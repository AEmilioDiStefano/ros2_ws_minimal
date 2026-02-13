#!/usr/bin/env python3
"""
terminal_orchestrator.py

Terminal menu orchestrator for Robot Legion playbooks.

Goals:
- Keep UI friendly for tall, narrow terminals.
- Provide simple playbook forms with variable prompts.
- Send ExecutePlaybook goals to one robot or all discovered robots.
- Use consistent, rough distance/time math across robots.
"""

from __future__ import annotations

import json
import math
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from fleet_orchestrator_interfaces.action import ExecutePlaybook
from .drive_profiles import load_profile_registry, resolve_robot_profile
from .playbook_strategies import compile_transit_xy_plans


ACTION_RE = re.compile(r"^/([^/]+)/execute_playbook$")
ACTION_SEND_GOAL_SVC_RE = re.compile(r"^/([^/]+)/execute_playbook/_action/send_goal$")
ACTION_TYPE = "fleet_orchestrator_interfaces/action/ExecutePlaybook"
ACTION_SEND_GOAL_TYPE = "fleet_orchestrator_interfaces/action/ExecutePlaybook_SendGoal"
UI_WIDTH = 31  # Match teleop's narrow terminal style.


@dataclass
class GoalReport:
    robot: str
    accepted: bool
    success: bool
    reason: str


@dataclass
class TwoLegPlan:
    """
    Two-segment robot-relative motion plan:
      rotate(theta1) -> transit(length1) -> rotate(theta2-theta1) -> transit(length2)
    """
    theta1: float
    theta2: float
    length1_m: float
    length2_m: float
    max_detour_m: float


class TerminalOrchestrator(Node):
    def __init__(self):
        super().__init__("terminal_orchestrator")

        # Rough motion constants used for distance -> duration estimates.
        self.declare_parameter("base_linear_mps", 0.40)
        self.declare_parameter("base_strafe_mps", 0.35)
        self.declare_parameter("base_angular_rps", 2.00)
        self.declare_parameter("default_speed_scale", 1.0)

        self.base_linear_mps = float(self.get_parameter("base_linear_mps").value)
        self.base_strafe_mps = float(self.get_parameter("base_strafe_mps").value)
        self.base_angular_rps = float(self.get_parameter("base_angular_rps").value)
        self.default_speed_scale = float(self.get_parameter("default_speed_scale").value)

        # Do not use name "_clients": rclpy.Node already uses that internally.
        self._action_clients: Dict[str, ActionClient] = {}
        self.selected_robot: Optional[str] = None  # None => all robots

        # Fleet-style JSON event emission for integration testing.
        self.declare_parameter("publish_fleet_topics", False)
        self.publish_fleet_topics = bool(self.get_parameter("publish_fleet_topics").value)
        self.task_pub = self.create_publisher(String, "/fo/task", 10)
        self.audit_pub = self.create_publisher(String, "/fo/audit", 10)

        # Best-effort registry for robot type/hardware previews.
        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        self._profile_registry = None
        try:
            self._profile_registry = load_profile_registry(profiles_path)
        except Exception:
            self._profile_registry = None

    # ---------------- UI helpers ----------------

    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        """
        Allow ROS graph discovery to populate before first menu render.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            if self._discover_action_servers():
                return

    def _clear(self):
        # ANSI clear + home cursor.
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def _divider(self) -> str:
        return "-" * UI_WIDTH

    def _fit(self, text: str) -> List[str]:
        text = str(text)
        if len(text) <= UI_WIDTH:
            return [text]
        words = text.split(" ")
        out: List[str] = []
        cur = ""
        for w in words:
            if not cur:
                cur = w
            elif len(cur) + 1 + len(w) <= UI_WIDTH:
                cur += " " + w
            else:
                out.append(cur)
                cur = w
        if cur:
            out.append(cur)
        return out

    def _print_screen(self, title: str, lines: List[str]):
        self._clear()
        print(self._divider())
        for ln in self._fit(title[:UI_WIDTH]):
            print(ln)
        print(self._divider())
        for ln in lines:
            for part in self._fit(ln):
                print(part)
        print(self._divider())

    def _pause(self, text: str = "Press Enter..."):
        input((text + " ").strip())

    def _prompt_float(
        self,
        label: str,
        default: Optional[float] = None,
        allow_zero: bool = True,
    ) -> Optional[float]:
        while True:
            if default is None:
                raw = input(f"{label}: ").strip()
            else:
                raw = input(f"{label} [{default}]: ").strip()
                if raw == "":
                    return float(default)

            if raw.lower() in ("q", "quit", "b", "back"):
                return None
            try:
                val = float(raw)
            except ValueError:
                print("Enter a number.")
                continue

            if not allow_zero and abs(val) < 1e-9:
                print("Value cannot be 0.")
                continue
            return val

    def _emit_json_event(self, topic_type: str, payload: dict):
        """
        Emit fleet-style JSON to terminal and optionally to /fo/task or /fo/audit.
        """
        # Pretty JSON for copy/paste into future fleet_orchestrator integration work.
        raw = json.dumps(payload, indent=2, sort_keys=True)
        print(raw)
        sys.stdout.flush()

        if not self.publish_fleet_topics:
            return

        msg = String()
        msg.data = raw
        if topic_type == "task":
            self.task_pub.publish(msg)
        elif topic_type == "audit":
            self.audit_pub.publish(msg)

    def _resolve_robot_profile_preview(self, robot: str) -> Dict[str, str]:
        out = {
            "drive_type": "unknown",
            "hardware": "unknown",
            "profile_name": "",
            "source": "none",
        }
        if not self._profile_registry:
            return out
        try:
            prof = resolve_robot_profile(self._profile_registry, robot)
            out["drive_type"] = str(prof.get("drive_type") or "unknown")
            out["hardware"] = str(prof.get("hardware") or "unknown")
            out["profile_name"] = str(prof.get("profile_name") or "")
            out["source"] = "registry"
        except Exception:
            pass
        return out

    def _twist_step_text(self, tw) -> str:
        vx = float(getattr(tw.linear, "x", 0.0))
        vy = float(getattr(tw.linear, "y", 0.0))
        wz = float(getattr(tw.angular, "z", 0.0))
        if abs(wz) > 1e-6 and abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return "rotate_left" if wz > 0.0 else "rotate_right"
        if abs(vx) > 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            return "forward" if vx > 0.0 else "backward"
        if abs(vy) > 1e-6 and abs(vx) < 1e-6 and abs(wz) < 1e-6:
            return "strafe_left" if vy > 0.0 else "strafe_right"
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            return "hold"
        return "mixed_motion"

    def _build_motion_preview(self, robot: str, goal: ExecutePlaybook.Goal) -> Dict[str, object]:
        profile = self._resolve_robot_profile_preview(robot)
        preview: Dict[str, object] = {
            "robot": robot,
            "drive_type": profile["drive_type"],
            "hardware": profile["hardware"],
            "profile_name": profile["profile_name"],
            "profile_source": profile["source"],
            "cardinal_mode": "relative_fallback",
            "strategy_id": "",
            "steps": [],
            "notes": [],
        }
        if goal.command_id != "transit_xy":
            return preview

        try:
            params = json.loads(goal.parameters_json or "{}")
        except Exception:
            params = {}
        speed_scale = max(0.05, min(1.5, float(params.get("speed", 1.0))))
        compiled = compile_transit_xy_plans(
            drive_type=str(profile["drive_type"]),
            hardware=str(profile["hardware"]),
            north_m=float(getattr(goal, "north_m", 0.0)),
            east_m=float(getattr(goal, "east_m", 0.0)),
            v_fwd=self.base_linear_mps * speed_scale,
            v_strafe=self.base_strafe_mps * speed_scale,
            w_rot=self.base_angular_rps * speed_scale,
            cardinal_mode="relative_fallback",
            heading_rad=None,
        )
        preview["strategy_id"] = compiled.strategy_id
        preview["steps"] = [
            {
                "index": i,
                "motion": self._twist_step_text(plan.twist),
                "duration_s": round(float(plan.duration_s), 3),
                "status_text": str(plan.status_text),
            }
            for i, plan in enumerate(compiled.plans, start=1)
        ]
        preview["notes"] = [
            "north/east uses robot body frame without heading sensor",
            "robots with different starting headings diverge in world frame",
        ]
        return preview

    # ---------------- Coordinated XY linear algebra planner ----------------

    def _rotation_params_from_delta(self, delta_rad: float, speed_scale: float) -> Dict[str, object]:
        """
        Convert a signed rotation delta into executor rotate params.
        """
        direction = "left" if delta_rad >= 0.0 else "right"
        duration_s = abs(delta_rad) / max(1e-6, self.base_angular_rps * speed_scale)
        return {
            "direction": direction,
            "duration_s": duration_s,
            "speed": speed_scale,
        }

    def _transit_params_from_length(self, length_m: float, speed_scale: float) -> Dict[str, object]:
        """
        Convert a signed path length into executor transit params.
        """
        direction = "forward" if length_m >= 0.0 else "backward"
        duration_s = abs(length_m) / max(1e-6, self.base_linear_mps * speed_scale)
        return {
            "direction": direction,
            "duration_s": duration_s,
            "speed": speed_scale,
        }

    def _line_detour_distance(self, waypoint: Tuple[float, float], goal_xy: Tuple[float, float]) -> float:
        """
        Max lateral deviation from the straight line start->goal is the perpendicular
        distance from the intermediate waypoint to that line.
        """
        wx, wy = waypoint
        gx, gy = goal_xy
        norm = math.hypot(gx, gy)
        if norm < 1e-9:
            return 0.0
        # 2D cross-product magnitude divided by |goal|.
        return abs(wx * gy - wy * gx) / norm

    def _solve_two_leg_lengths(self, dx: float, dy: float, theta1: float, theta2: float) -> Optional[Tuple[float, float]]:
        """
        Solve the 2x2 linear system:
          l1*[cos(theta1), sin(theta1)] + l2*[cos(theta2), sin(theta2)] = [dx, dy]

        This is the core linear algebra step that transforms two heading choices into
        two segment lengths that exactly reach the destination vector.
        """
        a11, a12 = math.cos(theta1), math.cos(theta2)
        a21, a22 = math.sin(theta1), math.sin(theta2)
        det = a11 * a22 - a12 * a21
        if abs(det) < 1e-6:
            return None
        inv11, inv12 = a22 / det, -a12 / det
        inv21, inv22 = -a21 / det, a11 / det
        l1 = inv11 * dx + inv12 * dy
        l2 = inv21 * dx + inv22 * dy
        return (l1, l2)

    def _signed_distance_to_line(self, point_xy: Tuple[float, float], line_dir_xy: Tuple[float, float]) -> float:
        """
        Signed perpendicular distance from point to the infinite line through origin
        with direction line_dir_xy.
        """
        px, py = point_xy
        lx, ly = line_dir_xy
        denom = math.hypot(lx, ly)
        if denom < 1e-9:
            return 0.0
        return (px * ly - py * lx) / denom

    def _plan_two_leg_deterministic(
        self,
        dx: float,
        dy: float,
        turn_side: str,
        max_detour_m: float,
        theta2_fixed: float,
        target_max_detour_m: float,
        ref_line_dir_xy: Tuple[float, float],
        max_samples: int = 120,
    ) -> Optional[TwoLegPlan]:
        """
        Deterministic two-leg planner using constrained linear algebra search.

        We keep a common final heading theta2_fixed for all robots, then scan theta1
        (same turn side, smaller magnitude) and solve segment lengths from:
          l1*u(theta1) + l2*u(theta2) = [dx, dy]

        Candidate quality is based on closeness to target_max_detour_m while enforcing:
        - forward+forward legs (l1,l2 > 0),
        - same-side rotations,
        - max detour bound.
        """
        side_sign = +1.0 if turn_side == "left" else -1.0
        min_abs = math.radians(4.0)
        max_abs = math.radians(70.0)
        theta2 = side_sign * min(max_abs, max(min_abs, abs(theta2_fixed)))
        h2 = abs(theta2)

        # theta1 must be smaller magnitude than theta2 for same-side second rotation.
        h1_min = min_abs
        h1_max = h2 - math.radians(2.0)
        if h1_max <= h1_min:
            return None

        best: Optional[TwoLegPlan] = None
        best_err = float("inf")

        for i in range(max_samples):
            ratio = i / max(1, max_samples - 1)
            h1 = h1_min + ratio * (h1_max - h1_min)
            theta1 = side_sign * h1

            solved = self._solve_two_leg_lengths(dx=dx, dy=dy, theta1=theta1, theta2=theta2)
            if solved is None:
                continue
            l1, l2 = solved
            # Enforce forward+forward style.
            if l1 <= 0.02 or l2 <= 0.02:
                continue

            # Same-side second rotation check.
            dtheta2 = theta2 - theta1
            if side_sign > 0 and dtheta2 <= 0.0:
                continue
            if side_sign < 0 and dtheta2 >= 0.0:
                continue

            wx = l1 * math.cos(theta1)
            wy = l1 * math.sin(theta1)
            d_waypoint = abs(self._signed_distance_to_line((wx, wy), ref_line_dir_xy))
            d_goal = abs(self._signed_distance_to_line((dx, dy), ref_line_dir_xy))
            # For a polyline, max distance from reference line occurs at segment endpoints.
            achieved = max(d_waypoint, d_goal)
            if achieved > max_detour_m + 1e-9:
                continue

            err = abs(achieved - target_max_detour_m)
            if err < best_err:
                best_err = err
                best = TwoLegPlan(
                theta1=theta1,
                theta2=theta2,
                length1_m=l1,
                length2_m=l2,
                max_detour_m=achieved,
            )
        return best

    def _robot_goal_vectors_for_formation(
        self,
        robots: List[str],
        anchor_robot: str,
        x_m: float,
        y_m: float,
        spacing_m: float,
        formation_dir_rad: float,
    ) -> Dict[str, Tuple[float, float]]:
        """
        Compute per-robot destination vectors in anchor-relative formation terms.

        Assumption (explicitly surfaced to user in UI/docs):
        robots begin in a line referenced to anchor_robot with spacing spacing_m
        and direction formation_dir_rad in the same body frame used by x/y.

        Linear algebra simplification:
        - Let v = [x, y] be anchor destination vector.
        - Let s = spacing * [cos(phi), sin(phi)] be formation axis vector.
        - Robot k (relative index from anchor) gets v + k*s.
        """
        ordered = list(robots)
        if anchor_robot not in ordered:
            return {r: (x_m, y_m) for r in ordered}

        anchor_idx = ordered.index(anchor_robot)
        sx = spacing_m * math.cos(formation_dir_rad)
        sy = spacing_m * math.sin(formation_dir_rad)

        out: Dict[str, Tuple[float, float]] = {}
        for idx, robot in enumerate(ordered):
            k = idx - anchor_idx
            out[robot] = (x_m + k * sx, y_m + k * sy)
        return out

    # ---------------- ROS discovery/action ----------------

    def _discover_action_servers(self) -> List[Tuple[str, str]]:
        """
        Returns list of (robot_name, action_name).
        """
        found: Dict[str, str] = {}

        # Primary path: native action graph API.
        try:
            action_names_and_types = self.get_action_names_and_types()
        except Exception:
            action_names_and_types = []

        for action_name, types in action_names_and_types:
            if ACTION_TYPE not in types:
                continue
            m = ACTION_RE.match(action_name)
            if not m:
                continue
            found[m.group(1)] = action_name

        # Fallback path: discover action servers via send_goal service endpoints.
        # This is more robust on some DDS/graph combinations where action introspection
        # lags while services are already visible.
        if not found:
            try:
                services_and_types = self.get_service_names_and_types()
            except Exception:
                services_and_types = []
            for service_name, types in services_and_types:
                if ACTION_SEND_GOAL_TYPE not in types:
                    continue
                m = ACTION_SEND_GOAL_SVC_RE.match(service_name)
                if not m:
                    continue
                robot = m.group(1)
                found[robot] = f"/{robot}/execute_playbook"

        pairs = sorted(found.items(), key=lambda p: p[0])
        return pairs

    def _robots_map(self) -> Dict[str, str]:
        return {robot: action for robot, action in self._discover_action_servers()}

    def _get_or_make_client(self, action_name: str) -> ActionClient:
        client = self._action_clients.get(action_name)
        if client is None:
            client = ActionClient(self, ExecutePlaybook, action_name)
            self._action_clients[action_name] = client
        return client

    def _send_goal(self, robot: str, action_name: str, goal: ExecutePlaybook.Goal) -> GoalReport:
        client = self._get_or_make_client(action_name)
        if not client.wait_for_server(timeout_sec=2.0):
            return GoalReport(robot=robot, accepted=False, success=False, reason="server unavailable")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            return GoalReport(robot=robot, accepted=False, success=False, reason="goal send timeout")

        goal_handle = send_future.result()
        if goal_handle is None:
            return GoalReport(robot=robot, accepted=False, success=False, reason="no goal handle")
        if not goal_handle.accepted:
            return GoalReport(robot=robot, accepted=False, success=False, reason="goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)
        if not result_future.done():
            return GoalReport(robot=robot, accepted=True, success=False, reason="result timeout")

        wrapped = result_future.result()
        result_msg = wrapped.result if wrapped is not None else None
        if result_msg is None:
            return GoalReport(robot=robot, accepted=True, success=False, reason="empty result")

        success = bool(getattr(result_msg, "success", True))
        reason = str(getattr(result_msg, "reason", "")).strip() or ("ok" if success else "failed")
        accepted = bool(getattr(result_msg, "accepted", True))
        return GoalReport(robot=robot, accepted=accepted, success=success, reason=reason)

    def _target_actions(self, robots_map: Dict[str, str]) -> List[Tuple[str, str]]:
        if self.selected_robot:
            action = robots_map.get(self.selected_robot)
            if action:
                return [(self.selected_robot, action)]
            return []
        return [(r, a) for r, a in robots_map.items()]

    def _spin_until(self, done_fn, timeout_sec: float, step_sec: float = 0.05) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            if done_fn():
                return True
            now = time.monotonic()
            if now >= deadline:
                return done_fn()
            rclpy.spin_once(self, timeout_sec=min(step_sec, deadline - now))
        return done_fn()

    def _dispatch_goal(self, goal_factory) -> List[GoalReport]:
        robots_map = self._robots_map()
        targets = self._target_actions(robots_map)
        reports: List[GoalReport] = []

        if not targets:
            return reports

        pending: Dict[str, Dict[str, object]] = {}

        # Phase 1: send all goals first so robots can start at roughly the same time.
        for robot, action_name in targets:
            client = self._get_or_make_client(action_name)
            if not client.wait_for_server(timeout_sec=2.0):
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason="server unavailable")
                )
                continue
            goal = goal_factory(robot)
            preview = self._build_motion_preview(robot, goal)
            dispatch_task = {
                "type": "playbook_command",
                "intent_id": str(goal.intent_id),
                "command_id": str(goal.command_id),
                "platform": {
                    "adapter_type": "terminal_orchestrator",
                    "platform_family": "robot_legion_teleop_python",
                },
                "parameters": json.loads(goal.parameters_json or "{}") if goal.parameters_json else {},
                "targets": robot,
                "trace": {
                    "translator": "terminal_orchestrator",
                    "confidence": 1.0,
                    "explanation": "manual terminal playbook dispatch",
                },
                "preview": preview,
            }
            self._emit_json_event("task", dispatch_task)
            send_future = client.send_goal_async(goal)
            pending[robot] = {"send_future": send_future, "intent_id": str(goal.intent_id)}

        if not pending:
            return reports

        # Wait for send_goal responses.
        self._spin_until(
            lambda: all(state["send_future"].done() for state in pending.values()),
            timeout_sec=5.0,
        )

        result_pending: Dict[str, Dict[str, object]] = {}
        for robot, state in pending.items():
            send_future = state["send_future"]
            if not send_future.done():
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason="goal send timeout")
                )
                continue

            goal_handle = send_future.result()
            if goal_handle is None:
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason="no goal handle")
                )
                continue
            if not goal_handle.accepted:
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason="goal rejected")
                )
                continue

            result_pending[robot] = {
                "future": goal_handle.get_result_async(),
                "intent_id": state.get("intent_id", ""),
            }

        if result_pending:
            self._spin_until(
                lambda: all(state["future"].done() for state in result_pending.values()),
                timeout_sec=90.0,
            )

        for robot, state in result_pending.items():
            result_future = state["future"]
            intent_id = str(state.get("intent_id", ""))
            if not result_future.done():
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "failed",
                        "details": "result timeout",
                    },
                )
                reports.append(
                    GoalReport(robot=robot, accepted=True, success=False, reason="result timeout")
                )
                continue

            wrapped = result_future.result()
            result_msg = wrapped.result if wrapped is not None else None
            if result_msg is None:
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "failed",
                        "details": "empty result",
                    },
                )
                reports.append(
                    GoalReport(robot=robot, accepted=True, success=False, reason="empty result")
                )
                continue

            success = bool(getattr(result_msg, "success", True))
            reason = str(getattr(result_msg, "reason", "")).strip() or ("ok" if success else "failed")
            accepted = bool(getattr(result_msg, "accepted", True))
            self._emit_json_event(
                "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "succeeded" if success else "failed",
                    "accepted": accepted,
                    "reason": reason,
                },
            )
            reports.append(
                GoalReport(robot=robot, accepted=accepted, success=success, reason=reason)
            )

        return reports

    # ---------------- Menus ----------------

    def _target_menu(self):
        while rclpy.ok():
            robots_map = self._robots_map()
            robots = sorted(robots_map.keys())
            lines = [
                "Select target robots",
                "",
                "a) all robots",
            ]
            for i, robot in enumerate(robots, start=1):
                lines.append(f"{i}) {robot}")
            lines.extend(["", "b) back"])
            self._print_screen("TARGET SELECT", lines)

            choice = input("Choice: ").strip().lower()
            if choice in ("b", "back", "q", "quit"):
                return
            if choice == "a":
                self.selected_robot = None
                return
            try:
                idx = int(choice)
            except ValueError:
                continue
            if 1 <= idx <= len(robots):
                self.selected_robot = robots[idx - 1]
                return

    def _render_reports(self, reports: List[GoalReport]):
        lines = ["Execution report", ""]
        if not reports:
            lines.extend(["No target robots found."])
        else:
            for rep in reports:
                if rep.accepted and rep.success:
                    lines.append(f"{rep.robot}: OK")
                else:
                    lines.append(f"{rep.robot}: FAIL")
                    lines.append(f"  {rep.reason}")
        self._print_screen("RESULT", lines)
        self._pause()

    def _confirm(self, summary_lines: List[str]) -> bool:
        self._print_screen("EXECUTION SUMMARY", summary_lines + ["", "EXECUTE? y/n"])
        answer = input("> ").strip().lower()
        return answer in ("y", "yes")

    # ---------------- Playbooks ----------------

    def _playbook_move_xy(self):
        self._print_screen(
            "PLAYBOOK: MOVE XY",
            [
                "detected-robot XY planner",
                "x: forward(+) backward(-)",
                "y: right(+) left(-)",
                "main robot goes straight",
                "others use 2-leg planner",
                "",
                "Type 'back' to cancel",
            ],
        )
        # Requirement: this playbook targets ALL currently detected robots.
        detected_robots = sorted(self._robots_map().keys())
        if not detected_robots:
            self._print_screen(
                "MOVE XY REQUIREMENT",
                [
                    "No detected robots.",
                    "Start robot bringup and retry.",
                ],
            )
            self._pause()
            return

        x_m = self._prompt_float("x meters", default=1.0)
        if x_m is None:
            return
        y_m = self._prompt_float("y meters", default=0.0)
        if y_m is None:
            return
        speed = self._prompt_float("speed scale", default=self.default_speed_scale, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        spacing_m = self._prompt_float("robot spacing m", default=0.45, allow_zero=False)
        if spacing_m is None:
            return
        formation_dir_rad = self._prompt_float("formation dir rad", default=0.0)
        if formation_dir_rad is None:
            return
        max_detour_m = self._prompt_float("MAX PATH OFFSET m", default=0.8, allow_zero=False)
        if max_detour_m is None:
            return
        max_detour_m = max(0.05, float(max_detour_m))

        turn_side_raw = input("turn side [left/right] [left]: ").strip().lower()
        turn_side = turn_side_raw if turn_side_raw in ("left", "right") else "left"

        # Ask user to pick main robot (anchor); all destinations are defined relative to it.
        self._print_screen(
            "MAIN ROBOT SELECT",
            [
                "Choose main robot (anchor)",
                "Formation offsets are",
                "interpreted from this robot.",
                "",
            ] + [f"{i+1}) {r}" for i, r in enumerate(detected_robots)],
        )
        main_choice = input("Main robot index [1]: ").strip()
        main_idx = 1
        if main_choice:
            try:
                main_idx = int(main_choice)
            except ValueError:
                main_idx = 1
        main_idx = min(max(main_idx, 1), len(detected_robots))
        main_robot = detected_robots[main_idx - 1]

        goal_vectors = self._robot_goal_vectors_for_formation(
            robots=detected_robots,
            anchor_robot=main_robot,
            x_m=float(x_m),
            y_m=float(y_m),
            spacing_m=float(spacing_m),
            formation_dir_rad=float(formation_dir_rad),
        )

        # Reference straight path is main robot's start->goal line.
        main_goal = goal_vectors.get(main_robot, (float(x_m), float(y_m)))
        ref_line = main_goal if math.hypot(main_goal[0], main_goal[1]) > 1e-9 else (1.0, 0.0)

        side_sign = +1.0 if turn_side == "left" else -1.0
        base_heading = math.atan2(float(main_goal[1]), float(main_goal[0])) if (abs(main_goal[0]) + abs(main_goal[1])) > 1e-9 else 0.0
        # Shared terminal heading for non-main robots so all end with same orientation.
        theta2_common = side_sign * min(math.radians(60.0), max(math.radians(12.0), abs(base_heading) + math.radians(8.0)))

        # Detour assignment rule:
        # - main robot: 0 (straight path)
        # - "second robot": next detected robot (excluding main) gets max_detour_m
        # - remaining robots: evenly spaced detour levels between 0 and max_detour_m
        detour_targets: Dict[str, float] = {main_robot: 0.0}
        non_main = [r for r in detected_robots if r != main_robot]
        secondary_robot = non_main[0] if non_main else None
        if secondary_robot:
            detour_targets[secondary_robot] = max_detour_m
        remaining = [r for r in non_main if r != secondary_robot]
        if remaining:
            step = max_detour_m / float(len(remaining) + 1)
            for i, robot in enumerate(remaining, start=1):
                detour_targets[robot] = i * step

        plans: Dict[str, TwoLegPlan] = {}
        failures: List[str] = []
        for robot in detected_robots:
            dx, dy = goal_vectors[robot]
            if robot == main_robot:
                # Main robot path: one rotate + one transit (straight to objective).
                heading = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-9 else 0.0
                dist = math.hypot(dx, dy)
                plans[robot] = TwoLegPlan(
                    theta1=heading,
                    theta2=heading,
                    length1_m=dist,
                    length2_m=0.0,
                    max_detour_m=abs(self._signed_distance_to_line((dx, dy), ref_line)),
                )
            else:
                plan = self._plan_two_leg_deterministic(
                    dx=dx,
                    dy=dy,
                    turn_side=turn_side,
                    max_detour_m=max_detour_m,
                    theta2_fixed=theta2_common,
                    target_max_detour_m=float(detour_targets.get(robot, 0.0)),
                    ref_line_dir_xy=ref_line,
                )
                if plan is None:
                    failures.append(robot)
                else:
                    plans[robot] = plan

        if failures:
            self._print_screen(
                "PLANNER FAILED",
                [
                    "No feasible deterministic plan",
                    "under current constraints for:",
                    ", ".join(failures),
                    "",
                    "Try: larger MAX PATH OFFSET",
                    "or different turn side.",
                ],
            )
            self._pause()
            return

        est_total_seq = 0.0
        for robot in target_robots:
            p = plans[robot]
            r1 = abs(p.theta1) / max(1e-6, self.base_angular_rps * speed)
            t1 = abs(p.length1_m) / max(1e-6, self.base_linear_mps * speed)
            r2 = abs(p.theta2 - p.theta1) / max(1e-6, self.base_angular_rps * speed)
            t2 = abs(p.length2_m) / max(1e-6, self.base_linear_mps * speed)
            est_total_seq = max(est_total_seq, r1 + t1 + r2 + t2)

        target_text = f"detected={len(detected_robots)}"
        summary = [
            f"Target: {target_text}",
            "Mode: detected-robot XY",
            "",
            f"Main robot: {main_robot}",
            f"Base x={x_m:.2f} y={y_m:.2f}",
            f"Spacing={spacing_m:.2f} m",
            f"Formation dir={formation_dir_rad:.3f} rad",
            f"Turn side={turn_side}",
            f"MAX PATH OFFSET={max_detour_m:.2f} m",
            "",
            f"Est phase total: {est_total_seq:.2f} s",
            "",
            "Frame: robot-relative",
            "(no heading alignment)",
        ]

        for robot in detected_robots:
            dx, dy = goal_vectors[robot]
            p = plans[robot]
            summary.append(f"{robot}: dx={dx:.2f} dy={dy:.2f}")
            summary.append(
                f"  th1={math.degrees(p.theta1):.1f}deg "
                f"th2={math.degrees(p.theta2):.1f}deg "
                f"detour={p.max_detour_m:.2f}m "
                f"target={detour_targets.get(robot, 0.0):.2f}m"
            )

        if not self._confirm(summary):
            return

        now = int(time.time())
        phase_reports: List[GoalReport] = []

        # Build phase helpers.
        def _build_phase_goal(robot: str, phase_label: str, command_id: str, params: Dict[str, object]) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_xy_{phase_label}_{robot}_{now}"
            g.command_id = command_id
            g.vehicle_ids = [robot]
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        # Emit a high-level planner event for future fleet_orchestrator integration.
        self._emit_json_event(
            "task",
            {
                "type": "playbook_command",
                "intent_id": f"term_xy_plan_{now}",
                "command_id": "transit_xy_deterministic_formation",
                "platform": {
                    "adapter_type": "terminal_orchestrator",
                    "platform_family": "robot_legion_teleop_python",
                },
                "targets": detected_robots,
                "parameters": {
                    "x_m": x_m,
                    "y_m": y_m,
                    "speed": speed,
                    "spacing_m": spacing_m,
                    "formation_dir_rad": formation_dir_rad,
                    "turn_side": turn_side,
                    "max_path_offset_m": max_detour_m,
                    "main_robot": main_robot,
                    "secondary_robot": secondary_robot,
                },
                "trace": {
                    "translator": "terminal_orchestrator",
                    "confidence": 1.0,
                    "explanation": "deterministic two-leg linear algebra planner",
                },
                "plans": {
                    robot: {
                        "dx": goal_vectors[robot][0],
                        "dy": goal_vectors[robot][1],
                        "theta1_deg": math.degrees(plans[robot].theta1),
                        "theta2_deg": math.degrees(plans[robot].theta2),
                        "length1_m": plans[robot].length1_m,
                        "length2_m": plans[robot].length2_m,
                        "max_detour_m": plans[robot].max_detour_m,
                        "target_detour_m": detour_targets.get(robot, 0.0),
                    }
                    for robot in detected_robots
                },
            },
        )

        # Phase 1: first rotate for all robots (parallel dispatch).
        def build_phase1(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._rotation_params_from_delta(delta_rad=p.theta1, speed_scale=speed)
            return _build_phase_goal(robot, "p1_rotate", "rotate", params)

        phase_reports.extend(self._dispatch_goal(build_phase1))
        if any((not r.success) for r in phase_reports[-len(detected_robots):]):
            self._render_reports(phase_reports)
            return

        # Phase 2: first transit for all robots.
        def build_phase2(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._transit_params_from_length(length_m=p.length1_m, speed_scale=speed)
            return _build_phase_goal(robot, "p2_transit", "transit", params)

        phase_reports.extend(self._dispatch_goal(build_phase2))
        if any((not r.success) for r in phase_reports[-len(detected_robots):]):
            self._render_reports(phase_reports)
            return

        # Phase 3: second rotate for all robots.
        def build_phase3(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._rotation_params_from_delta(delta_rad=(p.theta2 - p.theta1), speed_scale=speed)
            return _build_phase_goal(robot, "p3_rotate", "rotate", params)

        phase_reports.extend(self._dispatch_goal(build_phase3))
        if any((not r.success) for r in phase_reports[-len(detected_robots):]):
            self._render_reports(phase_reports)
            return

        # Phase 4: second transit for all robots.
        def build_phase4(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._transit_params_from_length(length_m=p.length2_m, speed_scale=speed)
            return _build_phase_goal(robot, "p4_transit", "transit", params)

        phase_reports.extend(self._dispatch_goal(build_phase4))
        self._render_reports(phase_reports)

    def _playbook_execute_all_commands(self):
        """
        Validation sweep:
        Execute one safe example of each supported playbook primitive on all target robots.
        This is intended as a quick integration test, not mission behavior.
        """
        self._print_screen(
            "PLAYBOOK: ALL COMMANDS",
            [
                "Runs all primitive commands",
                "on selected target robots.",
                "",
                "Type 'back' to cancel",
            ],
        )

        speed = self._prompt_float("speed scale", default=0.8, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        target_text = self.selected_robot or "ALL"
        summary = [
            f"Target: {target_text}",
            "",
            "Command sweep order:",
            "1) hold",
            "2) transit",
            "3) rotate",
            "4) strafe",
            "5) diagonal",
            "6) turn",
            "7) transit_xy",
            "",
            f"Speed scale: {speed:.2f}",
            "Purpose: playbook smoke test",
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())
        reports: List[GoalReport] = []

        # Fixed safe-ish parameter set for deterministic testing.
        command_plan = [
            ("hold", {"duration_s": 0.5, "speed": speed}),
            ("transit", {"direction": "forward", "duration_s": 0.8, "speed": speed}),
            ("rotate", {"direction": "left", "duration_s": 0.5, "speed": speed}),
            ("strafe", {"direction": "right", "duration_s": 0.7, "speed": speed}),
            ("diagonal", {"direction": "fwd_right", "duration_s": 0.7, "speed": speed}),
            ("turn", {"direction": "left", "duration_s": 0.7, "speed": speed}),
            ("transit_xy", {"north_m": 0.25, "east_m": 0.15, "speed": speed}),
        ]

        self._emit_json_event(
            "task",
            {
                "type": "playbook_command",
                "intent_id": f"term_all_commands_plan_{now}",
                "command_id": "playbook_command_sweep",
                "platform": {
                    "adapter_type": "terminal_orchestrator",
                    "platform_family": "robot_legion_teleop_python",
                },
                "targets": target_text,
                "parameters": {"speed": speed, "command_count": len(command_plan)},
                "trace": {
                    "translator": "terminal_orchestrator",
                    "confidence": 1.0,
                    "explanation": "manual all-commands validation sweep",
                },
            },
        )

        for idx, (command_id, params) in enumerate(command_plan, start=1):
            phase = f"c{idx}_{command_id}"

            def build_goal(robot: str, cid=command_id, p=params, ph=phase) -> ExecutePlaybook.Goal:
                g = ExecutePlaybook.Goal()
                g.intent_id = f"term_all_{ph}_{robot}_{now}"
                g.command_id = cid
                g.vehicle_ids = [robot]
                if cid == "transit_xy":
                    g.north_m = float(p.get("north_m", 0.0))
                    g.east_m = float(p.get("east_m", 0.0))
                g.parameters_json = json.dumps(p, separators=(",", ":"))
                return g

            # Dispatch each primitive in synchronized phase order across robots.
            reports.extend(self._dispatch_goal(build_goal))

        self._render_reports(reports)

    def _playbook_transit_distance(self):
        self._print_screen(
            "PLAYBOOK: TRANSIT DIST",
            [
                "Move forward/back by meters",
                "distance + => forward",
                "distance - => backward",
                "",
                "Type 'back' to cancel",
            ],
        )
        meters = self._prompt_float("distance meters", default=1.0, allow_zero=False)
        if meters is None:
            return
        speed = self._prompt_float("speed scale", default=self.default_speed_scale, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        direction = "forward" if meters >= 0.0 else "backward"
        duration_s = abs(meters) / max(1e-6, self.base_linear_mps * speed)
        target_text = self.selected_robot or "ALL"
        summary = [
            f"Target: {target_text}",
            "",
            f"Distance: {meters:.2f} m",
            f"Direction: {direction}",
            f"Duration: {duration_s:.2f} s",
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())

        def build_goal(robot: str) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_transit_{robot}_{now}"
            g.command_id = "transit"
            g.vehicle_ids = [robot]
            params = {
                "direction": direction,
                "duration_s": float(duration_s),
                "speed": float(speed),
            }
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        reports = self._dispatch_goal(build_goal)
        self._render_reports(reports)

    def _playbook_rotate_degrees(self):
        self._print_screen(
            "PLAYBOOK: ROTATE DEG",
            [
                "Rotate by degrees",
                "degrees + => left/ccw",
                "degrees - => right/cw",
                "",
                "Type 'back' to cancel",
            ],
        )
        degrees = self._prompt_float("degrees", default=45.0, allow_zero=False)
        if degrees is None:
            return
        speed = self._prompt_float("speed scale", default=self.default_speed_scale, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        direction = "left" if degrees >= 0.0 else "right"
        radians = abs(degrees) * 3.141592653589793 / 180.0
        duration_s = radians / max(1e-6, self.base_angular_rps * speed)
        target_text = self.selected_robot or "ALL"
        summary = [
            f"Target: {target_text}",
            "",
            f"Rotate: {degrees:.2f} deg",
            f"Direction: {direction}",
            f"Duration: {duration_s:.2f} s",
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())

        def build_goal(robot: str) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_rotate_{robot}_{now}"
            g.command_id = "rotate"
            g.vehicle_ids = [robot]
            params = {
                "direction": direction,
                "duration_s": float(duration_s),
                "speed": float(speed),
            }
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        reports = self._dispatch_goal(build_goal)
        self._render_reports(reports)

    # ---------------- Main loop ----------------

    def _main_menu(self) -> str:
        # Keep graph data fresh before rendering choices.
        rclpy.spin_once(self, timeout_sec=0.15)
        robots = sorted(self._robots_map().keys())
        active_target = self.selected_robot if self.selected_robot else "ALL"
        lines = [
            "Terminal playbook UI",
            "",
            f"Detected robots: {len(robots)}",
            f"Target: {active_target}",
            "",
            "1) Move objective (x,y)",
            "2) Execute ALL commands",
            "3) Transit distance",
            "4) Rotate degrees",
            "",
            "t) Select target",
            "r) Refresh",
            "q) Quit",
        ]
        if robots:
            lines.append("")
            lines.append("Robots:")
            for robot in robots:
                lines.append(f"- {robot}")
        else:
            lines.append("")
            lines.append("No execute_playbook servers")
        self._print_screen("TERMINAL ORCHESTRATOR", lines)
        return input("Choice: ").strip().lower()

    def run_ui(self):
        self._warmup_discovery(timeout_sec=1.2, spin_step_sec=0.1)
        while rclpy.ok():
            choice = self._main_menu()
            if choice == "q":
                return
            if choice == "r":
                # Redraw loop after a short graph update spin.
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            if choice == "t":
                self._target_menu()
                continue
            if choice == "1":
                self._playbook_move_xy()
                continue
            if choice == "2":
                self._playbook_execute_all_commands()
                continue
            if choice == "3":
                self._playbook_transit_distance()
                continue
            if choice == "4":
                self._playbook_rotate_degrees()
                continue


def main(args=None):
    rclpy.init(args=args)
    node = TerminalOrchestrator()
    try:
        node.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
