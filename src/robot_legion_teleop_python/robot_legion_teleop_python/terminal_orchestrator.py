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
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from fleet_orchestrator_interfaces.action import ExecutePlaybook


ACTION_RE = re.compile(r"^/([^/]+)/execute_playbook$")
ACTION_TYPE = "fleet_orchestrator_interfaces/action/ExecutePlaybook"
UI_WIDTH = 31  # Match teleop's narrow terminal style.


@dataclass
class GoalReport:
    robot: str
    accepted: bool
    success: bool
    reason: str


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

        self._clients: Dict[str, ActionClient] = {}
        self.selected_robot: Optional[str] = None  # None => all robots

    # ---------------- UI helpers ----------------

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

    # ---------------- ROS discovery/action ----------------

    def _discover_action_servers(self) -> List[Tuple[str, str]]:
        """
        Returns list of (robot_name, action_name).
        """
        pairs: List[Tuple[str, str]] = []
        try:
            action_names_and_types = self.get_action_names_and_types()
        except Exception:
            return pairs

        for action_name, types in action_names_and_types:
            if ACTION_TYPE not in types:
                continue
            m = ACTION_RE.match(action_name)
            if not m:
                continue
            pairs.append((m.group(1), action_name))
        pairs.sort(key=lambda p: p[0])
        return pairs

    def _robots_map(self) -> Dict[str, str]:
        return {robot: action for robot, action in self._discover_action_servers()}

    def _get_or_make_client(self, action_name: str) -> ActionClient:
        client = self._clients.get(action_name)
        if client is None:
            client = ActionClient(self, ExecutePlaybook, action_name)
            self._clients[action_name] = client
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

    def _dispatch_goal(self, goal_factory) -> List[GoalReport]:
        robots_map = self._robots_map()
        targets = self._target_actions(robots_map)
        reports: List[GoalReport] = []
        for robot, action_name in targets:
            goal = goal_factory(robot)
            reports.append(self._send_goal(robot, action_name, goal))
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
                "Move objective by x and y",
                "x: forward(+) backward(-)",
                "y: right(+) left(-)",
                "",
                "Type 'back' to cancel",
            ],
        )
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

        est_fx = abs(x_m) / max(1e-6, self.base_linear_mps * speed)
        est_fy = abs(y_m) / max(1e-6, self.base_strafe_mps * speed)
        est_total_seq = est_fx + est_fy
        target_text = self.selected_robot or "ALL"
        summary = [
            f"Target: {target_text}",
            "",
            f"{x_m:.2f} m in x dir",
            f"{y_m:.2f} m in right dir",
            "",
            f"Est fwd: {est_fx:.2f} s",
            f"Est strafe: {est_fy:.2f} s",
            f"Est total: {est_total_seq:.2f} s",
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())

        def build_goal(robot: str) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_xy_{robot}_{now}"
            g.command_id = "transit_xy"
            g.vehicle_ids = [robot]
            g.north_m = float(x_m)
            g.east_m = float(y_m)
            params = {"north_m": float(x_m), "east_m": float(y_m), "speed": float(speed)}
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        reports = self._dispatch_goal(build_goal)
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
        robots = sorted(self._robots_map().keys())
        active_target = self.selected_robot if self.selected_robot else "ALL"
        lines = [
            "Terminal playbook UI",
            "",
            f"Detected robots: {len(robots)}",
            f"Target: {active_target}",
            "",
            "1) Move objective (x,y)",
            "2) Transit distance",
            "3) Rotate degrees",
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
                self._playbook_transit_distance()
                continue
            if choice == "3":
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
