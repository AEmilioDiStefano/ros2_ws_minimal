#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
teleop_legion_key.py

GOAL
----
Provide a keyboard teleop program that:
     - cooked-mode printing while raw key reading is active
     - startup robot discovery + prompt-until-valid selection
     - press 'm' any time to switch robot (same prompt loop)
     - arrow keys supported (ESC [ A/B/C/D sequences)
     - speed profiles (i/o/p) and incremental scaling (q/r/w/e etc.)
     - republish last twist for "continuous motion" feel
     - offline watchdog: if /<robot>/cmd_vel loses subscribers, stop + reselect

  2) Works with heterogeneous-fleet architecture:
     - robot cmd_vel topics are namespaced: /<robot_name>/cmd_vel
     - drive type (diff vs mecanum) is auto-resolved using:
         a) heartbeat JSON on /<robot>/heartbeat   (preferred)
         b) config/robot_profiles.yaml registry    (fallback)
         c) safe default                            diff_drive
     - publishes active robot identity to BOTH legacy topics and new /legion topics

This file is intentionally heavily commented for newer ROS2 users.
"""

from __future__ import annotations

import json
import re
import sys
import select
import termios
import tty
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# These helpers come from our heterogeneous fleet tutorial.
# They let teleop learn drive_type from the same registry robots use locally.
from .drive_profiles import load_profile_registry, resolve_robot_profile


# We consider /<robot>/cmd_vel the "presence signal" that a robot is online.
# If that topic exists AND has at least one subscriber, teleop can drive it.
CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")


# ---------------------------------------------------------------------
# Terminal helpers (OLD behavior preserved)
# ---------------------------------------------------------------------
def restore_terminal_settings(settings):
    """Restore terminal settings (cooked mode)."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def set_raw_mode():
    """
    Put terminal into RAW mode.
    RAW mode lets us read keypresses instantly without waiting for Enter.
    """
    tty.setraw(sys.stdin.fileno())


def read_key_raw(timeout_s: float = 0.1) -> str:
    """
    Read a single keypress in RAW mode.

    Arrow keys arrive as escape sequences:
      ESC [ A  (up)
      ESC [ B  (down)
      ESC [ C  (right)
      ESC [ D  (left)

    This function reads those sequences and returns a combined string.
    """
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not rlist:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":  # ESC begins an escape sequence
        key += sys.stdin.read(2)
    return key


# ---------------------------------------------------------------------
# Small data container for easier future web integration
# ---------------------------------------------------------------------
@dataclass
class TeleopState:
    available_robots: List[str]
    active_robot: Optional[str]
    cmd_vel_topic: Optional[str]
    drive_type: str
    linear_speed: float
    strafe_speed: float
    angular_speed: float
    allow_cmd_vel_switching: bool
    wheel_separation: float


class RobotLegionTeleop(Node):
    """
    ROS2 node that implements the teleop program.

    Important ROS2 concept:
      - This node does not "connect" to a robot directly.
      - It only publishes Twist messages to a topic.
      - As long as the robot is on the ROS network and has a motor driver subscribed
        to that topic, it will move.
    """

    def __init__(self):
        super().__init__("robot_legion_teleop_python")

        # ---------------- Parameters (kept from OLD + extended) ----------------
        # If false, user can select a robot once but cannot switch afterwards.
        self.declare_parameter("allow_cmd_vel_switching", True)
        self.allow_cmd_vel_switching = bool(self.get_parameter("allow_cmd_vel_switching").value)

        # Used for circle-turn math (must match the robot's diff-drive geometry).
        # We keep this because circle turns are part of the OLD behavior.
        self.declare_parameter("wheel_separation", 0.18)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)

        # Optional: expose a separate strafe speed (useful for mecanum).
        self.declare_parameter("strafe_speed", 0.40)
        self.strafe_speed = float(self.get_parameter("strafe_speed").value)

        # ---------------- Drive-type registry (new architecture support) ----------------
        # We load the same registry file used elsewhere. This is a fallback.
        # Heartbeat is preferred because it reflects what the robot is actually running.
        self.profile_registry = load_profile_registry(None)

        # ---------------- Active robot publishers (legacy + new) ----------------
        # These topics let other systems (FPV mux, web UI, orchestrator tooling) know
        # which robot teleop currently controls.
        #
        # We publish BOTH legacy topics (from OLD) and new /legion/* topics to avoid
        # breaking older code.
        self.active_robot_pub_legacy = self.create_publisher(String, "/active_robot", 10)
        self.active_robot_teleop_pub_legacy = self.create_publisher(String, "/teleop/active_robot", 10)

        self.active_robot_pub = self.create_publisher(String, "/legion/active_robot", 10)
        self.active_robot_teleop_pub = self.create_publisher(String, "/legion/active_robot_teleop", 10)
        self.active_drive_pub = self.create_publisher(String, "/legion/active_drive_type", 10)

        # ---------------- Heartbeat subscription (new architecture support) ----------------
        # Heartbeat is best-effort. If it doesn't exist, everything still works.
        self.hb_sub = None
        self._last_hb_json: Optional[dict] = None

        # ---------------- Publisher created after robot selection ----------------
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None
        self.current_drive_type: str = "diff_drive"  # diff_drive or mecanum

        # ---------------- Speed profile (OLD behavior preserved) ----------------
        # These are operator-level speed scalars; they are NOT hardware calibration.
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # ---------------- Republish state (OLD behavior preserved) ----------------
        self.last_lin_mult = 0.0
        self.last_strafe_mult = 0.0  # new: for mecanum
        self.last_ang_mult = 0.0
        self.last_twist = Twist()
        self.is_moving = False

        # ---------------- Keymaps ----------------
        # The OLD keymap used (lin_mult, ang_mult).
        # We extend to (lin_mult, strafe_mult, ang_mult).
        #
        # IMPORTANT: We keep the *terminal function* of OLD:
        # - arrow keys work
        # - 7/9/1/3 are special circle turns (diff only)
        self.move_bindings: Dict[str, Tuple[str, Optional[int], Optional[int], Optional[int]]] = {}
        self._rebuild_move_bindings_for("diff_drive")

        self.speed_bindings = {
            "w": self._increase_both_speeds,
            "+": self._increase_both_speeds,
            "e": self._decrease_both_speeds,
            "-": self._decrease_both_speeds,
            "q": self._increase_linear_speed,
            "/": self._increase_linear_speed,
            "r": self._decrease_linear_speed,
            "*": self._decrease_linear_speed,
            "i": self._set_slow_profile,
            "o": self._set_medium_profile,
            "p": self._set_fast_profile,
        }

        # Timers (OLD behavior preserved)
        self.create_timer(0.1, self._republish_last_twist)
        self.create_timer(0.5, self._offline_watchdog)

        # Initial UI (OLD style)
        self._tprint(self.render_instructions(topic_name=None))
        self._tprint("[STARTUP] Discovering robots...")

    # -----------------------------------------------------------------
    # Terminal-safe printing (cooked-mode prints only)
    # -----------------------------------------------------------------
    def _tprint(self, text: str = ""):
        """
        Print reliably while we are using RAW mode for keypress reading.

        Why this exists:
          - In RAW mode, '\\n' may not return to column 0.
          - Over time your output drifts rightward.
          - Your thin SSH terminals become unreadable.

        Solution (OLD behavior):
          1) temporarily restore cooked mode
          2) print
          3) return to raw mode
        """
        if not sys.stdin.isatty():
            print(text)
            sys.stdout.flush()
            return

        try:
            restore_terminal_settings(self._terminal_settings)
        except Exception:
            print(text)
            sys.stdout.flush()
            return

        print(text if text else "")
        sys.stdout.flush()

        try:
            set_raw_mode()
        except Exception:
            pass

    # -----------------------------------------------------------------
    # ROS graph warm-up (fixes "empty list at startup" race)
    # -----------------------------------------------------------------
    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        """
        On startup, ROS discovery can lag briefly (DDS graph not fully populated).
        If we list robots immediately, you might see "(none found)" even though
        robots exist.

        This function spins briefly to let discovery complete, then returns early
        if any robots are seen.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            if self.list_available_robots():
                return

    # -----------------------------------------------------------------
    # State accessors (handy later for a web/LLM UI layer)
    # -----------------------------------------------------------------
    def get_state(self) -> TeleopState:
        robots = self.list_available_robots()
        return TeleopState(
            available_robots=robots,
            active_robot=self.current_robot_name,
            cmd_vel_topic=self.cmd_vel_topic,
            drive_type=self.current_drive_type,
            linear_speed=self.linear_speed,
            strafe_speed=self.strafe_speed,
            angular_speed=self.angular_speed,
            allow_cmd_vel_switching=self.allow_cmd_vel_switching,
            wheel_separation=self.wheel_separation,
        )

    # -----------------------------------------------------------------
    # Rendering (OLD-style look, but drive-type aware)
    # -----------------------------------------------------------------
    def render_instructions(self, topic_name: Optional[str]) -> str:
        """
        Return a multi-line instruction block as a string.
        We keep the OLD layout and add a small drive-type section.
        """
        lines = []
        lines.append("-------------------------------")
        lines.append("      ROBOT LEGION TELEOP      ")
        lines.append("-------------------------------")
        if topic_name:
            lines.append(f"Publishing Twist on: {topic_name}")
        else:
            lines.append("SELECT A ROBOT")
        lines.append(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        lines.append(f"wheel_separation (circle turns): {self.wheel_separation:.2f} m")

        if self.current_robot_name:
            lines.append("")
            lines.append(f"ACTIVE ROBOT: {self.current_robot_name}")
            lines.append(f"DRIVE TYPE : {self.current_drive_type}")

        lines.append("")
        lines.append("MOVEMENT:")

        if self.current_drive_type == "mecanum":
            # Same key *set* as OLD, different meaning for left/right:
            # - arrows / 4/6 strafe
            # - rotation on 0 and .
            lines.append("  8 forward")
            lines.append("  2 backward")
            lines.append("  4 strafe-left")
            lines.append("  6 strafe-right")
            lines.append("  7 diag fwd-left")
            lines.append("  9 diag fwd-right")
            lines.append("  1 diag back-left")
            lines.append("  3 diag back-right")
            lines.append("  0 rotate-left")
            lines.append("  . rotate-right")
            lines.append("")
            lines.append("ARROW KEYS ALSO WORK")
            lines.append(" [UP] forward")
            lines.append(" [DOWN] backward")
            lines.append(" [LEFT] strafe-left")
            lines.append(" [RIGHT] strafe-right")
        else:
            # diff_drive instructions from OLD
            lines.append("  8 forward")
            lines.append("  2 backward")
            lines.append("  4 rotate-left")
            lines.append("  6 rotate-right")
            lines.append("  7 circle forward-left")
            lines.append("  9 circle forward-right")
            lines.append("  1 circle backward-left")
            lines.append("  3 circle backward-right")
            lines.append("")
            lines.append("ARROW KEYS ALSO WORK")
            lines.append(" [UP] forward")
            lines.append(" [DOWN] backward")
            lines.append(" [LEFT] rotate left")
            lines.append(" [RIGHT] rotate right")

        lines.append("")
        lines.append("STOP:")
        lines.append("  [SPACE] OR 's' OR 5")
        lines.append("")
        lines.append("SPEED PROFILES:")
        lines.append("  i slow, o medium, p fast")
        lines.append("")
        lines.append("Speed scaling:")
        lines.append("  q OR / increase linear")
        lines.append("  r OR * decrease linear")
        lines.append("  w OR + increase speed")
        lines.append("  e OR - decrease speed")
        lines.append("")
        lines.append("Robot selection:")
        lines.append("  m choose robot")
        lines.append("")
        lines.append("CTRL-C to quit.")
        lines.append("-------------------------------")
        lines.append(self.render_current_speeds())
        return "\n".join(lines)

    def render_current_speeds(self) -> str:
        """
        Keep OLD one-liner, but include strafe speed so mecanum tuning is visible.
        """
        return (
            f"Linear: {self.linear_speed:.2f}  "
            f"Strafe: {self.strafe_speed:.2f}  "
            f"Angular: {self.angular_speed:.2f}"
        )

    def render_robot_list(self, robots: List[str]) -> str:
        """
        Old-style robot list. One robot per line with spacing for readability.
        """
        lines = []
        lines.append("")
        lines.append("======= AVAILABLE ROBOTS =======")
        if not robots:
            lines.append("")
            lines.append("  (none found)")
            lines.append("")
            lines.append("  - Try 'r' to refresh")
            lines.append("")
            lines.append("  - Check ROS_DOMAIN_ID and network")
            lines.append("  - Ensure motor_driver_node is running")
            lines.append("    (it must subscribe to /<robot>/cmd_vel)")
            lines.append("")
        else:
            lines.append("")
            for r in robots:
                lines.append(f"  - {r}")
                lines.append("")
        lines.append("================================")
        lines.append("")
        return "\n".join(lines)

    # -----------------------------------------------------------------
    # Discovery helpers (OLD behavior preserved)
    # -----------------------------------------------------------------
    def _topic_to_robot(self, topic: str) -> Optional[str]:
        m = CMD_VEL_RE.match(topic)
        return m.group(1) if m else None

    def _discover_candidate_cmd_vel_topics(self) -> List[str]:
        topics_and_types = self.get_topic_names_and_types()
        topics = [t for (t, _types) in topics_and_types]
        return [t for t in topics if CMD_VEL_RE.match(t)]

    def list_available_robots(self) -> List[str]:
        robots: Set[str] = set()
        for t in self._discover_candidate_cmd_vel_topics():
            subs = self.get_subscriptions_info_by_topic(t)
            if subs and len(subs) > 0:
                robot = self._topic_to_robot(t)
                if robot:
                    robots.add(robot)
        return sorted(robots)

    def _validate_robot_name(self, name: str) -> Tuple[bool, str]:
        """
        Validate by checking there is at least one subscriber on /<robot>/cmd_vel.
        This exactly matches OLD logic.
        """
        name = name.strip()
        if not name:
            return False, ""
        topic = f"/{name}/cmd_vel"
        subs = self.get_subscriptions_info_by_topic(topic)
        if subs and len(subs) > 0:
            return True, topic
        return False, topic

    # -----------------------------------------------------------------
    # Heartbeat + drive-type resolution (new architecture support)
    # -----------------------------------------------------------------
    def _heartbeat_cb(self, msg: String):
        """
        Heartbeat payload is expected to be JSON.
        We keep this best-effort: malformed JSON just yields None.
        """
        try:
            self._last_hb_json = json.loads(msg.data)
        except Exception:
            self._last_hb_json = None

    def _attach_heartbeat_sub(self, robot_name: str):
        """
        Subscribe to /<robot>/heartbeat.
        We also spin briefly to try to catch one heartbeat immediately.
        """
        self._last_hb_json = None

        if self.hb_sub is not None:
            try:
                self.destroy_subscription(self.hb_sub)
            except Exception:
                pass
            self.hb_sub = None

        topic = f"/{robot_name}/heartbeat"
        self.hb_sub = self.create_subscription(String, topic, self._heartbeat_cb, 10)

        # Short best-effort wait for one heartbeat message.
        deadline = time.monotonic() + 0.35
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._last_hb_json is not None:
                break

    def _resolve_drive_type_for_robot(self, robot_name: str) -> str:
        """
        Priority:
          1) heartbeat JSON field "drive_type"
          2) YAML registry resolve_robot_profile()
          3) fallback diff_drive

        Returns:
          "diff_drive" or "mecanum"
        """
        drive_type = "diff_drive"

        # YAML fallback
        try:
            prof = resolve_robot_profile(self.profile_registry, robot_name)
            dt = str(prof.get("drive_type", "diff_drive")).strip()
            if dt:
                drive_type = dt
        except Exception:
            pass

        # Heartbeat override
        if isinstance(self._last_hb_json, dict):
            dt = self._last_hb_json.get("drive_type", None)
            if isinstance(dt, str) and dt.strip():
                drive_type = dt.strip()

        return "mecanum" if drive_type == "mecanum" else "diff_drive"

    # -----------------------------------------------------------------
    # Keymap: OLD behavior, drive-type aware
    # -----------------------------------------------------------------
    def _rebuild_move_bindings_for(self, drive_type: str):
        """
        Build move bindings.

        For diff_drive:
          - Arrow left/right and 4/6 rotate (OLD behavior)
          - 7/9/1/3 are "circle" one-track turns (OLD behavior)

        For mecanum:
          - Keep the same keys so the operator workflow feels consistent
          - Arrow left/right and 4/6 become strafe
          - 7/9/1/3 become diagonals
          - Add 0/. for rotate (so mecanum can yaw without sacrificing strafe keys)
        """
        dt = (drive_type or "").strip()

        if dt == "mecanum":
            self.move_bindings = {
                # Arrow keys
                "\x1b[A": ("twist", 1, 0, 0),    # up -> forward
                "\x1b[B": ("twist", -1, 0, 0),   # down -> backward
                "\x1b[D": ("twist", 0, 1, 0),    # left -> strafe left
                "\x1b[C": ("twist", 0, -1, 0),   # right -> strafe right

                # Numpad
                "8": ("twist", 1, 0, 0),
                "2": ("twist", -1, 0, 0),
                "4": ("twist", 0, 1, 0),
                "6": ("twist", 0, -1, 0),

                # Diagonals (mecanum meaningful)
                "7": ("twist", 1, 1, 0),
                "9": ("twist", 1, -1, 0),
                "1": ("twist", -1, 1, 0),
                "3": ("twist", -1, -1, 0),

                # Rotation keys (added; doesn't exist in OLD, but doesn't break OLD)
                "0": ("twist", 0, 0, 1),   # rotate left
                ".": ("twist", 0, 0, -1),  # rotate right
            }
        else:
            # diff_drive bindings (OLD)
            self.move_bindings = {
                # Arrow keys
                "\x1b[A": ("twist", 1, 0, 0),     # up -> forward
                "\x1b[B": ("twist", -1, 0, 0),    # down -> backward
                "\x1b[D": ("twist", 0, 0, 1),     # left -> rotate left
                "\x1b[C": ("twist", 0, 0, -1),    # right -> rotate right

                # Numpad
                "8": ("twist", 1, 0, 0),
                "2": ("twist", -1, 0, 0),

                # Rotate in place
                "4": ("twist", 0, 0, 1),
                "6": ("twist", 0, 0, -1),

                # One-track-only circle turns (handled specially in execute loop)
                "7": ("circle", None, None, None),
                "9": ("circle", None, None, None),
                "1": ("circle", None, None, None),
                "3": ("circle", None, None, None),
            }

    # -----------------------------------------------------------------
    # Active robot publishing (legacy + new)
    # -----------------------------------------------------------------
    def _publish_active_robot(self):
        if not self.current_robot_name:
            return

        # Publish robot name
        msg = String()
        msg.data = self.current_robot_name

        # Legacy topics (do not break older FPV mux behavior)
        self.active_robot_pub_legacy.publish(msg)
        self.active_robot_teleop_pub_legacy.publish(msg)

        # New /legion topics
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

        # Publish drive type too (new)
        dt = String()
        dt.data = self.current_drive_type
        self.active_drive_pub.publish(dt)

    # -----------------------------------------------------------------
    # Topic application (selection -> publisher setup)
    # -----------------------------------------------------------------
    def _apply_cmd_vel_topic(self, new_topic: str):
        """
        Switch control to a new /<robot>/cmd_vel topic.

        Steps:
          1) refuse /cmd_vel (must be namespaced for multi-robot)
          2) destroy old publisher
          3) create new publisher
          4) attach heartbeat subscription
          5) resolve drive type and rebuild keymap
          6) print updated UI (OLD style)
        """
        if not new_topic.startswith("/"):
            new_topic = "/" + new_topic

        # Hard refusal because a shared /cmd_vel breaks multi-robot control.
        if new_topic == "/cmd_vel":
            self._tprint("[ERROR] Refusing /cmd_vel. Use /<robot>/cmd_vel for multi-robot architecture.")
            return

        robot = self._topic_to_robot(new_topic)
        if robot:
            self.current_robot_name = robot

        # Tear down old publisher to avoid accidentally commanding the previous robot.
        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass

        # Create publisher for the new robot
        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic

        # Heartbeat + drive type resolution
        if self.current_robot_name:
            self._attach_heartbeat_sub(self.current_robot_name)
            self.current_drive_type = self._resolve_drive_type_for_robot(self.current_robot_name)
        else:
            self.current_drive_type = "diff_drive"

        # Update key bindings based on drive type
        self._rebuild_move_bindings_for(self.current_drive_type)

        # Broadcast active selection to other subsystems
        self._publish_active_robot()

        # Print OLD-style UI block again (so the operator sees current mode)
        self._tprint(self.render_instructions(new_topic))
        self._tprint(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}  drive={self.current_drive_type}")

    # -----------------------------------------------------------------
    # Diff-drive circle turns (OLD behavior)
    # -----------------------------------------------------------------
    def _publish_one_track_circle(self, v_left: float, v_right: float):
        """
        ONE-TRACK-ONLY circle motion (diff-drive only).

        This matches OLD behavior exactly, based on track-speed math:

            v_left  = v - w*L/2
            v_right = v + w*L/2

        Inverse:
            v = (v_left + v_right)/2
            w = (v_right - v_left)/L
        """
        L = self.wheel_separation if self.wheel_separation > 1e-6 else 0.18
        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / L

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.last_twist = twist
        self.is_moving = True

        # Safe guard: publisher_ exists if a robot is selected
        if self.publisher_ is not None:
            self.publisher_.publish(twist)

    # -----------------------------------------------------------------
    # Republish + Offline watchdog (OLD behavior preserved)
    # -----------------------------------------------------------------
    def _republish_last_twist(self):
        """
        Re-publish movement so the robot continues moving even if the operator
        isn't constantly generating keypresses.

        This is a "human usability" feature that also makes demos smoother.
        """
        if not self.is_moving or self.publisher_ is None:
            return

        # If last_twist was a direct Twist, re-publish it.
        if (
            abs(self.last_twist.linear.x) > 1e-9 or
            abs(self.last_twist.linear.y) > 1e-9 or
            abs(self.last_twist.angular.z) > 1e-9
        ):
            self.publisher_.publish(self.last_twist)

    def _offline_watchdog(self):
        """
        If the current cmd_vel topic has no subscribers, treat robot as offline:
          - publish stop
          - reset internal state
          - destroy publisher
          - force selection again

        This preserves OLD behavior and prevents "publishing into the void".
        """
        if self.publisher_ is None or not self.cmd_vel_topic or not self.current_robot_name:
            return

        subs = self.get_subscriptions_info_by_topic(self.cmd_vel_topic)
        if subs and len(subs) > 0:
            return

        # Robot went offline
        self._tprint("")
        self._tprint(f"[OFFLINE] Robot '{self.current_robot_name}' offline (no subs on {self.cmd_vel_topic}).")
        self._tprint("[OFFLINE] Stopping and returning to robot selection.")

        try:
            self.publisher_.publish(Twist())
        except Exception:
            pass

        self.is_moving = False
        self.last_lin_mult = 0.0
        self.last_strafe_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()

        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass

        self.publisher_ = None
        self.cmd_vel_topic = None
        self.current_robot_name = None
        self.current_drive_type = "diff_drive"

    # -----------------------------------------------------------------
    # Speed modifiers (OLD behavior preserved)
    # -----------------------------------------------------------------
    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self.strafe_speed *= self.speed_step
        self._tprint(self.render_current_speeds())

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self.strafe_speed /= self.speed_step
        self._tprint(self.render_current_speeds())

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        self._tprint(self.render_current_speeds())

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        self._tprint(self.render_current_speeds())

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self.strafe_speed = float(self.get_parameter("strafe_speed").value)
        self._tprint(self.render_current_speeds())

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self.strafe_speed = float(self.get_parameter("strafe_speed").value) * (self.speed_step ** 10)
        self._tprint(self.render_current_speeds())

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self.strafe_speed = float(self.get_parameter("strafe_speed").value) * (self.speed_step ** 12)
        self._tprint(self.render_current_speeds())

    # -----------------------------------------------------------------
    # Robot selection prompt (OLD behavior preserved)
    # -----------------------------------------------------------------
    def _prompt_until_valid_robot(self):
        """
        OLD behavior:
          - print robot list
          - prompt until the user enters a valid robot name
          - 'r' refreshes discovery
        """
        first_pass = True

        while rclpy.ok():
            if first_pass:
                self._warmup_discovery(timeout_sec=1.0)
                first_pass = False

            robots = self.list_available_robots()
            self._tprint(self.render_robot_list(robots))

            # input() requires cooked mode, so we restore before reading.
            restore_terminal_settings(self._terminal_settings)
            try:
                user_input = input("[SELECT ROBOT] Enter robot name \n(or enter 'r' to refresh): ").strip()
            except Exception:
                set_raw_mode()
                continue

            # Return to raw mode after input
            set_raw_mode()

            if user_input.lower() == "r":
                self._warmup_discovery(timeout_sec=0.5)
                continue

            ok, topic = self._validate_robot_name(user_input)
            if not ok:
                self._tprint(f"[INVALID] '{user_input}' not available (no subs on {topic}). Try again.\n")
                continue

            # If switching disabled and we already have a robot, refuse.
            if not self.allow_cmd_vel_switching and self.publisher_ is not None:
                self._tprint("[ROBOT SWITCH] Switching disabled (allow_cmd_vel_switching:=False).")
                return

            self._apply_cmd_vel_topic(topic)
            return

    # -----------------------------------------------------------------
    # Main loop (OLD behavior preserved, extended for mecanum)
    # -----------------------------------------------------------------
    def run(self):
        """
        Main teleop loop:
          - initialize terminal settings
          - enter raw mode
          - prompt for robot selection
          - read keys and publish Twist continuously
        """
        self._terminal_settings = termios.tcgetattr(sys.stdin)
        set_raw_mode()

        # initial selection (OLD)
        self._prompt_until_valid_robot()

        try:
            while rclpy.ok():
                # spin_once is important even for teleop so we:
                # - receive heartbeat messages
                # - keep DDS discovery updated
                rclpy.spin_once(self, timeout_sec=0.0)

                key = read_key_raw(timeout_s=0.1)
                if key == "":
                    continue
                if key == "\x03":  # Ctrl-C
                    break

                # If no robot selected (startup/offline), guide operator
                if self.publisher_ is None:
                    if key == "m":
                        self._prompt_until_valid_robot()
                    elif key in self.speed_bindings:
                        self.speed_bindings[key]()
                    elif key in (" ", "5", "s"):
                        self._tprint("[STOP] (no robot selected)")
                    else:
                        self._tprint("[INFO] No robot selected. Press 'm' to select a robot.")
                    continue

                # STOP keys (OLD)
                if key in (" ", "5", "s"):
                    self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_strafe_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()
                    self._tprint("[STOP]")
                    continue

                # Switch robot (OLD)
                if key == "m":
                    self._prompt_until_valid_robot()
                    continue

                # Speed controls (OLD)
                if key in self.speed_bindings:
                    self.speed_bindings[key]()
                    continue

                # Movement keys (drive-type aware)
                if key in self.move_bindings:
                    mode, lin_mult, strafe_mult, ang_mult = self.move_bindings[key]

                    # Diff-drive circle turns (OLD exact behavior)
                    if mode == "circle" and self.current_drive_type == "diff_drive":
                        S = self.linear_speed
                        if key == "7":
                            self._publish_one_track_circle(v_left=0.0, v_right=+S)
                            continue
                        if key == "9":
                            self._publish_one_track_circle(v_left=+S, v_right=0.0)
                            continue
                        if key == "1":
                            self._publish_one_track_circle(v_left=0.0, v_right=-S)
                            continue
                        if key == "3":
                            self._publish_one_track_circle(v_left=-S, v_right=0.0)
                            continue

                    # Normal twist build:
                    # - linear.x from lin_mult
                    # - linear.y from strafe_mult (mecanum only; diff ignores)
                    # - angular.z from ang_mult
                    self.last_lin_mult = float(lin_mult or 0.0)
                    self.last_strafe_mult = float(strafe_mult or 0.0)
                    self.last_ang_mult = float(ang_mult or 0.0)

                    twist = Twist()
                    twist.linear.x = self.linear_speed * self.last_lin_mult

                    # Only mecanum uses strafe. Diff robots will just ignore linear.y
                    # (their motor driver will not act on it).
                    twist.linear.y = self.strafe_speed * self.last_strafe_mult

                    twist.angular.z = self.angular_speed * self.last_ang_mult

                    self.last_twist = twist
                    self.is_moving = True
                    self.publisher_.publish(twist)
                    continue

                # Unknown keys: ignore (keeps terminal clean)

        finally:
            # Always try to stop the robot on exit
            try:
                if self.publisher_ is not None:
                    self.publisher_.publish(Twist())
            except Exception:
                pass
            restore_terminal_settings(self._terminal_settings)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
