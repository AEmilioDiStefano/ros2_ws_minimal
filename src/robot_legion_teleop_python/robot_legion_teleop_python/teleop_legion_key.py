#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for swarm robotics applications 

Core behaviors:
- Robots are identified by "robot name" (default = Linux username on robot)
- A robot is considered "available" if there is at least one subscriber on:
    /<robot_name>/cmd_vel
  (i.e., motor_driver_node is running on that robot)

Startup:
- Prints list of available robots, prompts user to pick one.
- Keeps prompting until a valid robot name is entered.

Switching:
- Press 'm' any time to switch robot (same validation loop).

Offline detection:
- While driving, if the current robot stops subscribing to /<robot>/cmd_vel,
  teleop stops and returns to selection prompt.

Tracked robot turning:
- '4' rotates LEFT  (left track backward, right track forward)  -> angular.z positive
- '6' rotates RIGHT (left track forward, right track backward)  -> angular.z negative

Circle turns (smallest radius; ONE TRACK ONLY):
- 7 forward-left  : RIGHT track forward, LEFT track stopped
- 9 forward-right : LEFT track forward, RIGHT track stopped
- 1 backward-left : RIGHT track backward, LEFT track stopped
- 3 backward-right: LEFT track backward, RIGHT track stopped

Terminal reliability note:
- This program reads keys in RAW mode for instant keypress capture.
- Printing while RAW is active can cause newline behavior that "drifts" output rightward
  (because \n may not return to column 0).
- We fix that by ALWAYS printing in cooked mode, then returning to raw.

NEW (minimal additions):
- Teleop respects the shared per-robot control lock.
  Teleop will only publish cmd_vel if it owns the lock for the active robot.

Lock protocol (std_msgs/String JSON):
  /control_lock/request
  /control_lock/state
  /control_lock/response/<client_id>
"""

import re
import sys
import select
import termios
import tty
import time
import json
import os
import socket
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def set_raw_mode():
    tty.setraw(sys.stdin.fileno())


def read_key_raw(timeout_s: float = 0.1) -> str:
    """
    Read a single keypress (with arrow-key escape sequence support).
    Assumes terminal is already in raw mode.
    """
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not rlist:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":  # start of escape sequence
        # Typical arrow keys arrive as ESC [ A/B/C/D (3 chars total including ESC)
        key += sys.stdin.read(2)
    return key


def _make_client_id() -> str:
    host = socket.gethostname().strip() or "host"
    # ROS topic name-safe: only alphanumerics and underscores
    host = re.sub(r"[^A-Za-z0-9_]", "_", host)

    pid = os.getpid()
    return f"teleop_{host}_{pid}"

class LockClient:
    """
    Minimal lock client to share the same lock as the browser UI.
    Topic JSON over std_msgs/String.

    Request: /control_lock/request
      {"action":"acquire|release|heartbeat","robot":"robotname","client_id":"..."}
    State: /control_lock/state
      {"locks": {"robotname": "owner_client_id", ...}}
    Response: /control_lock/response/<client_id>
      {"ok": true|false, "action":"...", "robot":"...", "owner":"...", "reason":"..."}
    """
    def __init__(self, node: Node, client_id: str):
        self.node = node
        self.client_id = client_id

        self.req_pub = node.create_publisher(String, "/control_lock/request", 50)

        self._locks: Dict[str, str] = {}
        self._last_resp: Optional[Dict] = None

        self._state_sub = node.create_subscription(String, "/control_lock/state", self._on_state, 10)
        self._resp_sub = node.create_subscription(
            String,
            f"/control_lock/response/{client_id}",
            self._on_resp,
            10,
        )

    def _on_state(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._locks = dict(data.get("locks") or {})
        except Exception:
            self._locks = {}

    def _on_resp(self, msg: String):
        try:
            self._last_resp = json.loads(msg.data)
        except Exception:
            self._last_resp = None

    def owner(self, robot: str) -> Optional[str]:
        return self._locks.get(robot)

    def _publish_req(self, action: str, robot: str):
        m = String()
        m.data = json.dumps({"action": action, "robot": robot, "client_id": self.client_id})
        self.req_pub.publish(m)

    def acquire(self, robot: str, timeout_sec: float = 1.0) -> bool:
        self._last_resp = None
        self._publish_req("acquire", robot)
        t0 = time.time()
        while (time.time() - t0) < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self._last_resp
                and self._last_resp.get("action") == "acquire"
                and self._last_resp.get("robot") == robot
            ):
                return bool(self._last_resp.get("ok"))
        return False

    def release(self, robot: str, timeout_sec: float = 1.0) -> bool:
        self._last_resp = None
        self._publish_req("release", robot)
        t0 = time.time()
        while (time.time() - t0) < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self._last_resp
                and self._last_resp.get("action") == "release"
                and self._last_resp.get("robot") == robot
            ):
                return bool(self._last_resp.get("ok"))
        return False

    def heartbeat(self, robot: str):
        self._publish_req("heartbeat", robot)


@dataclass
class TeleopState:
    available_robots: List[str]
    active_robot: Optional[str]
    cmd_vel_topic: Optional[str]
    linear_speed: float
    angular_speed: float
    allow_cmd_vel_switching: bool
    wheel_separation: float


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__("robot_legion_teleop_python")

        # Optional parameters
        self.declare_parameter("allow_cmd_vel_switching", True)
        self.allow_cmd_vel_switching = bool(self.get_parameter("allow_cmd_vel_switching").value)

        # Used for circle-turn math (must match motor_driver_node)
        self.declare_parameter("wheel_separation", 0.18)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)

        # Active robot publishers (used by fpv_camera_mux)
        self.active_robot_pub = self.create_publisher(String, "/active_robot", 10)
        self.active_robot_teleop_pub = self.create_publisher(String, "/teleop/active_robot", 10)

        # Publisher created after robot selection
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None

        # --- NEW: lock client (minimal addition) ---
        self.lock_client_id = _make_client_id()
        self.lock = LockClient(self, self.lock_client_id)
        self.lock_owned_for_active_robot: bool = False
        self.create_timer(1.0, self._lock_heartbeat_timer)
        # ------------------------------------------

        # Speed profile
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # Republish state
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()
        self.is_moving = False

        # KEYMAP (7/9/1/3 handled specially for one-track-only circles)
        self.move_bindings = {
            # Arrow keys
            "\x1b[A": ("twist", 1, 0),     # up
            "\x1b[B": ("twist", -1, 0),    # down
            "\x1b[D": ("twist", 0, 1),     # left  (rotate left)
            "\x1b[C": ("twist", 0, -1),    # right (rotate right)

            # Numpad
            "8": ("twist", 1, 0),          # forward
            "2": ("twist", -1, 0),         # backward

            # Rotate in place
            "4": ("twist", 0, 1),          # rotate LEFT
            "6": ("twist", 0, -1),         # rotate RIGHT

            # One-track-only circle turns (handled specially)
            "7": ("circle", None, None),   # forward-left  (right track only)
            "9": ("circle", None, None),   # forward-right (left track only)
            "1": ("circle", None, None),   # backward-left (right track only)
            "3": ("circle", None, None),   # backward-right(left track only)
        }

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

        # Timers
        self.create_timer(0.1, self._republish_last_twist)
        self.create_timer(0.5, self._offline_watchdog)

        # Initial UI
        self._tprint(self.render_instructions(topic_name=None))
        self._tprint("[STARTUP] Discovering robots...")
        self._tprint(f"[LOCK] teleop lock_client_id: {self.lock_client_id}")

    # ---------------- NEW: lock helpers (minimal addition) ----------------

    def _lock_heartbeat_timer(self):
        if self.current_robot_name and self.lock_owned_for_active_robot:
            if self.lock.owner(self.current_robot_name) == self.lock_client_id:
                self.lock.heartbeat(self.current_robot_name)
            else:
                # lock expired or taken elsewhere
                self.lock_owned_for_active_robot = False
                self.is_moving = False
                self.last_lin_mult = 0.0
                self.last_ang_mult = 0.0
                self.last_twist = Twist()
                self._tprint(f"[LOCK] Lost lock for '{self.current_robot_name}'. Teleop motion disabled until lock reacquired.")

    def _release_lock_if_owned(self):
        if not self.current_robot_name:
            self.lock_owned_for_active_robot = False
            return
        if self.lock.owner(self.current_robot_name) == self.lock_client_id:
            self.lock.release(self.current_robot_name)
        self.lock_owned_for_active_robot = False

    def _try_acquire_lock_for_robot(self, robot: str):
        self.lock_owned_for_active_robot = False
        ok = self.lock.acquire(robot)
        if ok:
            self.lock_owned_for_active_robot = True
            self._tprint(f"[LOCK] Acquired lock for '{robot}'.")
        else:
            owner = self.lock.owner(robot)
            self._tprint(f"[LOCK] Lock denied for '{robot}'. Current owner: {owner or '(unknown)'}")
            self._tprint("[LOCK] Active robot still switches (for video), but teleop motion is DISABLED for this robot.")

    def _can_publish_cmd(self) -> bool:
        if self.publisher_ is None or not self.current_robot_name:
            return False
        return self.lock_owned_for_active_robot and (self.lock.owner(self.current_robot_name) == self.lock_client_id)

    # ---------------- Terminal output (cooked mode only) ----------------

    def _tprint(self, text: str = ""):
        """
        Print reliably even while the program uses RAW mode for keypress reading.
        We temporarily restore "cooked" terminal settings for printing.
        """
        # If stdin isn't a tty, just print normally (useful for piping/logging)
        if not sys.stdin.isatty():
            print(text)
            sys.stdout.flush()
            return

        try:
            # Put terminal back to cooked for correct newline behavior
            restore_terminal_settings(self._terminal_settings)
        except Exception:
            # If settings aren't initialized yet, best effort print
            print(text)
            sys.stdout.flush()
            return

        # Normal printing in cooked mode
        if text:
            print(text)
        else:
            print("")
        sys.stdout.flush()

        # Back to raw so key reading continues to work
        try:
            set_raw_mode()
        except Exception:
            pass

    # ---------------- ROS graph warm-up (startup discovery fix) ----------------

    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        """
        Warm up DDS/ROS graph discovery so get_topic_names_and_types() /
        get_subscriptions_info_by_topic() are correct on first display.

        Why this exists:
        - On startup, the ROS graph cache may not yet include remote peers.
        - If we query immediately, we may see (none found) even though robots exist.
        - A short spin_once loop processes discovery traffic and populates the graph.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            # Stop early as soon as we see any available robot
            if self.list_available_robots():
                return

    # ---------------- State for easy web integration ----------------

    def get_state(self) -> TeleopState:
        robots = self.list_available_robots()
        return TeleopState(
            available_robots=robots,
            active_robot=self.current_robot_name,
            cmd_vel_topic=self.cmd_vel_topic,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            allow_cmd_vel_switching=self.allow_cmd_vel_switching,
            wheel_separation=self.wheel_separation,
        )

    def get_state_dict(self) -> Dict:
        """
        JSON-friendly dict version of state (plug-and-play for future HTTP/WebSocket API).
        """
        s = self.get_state()
        return {
            "available_robots": s.available_robots,
            "active_robot": s.active_robot,
            "cmd_vel_topic": s.cmd_vel_topic,
            "linear_speed": s.linear_speed,
            "angular_speed": s.angular_speed,
            "allow_cmd_vel_switching": s.allow_cmd_vel_switching,
            "wheel_separation": s.wheel_separation,
        }

    # ---------------- Renderers (return strings; terminal/web can reuse) ----------------

    def render_instructions(self, topic_name: Optional[str]) -> str:
        lines = []
        lines.append("-------------------------------")
        lines.append("      ROBOT LEGION TELEOP      ")
        lines.append("-------------------------------")
        if topic_name:
            lines.append(f"Publishing Twist on: {topic_name}")
        else:
            lines.append("SELECT A ROBOT")
        lines.append(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        lines.append(f"wheel_separation (for circle turns): {self.wheel_separation:.2f} m")
        if self.current_robot_name:
            lines.append("")
            lines.append(f"ACTIVE ROBOT: {self.current_robot_name}")
            # NEW: small lock status display
            owner = self.lock.owner(self.current_robot_name)
            if owner:
                lines.append(f"LOCK OWNER: {owner}")
            else:
                lines.append("LOCK OWNER: (none)")
            lines.append(f"TELEOP LOCK HELD: {self.lock_owned_for_active_robot}")
        lines.append("")
        lines.append("MOVEMENT:")
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
        return f"Linear Speed: {self.linear_speed:.2f}  Angular Speed: {self.angular_speed:.2f}"

    def render_robot_list(self, robots: List[str]) -> str:
        lines = []
        lines.append("")
        lines.append("======= AVAILABLE ROBOTS =======")
        if not robots:
            lines.append("")
            lines.append("  (none found)")
            lines.append("")
            lines.append("  - Try 'r' to refresh")
            lines.append("")
            lines.append("  - Make sure each robot is")
            lines.append("    on the same ROS_DOMAIN_ID")
            lines.append("    and network.")
            lines.append("")
            lines.append("  - Make sure your robots are")
            lines.append("    currently running at least")
            lines.append("    one node.")
            lines.append("")
        else:
            lines.append("")
            for r in robots:
                # NEW: show lock ownership in the list
                owner = self.lock.owner(r)
                lines.append(f"  - {r}")
                if owner:
                    lines.append(f"    LOCKED by: {owner}")
                lines.append("")
        lines.append("================================")
        lines.append("")
        return "\n".join(lines)

    # ---------------- Discovery helpers ----------------

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
        name = name.strip()
        if not name:
            return False, ""
        topic = f"/{name}/cmd_vel"
        subs = self.get_subscriptions_info_by_topic(topic)
        if subs and len(subs) > 0:
            return True, topic
        return False, topic

    # ---------------- Active robot helpers ----------------

    def _publish_active_robot(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

    # ---------------- Topic application ----------------

    def _apply_cmd_vel_topic(self, new_topic: str):
        if not new_topic.startswith("/"):
            new_topic = "/" + new_topic

        if new_topic == "/cmd_vel":
            self._tprint("[ERROR] Refusing to use /cmd_vel. Use /<robot>/cmd_vel for multi-robot architecture.")
            return

        # NEW: release lock for previous robot if we owned it
        prev_robot = self.current_robot_name
        if prev_robot and (self.lock.owner(prev_robot) == self.lock_client_id):
            self.lock.release(prev_robot)
        self.lock_owned_for_active_robot = False

        robot = self._topic_to_robot(new_topic)
        if robot:
            self.current_robot_name = robot

        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass

        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic

        # Always publish active robot (so video can follow selection even if lock denied)
        self._publish_active_robot()

        # NEW: attempt to acquire lock for the selected robot
        if self.current_robot_name:
            self._try_acquire_lock_for_robot(self.current_robot_name)

        self._tprint(self.render_instructions(new_topic))
        self._tprint(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")

    # ---------------- Circle-turn helper ----------------

    def _publish_one_track_circle(self, v_left: float, v_right: float):
        """
        Force one-track-only motion by solving for Twist that yields the desired track speeds:

            v_left  = v - w*L/2
            v_right = v + w*L/2

        Inverse:
            v = (v_left + v_right)/2
            w = (v_right - v_left)/L
        """
        if not self._can_publish_cmd():
            return

        L = self.wheel_separation if self.wheel_separation > 1e-6 else 0.18
        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / L

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.last_twist = twist
        self.is_moving = True

        self.publisher_.publish(twist)

    # ---------------- Republish + Offline watchdog ----------------

    def _republish_last_twist(self):
        if not self.is_moving or self.publisher_ is None:
            return

        # NEW: respect lock
        if not self._can_publish_cmd():
            return

        if abs(self.last_twist.linear.x) > 1e-9 or abs(self.last_twist.angular.z) > 1e-9:
            self.publisher_.publish(self.last_twist)
            return

        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return

        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.angular.z = self.angular_speed * self.last_ang_mult
        self.publisher_.publish(twist)

    def _offline_watchdog(self):
        """
        If current cmd_vel topic has no subscribers, the robot is "offline".
        Stop and force re-selection.
        """
        if self.publisher_ is None or not self.cmd_vel_topic or not self.current_robot_name:
            return

        subs = self.get_subscriptions_info_by_topic(self.cmd_vel_topic)
        if subs and len(subs) > 0:
            return

        # Robot went offline
        self._tprint("")
        self._tprint(f"[OFFLINE] Robot '{self.current_robot_name}' appears offline (no subscribers on {self.cmd_vel_topic}).")
        self._tprint("[OFFLINE] Stopping and returning to robot selection.")
        try:
            # NEW: only publish stop if we own lock
            if self._can_publish_cmd():
                self.publisher_.publish(Twist())
        except Exception:
            pass

        # NEW: release lock if owned
        self._release_lock_if_owned()

        self.is_moving = False
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()

        # Destroy publisher so main loop forces selection again
        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass
        self.publisher_ = None
        self.cmd_vel_topic = None
        self.current_robot_name = None

    # ---------------- Speed modifiers ----------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._tprint(self.render_current_speeds())

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
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
        self._tprint(self.render_current_speeds())

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._tprint(self.render_current_speeds())

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._tprint(self.render_current_speeds())

    # ---------------- Robot selection prompt ----------------

    def _prompt_until_valid_robot(self):
        """
        Print robots, prompt until user enters a valid robot name.
        """
        first_pass = True

        while rclpy.ok():
            # Warm up discovery so the initial list is correct on startup.
            if first_pass:
                self._warmup_discovery(timeout_sec=1.0)
                first_pass = False

            robots = self.list_available_robots()
            self._tprint(self.render_robot_list(robots))

            # We MUST be in cooked mode for input()
            restore_terminal_settings(self._terminal_settings)
            try:
                user_input = input("[SELECT ROBOT] Enter robot name \n(or enter 'r' to refresh): ").strip()
            except Exception:
                # Return to raw and retry
                set_raw_mode()
                continue

            # Back to raw for key reading after input()
            set_raw_mode()

            if user_input.lower() == "r":
                # Also warm up on refresh so the very next listing is accurate.
                self._warmup_discovery(timeout_sec=0.5)
                continue

            ok, topic = self._validate_robot_name(user_input)
            if not ok:
                self._tprint(f"[INVALID] '{user_input}' is not an available robot (no subscriber on {topic}). Try again.\n")
                continue

            if not self.allow_cmd_vel_switching and self.publisher_ is not None:
                self._tprint("[ROBOT SWITCH] Switching disabled (allow_cmd_vel_switching:=False).")
                return

            self._apply_cmd_vel_topic(topic)
            return

    # ---------------- Main loop ----------------

    def run(self):
        # Save terminal settings once so all printing can temporarily restore them.
        self._terminal_settings = termios.tcgetattr(sys.stdin)

        # Enter raw mode for key reading
        set_raw_mode()

        # initial selection (uses input, which temporarily flips to cooked itself)
        self._prompt_until_valid_robot()

        try:
            while rclpy.ok():
                key = read_key_raw(timeout_s=0.1)
                if key == "":
                    continue
                if key == "\x03":  # Ctrl-C
                    break

                # If we currently have no robot selected (offline or startup), force selection
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

                # Movement keys
                if key in self.move_bindings:
                    # NEW: respect lock (ignore motion if we don't own it)
                    if not self._can_publish_cmd():
                        self.is_moving = False
                        continue

                    mode, lin_mult, ang_mult = self.move_bindings[key]

                    if mode == "circle":
                        S = self.linear_speed
                        if key == "7":
                            self._publish_one_track_circle(v_left=0.0, v_right=+S)
                        elif key == "9":
                            self._publish_one_track_circle(v_left=+S, v_right=0.0)
                        elif key == "1":
                            self._publish_one_track_circle(v_left=0.0, v_right=-S)
                        elif key == "3":
                            self._publish_one_track_circle(v_left=-S, v_right=0.0)
                        continue

                    # Normal twist
                    self.last_lin_mult = float(lin_mult)
                    self.last_ang_mult = float(ang_mult)

                    twist = Twist()
                    twist.linear.x = self.linear_speed * self.last_lin_mult
                    twist.angular.z = self.angular_speed * self.last_ang_mult

                    self.last_twist = twist
                    self.is_moving = True
                    self.publisher_.publish(twist)
                    continue

                # Stop
                if key in (" ", "5", "s"):
                    # NEW: only publish stop if we own lock; always clear local state
                    if self._can_publish_cmd():
                        self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()
                    self._tprint("[STOP]")
                    continue

                # Switch robot
                if key == "m":
                    # NEW: publish stop only if we own lock (optional but safe)
                    if self._can_publish_cmd():
                        self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()

                    self._prompt_until_valid_robot()
                    continue

                # Speed controls
                if key in self.speed_bindings:
                    self.speed_bindings[key]()
                    continue

        finally:
            try:
                if self.publisher_ is not None:
                    # NEW: stop only if we own lock
                    if self._can_publish_cmd():
                        self.publisher_.publish(Twist())
            except Exception:
                pass

            # NEW: release lock if owned
            self._release_lock_if_owned()

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
