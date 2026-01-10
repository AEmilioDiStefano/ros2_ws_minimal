#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for swarm robotics applications:
- On startup, teleop scans the network and prints a list of available robots.
- "Available robot" means: there exists a topic /<robot>/cmd_vel of type geometry_msgs/msg/Twist
  AND there is at least one subscriber on that topic (i.e., a motor driver node is listening).
- User is prompted to enter a robot name. Invalid names are rejected and re-prompted.
- Once a valid robot is selected, teleop prints instructions and begins control.

Robot switching:
- Press 'm' anytime to rescan and switch robots (validated against available robots).

Publishing behavior:
- Teleop publishes Twist to /<robot_name>/cmd_vel
- Teleop publishes active robot name to:
    - /active_robot
    - /teleop/active_robot   (used by fpv_camera_mux)

Tracked robot turning:
- '4' rotates LEFT  (left track backward, right track forward)  -> angular.z positive
- '6' rotates RIGHT (left track forward, right track backward)  -> angular.z negative

Circle turns (smallest radius; ONE TRACK ONLY):
- 7 forward-left  : RIGHT track forward, LEFT track stopped
- 9 forward-right : LEFT track forward, RIGHT track stopped
- 1 backward-left : RIGHT track backward, LEFT track stopped
- 3 backward-right: LEFT track backward, RIGHT track stopped
"""

import sys
import time
import select
import termios
import tty
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ---------------- Terminal helpers ----------------

def get_key(settings):
    """Read a single keypress (with arrow-key escape sequence support)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # start of escape sequence
            key += sys.stdin.read(2)
        return key
    return ''


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


# ---------------- Teleop Node ----------------

class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__('robot_legion_teleop_python')

        # Optional startup hints (we still validate against discovered robots)
        self.declare_parameter('startup_robot', '')
        self.declare_parameter('startup_cmd_vel_topic', '')

        # Safety: if False, 'm' will NOT change cmd_vel topic (real robot mode).
        self.declare_parameter('allow_cmd_vel_switching', True)
        self.allow_cmd_vel_switching = bool(self.get_parameter('allow_cmd_vel_switching').value)

        # Match motor_driver_node default wheel_separation (used for circle-turn math)
        self.declare_parameter('wheel_separation', 0.18)  # meters between tracks
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)

        # Active robot publishers
        self.active_robot_pub = self.create_publisher(String, '/active_robot', 10)
        self.active_robot_teleop_pub = self.create_publisher(String, '/teleop/active_robot', 10)

        # Teleop publisher (created once we have a target)
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None

        # Base speed profile (SLOW)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        # Derived profiles
        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # Last commanded direction (for republish robustness)
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()
        self.is_moving = False

        # KEYMAP
        self.move_bindings = {
            # Arrow keys
            '\x1b[A': ('twist', 1, 0),     # up
            '\x1b[B': ('twist', -1, 0),    # down
            '\x1b[D': ('twist', 0, 1),     # left  (rotate left)
            '\x1b[C': ('twist', 0, -1),    # right (rotate right)

            # Numpad
            '8': ('twist', 1, 0),          # forward
            '2': ('twist', -1, 0),         # backward

            # Rotate in place
            '4': ('twist', 0, 1),          # rotate LEFT
            '6': ('twist', 0, -1),         # rotate RIGHT

            # One-track-only circle turns (handled specially)
            '7': ('circle', None, None),   # forward-left  (right track only)
            '9': ('circle', None, None),   # forward-right (left track only)
            '1': ('circle', None, None),   # backward-left (right track only)
            '3': ('circle', None, None),   # backward-right(left track only)
        }

        self.speed_bindings = {
            'w': self._increase_both_speeds,
            '+': self._increase_both_speeds,
            'e': self._decrease_both_speeds,
            '-': self._decrease_both_speeds,
            'q': self._increase_linear_speed,
            '/': self._increase_linear_speed,
            'r': self._decrease_linear_speed,
            '*': self._decrease_linear_speed,
            'i': self._set_slow_profile,
            'o': self._set_medium_profile,
            'p': self._set_fast_profile,
        }

        # Republish timer for robustness across Wi-Fi/DDS.
        self.create_timer(0.1, self._republish_last_twist)

        # Record any startup hints (still validated later)
        self.startup_robot_hint = self.get_parameter('startup_robot').get_parameter_value().string_value.strip()
        self.startup_topic_hint = self.get_parameter('startup_cmd_vel_topic').get_parameter_value().string_value.strip()

        # NOTE: We do not immediately print instructions here anymore.
        # We first perform TRUE SWARM startup selection in run() before switching terminal to raw mode.

    # ---------------- Printing ----------------

    def _print_instructions(self, topic_name: Optional[str]):
        print("--------------------------------------------------")
        print(" Robot Legion Teleop (Python) - TRUE SWARM MODE")
        print("--------------------------------------------------")
        if topic_name:
            print(f"Publishing Twist on: {topic_name}")
        else:
            print("Publishing Twist on: (none yet)")
        print(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        print(f"wheel_separation (for circle turns): {self.wheel_separation:.3f} m")
        if self.current_robot_name:
            print(f"Active robot: {self.current_robot_name}")
        print("")
        print("For best results, use the numpad (with numlock enabled) on your keyboard to control robots")
        print("")
        print("Movement:")
        print("  8 forward, 2 backward")
        print("  4 rotate-left (in-place), 6 rotate-right (in-place)")
        print("  7 forward-left circle  (RIGHT track only)")
        print("  9 forward-right circle (LEFT  track only)")
        print("  1 backward-left circle (RIGHT track only)")
        print("  3 backward-right circle(LEFT  track only)")
        print("  Arrow keys also work")
        print("")
        print("Stop:")
        print("  [SPACE], 's', or 5")
        print("")
        print("Speed profiles:")
        print("  i slow, o medium, p fast")
        print("")
        print("Speed scaling:")
        print("  w/+ increase both, e/- decrease both")
        print("  q// increase linear, r/* decrease linear")
        print("")
        print("Robot selection:")
        print("  m rescan + choose robot")
        print("")
        print("CTRL-C to quit.")
        print("--------------------------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        print(f"Linear Speed: {self.linear_speed:.3f}  Angular Speed: {self.angular_speed:.3f}")

    # ---------------- Robot discovery (TRUE SWARM) ----------------

    def _extract_robot_name_from_topic(self, topic: str) -> Optional[str]:
        """
        Expect /<robot>/cmd_vel. Return <robot> if match, else None.
        """
        if not topic:
            return None
        if not topic.startswith('/'):
            topic = '/' + topic
        parts = topic.split('/')
        # ['', '<robot>', 'cmd_vel']
        if len(parts) == 3 and parts[2] == 'cmd_vel' and parts[1]:
            return parts[1]
        return None

    def _discover_available_robots(self) -> Tuple[List[str], Dict[str, str]]:
        """
        Returns:
          - sorted list of robot names
          - mapping {robot_name: cmd_vel_topic}
        Criteria:
          - topic matches /<robot>/cmd_vel
          - type includes geometry_msgs/msg/Twist
          - has >=1 subscriber (motor driver node listening)
        """
        robots: Dict[str, str] = {}

        # Graph: topics + types
        topic_map = self.get_topic_names_and_types()

        for topic_name, types in topic_map:
            robot = self._extract_robot_name_from_topic(topic_name)
            if not robot:
                continue

            # Must be Twist topic
            if not any(t.endswith('geometry_msgs/msg/Twist') or t == 'geometry_msgs/msg/Twist' for t in types):
                continue

            # Must have at least one subscriber
            try:
                subs_info = self.get_subscriptions_info_by_topic(topic_name)
            except Exception:
                subs_info = []

            if subs_info and len(subs_info) > 0:
                robots[robot] = topic_name

        robot_list = sorted(robots.keys())
        return robot_list, robots

    def _print_available_robots(self, robots: List[str]):
        print("\n================= AVAILABLE ROBOTS =================")
        if not robots:
            print("No robots found yet.")
            print("A robot is considered 'available' if it has a /<robot>/cmd_vel topic")
            print("of type geometry_msgs/msg/Twist AND a motor driver is subscribed to it.")
        else:
            for r in robots:
                print(f" - {r}")
        print("====================================================\n")

    def _prompt_user_for_robot(self, robots: List[str]) -> Optional[str]:
        """
        Prompt until the user enters a valid robot name.
        Returns selected robot name, or None if user cancels (not used on startup).
        """
        robots_set = set(robots)

        while True:
            user_input = input("Enter a robot name (or press Enter to rescan): ").strip()
            if user_input == '':
                return None  # rescan
            if user_input in robots_set:
                return user_input
            print(f"[INVALID] '{user_input}' is not an available robot. Please try again.\n")

    def _true_swarm_startup_select_robot(self):
        """
        TRUE SWARM startup flow:
          - discover robots
          - print list
          - prompt until valid name chosen
        """
        # First: if hints exist, we can try to prefer them, but we still show list & validate.
        preferred_robot: Optional[str] = None
        if self.startup_topic_hint:
            inferred = self._extract_robot_name_from_topic(self.startup_topic_hint.strip())
            if inferred:
                preferred_robot = inferred
        elif self.startup_robot_hint:
            preferred_robot = self.startup_robot_hint

        while rclpy.ok():
            robots, robot_to_topic = self._discover_available_robots()
            self._print_available_robots(robots)

            # If preferred_robot exists and is valid, offer it as a shortcut.
            if preferred_robot and preferred_robot in robot_to_topic:
                ans = input(f"Press Enter to choose '{preferred_robot}', or type another robot name: ").strip()
                if ans == '':
                    chosen = preferred_robot
                else:
                    if ans in robot_to_topic:
                        chosen = ans
                    else:
                        print(f"[INVALID] '{ans}' is not an available robot. Try again.\n")
                        continue
            else:
                chosen = self._prompt_user_for_robot(robots)
                if chosen is None:
                    # User requested rescan (or there were no robots)
                    if not robots:
                        input("Press Enter to rescan...")
                    continue

            # Apply the chosen robot
            topic = robot_to_topic.get(chosen, f'/{chosen}/cmd_vel')
            self._apply_cmd_vel_topic(topic, announce_startup=True)
            return

    # ---------------- Active robot helpers ----------------

    def _publish_active_robot(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

    # ---------------- Topic application ----------------

    def _apply_cmd_vel_topic(self, new_topic: str, announce_startup: bool = False):
        if not new_topic.startswith('/'):
            new_topic = '/' + new_topic

        if new_topic == '/cmd_vel':
            print("[ERROR] Refusing to use /cmd_vel. Use /<robot_name>/cmd_vel for multi-robot architecture.")
            return

        new_robot_name = self._extract_robot_name_from_topic(new_topic)
        if new_robot_name:
            self.current_robot_name = new_robot_name
            self._publish_active_robot()

        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass

        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic

        if announce_startup:
            self._print_instructions(new_topic)
            print(f"[STARTUP] Now controlling: {self.current_robot_name}")
        else:
            print(f"[ROBOT SWITCH] Now publishing Twist to: {new_topic}")
            if self.current_robot_name:
                print(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")

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
        L = self.wheel_separation if self.wheel_separation > 1e-6 else 0.18
        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / L

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.last_twist = twist
        self.is_moving = True
        self.publisher_.publish(twist)

    # ---------------- Republish (robust driving) ----------------

    def _republish_last_twist(self):
        if not self.is_moving or self.publisher_ is None:
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

    # ---------------- Speed modifiers ----------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._print_current_speeds()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self._print_current_speeds()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        self._print_current_speeds()

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        self._print_current_speeds()

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self._print_current_speeds()

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._print_current_speeds()

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._print_current_speeds()

    # ---------------- Robot switching (validated) ----------------

    def _switch_robot_prompt_validated(self, settings):
        """
        'm' key behavior: rescan available robots (those with subscribed /<robot>/cmd_vel)
        and force user to choose a valid robot.
        """
        if not self.allow_cmd_vel_switching:
            print("[ROBOT SWITCH] cmd_vel switching disabled (allow_cmd_vel_switching:=False).")
            print(f"              Staying on: {self.cmd_vel_topic}")
            return

        restore_terminal_settings(settings)
        try:
            while True:
                robots, robot_to_topic = self._discover_available_robots()
                self._print_available_robots(robots)

                if not robots:
                    input("No robots available. Press Enter to rescan (or CTRL-C to quit)...")
                    continue

                ans = input("Enter robot name to control (or press Enter to cancel): ").strip()
                if ans == '':
                    break
                if ans not in robot_to_topic:
                    print(f"[INVALID] '{ans}' is not an available robot. Try again.\n")
                    continue

                self._apply_cmd_vel_topic(robot_to_topic[ans], announce_startup=False)
                break
        finally:
            tty.setraw(sys.stdin.fileno())

    # ---------------- Main loop ----------------

    def run(self):
        # TRUE SWARM STARTUP SELECTION (before terminal raw mode)
        self._true_swarm_startup_select_robot()

        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)
                if key == '':
                    continue
                if key == '\x03':  # Ctrl-C
                    break

                if self.publisher_ is None:
                    # Should not happen because startup selection enforces a robot,
                    # but keep behavior safe.
                    if key == 'm':
                        self._switch_robot_prompt_validated(settings)
                    elif key in self.speed_bindings:
                        self.speed_bindings[key]()
                    continue

                if key in self.move_bindings:
                    mode, lin_mult, ang_mult = self.move_bindings[key]

                    if mode == 'circle':
                        S = self.linear_speed

                        if key == '7':
                            self._publish_one_track_circle(v_left=0.0, v_right=+S)
                        elif key == '9':
                            self._publish_one_track_circle(v_left=+S, v_right=0.0)
                        elif key == '1':
                            self._publish_one_track_circle(v_left=0.0, v_right=-S)
                        elif key == '3':
                            self._publish_one_track_circle(v_left=-S, v_right=0.0)
                        continue

                    self.last_lin_mult = float(lin_mult)
                    self.last_ang_mult = float(ang_mult)

                    twist = Twist()
                    twist.linear.x = self.linear_speed * self.last_lin_mult
                    twist.angular.z = self.angular_speed * self.last_ang_mult

                    self.last_twist = twist
                    self.is_moving = True
                    self.publisher_.publish(twist)
                    continue

                if key in (' ', '5', 's'):
                    self.publisher_.publish(Twist())
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()
                    print("[STOP]")
                    continue

                if key == 'm':
                    self._switch_robot_prompt_validated(settings)
                    continue

                if key in self.speed_bindings:
                    self.speed_bindings[key]()
                    continue

        finally:
            if self.publisher_ is not None:
                self.publisher_.publish(Twist())
            restore_terminal_settings(settings)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
