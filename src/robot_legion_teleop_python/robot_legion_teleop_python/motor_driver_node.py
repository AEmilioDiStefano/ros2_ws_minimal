#!/usr/bin/env python3
import time
import getpass
import logging
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .drive_profiles import load_profile_registry, resolve_robot_profile
from .hardware_interface import HardwareInterface
from .audit_logger import AuditLogger

LOG = logging.getLogger("motor_driver_node")


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # Robot naming
        self.declare_parameter("robot_name", getpass.getuser())
        self.robot_name = self.get_parameter("robot_name").value.strip() or getpass.getuser()

        # Parameters
        self.declare_parameter("wheel_separation", 0.18)   # meters
        self.declare_parameter("max_linear_speed", 0.4)    # m/s
        self.declare_parameter("max_angular_speed", 2.0)   # rad/s
        self.declare_parameter("max_pwm", 100)             # percent

        # Topic this robot listens to
        self.declare_parameter("cmd_vel_topic", f"/{self.robot_name}/cmd_vel")
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value.strip() or f"/{self.robot_name}/cmd_vel"

        # Hardware abstraction - load profile-based GPIO mapping when available
        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None

        gpio_map = {}
        try:
            reg = load_profile_registry(profiles_path)
            prof = resolve_robot_profile(reg, self.robot_name)
            gpio_map = prof.get("gpio", {}) or {}
            self.get_logger().info(f"[{self.robot_name}] Loaded GPIO map from profile: {list(gpio_map.keys())}")
        except Exception as ex:
            # Log the exception; drive_profiles.py should have already tried fallbacks.
            self.get_logger().warning(f"[{self.robot_name}] Failed to load profile registry: {ex}")
            self.get_logger().warning(f"[{self.robot_name}] Will run in mock hardware mode. To fix, pass profiles_path parameter or rebuild package.")

        # Normalize common profile naming and initialize hardware interface.
        gpio_map = self._normalize_gpio_map(gpio_map)
        self.hw = HardwareInterface(gpio_map)

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Watchdog
        self.last_cmd_time = time.time()
        self.timeout_sec = 0.5
        self.create_timer(0.1, self._watchdog)

        # Audit logging for DIU compliance
        audit_log_path = os.environ.get("ROBOT_AUDIT_LOG_PATH") or f"/tmp/robot_{self.robot_name}_audit.jsonl"
        self.audit = AuditLogger(self, "motor_driver", audit_log_path)

        self.get_logger().info(f"[{self.robot_name}] Motor driver listening on {self.cmd_vel_topic}")
        self.get_logger().info(f"[{self.robot_name}] Audit log: {audit_log_path}")

    # --------------------------------------------------

    def _normalize_gpio_map(self, gpio_map: dict) -> dict:
        """Normalize different hardware profile key names into a common
        mapping consumed by HardwareInterface (en_left,in1_left,in2_left,en_right,...).

        This keeps motor mixing logic independent of exact profile naming.
        """
        if not gpio_map:
            return {}

        # If profile already uses expected keys, return as-is
        expected = ("en_left", "in1_left", "in2_left", "en_right", "in1_right", "in2_right")
        if all(k in gpio_map for k in ("en_left", "in1_left", "in2_left")) or all(k in gpio_map for k in ("fl_pwm", "fr_pwm")):
            # Map TB6612 style (fl/fr -> left/right) to en_left/en_right
            mapped = {}
            if "en_left" in gpio_map:
                mapped.update(gpio_map)
                return mapped

        mapped = {}
        # TB6612 dual naming -> map front-left/front-right to left/right
        if "fl_pwm" in gpio_map and "fr_pwm" in gpio_map:
            mapped["en_left"] = gpio_map.get("fl_pwm")
            mapped["in1_left"] = gpio_map.get("fl_in1")
            mapped["in2_left"] = gpio_map.get("fl_in2")

            mapped["en_right"] = gpio_map.get("fr_pwm")
            mapped["in1_right"] = gpio_map.get("fr_in1")
            mapped["in2_right"] = gpio_map.get("fr_in2")
            return mapped

        # hbridge_2ch style mapping
        if "en_left" in gpio_map or "in1_left" in gpio_map:
            mapped.update(gpio_map)
            return mapped

        # older-style keys (en_left/en_right names may vary)
        # try common alternative keys
        if "ena" in gpio_map and "enb" in gpio_map:
            mapped["en_left"] = gpio_map.get("ena")
            mapped["en_right"] = gpio_map.get("enb")
            mapped["in1_left"] = gpio_map.get("in1_left")
            mapped["in2_left"] = gpio_map.get("in2_left")
            mapped["in1_right"] = gpio_map.get("in1_right")
            mapped["in2_right"] = gpio_map.get("in2_right")
            return mapped

        # fallback: return original map, HardwareInterface will mock if keys absent
        return gpio_map

    # --------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        start_time = time.time()

        wheel_sep = float(self.get_parameter("wheel_separation").value)
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_ang = float(self.get_parameter("max_angular_speed").value)

        v = max(-max_lin, min(max_lin, msg.linear.x))
        w = max(-max_ang, min(max_ang, msg.angular.z))

        # Log motor command for audit trail
        self.audit.log_command(
            robot=self.robot_name,
            source="cmd_vel",
            command_id="twist",
            parameters={"linear_x": float(msg.linear.x), "angular_z": float(msg.angular.z)},
            status="received",
            duration_s=time.time() - start_time,
        )

        # ===============================
        # TANK-SPIN OVERRIDE
        # ===============================
        if abs(v) < 1e-3 and abs(w) > 1e-3:
            spin_speed = 0.7 * max_lin
            direction = 1.0 if w > 0.0 else -1.0
            v_left = -direction * spin_speed
            v_right = +direction * spin_speed
            self._set_motor_outputs(v_left, v_right)
            return

        # ===============================
        # NORMAL DIFF-DRIVE MODE
        # ===============================
        v_left = v - (w * wheel_sep / 2.0)
        v_right = v + (w * wheel_sep / 2.0)
        self._set_motor_outputs(v_left, v_right)

    # --------------------------------------------------

    def _watchdog(self):
        if time.time() - self.last_cmd_time > self.timeout_sec:
            self._set_motor_outputs(0.0, 0.0)

    # --------------------------------------------------

    def _set_motor_outputs(self, v_left: float, v_right: float):
        # Convert speeds to duty+direction and delegate to hardware interface
        max_lin = float(self.get_parameter("max_linear_speed").value)
        max_pwm = float(self.get_parameter("max_pwm").value)

        def speed_to_pwm(v):
            ratio = max(-1.0, min(1.0, v / max_lin))
            direction = 1 if ratio >= 0 else -1
            duty = abs(ratio) * max_pwm
            return duty, direction

        left_duty, left_dir = speed_to_pwm(v_left)
        right_duty, right_dir = speed_to_pwm(v_right)

        # Use the hardware abstraction layer to actually set pins/PWM
        try:
            self.hw.set_motor(left_duty, left_dir, right_duty, right_dir)
        except Exception as e:
            LOG.warning("Failed to set motor outputs: %s", e)

    # --------------------------------------------------

    def destroy_node(self):
        try:
            self._set_motor_outputs(0.0, 0.0)
        except Exception:
            pass
        try:
            if hasattr(self, "hw"):
                self.hw.stop()
        except Exception:
            pass
        try:
            if hasattr(self, "audit"):
                self.audit.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
