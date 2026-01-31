#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
motor_driver_node.py

ROLE
----
This node runs ON THE ROBOT and converts a generic command:
    geometry_msgs/Twist on /<robot>/cmd_vel
into:
    motor outputs (GPIO PWM + direction pins)

This is the key "adapter boundary" that makes the fleet scalable:
- Teleop and fleet_orchestrator only care about cmd_vel and execute_playbook.
- Each robot's wiring + drive type lives in robot_profiles.yaml.

SUPPORTED DRIVE TYPES
---------------------
- diff_drive
- mecanum

SUPPORTED HARDWARE ADAPTERS (example)
-------------------------------------
- hbridge_2ch   (left/right)
- tb6612_4ch    (fl/fr/rl/rr)

IMPORTANT NOTE FOR NEW ROS2 USERS
---------------------------------
We keep this file pure Python and avoid exotic dependencies.

GPIO libraries differ by platform:
- On Raspberry Pi, you might use RPi.GPIO or gpiozero or pigpio.
- On Jetson, you might use Jetson.GPIO.
- On a dev laptop, GPIO won't exist.

So this node:
- tries to import a GPIO backend
- if it cannot, it enters "DRY RUN" mode:
    it logs what it *would* do, but does not touch GPIO pins

That makes it debuggable anywhere.
"""

from __future__ import annotations

import getpass
import math
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .drive_profiles import load_profile_registry, resolve_robot_profile


# ---------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# ---------------------------------------------------------------------
# GPIO abstraction layer
# ---------------------------------------------------------------------
class GpioBackend:
    """
    Tiny interface so we can swap GPIO implementations.

    The node needs:
      - set_motor(channel_name, value in [-1..+1])
      - stop_all()

    Where:
      value sign controls direction, magnitude controls PWM duty cycle.
    """

    def set_motor(self, name: str, value: float) -> None:
        raise NotImplementedError

    def stop_all(self) -> None:
        raise NotImplementedError


class DryRunGpio(GpioBackend):
    """
    Used when no GPIO library is available.
    Logs motor commands at a low rate so terminals stay readable.
    """
    def __init__(self, logger):
        self._log = logger
        self._last_log_t = 0.0

    def set_motor(self, name: str, value: float) -> None:
        now = time.monotonic()
        # log at most ~5 Hz so we don't spam
        if (now - self._last_log_t) > 0.20:
            self._log.info(f"[DRYRUN] motor {name}: {value:+.2f}")
            self._last_log_t = now

    def stop_all(self) -> None:
        self._log.warn("[DRYRUN] stop_all()")


class SimpleRpiGpioBackend(GpioBackend):
    """
    A minimal Raspberry Pi GPIO backend using RPi.GPIO if available.

    If you prefer gpiozero/pigpio later, you can add another backend class.
    """
    def __init__(self, logger, gpio_map: Dict[str, Dict[str, Any]], pwm_hz: int = 20000):
        self._log = logger
        self._gpio_map = gpio_map
        self._pwm_hz = int(pwm_hz)

        try:
            import RPi.GPIO as GPIO  # type: ignore
            self.GPIO = GPIO
        except Exception as e:
            raise RuntimeError(f"RPi.GPIO import failed: {e}")

        GPIO = self.GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self._pwms = {}

        # gpio_map is like:
        #   {"left":{"pwm":18,"in1":23,"in2":24}, "right":{...}}
        for name, pins in gpio_map.items():
            pwm_pin = int(pins["pwm"])
            in1 = int(pins["in1"])
            in2 = int(pins["in2"])

            GPIO.setup(pwm_pin, GPIO.OUT)
            GPIO.setup(in1, GPIO.OUT)
            GPIO.setup(in2, GPIO.OUT)

            pwm = GPIO.PWM(pwm_pin, self._pwm_hz)
            pwm.start(0.0)
            self._pwms[name] = (pwm, in1, in2)

        self._log.info("[GPIO] RPi.GPIO backend initialized")

    def set_motor(self, name: str, value: float) -> None:
        value = clamp(value, -1.0, 1.0)
        pwm, in1, in2 = self._pwms[name]
        GPIO = self.GPIO

        # Direction
        if value >= 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)

        duty = abs(value) * 100.0  # PWM duty in percent
        pwm.ChangeDutyCycle(clamp(duty, 0.0, 100.0))

    def stop_all(self) -> None:
        for name in list(self._pwms.keys()):
            self.set_motor(name, 0.0)


# ---------------------------------------------------------------------
# MotorDriver node
# ---------------------------------------------------------------------
class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # Robot identity follows your convention: default = Linux username.
        self.declare_parameter("robot_name", getpass.getuser())
        self.declare_parameter("profiles_path", "")

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None

        # Load and resolve profile.
        registry = load_profile_registry(profiles_path)
        prof = resolve_robot_profile(registry, self.robot)

        self.profile_name = prof["profile_name"]
        self.drive_type = prof["drive_type"]
        self.hardware = prof["hardware"]
        self.params = prof["params"]
        self.gpio_map = prof["gpio"]

        # The cmd_vel topic is always /<robot>/cmd_vel (or template)
        tmpl = str(prof.get("cmd_vel_topic_template", "/{robot}/cmd_vel"))
        self.cmd_vel_topic = tmpl.format(robot=self.robot)

        # Tuning params with safe defaults.
        self.max_linear = float(self.params.get("max_linear_mps", 0.60))
        self.max_strafe = float(self.params.get("max_strafe_mps", 0.50))
        self.max_angular = float(self.params.get("max_angular_rps", 3.00))

        self.wheel_sep = float(self.params.get("wheel_separation_m", 0.18))
        self.wheelbase = float(self.params.get("wheelbase_m", 0.20))
        self.trackwidth = float(self.params.get("trackwidth_m", 0.20))

        self.alpha = clamp(float(self.params.get("smoothing_alpha", 0.20)), 0.0, 0.95)
        self.cmd_timeout_s = float(self.params.get("cmd_timeout_s", 0.50))

        # State for smoothing and watchdog.
        self._last_cmd_t = time.monotonic()
        self._smoothed = (0.0, 0.0, 0.0)  # (vx, vy, wz)

        # Choose a GPIO backend.
        self.gpio = self._make_gpio_backend()

        # Subscribe to cmd_vel.
        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10)

        # Timer watchdog: stop if cmd_vel disappears.
        self.create_timer(0.05, self._watchdog_tick)  # 20 Hz watchdog

        # Thin-terminal friendly logs.
        self.get_logger().info(f"[{self.robot}] motor_driver online")
        self.get_logger().info(f"[{self.robot}] profile={self.profile_name} drive={self.drive_type} hw={self.hardware}")
        self.get_logger().info(f"[{self.robot}] cmd_vel={self.cmd_vel_topic} timeout={self.cmd_timeout_s:.2f}s")

    # ---------------- backend selection ----------------
    def _make_gpio_backend(self) -> GpioBackend:
        """
        Decide which GPIO backend to use.

        If RPi.GPIO is unavailable, we use DryRun mode.
        """
        # If gpio_map looks empty, we also dry run.
        if not isinstance(self.gpio_map, dict) or len(self.gpio_map.keys()) == 0:
            self.get_logger().warn("[GPIO] No gpio map found; using DRY RUN backend")
            return DryRunGpio(self.get_logger())

        try:
            # You can add other backends here based on self.hardware.
            # For now, we use RPi.GPIO for both hbridge_2ch and tb6612_4ch,
            # since both are just "motor channels with pwm/in1/in2".
            return SimpleRpiGpioBackend(self.get_logger(), self.gpio_map)
        except Exception as e:
            self.get_logger().warn(f"[GPIO] Backend init failed, using DRY RUN: {e}")
            return DryRunGpio(self.get_logger())

    # ---------------- cmd_vel handling ----------------
    def _cmd_vel_cb(self, msg: Twist) -> None:
        """
        Called whenever a Twist arrives.

        We:
          1) clamp to max speeds
          2) smooth to reduce jerk
          3) mix into motor channel outputs
        """
        self._last_cmd_t = time.monotonic()

        # Extract desired velocities.
        vx = clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vy = clamp(msg.linear.y, -self.max_strafe, self.max_strafe)
        wz = clamp(msg.angular.z, -self.max_angular, self.max_angular)

        # Smooth: exponential moving average.
        sx, sy, sz = self._smoothed
        sx = lerp(vx, sx, self.alpha)
        sy = lerp(vy, sy, self.alpha)
        sz = lerp(wz, sz, self.alpha)
        self._smoothed = (sx, sy, sz)

        # Convert to motor outputs in [-1..+1]
        if self.drive_type == "mecanum":
            outputs = self._mix_mecanum(sx, sy, sz)
        else:
            outputs = self._mix_diff(sx, sz)

        # Apply to motors
        for name, val in outputs.items():
            self.gpio.set_motor(name, val)

    def _mix_diff(self, vx: float, wz: float) -> Dict[str, float]:
        """
        Differential drive mixing.

        We convert (vx, wz) into left/right wheel speeds, then normalize to [-1..+1].

        wheel speeds:
          v_left  = vx - wz * L/2
          v_right = vx + wz * L/2
        """
        L = max(1e-6, self.wheel_sep)
        v_left = vx - wz * (L / 2.0)
        v_right = vx + wz * (L / 2.0)

        # Normalize by max_linear (simple) so outputs stay in [-1..+1].
        # This is not perfect physics, but it's stable and simple for early demos.
        denom = max(1e-6, self.max_linear)
        left_out = clamp(v_left / denom, -1.0, 1.0)
        right_out = clamp(v_right / denom, -1.0, 1.0)

        return {"left": left_out, "right": right_out}

    def _mix_mecanum(self, vx: float, vy: float, wz: float) -> Dict[str, float]:
        """
        Mecanum mixing.

        A common simplified mixing:
          fl = vx + vy + wz*K
          fr = vx - vy - wz*K
          rl = vx - vy + wz*K
          rr = vx + vy - wz*K

        K scales rotation based on geometry. We use (L+W)/2 as a simple proxy.
        """
        L = max(1e-6, self.wheelbase)
        W = max(1e-6, self.trackwidth)
        K = (L + W) / 2.0

        fl = vx + vy + wz * K
        fr = vx - vy - wz * K
        rl = vx - vy + wz * K
        rr = vx + vy - wz * K

        # Normalize so the largest magnitude becomes 1.0
        m = max(abs(fl), abs(fr), abs(rl), abs(rr), 1e-6)
        fl, fr, rl, rr = fl / m, fr / m, rl / m, rr / m

        return {"fl": clamp(fl, -1.0, 1.0),
                "fr": clamp(fr, -1.0, 1.0),
                "rl": clamp(rl, -1.0, 1.0),
                "rr": clamp(rr, -1.0, 1.0)}

    # ---------------- watchdog ----------------
    def _watchdog_tick(self) -> None:
        """
        Stops the robot if cmd_vel has not arrived recently.

        This is critical for safety:
        - if teleop crashes
        - if WiFi drops
        - if orchestrator stops publishing
        """
        now = time.monotonic()
        if (now - self._last_cmd_t) > self.cmd_timeout_s:
            # Stop only once per "timeout episode" to avoid log spam.
            self._last_cmd_t = now
            self._smoothed = (0.0, 0.0, 0.0)
            self.gpio.stop_all()
            self.get_logger().warn(f"[{self.robot}] cmd_vel timeout -> STOP")


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.gpio.stop_all()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
