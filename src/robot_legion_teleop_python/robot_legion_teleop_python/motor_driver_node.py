#!/usr/bin/env python3
import os
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.logging import get_logger

from geometry_msgs.msg import Twist

from robot_legion_teleop_python.drive_profiles import (
    load_robot_profiles_yaml,
    resolve_profile_for_robot,
    compute_motor_commands,
)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class MotorPins:
    pwm: int
    in1: int
    in2: int


class SimpleRpiGpioBackend:
    """
    Minimal GPIO backend using RPi.GPIO PWM.

    IMPORTANT:
      - Software PWM is typically reliable in the ~200–2000 Hz range.
      - Very high PWM frequencies (e.g., 20 kHz) often result in no usable drive on SBC software PWM.
    """

    def __init__(self, logger, gpio_map: dict, pwm_hz: int = 1000, min_duty: float = 0.0):
        self.log = logger
        self.gpio_map = gpio_map
        self._pwm_hz = int(pwm_hz)
        self._min_duty = clamp(float(min_duty), 0.0, 99.0)

        self._GPIO = None
        self._pwm_objs = {}  # motor_name -> PWM object

        try:
            import RPi.GPIO as GPIO  # type: ignore
            self._GPIO = GPIO
        except Exception as e:
            self.log.error(f"RPi.GPIO import failed; running in DRYRUN mode: {e}")
            self._GPIO = None

        if self._GPIO is None:
            return

        GPIO = self._GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Configure pins + PWM channels
        for motor_name, pins in self.gpio_map.items():
            mp = MotorPins(**pins)
            for p in (mp.pwm, mp.in1, mp.in2):
                GPIO.setup(p, GPIO.OUT)
                GPIO.output(p, GPIO.LOW)

            pwm = GPIO.PWM(mp.pwm, self._pwm_hz)
            pwm.start(0.0)
            self._pwm_objs[motor_name] = pwm

        self.log.info(f"GPIO backend ready (RPi.GPIO, pwm_hz={self._pwm_hz}, min_duty={self._min_duty})")

    def shutdown(self):
        if self._GPIO is None:
            return
        try:
            for pwm in self._pwm_objs.values():
                pwm.ChangeDutyCycle(0.0)
                pwm.stop()
            self._GPIO.cleanup()
        except Exception as e:
            self.log.warn(f"GPIO cleanup warning: {e}")

    def set_motor(self, motor_name: str, value: float):
        """
        value is expected in [-1.0, 1.0]
        """
        value = clamp(float(value), -1.0, 1.0)
        duty = abs(value) * 100.0  # PWM duty in percent
        # Optional floor to overcome stiction (e.g., small DC motors)
        if duty > 0.0 and self._min_duty > 0.0:
            duty = self._min_duty + (duty * (100.0 - self._min_duty) / 100.0)
        duty = clamp(duty, 0.0, 100.0)

        if motor_name not in self.gpio_map:
            self.log.warn(f"Unknown motor '{motor_name}' in gpio_map")
            return

        mp = MotorPins(**self.gpio_map[motor_name])

        # DRYRUN mode: just log
        if self._GPIO is None:
            self.log.info(f"[DRYRUN] set_motor {motor_name}: value={value:.3f} duty={duty:.1f}")
            return

        GPIO = self._GPIO

        if value > 0.0:
            GPIO.output(mp.in1, GPIO.HIGH)
            GPIO.output(mp.in2, GPIO.LOW)
        elif value < 0.0:
            GPIO.output(mp.in1, GPIO.LOW)
            GPIO.output(mp.in2, GPIO.HIGH)
        else:
            GPIO.output(mp.in1, GPIO.LOW)
            GPIO.output(mp.in2, GPIO.LOW)

        pwm = self._pwm_objs.get(motor_name)
        if pwm is not None:
            pwm.ChangeDutyCycle(duty)


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # Load robot profiles (drive type + gpio map + kinematics)
        profiles_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "robot_profiles.yaml",
        )
        profiles_path = os.path.abspath(profiles_path)
        self.declare_parameter("profiles_path", profiles_path)
        profiles_path = self.get_parameter("profiles_path").get_parameter_value().string_value

        self.all_profiles = load_robot_profiles_yaml(profiles_path)
        self.robot = os.environ.get("ROBOT_NAME", "")
        if not self.robot:
            # allow a param override too
            self.declare_parameter("robot_name", "")
            self.robot = self.get_parameter("robot_name").get_parameter_value().string_value.strip()

        if not self.robot:
            self.robot = "robot"

        self.profile = resolve_profile_for_robot(self.all_profiles, self.robot)
        self.params = self.profile.get("params", {}) if self.profile else {}

        self.drive_type = self.profile.get("drive_type", "diff_drive")
        self.gpio_map = self.profile.get("gpio_map", {})

        # Topics / timeouts
        self.cmd_vel_topic = f"/{self.robot}/cmd_vel"
        self.cmd_timeout_s = float(self.params.get("cmd_timeout_s", 0.50))
        # PWM tuning (software PWM on SBCs works best around 200–2000 Hz)
        self.pwm_hz = int(self.params.get("pwm_hz", 1000))
        self.min_pwm_duty = float(self.params.get("min_pwm_duty", 0.0))

        self.get_logger().info(f"[{self.robot}] drive_type={self.drive_type}")
        self.get_logger().info(f"[{self.robot}] cmd_vel={self.cmd_vel_topic} timeout={self.cmd_timeout_s:.2f}s")
        self.get_logger().info(f"[{self.robot}] pwm_hz={self.pwm_hz} min_pwm_duty={self.min_pwm_duty}")

        # Backend
        self.gpio = self._make_gpio_backend()

        # cmd_vel input
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, qos)

        self.last_cmd_time = 0.0
        self.last_twist = Twist()

        # Watchdog loop
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

    def _make_gpio_backend(self):
        try:
            return SimpleRpiGpioBackend(self.get_logger(), self.gpio_map, pwm_hz=self.pwm_hz, min_duty=self.min_pwm_duty)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO backend: {e}")
            return SimpleRpiGpioBackend(self.get_logger(), self.gpio_map)  # DRYRUN fallback

    def _on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = time.time()

    def _stop_all(self):
        # best-effort: set all motors to 0
        for motor_name in self.gpio_map.keys():
            self.gpio.set_motor(motor_name, 0.0)

    def _tick(self):
        now = time.time()
        dt = now - self.last_cmd_time if self.last_cmd_time else 999.0

        if dt > self.cmd_timeout_s:
            self.get_logger().warn(f"[{self.robot}] cmd_vel timeout -> STOP")
            self._stop_all()
            return

        cmds = compute_motor_commands(self.drive_type, self.params, self.last_twist)

        # cmds is dict motor_name -> [-1..1]
        for motor_name, value in cmds.items():
            self.gpio.set_motor(motor_name, value)

    def destroy_node(self):
        try:
            self._stop_all()
            self.gpio.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
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
