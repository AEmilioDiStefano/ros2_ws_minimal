#!/usr/bin/env python3
"""
hardware_interface.py

Thin hardware abstraction layer for motor outputs.

Goals:
- Provide a consistent API for setting motor directions and PWM duty
- Allow a safe mock implementation when RPi.GPIO is not available (desktop)
- Keep behavior simple and well-documented for new engineers

Usage:
  from robot_legion_teleop_python.hardware_interface import HardwareInterface
  hw = HardwareInterface(gpio_map)
  hw.set_motor(left_duty, left_dir, right_duty, right_dir)
  hw.stop()

The gpio_map is a mapping of expected keys (names are flexible, see
drive_profiles.yaml) such as:
  {
    "en_left": 12,
    "in1_left": 23,
    "in2_left": 22,
    "en_right": 13,
    "in1_right": 27,
    "in2_right": 17,
  }

This module intentionally keeps hardware logic separated from motor mixing
so drive-type code can remain device-agnostic.
"""

from typing import Dict, Optional
import logging

try:
    import RPi.GPIO as GPIO  # type: ignore
    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False

LOG = logging.getLogger("hardware_interface")


class HardwareInterface:
    """Abstraction for simple 2-channel differential motor outputs.

    This implementation focuses on the common H-bridge configuration used
    in our profiles. It exposes a minimal API so other modules need not
    import or depend on RPi.GPIO directly.
    """

    def __init__(self, gpio_map: Optional[Dict[str, int]] = None):
        self.gpio_map = gpio_map or {}
        self.left_pwm = None
        self.right_pwm = None
        # Allow per-motor polarity inversion via profile flags. Accept several key names.
        self.invert_left = bool(
            self.gpio_map.get("invert_left")
            or self.gpio_map.get("left_inverted")
            or self.gpio_map.get("left_polarity_invert")
        )
        self.invert_right = bool(
            self.gpio_map.get("invert_right")
            or self.gpio_map.get("right_inverted")
            or self.gpio_map.get("right_polarity_invert")
        )

        if GPIO_AVAILABLE and self.gpio_map:
            try:
                self._setup_gpio()
                LOG.info("GPIO hardware interface initialized")
                LOG.info("GPIO map keys: %s; invert_left=%s invert_right=%s", list(self.gpio_map.keys()), self.invert_left, self.invert_right)
            except Exception as e:
                LOG.warning("GPIO init failed: %s", e)
                self._mock_mode()
        else:
            self._mock_mode()

    def _mock_mode(self):
        """Fallback that doesn't touch hardware; logs instead.
        This makes the package runnable on a laptop for development/testing.
        """
        self._mock = True
        if not GPIO_AVAILABLE:
            LOG.warning("RPi.GPIO not available - using mock hardware interface")
        else:
            LOG.warning("GPIO map empty - using mock hardware interface")

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # collect pins we expect for a 2-channel H-bridge
        pins = [
            self.gpio_map.get("in1_left"),
            self.gpio_map.get("in2_left"),
            self.gpio_map.get("in1_right"),
            self.gpio_map.get("in2_right"),
            self.gpio_map.get("en_left"),
            self.gpio_map.get("en_right"),
        ]
        pins = [p for p in pins if p is not None]

        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)

        # PWM channels
        self.left_pwm = GPIO.PWM(self.gpio_map.get("en_left"), 1000)
        self.right_pwm = GPIO.PWM(self.gpio_map.get("en_right"), 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        self._mock = False

    def set_motor(self, left_duty: float, left_dir: int, right_duty: float, right_dir: int):
        """Set motor outputs.

        left_dir/right_dir: 1 for forward, -1 for reverse
        duty: 0..100
        """
        if getattr(self, "_mock", True):
            LOG.debug("MOCK set_motor: L(duty=%s,dir=%s) R(duty=%s,dir=%s)", left_duty, left_dir, right_duty, right_dir)
            return

        # Apply configured polarity inversion if requested by the profile
        if self.invert_left:
            left_dir = -left_dir
        if self.invert_right:
            right_dir = -right_dir

        # left motor pins
        in1 = self.gpio_map.get("in1_left")
        in2 = self.gpio_map.get("in2_left")
        if in1 is not None and in2 is not None:
            GPIO.output(in1, GPIO.HIGH if left_dir > 0 else GPIO.LOW)
            GPIO.output(in2, GPIO.LOW if left_dir > 0 else GPIO.HIGH)

        # right motor pins
        in3 = self.gpio_map.get("in1_right")
        in4 = self.gpio_map.get("in2_right")
        if in3 is not None and in4 is not None:
            GPIO.output(in3, GPIO.HIGH if right_dir > 0 else GPIO.LOW)
            GPIO.output(in4, GPIO.LOW if right_dir > 0 else GPIO.HIGH)

        # set PWM duty
        if self.left_pwm is not None:
            self.left_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(left_duty))))
        if self.right_pwm is not None:
            self.right_pwm.ChangeDutyCycle(max(0.0, min(100.0, float(right_duty))))

    def stop(self):
        """Stop motors and cleanup if using real GPIO"""
        if getattr(self, "_mock", True):
            LOG.debug("MOCK stop called")
            return
        try:
            if self.left_pwm is not None:
                self.left_pwm.stop()
            if self.right_pwm is not None:
                self.right_pwm.stop()
            GPIO.cleanup()
        except Exception as e:
            LOG.warning("Error cleaning up GPIO: %s", e)
