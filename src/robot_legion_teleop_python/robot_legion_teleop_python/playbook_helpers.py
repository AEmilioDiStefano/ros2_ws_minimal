#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
playbook_helpers.py

This module holds "tiny autonomy helpers" used by unit_executor_action_server.py.

We keep this logic separate from the ActionServer node because:
- It makes the action server easier to read.
- It makes it easier to reuse the same timed-motion logic elsewhere.

This is intentionally simple: for early DIU-style demos, a lot of "autonomy"
can be mocked by timed Twist commands.
"""

import time
from dataclasses import dataclass
from typing import Callable

from geometry_msgs.msg import Twist


@dataclass
class TimedTwistPlan:
    """
    A tiny plan: publish one Twist for a duration.

    twist:
      The Twist to publish repeatedly.
    duration_s:
      How long to publish for.
    status_text:
      Text that gets shown in feedback (human readable).
    """
    twist: Twist
    duration_s: float
    status_text: str


def run_timed_twist(
    publish_fn: Callable[[Twist], None],
    feedback_fn: Callable[[float, str], None],
    plan: TimedTwistPlan,
    rate_hz: float = 20.0,
    stop_at_end: bool = True,
):
    """
    Publish a Twist for plan.duration_s with periodic feedback.

    publish_fn:
      A callback that publishes Twist (usually node.publisher.publish).
    feedback_fn:
      A callback that emits progress feedback (percent, status text).
    plan:
      TimedTwistPlan described above.
    rate_hz:
      Publishing frequency (default 20 Hz).
    stop_at_end:
      If True, publish a zero Twist at the end (safety).

    IMPORTANT:
    - We do time.sleep() here, which is "good enough" for demo autonomy.
    - If you later need real-time correctness, you'd convert to ROS Timers.
    """

    start = time.monotonic()
    end = start + max(0.0, float(plan.duration_s))

    # Prevent divide-by-zero and prevent absurd values.
    safe_rate = max(1.0, float(rate_hz))
    period = 1.0 / safe_rate

    last_fb = 0.0

    while True:
        now = time.monotonic()
        if now >= end:
            break

        # Publish the motion command.
        publish_fn(plan.twist)

        # Emit feedback about ~2 Hz (human-friendly).
        if (now - last_fb) >= 0.5:
            denom = (end - start)
            frac = (now - start) / denom if denom > 1e-9 else 1.0
            feedback_fn(max(0.0, min(1.0, frac)) * 100.0, plan.status_text)
            last_fb = now

        time.sleep(period)

    # Safety: stop the robot at the end unless explicitly disabled.
    if stop_at_end:
        publish_fn(Twist())
        feedback_fn(100.0, "completed")
