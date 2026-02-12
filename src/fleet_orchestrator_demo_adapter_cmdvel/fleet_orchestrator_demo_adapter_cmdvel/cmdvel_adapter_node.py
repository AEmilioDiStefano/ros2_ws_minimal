#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
cmdvel_adapter_node.py (DEMO ONLY)

Subscribes:
  /fo/task   PlaybookCommand JSON on std_msgs/String

Publishes (demo mapping):
  /<robot>/cmd_vel   geometry_msgs/Twist

This is explicitly NOT the orchestrator. It is a replaceable adapter used to
demonstrate drop-in integration with an existing ROS 2 control stack such as:
- your teleop publishing /<robot>/cmd_vel
- your motor driver subscribing /<robot>/cmd_vel  :contentReference[oaicite:2]{index=2}

During DIU sprints, this adapter would instead call platform-native playbook APIs.
"""

import json
import getpass
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CmdVelDemoAdapter(Node):
    def __init__(self):
        super().__init__("fleet_orchestrator_cmdvel_demo_adapter")

        self.declare_parameter("default_robot", getpass.getuser())
        self.declare_parameter("linear_speed", 0.2)   # m/s demo
        self.declare_parameter("angular_speed", 0.6)  # rad/s demo

        self.default_robot = str(self.get_parameter("default_robot").value).strip() or getpass.getuser()
        self.lin = float(self.get_parameter("linear_speed").value)
        self.ang = float(self.get_parameter("angular_speed").value)

        self._pubs: Dict[str, rclpy.publisher.Publisher] = {}

        self.create_subscription(String, "/fo/task", self._on_task, 10)
        self.get_logger().info("CmdVelDemoAdapter ready: /fo/task -> /<robot>/cmd_vel (demo mapping)")

    def _pub_for(self, robot: str):
        if robot not in self._pubs:
            self._pubs[robot] = self.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
        return self._pubs[robot]

    def _safe_json(self, s: str) -> dict | None:
        try:
            return json.loads(s)
        except Exception:
            return None

    def _on_task(self, msg: String):
        d = self._safe_json(msg.data or "")
        if not d or d.get("type") != "playbook_command":
            return

        command_id = str(d.get("command_id", ""))
        params = d.get("parameters") if isinstance(d.get("parameters"), dict) else {}
        intent_id = str(d.get("intent_id", ""))

        # Demo targeting: allow passing vehicle_ids, else use default_robot
        targets = []
        if isinstance(params.get("vehicle_ids"), list):
            targets = [str(x) for x in params["vehicle_ids"] if str(x).strip()]
        if not targets:
            targets = [self.default_robot]

        # Demo mapping:
        # - hold_position -> publish zero twist
        # - transit with east_m/north_m -> simple forward motion (demo only)
        for robot in targets:
            pub = self._pub_for(robot)
            tw = Twist()

            if command_id == "hold_position":
                tw.linear.x = 0.0
                tw.angular.z = 0.0
                pub.publish(tw)
                self.get_logger().info(f"[DEMO] intent={intent_id} hold_position -> /{robot}/cmd_vel (stop)")
                continue

            if command_id == "transit":
                # demo: if east_m or north_m present, just drive forward for a moment
                # (real integration would call platform playbook / navigation)
                tw.linear.x = self.lin
                tw.angular.z = 0.0
                pub.publish(tw)
                self.get_logger().info(f"[DEMO] intent={intent_id} transit -> /{robot}/cmd_vel (forward demo)")
                continue

            # unknown / formation -> no actuator command (log only)
            self.get_logger().warn(f"[DEMO] intent={intent_id} command={command_id} not mapped to cmd_vel (as expected).")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDemoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
