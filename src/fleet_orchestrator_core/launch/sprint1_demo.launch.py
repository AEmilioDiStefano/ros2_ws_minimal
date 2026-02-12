#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fleet_orchestrator_core",
            executable="intent_gateway",
            name="intent_gateway",
            output="screen",
            parameters=[{"operator_id": "operator", "client_id": "client"}],
        ),
        Node(
            package="fleet_orchestrator_core",
            executable="intent_translator",
            name="intent_translator",
            output="screen",
            parameters=[{"translator_name": "rules_v1"}],
        ),
        Node(
            package="fleet_orchestrator_core",
            executable="audit_logger",
            name="audit_logger",
            output="screen",
            parameters=[{"log_dir": "/tmp/fleet_orchestrator_logs"}],
        ),
    ])
