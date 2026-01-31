#!/usr/bin/env python3
# SPDX-L:contentReference[oaicite:19]{index=19}Proprietary

"""
robot_bringup.launch.py

Robot-side bringup for ONE robot.

Starts:
  - motor_driver_node (always)
  - heartbeat_node    (always)
  - unit_executor_action_server (optional; default true)
  - fpv_control_arbiter         (optional; default false)

WHY THIS LAUNCH FILE EXISTS
---------------------------
It ensures every robot in a heterogeneous fleet comes up the same way,
with differences controlled only by robot_profiles.yaml.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_robot_name = os.environ.get("USER", "robot")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value=default_robot_name,
        description="Robot identity name. Default: Linux username.",
    )

    profiles_path_arg = DeclareLaunchArgument(
        "profiles_path",
        default_value="",
        description=(
            "Optional explicit path to robot_profiles.yaml. "
            "If empty, nodes use installed share/<pkg>/config/robot_profiles.yaml."
        ),
    )

    enable_playbook_arg = DeclareLaunchArgument(
        "enable_playbook",
        default_value="true",
        description="Start unit_executor_action_server (robot-side playbook executor).",
    )

    enable_fpv_arg = DeclareLaunchArgument(
        "enable_fpv",
        default_value="false",
        description="Start fpv_control_arbiter (robot-side control arbitration).",
    )

    common_params = [
        {"robot_name": LaunchConfiguration("robot_name")},
        {"profiles_path": LaunchConfiguration("profiles_path")},
    ]

    motor_driver = Node(
        package="robot_legion_teleop_python",
        executable="motor_driver_node",
        name="motor_driver_node",
        output="screen",
        parameters=common_params,
    )

    heartbeat = Node(
        package="robot_legion_teleop_python",
        executable="heartbeat_node",
        name="heartbeat_node",
        output="screen",
        parameters=common_params,
    )

    unit_executor = Node(
        package="robot_legion_teleop_python",
        executable="unit_executor_action_server",
        name="unit_executor_action_server",
        output="screen",
        parameters=common_params,
        condition=IfCondition(LaunchConfiguration("enable_playbook")),
    )

    fpv_arbiter = Node(
        package="robot_legion_teleop_python",
        executable="fpv_control_arbiter",
        name="fpv_control_arbiter",
        output="screen",
        parameters=common_params,
        condition=IfCondition(LaunchConfiguration("enable_fpv")),
    )

    return LaunchDescription([
        robot_name_arg,
        profiles_path_arg,
        enable_playbook_arg,
        enable_fpv_arg,
        motor_driver,
        heartbeat,
        unit_executor,
        fpv_arbiter,
    ])
