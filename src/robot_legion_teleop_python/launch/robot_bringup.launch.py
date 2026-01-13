from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import re
import getpass


def _sanitize_ros_name(name: str) -> str:
    # ROS name/topic-safe: alphanumerics + underscore only
    s = re.sub(r"[^A-Za-z0-9_]", "_", (name or "").strip())
    s = s.strip("_")
    return s or "robot1"


def _default_robot_name_from_username() -> str:
    # Linux username (like you requested)
    try:
        u = getpass.getuser()
        if u:
            return _sanitize_ros_name(u)
    except Exception:
        pass

    # Fallback: env var USER
    u = os.environ.get("USER", "").strip()
    if u:
        return _sanitize_ros_name(u)

    return "robot1"


def _make_nodes(context, *args, **kwargs):
    robot_name = _default_robot_name_from_username()

    video_device = LaunchConfiguration("video_device").perform(context)
    use_camera = LaunchConfiguration("use_camera").perform(context).lower() in ("1", "true", "yes", "on")

    nodes = []

    # Motor driver under /<robot_name>/...
    nodes.append(
        Node(
            package="robot_legion_teleop_python",
            executable="motor_driver_node",
            name="motor_driver_node",
            namespace=robot_name,
            output="screen",
            parameters=[
                {"robot_name": robot_name},
                {"cmd_vel_topic": f"/{robot_name}/cmd_vel"},
            ],
        )
    )

    # Camera publisher under /<robot_name>/image_raw (USB webcam)
    if use_camera:
        nodes.append(
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="camera",
                namespace=robot_name,
                output="screen",
                parameters=[
                    {"video_device": video_device},
                ],
                # In the namespace, image_raw becomes /<robot_name>/image_raw automatically,
                # but we keep this explicit for clarity and future node swaps.
                remappings=[
                    ("image_raw", "image_raw"),
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            OpaqueFunction(function=_make_nodes),
        ]
    )
