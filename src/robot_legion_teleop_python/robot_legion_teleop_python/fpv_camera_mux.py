#!/usr/bin/env python3
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FpvCameraMux(Node):
    def __init__(self):
        super().__init__('fpv_camera_mux')

        # Config
        self.declare_parameter('camera_topic_suffix', '/camera/image_raw')
        self.declare_parameter('output_topic', '/fpv_camera/image_raw')
        self.declare_parameter('startup_robot', '')  # optional convenience

        self.camera_topic_suffix = self.get_parameter('camera_topic_suffix').get_parameter_value().string_value.strip()
        if not self.camera_topic_suffix.startswith('/'):
            self.camera_topic_suffix = '/' + self.camera_topic_suffix

        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value.strip()
        if not self.output_topic.startswith('/'):
            self.output_topic = '/' + self.output_topic

        startup_robot = self.get_parameter('startup_robot').get_parameter_value().string_value.strip()

        # Subscribers we create per robot
        self.camera_subscribers: Dict[str, Subscription] = {}

        # Latest frame per robot
        self.latest_frames: Dict[str, Image] = {}

        # Active robot
        self.active_robot: Optional[str] = startup_robot if startup_robot else None

        # Output publisher
        self.output_pub = self.create_publisher(Image, self.output_topic, 10)

        # Subscribe to active robot topic from teleop
        self.active_robot_sub = self.create_subscription(
            String,
            '/teleop/active_robot',
            self.active_robot_callback,
            10
        )

        # If startup_robot provided, ensure subscription exists immediately
        if self.active_robot:
            self._ensure_robot_subscription(self.active_robot)

        # Timer to republish active frame
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info(f'FPV camera mux initialized. Output: {self.output_topic}')
        self.get_logger().info(f'Camera suffix: {self.camera_topic_suffix}')
        if self.active_robot:
            self.get_logger().info(f'Starting active robot: {self.active_robot}')

    def _robot_camera_topic(self, robot_name: str) -> str:
        return f'/{robot_name}{self.camera_topic_suffix}'

    def _ensure_robot_subscription(self, robot_name: str):
        if robot_name in self.camera_subscribers:
            return

        topic = self._robot_camera_topic(robot_name)

        sub = self.create_subscription(
            Image,
            topic,
            lambda msg, name=robot_name: self.camera_callback(msg, name),
            10
        )

        self.camera_subscribers[robot_name] = sub
        self.get_logger().info(f'FPV mux subscribed to {topic}')

    def active_robot_callback(self, msg: String):
        robot = msg.data.strip()
        if not robot:
            return

        self.active_robot = robot
        self._ensure_robot_subscription(robot)
        self.get_logger().info(f'FPV active robot set to: {self.active_robot}')

    def camera_callback(self, msg: Image, robot_name: str):
        self.latest_frames[robot_name] = msg

    def timer_callback(self):
        if not self.active_robot:
            return

        frame = self.latest_frames.get(self.active_robot)
        if frame is not None:
            self.output_pub.publish(frame)


def main(args=None):
    rclpy.init(args=args)
    node = FpvCameraMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
