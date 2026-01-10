#!/usr/bin/env python3
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LegionCameraNode(Node):
    def __init__(self):
        super().__init__('legion_camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 20.0)

        # Robot/topic selection (robot-agnostic)
        # Priority:
        #   1) image_topic (explicit override)
        #   2) robot_name -> /<robot_name>/camera/image_raw
        #   3) relative 'camera/image_raw' (namespaced-friendly)
        self.declare_parameter('robot_name', '')
        self.declare_parameter('image_topic', '')

        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_rate = float(self.get_parameter('frame_rate').value)

        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value.strip()
        image_topic_override = self.get_parameter('image_topic').get_parameter_value().string_value.strip()

        if image_topic_override:
            self.image_topic = image_topic_override
        elif robot_name:
            self.image_topic = f'/{robot_name}/camera/image_raw'
        else:
            self.image_topic = 'camera/image_raw'  # relative

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at index {camera_index}')
        else:
            self.get_logger().info(f'Opened camera at index {camera_index}')

        self.get_logger().info(f'Publishing camera images on: {self.image_topic}')
        if robot_name:
            self.get_logger().info(f'Camera node robot_name: {robot_name}')

        period = 1.0 / max(1.0, self.frame_rate)
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegionCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
