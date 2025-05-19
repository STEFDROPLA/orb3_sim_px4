#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Create a publisher on the /camera topic
        self.publisher_ = self.create_publisher(Image, '/camera', 10)

        # Initialize OpenCV video capture (camera index 0)
        self.cap = cv2.VideoCapture(0)

        # Set resolution to 640x480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera /dev/video0")
            exit(1)

        # Convert OpenCV images to ROS2 Image messages
        self.bridge = CvBridge()

        # Publish at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        # Convert BGR (OpenCV default) to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Convert to ROS2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')

        # Add timestamp and frame ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Published one RGB frame with timestamp.")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
