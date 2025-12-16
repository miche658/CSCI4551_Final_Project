#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Parameters
        self.declare_parameter('video_path', 'video.mp4')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('fps', 30.0)      # publishing rate
        self.declare_parameter('loop', True)     # loop video when finished

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        # Open video
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video file: {video_path}")
            raise RuntimeError("Failed to open video file")

        # Publisher
        self.pub = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()

        # Timer based on FPS
        timer_period = 1.0 / self.fps if self.fps > 0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Publishing video '{video_path}' on topic '{image_topic}' at {self.fps} FPS"
        )

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            # End of video
            if self.loop:
                self.get_logger().info("Reached end of video, looping to start")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    self.get_logger().error("Failed to read frame after looping")
                    return
            else:
                self.get_logger().info("Reached end of video, stopping timer")
                self.timer.cancel()
                return

        # Convert OpenCV image (BGR) to ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

