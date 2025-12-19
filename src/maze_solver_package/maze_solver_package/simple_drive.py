#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class SimpleForward(Node):
    def __init__(self):
        super().__init__('simple_forward')
        
        # Create publisher for velocity commands w/ TwistStamped
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10
        )
        
        # Publish velocity commands at 10 Hz:
        self.timer = self.create_timer(0.1, self.move_forward)
        
        self.get_logger().info('Simple Forward node active')
    
    def move_forward(self):
        """Publish velocity command to move the robot forward"""
        msg = TwistStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = 0.2  # meters per second
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        
        # Rotation
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    simple_forward_node = SimpleForward()
    
    try:
        rclpy.spin(simple_forward_node)
    except KeyboardInterrupt:
        # Stop the robot before shutting down
        stop_msg = TwistStamped()
        stop_msg.header.stamp = simple_forward_node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        simple_forward_node.velocity_publisher.publish(stop_msg)
        simple_forward_node.get_logger().info('Stopping robot')
    finally:
        simple_forward_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
