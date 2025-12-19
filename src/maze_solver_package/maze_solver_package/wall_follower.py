#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.velocity_publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.get_logger().info('Wall Follower node started')
    
    def scan_callback(self, msg):
        # Get sensor readings
        front = min(msg.ranges[0:15] + msg.ranges[-15:])
        right = min(msg.ranges[265:275])
        
        # Create velocity command
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'base_link'
        
        # Very tight wall following parameters
        target_dist = 0.18  # Much closer to wall
        speed = 0.20
        turn_rate = 1.2  # Sharper turns
        
        # Control logic
        if front < 0.35:
            # Turn left when obstacle ahead - slow down more
            vel_msg.twist.linear.x = 0.03
            vel_msg.twist.angular.z = turn_rate * 1.1
        elif right < 0.15:
            # Too close - veer left
            vel_msg.twist.linear.x = speed * 0.5
            vel_msg.twist.angular.z = turn_rate * 0.6
        elif right > 0.35:
            # Too far - turn right to find wall
            vel_msg.twist.linear.x = speed * 0.8
            vel_msg.twist.angular.z = -turn_rate * 0.8
        elif right > target_dist + 0.03:
            # Drifting away - turn right
            vel_msg.twist.linear.x = speed * 0.9
            vel_msg.twist.angular.z = -turn_rate * 0.5
        else:
            # Good - go straight
            vel_msg.twist.linear.x = speed
            vel_msg.twist.angular.z = 0.0
        
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        node.velocity_publisher.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
