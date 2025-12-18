#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleDrive(Node):
    def __init__(self):
        super().__init__('simple_drive')
        
        # Declare parameter for robot namespace
        self.declare_parameter('robot_namespace', 'tb3_0')
        robot_ns = self.get_parameter('robot_namespace').value
        
        # Create publisher to cmd_vel topic
        self.publisher = self.create_publisher(
            Twist, 
            f'/{robot_ns}/cmd_vel', 
            10
        )
        
        # Create timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info(f'Publishing to /{robot_ns}/cmd_vel')
    
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDrive()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
