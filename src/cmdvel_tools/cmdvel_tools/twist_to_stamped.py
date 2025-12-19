import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.declare_parameter('in_topic', '/cmd_vel_nav')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, out_topic, 10)

        self.get_logger().info(f'Converting Twist {in_topic} -> TwistStamped {out_topic}')

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(TwistToStamped())
    rclpy.shutdown()
