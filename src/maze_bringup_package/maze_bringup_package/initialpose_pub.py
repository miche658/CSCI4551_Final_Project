import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initialpose_pub')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.sent = False
        self.timer = self.create_timer(2.0, self.send_once)  # delay so slam_toolbox is alive

    def send_once(self):
        if self.sent:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0  # yaw = 0

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(10.0) ** 2

        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info("Published /initialpose at (0,0,0) in map frame once.")

def main():
    rclpy.init()
    node = InitialPosePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
