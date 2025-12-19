import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.default_odom_frame = self.get_parameter("odom_frame").value
        self.default_base_frame = self.get_parameter("base_frame").value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, 50)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp

        # Prefer what the message says, fall back to params
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.default_odom_frame
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else self.default_base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
