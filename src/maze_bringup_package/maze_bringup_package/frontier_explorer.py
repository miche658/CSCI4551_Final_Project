import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleMover(Node):
    def __init__(self):
        super().__init__("simple_mover")

        # Declare parameters (just the command topic for now)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timer to move the robot forward at a constant speed
        self.timer = self.create_timer(1.0, self.move_forward)  # 1Hz rate

    def move_forward(self):
        """Move the robot forward at a constant speed."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Move forward at 0.2 m/s

        # Publish the movement command
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info("Moving forward at 0.2 m/s")

def main():
    rclpy.init()
    node = SimpleMover()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

