import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def snap(x, grid):
    return round(x / grid) * grid

class FrontierCoordinator(Node):
    def __init__(self):
        super().__init__("frontier_coordinator")

        self.declare_parameter("claim_radius_m", 0.75)
        self.declare_parameter("grid_snap_m", 0.25)

        self.claim_radius = float(self.get_parameter("claim_radius_m").value)
        self.grid_snap = float(self.get_parameter("grid_snap_m").value)

        self.claimed = {}  # (x,y) -> "tb3_0" or "tb3_1"

        self.sub0 = self.create_subscription(PoseStamped, "/tb3_0/frontier/proposal", self.on_prop0, 10)
        self.sub1 = self.create_subscription(PoseStamped, "/tb3_1/frontier/proposal", self.on_prop1, 10)

        self.pub0 = self.create_publisher(PoseStamped, "/tb3_0/frontier/approved", 10)
        self.pub1 = self.create_publisher(PoseStamped, "/tb3_1/frontier/approved", 10)

    def too_close(self, x, y):
        for (cx, cy) in self.claimed.keys():
            if math.hypot(x - cx, y - cy) <= self.claim_radius:
                return True
        return False

    def handle_msg(self, robot, msg, pub):
        x = msg.pose.position.x
        y = msg.pose.position.y

        if self.too_close(x, y):
            self.get_logger().info(f"Reject {robot} proposal ({x:.2f},{y:.2f}) too close to claim")
            return

        k = (snap(x, self.grid_snap), snap(y, self.grid_snap))
        self.claimed[k] = robot
        self.get_logger().info(f"Approve {robot} claim at ({k[0]:.2f},{k[1]:.2f})")

        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        pub.publish(out)

    def on_prop0(self, msg): self.handle_msg("tb3_0", msg, self.pub0)
    def on_prop1(self, msg): self.handle_msg("tb3_1", msg, self.pub1)

def main():
    rclpy.init()
    node = FrontierCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
