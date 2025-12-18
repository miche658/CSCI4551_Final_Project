import math
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

FREE = 0
UNKNOWN = -1

def neighbors4(i, w, h):
    x = i % w
    y = i // w
    out = []
    if x > 0: out.append(i - 1)
    if x < w - 1: out.append(i + 1)
    if y > 0: out.append(i - w)
    if y < h - 1: out.append(i + w)
    return out

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic_debug", "/frontier_goal")
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("replan_period_sec", 3.0)
        self.declare_parameter("goal_tolerance_m", 0.35)

        map_topic = self.get_parameter("map_topic").value
        self.sub = self.create_subscription(OccupancyGrid, map_topic, self.on_map, 10)

        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic_debug").value, 10)

        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active(localizer='slam_toolbox')
        self.map = None
        self.last_goal = None
        self.blacklist = set()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = "base_footprint"   
        self.timer = self.create_timer(self.get_parameter("replan_period_sec").value, self.tick)

    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def tick(self):
        if self.map is None:
            return

        # If Nav2 is currently driving, do not spam new goals
        if self.nav.isTaskComplete() is False:
            return

        # If a task finished, check result and blacklist failed goals
        if self.last_goal is not None:
            res = self.nav.getResult()
            if res != TaskResult.SUCCEEDED:
                self.blacklist.add((round(self.last_goal.pose.position.x, 2), round(self.last_goal.pose.position.y, 2)))

        goal = self.compute_best_frontier_goal()
        if goal is None:
            self.get_logger().info("No frontiers found. Exploration complete or map not updating.")
            return

        self.last_goal = goal
        self.goal_pub.publish(goal)
        self.nav.goToPose(goal)

    def compute_best_frontier_goal(self):
        m = self.map
        w, h = m.info.width, m.info.height
        data = m.data
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        # frontier cell: UNKNOWN with at least one FREE neighbor
        is_frontier = [False] * (w * h)
        for i in range(w * h):
            if data[i] != UNKNOWN:
                continue
            for nb in neighbors4(i, w, h):
                if data[nb] == FREE:
                    is_frontier[i] = True
                    break

        visited = [False] * (w * h)
        clusters = []

        for i in range(w * h):
            if not is_frontier[i] or visited[i]:
                continue
            q = deque([i])
            visited[i] = True
            cluster = []
            while q:
                j = q.popleft()
                cluster.append(j)
                for nb in neighbors4(j, w, h):
                    if is_frontier[nb] and not visited[nb]:
                        visited[nb] = True
                        q.append(nb)
            clusters.append(cluster)

        min_sz = int(self.get_parameter("min_cluster_size").value)
        clusters = [c for c in clusters if len(c) >= min_sz]
        if not clusters:
            return None

        # Get robot pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.base_frame, rclpy.time.Time()
            )
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed map->{self.base_frame}: {e}")
            return None


        best = None
        best_score = -1e9

        for c in clusters:
            # centroid in grid coords
            xs = [(idx % w) for idx in c]
            ys = [(idx // w) for idx in c]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)

            # convert to world
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res

            # score: prefer larger clusters, closer distance
            dist = math.hypot(wx - rx, wy - ry)
            score = (len(c) * 1.0) - (dist * 20.0)

            gx, gy = round(wx, 2), round(wy, 2)
            key = (gx, gy)
            if key in self.blacklist:
                continue

            if score > best_score:
                best_score = score
                best = (wx, wy)

        if best is None:
            return None

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(best[0])
        goal.pose.position.y = float(best[1])
        goal.pose.orientation.w = 1.0
        return goal

def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

