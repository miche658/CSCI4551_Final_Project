import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

UNKNOWN = -1
FREE = 0
OCC = 100

class MapMergeNode(Node):
    def __init__(self):
        super().__init__('map_merge_node')

        self.sub0 = self.create_subscription(OccupancyGrid, '/tb3_0/map', self.cb0, 10)
        self.sub1 = self.create_subscription(OccupancyGrid, '/tb3_1/map', self.cb1, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.map0 = None
        self.map1 = None

        self.timer = self.create_timer(0.5, self.tick)

    def cb0(self, msg): self.map0 = msg
    def cb1(self, msg): self.map1 = msg

    def compatible(self, a: OccupancyGrid, b: OccupancyGrid) -> bool:
        if a.info.width != b.info.width: return False
        if a.info.height != b.info.height: return False
        if abs(a.info.resolution - b.info.resolution) > 1e-9: return False
        if a.info.origin.position.x != b.info.origin.position.x: return False
        if a.info.origin.position.y != b.info.origin.position.y: return False
        return True

    def merge_cell(self, c0: int, c1: int) -> int:
        if c0 == OCC or c1 == OCC:
            return OCC
        if c0 == FREE or c1 == FREE:
            return FREE
        return UNKNOWN

    def tick(self):
        if self.map0 is None or self.map1 is None:
            return
        if not self.compatible(self.map0, self.map1):
            self.get_logger().warn('Maps not compatible (different size/origin/resolution). Not merging.')
            return

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map0.header.frame_id
        out.info = self.map0.info

        data0 = self.map0.data
        data1 = self.map1.data
        merged = [self.merge_cell(data0[i], data1[i]) for i in range(len(data0))]
        out.data = merged

        self.pub.publish(out)

def main():
    rclpy.init()
    node = MapMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
