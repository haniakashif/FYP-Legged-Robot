import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class CeilingFilter(Node):
    def __init__(self):
        super().__init__('ceiling_filter')

        # 1. Config: Max height (Z) relative to the sensor frame
        self.max_height = 0.15  # <--- EDIT THIS (meters)
        # Any point higher than this (relative to sensor) gets deleted.

        self.sub = self.create_subscription(
            PointCloud2, '/rgbd_camera/points', self.callback, 10)

        self.pub = self.create_publisher(
            PointCloud2, '/filtered_points', 10)

    def callback(self, msg):
        # 1. Read points
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # 2. Filter logic (Keep only points BELOW max_height)
        filtered_points = []
        for p in points:
            if p[2] < self.max_height:
                filtered_points.append(p)

        # 3. Publish new cloud
        if not filtered_points:
            return

        new_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CeilingFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
