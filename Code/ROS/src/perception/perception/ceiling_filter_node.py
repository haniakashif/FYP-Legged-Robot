import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class CeilingFilter(Node):
    def __init__(self):
        super().__init__('ceiling_filter')

        # Points at or above this Z (relative to sensor) go to /ceiling_points
        self.max_height = 0.15

        self.sub = self.create_subscription(
            PointCloud2, '/rgbd_camera/points', self.callback, 10)
        self.pub_floor = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.pub_ceiling = self.create_publisher(PointCloud2, '/ceiling_points', 10)

    def callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        floor_pts = []
        ceiling_pts = []
        for p in points:
            if p[2] < self.max_height:
                floor_pts.append(p)
            else:
                ceiling_pts.append(p)

        if not floor_pts:
            return

        self.pub_floor.publish(pc2.create_cloud_xyz32(msg.header, floor_pts))
        self.pub_ceiling.publish(pc2.create_cloud_xyz32(msg.header, ceiling_pts))


def main(args=None):
    rclpy.init(args=args)
    node = CeilingFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
