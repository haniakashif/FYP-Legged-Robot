import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class CeilingFlipper(Node):
    def __init__(self):
        super().__init__('ceiling_flipper')
        self.sub = self.create_subscription(
            PointCloud2, '/ceiling_points', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/ceiling_points_flipped', 10)

    def callback(self, msg):
        points = list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))
        
        if not points:
            return

        # Flip Y axis — ceiling becomes floor in flipped frame
        flipped = [(p[0], p[1], -p[2]) for p in points]
        
        out = pc2.create_cloud_xyz32(msg.header, flipped)
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CeilingFlipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()