import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
import numpy as np

class CeilingMapFlipper(Node):
    def __init__(self):
        super().__init__('ceiling_map_flipper')
        self.sub = self.create_subscription(
            GridMap, '/ceiling_elevation_map', self.callback, 10)
        self.pub = self.create_publisher(
            GridMap, '/ceiling_elevation_map_flipped', 10)

    def callback(self, msg):
        if 'elevation' not in msg.layers:
            return

        idx = msg.layers.index('elevation')
        data = msg.data[idx]

        # Negate elevation values — inverse of the Z flip done on the pointcloud
        arr = np.array(data.data, dtype=np.float32)
        arr = -arr  # flip back

        msg.data[idx].data = arr.tolist()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CeilingMapFlipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()