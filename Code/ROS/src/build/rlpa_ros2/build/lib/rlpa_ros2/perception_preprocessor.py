import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from grid_map_msgs.msg import GridMap
import numpy as np

class PerceptionPreprocessor(Node):
    def __init__(self):
        super().__init__('perception_preprocessor')

        self.max_height = 0.20 
        
        self.pc_sub = self.create_subscription(
            PointCloud2, '/rgbd_camera/points', self.pc_callback, 10)
        
        self.floor_pub = self.create_publisher(
            PointCloud2, '/filtered_points', 10)
        
        self.ceil_pub = self.create_publisher(
            PointCloud2, '/ceiling_points_flipped', 10)

        self.map_sub = self.create_subscription(
            GridMap, '/ceiling_elevation_map', self.map_callback, 10)
        
        self.map_pub = self.create_publisher(
            GridMap, '/ceiling_elevation_map_flipped', 10)

        self.get_logger().info("Y-Forward Perception Preprocessor Active.")

    def pc_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        floor_pts = []
        ceil_flipped_pts = []

        for p in points:
            # Gazebo outputs the camera naturally: p[0]=Forward, p[1]=Left, p[2]=Up
            # Because the THex robot faces the Y-axis, we map the coordinates:
            x_base = -float(p[1])         # Camera Left -> Robot Right (-X)
            y_base = float(p[0]) + 0.05   # Camera Forward -> Robot Forward (+Y), plus 5cm mount offset
            z_base = float(p[2]) + 0.15   # Camera Up -> Robot Up (+Z), plus 15cm mount offset

            # Filter points based on true depth from the lens (p[0])
            if float(p[0]) < 0.05 or float(p[0]) > 3.0:
                continue

            # Split floor vs ceiling based on robot height
            if z_base < self.max_height:
                floor_pts.append((x_base, y_base, z_base))
            else:
                ceil_flipped_pts.append((x_base, y_base, -z_base)) 

        # Assign to base_link so ElevationMapping processes it instantly
        new_header = msg.header
        new_header.frame_id = 'base_link' 

        if floor_pts:
            floor_msg = pc2.create_cloud_xyz32(new_header, floor_pts)
            self.floor_pub.publish(floor_msg)

        if ceil_flipped_pts:
            ceil_msg = pc2.create_cloud_xyz32(new_header, ceil_flipped_pts)
            self.ceil_pub.publish(ceil_msg)

    def map_callback(self, msg):
        if 'elevation' not in msg.layers: return
        idx = msg.layers.index('elevation')
        arr = np.array(msg.data[idx].data, dtype=np.float32)
        msg.data[idx].data = (-arr).tolist()
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()