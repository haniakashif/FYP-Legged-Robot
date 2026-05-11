import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PerceptionPreprocessor(Node):
    def __init__(self):
        super().__init__('perception_preprocessor')
        
        self.pc_sub = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.pc_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/aligned_cave_cloud', 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/perception/spatial_commands', 10)

        # --- KINEMATIC LIMITS ---
        self.min_height = 0.06
        self.max_height = 0.12
        self.min_sprawl = 0.25
        self.max_sprawl = 0.50

        # --- LOOKAHEAD BOUNDING BOX ---
        self.lookahead_y_min = 0.05  # Look 5cm ahead
        self.lookahead_y_max = 0.50  # Look 50cm ahead
        
        self.get_logger().info("Axis-Swapped Spatial Preprocessor Active.")

    def pc_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        aligned_points = []
        
        # Track highest floor rock, lowest ceiling, and closest walls
        floor_z_max = -np.inf
        ceil_z_min = np.inf
        left_wall_x = -np.inf   # Negative X side
        right_wall_x = np.inf   # Positive X side

        for p in points:
            # THE EMPIRICAL MAPPING:
            # p[0] = Depth (Forward)
            # p[1] = Height (Up/Down) 
            # p[2] = Width (Left/Right)
            
            # Adjust these params based on the location of the camera link in the URDF
            # Map Width to X, Depth to Y, and Height to Z
            base_x = -float(p[2])         # Width -> Base X (Right)
            base_y = float(p[0]) + 0.13   # Depth -> Base Y (Forward) [+5cm offset]
            base_z = float(p[1]) + 0.06   # Height -> Base Z (Up) [+15cm offset]
            
            aligned_points.append([base_x, base_y, base_z])

            # 2. CHECK THE LOOKAHEAD BOX (Only compute obstacles physically in front of us)
            if self.lookahead_y_min < base_y < self.lookahead_y_max:
                
                # Ceiling/Floor Detection (Sample a 30cm wide strip in the center)
                if -0.15 < base_x < 0.15:
                    if base_z < 0.10: # Floor
                        if base_z > floor_z_max: floor_z_max = base_z
                    else: # Ceiling
                        if base_z < ceil_z_min: ceil_z_min = base_z
                
                # Wall Detection (Sample a mid-height band where the body is)
                if 0.05 < base_z < 0.25:
                    if base_x < 0.0 and base_x > left_wall_x:
                        left_wall_x = base_x
                    if base_x > 0.0 and base_x < right_wall_x:
                        right_wall_x = base_x

        # Publish perfectly leveled cloud to RViz
        new_header = msg.header
        new_header.frame_id = 'base_link'
        if aligned_points:
            cloud_msg = pc2.create_cloud_xyz32(new_header, aligned_points)
            self.cloud_pub.publish(cloud_msg)

        # 3. CALCULATE DECOUPLED COMMANDS
        if floor_z_max == -np.inf: floor_z_max = 0.0
        if ceil_z_min == np.inf: ceil_z_min = 1.0
        if left_wall_x == -np.inf: left_wall_x = -0.5
        if right_wall_x == np.inf: right_wall_x = 0.5

        tunnel_width = right_wall_x - left_wall_x

        # --- HEIGHT LOGIC (CRAWL) ---
        # Try to maintain a 12cm buffer below the ceiling. 
        # If the ceiling drops, the target_height drops to keep the buffer.
        target_height = ceil_z_min - 0.12
        target_height = np.clip(target_height, self.min_height, self.max_height)

        # --- SPRAWL LOGIC (SQUEEZE) ---
        # Try to maintain a 10cm buffer between the legs and the walls.
        # If the walls get tight, the sprawl gets narrow.
        target_sprawl = tunnel_width - 0.10
        target_sprawl = np.clip(target_sprawl, self.min_sprawl, self.max_sprawl)

        # Publish independent commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(target_height), float(target_sprawl)]
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()