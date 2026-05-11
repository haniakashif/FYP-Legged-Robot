import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, Float64
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PerceptionPreprocessor(Node):
    def __init__(self):
        super().__init__('perception_preprocessor')
        
        self.pc_sub = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.pc_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/aligned_cave_cloud', 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/perception/spatial_commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/perception/spatial_marker', 10)
        self.lh_sub = self.create_subscription(Float64, '/perception/lookahead_max', self.lookahead_cb, 10)

        # --- KINEMATIC LIMITS ---
        self.min_height = 0.06
        self.max_height = 0.12
        self.min_sprawl = 0.25
        self.max_sprawl = 0.50

        # --- LOOKAHEAD BOUNDING BOX ---
        self.lookahead_y_min = 0.02  # Look 2cm ahead
        self.lookahead_y_max = 0.30  # Look 30cm ahead
        
        self.get_logger().info("Robust Spatial Preprocessor Active.")

    def lookahead_cb(self, msg):
        self.lookahead_y_max = msg.data
        self.get_logger().info(f"Received new Lookahead Max: {self.lookahead_y_max:.2f}m")

    def pc_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        aligned_points = []
        
        floor_z_max = -np.inf
        ceil_z_min = np.inf
        left_wall_x = -np.inf   
        right_wall_x = np.inf   

        for p in points:
            # Map Width to X, Depth to Y, and Height to Z
            base_x = -float(p[2])         # Width -> Base X (Right)
            base_y = float(p[0]) + 0.13   # Depth -> Base Y (Forward) [+13cm camera offset]
            base_z = float(p[1]) + 0.06   # Height -> Base Z (Up) [+6cm camera offset]
            
            aligned_points.append([base_x, base_y, base_z])

            # CHECK THE LOOKAHEAD BOX
            if self.lookahead_y_min < base_y < self.lookahead_y_max:
                
                # Ceiling/Floor Detection (Sample a 30cm wide strip in the center)
                if -0.15 < base_x < 0.15:
                    # FIX: Since base_link is 0.0, the floor is ALWAYS negative, ceiling is ALWAYS positive
                    if base_z < 0.0: 
                        if base_z > floor_z_max: floor_z_max = base_z
                    else: 
                        if base_z < ceil_z_min: ceil_z_min = base_z
                
                # Wall Detection (Sample a narrow band exactly at the robot's body height)
                # FIX: Lowered to -0.05 -> 0.05 so it doesn't accidentally scan the low ceilings!
                if -0.05 < base_z < 0.05:
                    if base_x < 0.0 and base_x > left_wall_x:
                        left_wall_x = base_x
                    if base_x > 0.0 and base_x < right_wall_x:
                        right_wall_x = base_x

        new_header = msg.header
        new_header.frame_id = 'base_link'
        if aligned_points:
            cloud_msg = pc2.create_cloud_xyz32(new_header, aligned_points)
            self.cloud_pub.publish(cloud_msg)

        # 3. CALCULATE DECOUPLED COMMANDS
        if floor_z_max == -np.inf: floor_z_max = -0.10 # Assume standard floor if unseen
        if ceil_z_min == np.inf: ceil_z_min = 1.0      # Assume open sky if unseen
        if left_wall_x == -np.inf: left_wall_x = -0.5
        if right_wall_x == np.inf: right_wall_x = 0.5

        # Calculate true geometric constraints
        tunnel_height = ceil_z_min - floor_z_max
        tunnel_width = right_wall_x - left_wall_x

        # --- HEIGHT LOGIC (CRAWL) ---
        # The robot needs about 13cm of total vertical space (body thickness + camera)
        target_height = tunnel_height - 0.15
        target_height = np.clip(target_height, self.min_height, self.max_height)

        # --- SPRAWL LOGIC (SQUEEZE) ---
        # The robot needs about 10cm of buffer between its leg span and the walls
        target_sprawl = tunnel_width - 0.10
        target_sprawl = np.clip(target_sprawl, self.min_sprawl, self.max_sprawl)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(target_height), float(target_sprawl)]
        self.cmd_pub.publish(cmd_msg)

        # --- 4. PUBLISH RVIZ VISUALIZATION MARKER ---
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'spatial_constraint'
        marker.id = 0
        
        # Keep as a CUBE, but we will flatten it into a line
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position: Center of the lookahead box on the Y axis
        marker.pose.position.x = 0.0
        marker.pose.position.y = (self.lookahead_y_min + self.lookahead_y_max) / 2.0
        
        # The floor is negative relative to base_link. 
        # Adding floor_z_max anchors the marker to the physical ground!
        marker.pose.position.z = float(floor_z_max + target_height)
        
        marker.pose.orientation.w = 1.0
        
        # Scale: 
        # X (Width) = Target Sprawl
        # Y (Depth) = 0.01 (1cm - very thin)
        # Z (Height) = 0.01 (1cm - very thin)
        marker.scale.x = float(target_sprawl)
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        
        # Color: Bright Green
        marker.color.a = 1.0 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()