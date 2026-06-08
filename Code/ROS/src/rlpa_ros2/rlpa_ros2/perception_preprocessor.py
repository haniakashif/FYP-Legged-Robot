import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Float64MultiArray, Float64
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation as R

class PerceptionPreprocessor(Node):
    def __init__(self):
        super().__init__('perception_preprocessor')
        
        self.pc_sub = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.pc_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self.imu_callback, 10)
        
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
        
        # --- STATE & SMOOTHING ---
        self.current_rot = R.identity()  
        
        # Exponential Moving Average (EMA) 
        self.ema_alpha = 0.15  
        self.smoothed_height = self.max_height
        self.smoothed_sprawl = self.max_sprawl

        self.get_logger().info("Gravity-Aligned Robust Spatial Preprocessor Active.")

    def lookahead_cb(self, msg):
        self.lookahead_y_max = msg.data
        self.get_logger().info(f"Received new Lookahead Max: {self.lookahead_y_max:.2f}m")

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        # If the IMU gives us a valid quaternion, use it
        if not all(abs(v) <= 1e-6 for v in q):
            try:
                self.current_rot = R.from_quat(q)
                return
            except ValueError:
                pass
                
        # --- THE FIX: CALCULATE GRAVITY FROM ACCELERATION ---
        # If quaternion is [0,0,0,0], use the linear acceleration vector
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Avoid math errors if in freefall or zeroed out
        if abs(ax) < 0.1 and abs(ay) < 0.1 and abs(az) < 0.1:
            return
            
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        
        # Create a rotation that ONLY un-pitches and un-rolls the robot (keeps Yaw at 0)
        self.current_rot = R.from_euler('xyz', [roll, pitch, 0.0])

    def pc_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_pts = []

        for p in points:
            # Map Width to X, Depth to Y, and Height to Z 
            base_x = -float(p[2])
            base_y = float(p[0]) + 0.13
            base_z = float(p[1]) + 0.06
            raw_pts.append([base_x, base_y, base_z])

        if not raw_pts:
            return

        pts_array = np.array(raw_pts)

        # --- 1. GRAVITY ALIGNMENT ---
        pts_world = self.current_rot.apply(pts_array)
        yaw = self.current_rot.as_euler('xyz')[2]
        r_yaw_inv = R.from_euler('z', -yaw)
        pts_aligned = r_yaw_inv.apply(pts_world)

        new_header = msg.header
        new_header.frame_id = 'base_link'
        cloud_msg = pc2.create_cloud_xyz32(new_header, pts_aligned.tolist())
        self.cloud_pub.publish(cloud_msg)

        # --- 2. PERCENTILE-BASED OBSTACLE EXTRACTION ---
        y_mask = (pts_aligned[:, 1] > self.lookahead_y_min) & (pts_aligned[:, 1] < self.lookahead_y_max)
        box_pts = pts_aligned[y_mask]

        if len(box_pts) > 0:
            center_mask = (box_pts[:, 0] > -0.15) & (box_pts[:, 0] < 0.15)
            center_pts = box_pts[center_mask]
            
            floor_zs = center_pts[center_pts[:, 2] < 0.0][:, 2] if len(center_pts) > 0 else []
            ceil_zs = center_pts[center_pts[:, 2] > 0.0][:, 2] if len(center_pts) > 0 else []

            height_mask = (box_pts[:, 2] > 0.0) & (box_pts[:, 2] < 0.20)
            wall_pts = box_pts[height_mask]
            
            left_xs = wall_pts[wall_pts[:, 0] < 0.0][:, 0] if len(wall_pts) > 0 else []
            right_xs = wall_pts[wall_pts[:, 0] > 0.0][:, 0] if len(wall_pts) > 0 else []

            floor_z_max = np.percentile(floor_zs, 95) if len(floor_zs) > 10 else -0.10
            ceil_z_min  = np.percentile(ceil_zs, 5) if len(ceil_zs) > 10 else 1.0
            left_wall_x = np.percentile(left_xs, 95) if len(left_xs) > 10 else -0.5
            right_wall_x = np.percentile(right_xs, 5) if len(right_xs) > 10 else 0.5
        else:
            # FIX: If the box is empty, assume open space! Do not return early.
            floor_z_max = -0.10
            ceil_z_min = 1.0
            left_wall_x = -0.5
            right_wall_x = 0.5

        # --- 3. CALCULATE RAW COMMANDS ---
        tunnel_height = ceil_z_min - floor_z_max
        tunnel_width = right_wall_x - left_wall_x

        target_height = np.clip(tunnel_height - 0.15, self.min_height, self.max_height)
        target_sprawl = np.clip(tunnel_width - 0.10, self.min_sprawl, self.max_sprawl)

        # --- 4. EXPONENTIAL MOVING AVERAGE (EMA) ---
        self.smoothed_height = (self.ema_alpha * target_height) + ((1 - self.ema_alpha) * self.smoothed_height)
        self.smoothed_sprawl = (self.ema_alpha * target_sprawl) + ((1 - self.ema_alpha) * self.smoothed_sprawl)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(self.smoothed_height), float(self.smoothed_sprawl)]
        self.cmd_pub.publish(cmd_msg)

        # --- 5. PUBLISH MARKER ---
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'spatial_constraint'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = (self.lookahead_y_min + self.lookahead_y_max) / 2.0
        marker.pose.position.z = float(floor_z_max + self.smoothed_height)
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = float(self.smoothed_sprawl)
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        
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