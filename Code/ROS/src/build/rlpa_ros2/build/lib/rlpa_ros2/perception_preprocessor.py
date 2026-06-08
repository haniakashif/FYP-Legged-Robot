import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, Float64
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.ndimage import distance_transform_edt

GRID_RES = 0.05  # metres per cell for the debug clearance grid


class PerceptionPreprocessor(Node):
    def __init__(self):
        super().__init__('perception_preprocessor')

        self.pc_sub = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.pc_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/aligned_cave_cloud', 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/perception/spatial_commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/perception/spatial_marker', 10)
        self.lh_sub = self.create_subscription(Float64, '/perception/lookahead_max', self.lookahead_cb, 10)

        # Debug: per-cell clearance analysis (mirrors map2apf3 Phases 2-3)
        self.pub_h_avail   = self.create_publisher(PointCloud2, '/debug_h_avail_pc',     10)
        self.pub_w_avail   = self.create_publisher(PointCloud2, '/debug_w_avail_pc',     10)
        self.pub_z_targets = self.create_publisher(PointCloud2, '/debug_z_targets_pc',   10)
        self.pub_safe_mask = self.create_publisher(PointCloud2, '/debug_safe_spaces_pc', 10)

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

    # =========================================================================
    #  DEBUG: Grid-based clearance analysis + PointCloud2 publishing
    # =========================================================================

    def _publish_xyz_intensity(self, publisher, header, xyz, intensities):
        if len(xyz) == 0:
            return
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pts = np.column_stack((xyz, intensities)).astype(np.float32)
        publisher.publish(pc2.create_cloud(header, fields, pts))

    def _compute_grid_debug(self, pts, header):
        """
        Bin the aligned point cloud into a 2D (lateral × forward) grid,
        compute per-cell floor/ceiling heights, vertical clearance h_avail,
        EDT lateral clearance w_avail, and optimal body height opt_z via a
        posture sweep. Publishes four debug PointCloud2 topics.

        Points are in base_link frame:
          pts[:,0] = base_x (lateral, +right)
          pts[:,1] = base_y (forward, +ahead)
          pts[:,2] = base_z (vertical, 0 = base_link CoM, + = above)
        """
        bx, by, bz = pts[:, 0], pts[:, 1], pts[:, 2]

        # Restrict to lookahead box
        in_box = (by > self.lookahead_y_min) & (by < self.lookahead_y_max)
        if in_box.sum() < 10:
            return
        bx, by, bz = bx[in_box], by[in_box], bz[in_box]

        # --- Build 2D grid ---
        x_min, x_max = float(bx.min()), float(bx.max())
        y_min, y_max = self.lookahead_y_min, self.lookahead_y_max

        n_cols = max(1, int(np.ceil((x_max - x_min) / GRID_RES)))
        n_rows = max(1, int(np.ceil((y_max - y_min) / GRID_RES)))

        col_idx = np.clip(((bx - x_min) / GRID_RES).astype(int), 0, n_cols - 1)
        row_idx = np.clip(((by - y_min) / GRID_RES).astype(int), 0, n_rows - 1)
        flat_idx = col_idx * n_rows + row_idx

        # Per-cell floor (highest z ≤ 0) and ceiling (lowest z > 0)
        floor_z = np.full(n_cols * n_rows, np.nan)
        ceil_z  = np.full(n_cols * n_rows, np.nan)
        for fi in np.unique(flat_idx):
            zs  = bz[flat_idx == fi]
            neg = zs[zs <= 0.0]
            pos = zs[zs > 0.0]
            if len(neg):
                floor_z[fi] = neg.max()
            if len(pos):
                ceil_z[fi]  = pos.min()
        floor_z = floor_z.reshape(n_cols, n_rows)
        ceil_z  = ceil_z.reshape(n_cols, n_rows)

        # --- Phase 2: Clearance Computation ---
        valid   = ~np.isnan(floor_z) & ~np.isnan(ceil_z)
        h_avail = np.where(valid, ceil_z - floor_z, np.nan)

        # w_avail: EDT distance (m) from nearest too-tight cell
        walls   = valid & (h_avail < self.min_height)
        w_avail = np.where(valid, distance_transform_edt(~walls) * GRID_RES, np.nan)

        # --- Phase 3: Posture Sweep → opt_z per cell ---
        z_candidates = np.linspace(self.min_height, self.max_height, 12)
        best_score   = np.full((n_cols, n_rows), -np.inf)
        best_z       = np.full((n_cols, n_rows), np.nan)

        for z in z_candidates:
            ratio = (z - self.min_height) / max(self.max_height - self.min_height, 1e-6)
            s     = self.max_sprawl - ratio * (self.max_sprawl - self.min_sprawl)
            score = np.minimum(h_avail - z, w_avail - s)
            score = np.where((z > h_avail) | (s > w_avail), -np.inf, score)
            improved   = score > best_score
            best_score = np.where(improved, score, best_score)
            best_z     = np.where(improved, z,     best_z)

        safe_mask = best_score > -np.inf

        # Grid cell centres in base_link frame (for publishing)
        ci_arr, ri_arr = np.meshgrid(np.arange(n_cols), np.arange(n_rows), indexing='ij')
        wx = (x_min + (ci_arr + 0.5) * GRID_RES).astype(np.float32)
        wy = (y_min + (ri_arr + 0.5) * GRID_RES).astype(np.float32)

        def extract(mask2d, intensity_arr, z_vals=None):
            m = mask2d & np.isfinite(intensity_arr)
            if not np.any(m):
                return np.zeros((0, 3)), np.zeros(0)
            wz = z_vals[m] if z_vals is not None else np.zeros(m.sum(), dtype=np.float32)
            return np.column_stack((wx[m], wy[m], wz)), intensity_arr[m]

        # /debug_h_avail_pc — vertical clearance (m), flat at z=0
        xyz, inten = extract(valid, h_avail)
        self._publish_xyz_intensity(self.pub_h_avail, header, xyz, inten)

        # /debug_w_avail_pc — lateral EDT clearance (m), flat at z=0
        xyz, inten = extract(valid, w_avail)
        self._publish_xyz_intensity(self.pub_w_avail, header, xyz, inten)

        # /debug_z_targets_pc — optimal body height; z placed at floor + best_z/2
        z_3d = np.where(safe_mask, floor_z + best_z / 2.0, np.nan)
        xyz, inten = extract(safe_mask, best_z, z_vals=z_3d)
        self._publish_xyz_intensity(self.pub_z_targets, header, xyz, inten)

        # /debug_safe_spaces_pc — 1.0 = navigable, 0.0 = too tight, flat at z=0
        safe_float = np.where(valid, safe_mask.astype(np.float32), np.nan)
        xyz, inten = extract(valid, safe_float)
        self._publish_xyz_intensity(self.pub_safe_mask, header, xyz, inten)

    # =========================================================================
    #  MAIN CALLBACK
    # =========================================================================

    def pc_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        aligned_points = []

        floor_z_max  = -np.inf
        ceil_z_min   =  np.inf
        left_wall_x  = -np.inf
        right_wall_x =  np.inf

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
                    if base_z < 0.0:
                        if base_z > floor_z_max: floor_z_max = base_z
                    else:
                        if base_z < ceil_z_min:  ceil_z_min  = base_z

                # Wall Detection (narrow band at body height)
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

            self._compute_grid_debug(np.array(aligned_points, dtype=np.float32), new_header)

        if floor_z_max == -np.inf: floor_z_max = -0.10
        if ceil_z_min  ==  np.inf: ceil_z_min  =  1.0
        if left_wall_x == -np.inf: left_wall_x  = -0.5
        if right_wall_x ==  np.inf: right_wall_x =  0.5

        # Calculate true geometric constraints
        tunnel_height = ceil_z_min   - floor_z_max
        tunnel_width  = right_wall_x - left_wall_x

        # --- HEIGHT LOGIC (CRAWL) ---
        target_height = np.clip(tunnel_height - 0.15, self.min_height, self.max_height)

        # --- SPRAWL LOGIC (SQUEEZE) ---
        target_sprawl = np.clip(tunnel_width  - 0.10, self.min_sprawl, self.max_sprawl)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(target_height), float(target_sprawl)]
        self.cmd_pub.publish(cmd_msg)

        # --- RVIZ VISUALIZATION MARKER ---
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'spatial_constraint'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = (self.lookahead_y_min + self.lookahead_y_max) / 2.0
        marker.pose.position.z = float(floor_z_max + target_height)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(target_sprawl)
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
