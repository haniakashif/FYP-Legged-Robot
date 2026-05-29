import rclpy
import numpy as np
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray

from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.ndimage import distance_transform_edt, gaussian_filter1d


class APFPlannerNode(Node):
    K_ATTR       = 1.0
    K_REP        = 0.08
    D0           = 0.6
    GRAD_STEP    = 0.3
    MAX_ITER     = 2000
    CONVERGE_M   = 0.06
    HEIGHT_SIGMA = 2.5

    Z_MIN = 0.04
    Z_MAX = 0.11
    S_MIN = 0.25
    S_MAX = 0.50

    def __init__(self):
        super().__init__('apf_planner')

        np.seterr(invalid='ignore')

        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil  = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        self.pub_apf_path     = self.create_publisher(PointCloud2,       '/apf_path_pc',                 10)
        self.pub_spatial_cmds = self.create_publisher(Float64MultiArray, '/perception/spatial_commands',  10)
        self.pub_h_avail      = self.create_publisher(PointCloud2,       '/debug_h_avail_pc',             10)
        self.pub_w_avail      = self.create_publisher(PointCloud2,       '/debug_w_avail_pc',             10)
        self.pub_z_targets    = self.create_publisher(PointCloud2,       '/debug_z_targets_pc',           10)
        self.pub_safe_mask    = self.create_publisher(PointCloud2,       '/debug_safe_spaces_pc',         10)
        self.pub_apf_field    = self.create_publisher(PointCloud2,       '/debug_apf_field_pc',           10)

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 3.0)

        self.get_logger().info('APF Planner Active.')

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        idx = msg.layers.index(name)
        d = msg.data[idx]
        n_cols, n_rows = d.layout.dim[0].size, d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def get_span_from_z(self, z_arr: np.ndarray) -> np.ndarray:
        ratio = (z_arr - self.Z_MIN) / (self.Z_MAX - self.Z_MIN)
        return self.S_MAX - ratio * (self.S_MAX - self.S_MIN)

    def world_to_grid(self, wx: float, wy: float, info) -> tuple[int, int]:
        cx, cy = info.pose.position.x, info.pose.position.y
        half_x, half_y = info.length_x / 2.0, info.length_y / 2.0
        res = info.resolution
        col = int((cy + half_y - wy) / res - 0.5)
        row = int((cx + half_x - wx) / res - 0.5)
        return col, row

    def compute_clearances(self, floor, ceil, res):
        valid = ~np.isnan(floor) & ~np.isnan(ceil)
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        walls = valid & (clearance_raw < self.Z_MIN)
        d_xy = distance_transform_edt(~walls) * res
        h_avail = np.where(valid, clearance_raw, np.nan)
        w_avail = np.where(valid, d_xy, np.nan)
        return h_avail, w_avail, floor, valid

    def optimize_posture(self, h_avail, w_avail):
        z_candidates = np.linspace(self.Z_MIN, self.Z_MAX, 15)
        best_score = np.full_like(h_avail, -np.inf)
        best_z = np.full_like(h_avail, np.nan)

        for z in z_candidates:
            s = self.get_span_from_z(z)
            score = np.minimum(h_avail - z, w_avail - s)
            score = np.where((z > h_avail) | (s > w_avail), -np.inf, score)
            improved = score > best_score
            best_score = np.where(improved, score, best_score)
            best_z = np.where(improved, z, best_z)

        return best_z, best_score > -np.inf

    def compute_apf(self, safe_mask, w_avail, goal_col, goal_row):
        c_idx, r_idx = np.meshgrid(
            np.arange(safe_mask.shape[0]), np.arange(safe_mask.shape[1]), indexing='ij')
        U_attr = self.K_ATTR * ((c_idx - goal_col)**2 + (r_idx - goal_row)**2).astype(np.float32)
        w = np.where(safe_mask, w_avail, np.nan)
        in_range = safe_mask & (w < self.D0) & (w > 1e-6)
        U_rep = np.zeros_like(U_attr)
        U_rep[in_range] = self.K_REP * (1.0 / w[in_range] - 1.0 / self.D0) ** 2
        return np.where(safe_mask, U_attr + U_rep, np.inf)

    def gradient_descent_path(self, U, start_col, start_row, goal_col, goal_row, res):
        n_cols, n_rows = U.shape
        path = []
        finite_mask = np.isfinite(U)
        if not finite_mask.any():
            return path

        if not finite_mask[start_col, start_row]:
            safe_cols, safe_rows = np.where(finite_mask)
            dists = np.hypot(safe_cols - start_col, safe_rows - start_row)
            best = np.argmin(dists)
            start_col, start_row = int(safe_cols[best]), int(safe_rows[best])

        c, r = float(start_col), float(start_row)
        converge_cells = self.CONVERGE_M / res

        for _ in range(self.MAX_ITER):
            path.append((c, r))
            if np.hypot(c - goal_col, r - goal_row) < converge_cells:
                path.append((float(goal_col), float(goal_row)))
                break

            ci = int(np.clip(round(c), 1, n_cols - 2))
            ri = int(np.clip(round(r), 1, n_rows - 2))
            U_c = U[ci, ri]

            U_cp1 = U[ci+1, ri] if np.isfinite(U[ci+1, ri]) else U_c
            U_cm1 = U[ci-1, ri] if np.isfinite(U[ci-1, ri]) else U_c
            U_rp1 = U[ci, ri+1] if np.isfinite(U[ci, ri+1]) else U_c
            U_rm1 = U[ci, ri-1] if np.isfinite(U[ci, ri-1]) else U_c

            dU_dc = (U_cp1 - U_cm1) / 2.0
            dU_dr = (U_rp1 - U_rm1) / 2.0
            grad_mag = np.hypot(dU_dc, dU_dr)

            if not np.isfinite(grad_mag) or grad_mag < 1e-9:
                dc, dr = goal_col - c, goal_row - r
                norm = np.hypot(dc, dr) + 1e-9
                c += self.GRAD_STEP * dc / norm
                r += self.GRAD_STEP * dr / norm
            else:
                c -= self.GRAD_STEP * dU_dc / grad_mag
                r -= self.GRAD_STEP * dU_dr / grad_mag

            c = float(np.clip(c, 0, n_cols - 1))
            r = float(np.clip(r, 0, n_rows - 1))

        return path

    def sample_smooth_height(self, path, opt_z, floor):
        n_cols, n_rows = opt_z.shape
        raw_heights, floor_heights = [], []

        for (c, r) in path:
            ci = int(np.clip(round(c), 0, n_cols - 1))
            ri = int(np.clip(round(r), 0, n_rows - 1))
            z, f = opt_z[ci, ri], floor[ci, ri]
            if np.isnan(z) and raw_heights:
                z = raw_heights[-1]
            elif np.isnan(z):
                z = self.Z_MIN
            raw_heights.append(z)
            floor_heights.append(f if not np.isnan(f) else 0.0)

        heights_smooth = gaussian_filter1d(np.array(raw_heights, dtype=np.float32), sigma=self.HEIGHT_SIGMA)
        return np.clip(heights_smooth, self.Z_MIN, self.Z_MAX), np.array(floor_heights, dtype=np.float32)

    def publish_debug_cloud(self, publisher, header, info, values, base_z, mask, is_flat=True):
        if not np.any(mask):
            return
        res = info.resolution
        cx, cy = info.pose.position.x, info.pose.position.y
        half_x, half_y = info.length_x / 2.0, info.length_y / 2.0
        cols, rows = np.where(mask)
        world_y = cy + half_y - (cols + 0.5) * res
        world_x = cx + half_x - (rows + 0.5) * res
        intensities = values[cols, rows]
        world_z = np.zeros_like(world_x) if is_flat else base_z[cols, rows] + intensities / 2.0
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        publisher.publish(pc2.create_cloud(header, fields, np.column_stack((world_x, world_y, world_z, intensities))))

    def publish_apf_path(self, header, info, path, heights_smooth, floor_heights):
        if not path:
            return
        res = info.resolution
        cx, cy = info.pose.position.x, info.pose.position.y
        half_x, half_y = info.length_x / 2.0, info.length_y / 2.0
        pts = []
        for i, (c, r) in enumerate(path):
            wy = cy + half_y - (c + 0.5) * res
            wx = cx + half_x - (r + 0.5) * res
            pts.append([wx, wy, floor_heights[i] + heights_smooth[i] / 2.0, heights_smooth[i]])

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub_apf_path.publish(pc2.create_cloud(header, fields, np.array(pts, dtype=np.float32)))

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            return

        res   = floor_msg.info.resolution
        floor = self.get_layer(floor_msg, 'elevation')
        ceil  = self.get_layer(ceil_msg,  'elevation')

        h_avail, w_avail, f_base, valid = self.compute_clearances(floor, ceil, res)
        opt_z, safe_mask = self.optimize_posture(h_avail, w_avail)

        if np.sum(safe_mask) < 5:
            return

        goal_col, goal_row = self.world_to_grid(
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            floor_msg.info)
        goal_col = int(np.clip(goal_col, 0, safe_mask.shape[0] - 1))
        goal_row = int(np.clip(goal_row, 0, safe_mask.shape[1] - 1))

        start_col = safe_mask.shape[0] // 2
        start_row = safe_mask.shape[1] // 2

        U = self.compute_apf(safe_mask, w_avail, goal_col, goal_row)
        path = self.gradient_descent_path(U, start_col, start_row, goal_col, goal_row, res)

        if not path:
            return

        heights_smooth, floor_heights = self.sample_smooth_height(path, opt_z, floor)

        # Publish height + sprawl at the robot's immediate position to the RL pipeline
        target_height = float(heights_smooth[0])
        target_sprawl = float(self.get_span_from_z(np.array(target_height)))

        cmd = Float64MultiArray()
        cmd.data = [target_height, target_sprawl]
        self.pub_spatial_cmds.publish(cmd)

        self.get_logger().info(
            f'APF -> height: {target_height:.3f}  sprawl: {target_sprawl:.3f}  path: {len(path)} pts')

        self.publish_apf_path(floor_msg.header, floor_msg.info, path, heights_smooth, floor_heights)

        # Debug clouds
        self.publish_debug_cloud(self.pub_h_avail,   floor_msg.header, floor_msg.info, h_avail,                  f_base,    valid,     is_flat=True)
        self.publish_debug_cloud(self.pub_w_avail,   floor_msg.header, floor_msg.info, w_avail,                  f_base,    valid,     is_flat=True)
        self.publish_debug_cloud(self.pub_z_targets, floor_msg.header, floor_msg.info, opt_z,                    f_base,    safe_mask, is_flat=False)
        self.publish_debug_cloud(self.pub_safe_mask, floor_msg.header, floor_msg.info, safe_mask.astype(np.float32), f_base, valid,     is_flat=True)
        U_vis = np.where(safe_mask, np.clip(U, 0, np.percentile(U[safe_mask], 95) if safe_mask.any() else 1), np.nan)
        self.publish_debug_cloud(self.pub_apf_field, floor_msg.header, floor_msg.info, U_vis, np.zeros_like(U_vis), safe_mask & ~np.isnan(U_vis), is_flat=True)


def main(args=None):
    rclpy.init(args=args)
    node = APFPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
