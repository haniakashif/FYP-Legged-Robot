import rclpy
import numpy as np
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.ndimage import distance_transform_edt, gaussian_filter1d


class PostureOptimizerDebug(Node):
    # ------------------------------------------------------------------ #
    #  APF tuning constants                                               #
    # ------------------------------------------------------------------ #
    K_ATTR      = 1.0
    K_REP       = 0.08
    D0          = 0.6
    GRAD_STEP   = 0.3
    MAX_ITER    = 2000
    CONVERGE_M  = 0.06
    HEIGHT_SIGMA = 2.5

    def __init__(self):
        super().__init__('posture_optimizer_debug')

        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil  = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        self.pub_h_avail   = self.create_publisher(PointCloud2, '/debug_h_avail_pc',     10)
        self.pub_w_avail   = self.create_publisher(PointCloud2, '/debug_w_avail_pc',     10)
        self.pub_z_targets = self.create_publisher(PointCloud2, '/debug_z_targets_pc',   10)
        self.pub_safe_mask = self.create_publisher(PointCloud2, '/debug_safe_spaces_pc', 10)
        self.pub_apf_field = self.create_publisher(PointCloud2, '/debug_apf_field_pc',   10)
        self.pub_apf_path  = self.create_publisher(PointCloud2, '/apf_path_pc',          10)

        self.z_min = 0.15
        self.z_max = 0.40
        self.s_min = 0.40
        self.s_max = 0.80
        self.l_rob = 0.50

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)

        self.get_logger().info(
            'PostureOptimizerDebug (APF) ready. '
            f'Goal: ({self.get_parameter("goal_x").value}, '
            f'{self.get_parameter("goal_y").value})'
        )

    # ================================================================== #

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        idx    = msg.layers.index(name)
        d      = msg.data[idx]
        n_cols = d.layout.dim[0].size
        n_rows = d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def get_span_from_z(self, z_arr: np.ndarray) -> np.ndarray:
        ratio = (z_arr - self.z_min) / (self.z_max - self.z_min)
        return self.s_max - ratio * (self.s_max - self.s_min)

    def compute_clearances(self, floor: np.ndarray, ceil: np.ndarray, res: float):
        valid         = ~np.isnan(floor) & ~np.isnan(ceil)
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        walls         = valid & (clearance_raw < self.z_min)
        d_xy          = distance_transform_edt(~walls) * res
        h_avail       = np.where(valid, clearance_raw, np.nan)
        w_avail       = np.where(valid, d_xy,          np.nan)
        return h_avail, w_avail, floor, valid

    def optimize_posture(self, h_avail: np.ndarray, w_avail: np.ndarray):
        z_candidates = np.linspace(self.z_min, self.z_max, 15)
        best_score   = np.full_like(h_avail, -np.inf)
        best_z       = np.full_like(h_avail,  np.nan)

        for z in z_candidates:
            s        = self.get_span_from_z(z)
            margin_z = h_avail - z
            margin_w = w_avail - s
            score    = np.minimum(margin_z, margin_w)
            score    = np.where((z > h_avail) | (s > w_avail), -np.inf, score)
            improved = score > best_score
            best_score = np.where(improved, score, best_score)
            best_z     = np.where(improved, z,     best_z)

        is_safe = best_score > -np.inf
        return best_z, is_safe

    def world_to_grid(self, wx: float, wy: float, info) -> tuple[int, int]:
        cx     = info.pose.position.x
        cy     = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0
        res    = info.resolution
        col = int((cy + half_y - wy) / res - 0.5)
        row = int((cx + half_x - wx) / res - 0.5)
        return col, row

    def grid_to_world(self, col: float, row: float, info) -> tuple[float, float]:
        cx     = info.pose.position.x
        cy     = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0
        res    = info.resolution
        wy = cy + half_y - (col + 0.5) * res
        wx = cx + half_x - (row + 0.5) * res
        return wx, wy

    def compute_apf(self, safe_mask, w_avail, goal_col, goal_row) -> np.ndarray:
        c_idx, r_idx = np.meshgrid(
            np.arange(safe_mask.shape[0]),
            np.arange(safe_mask.shape[1]),
            indexing='ij'
        )
        U_attr = self.K_ATTR * ((c_idx - goal_col)**2 + (r_idx - goal_row)**2).astype(np.float32)

        w = np.where(safe_mask, w_avail, np.nan)
        in_range = safe_mask & (w < self.D0) & (w > 1e-6)
        U_rep = np.zeros_like(U_attr)
        U_rep[in_range] = self.K_REP * (1.0 / w[in_range] - 1.0 / self.D0) ** 2

        U = U_attr + U_rep
        return np.where(safe_mask, U, np.inf)

    def gradient_descent_path(self, U, start_col, start_row, goal_col, goal_row, res):
        n_cols, n_rows = U.shape
        path = []

        finite_mask = np.isfinite(U)
        if not finite_mask.any():
            self.get_logger().warn('APF: no finite cells in U — skipping path')
            return path
        if not finite_mask[start_col, start_row]:
            safe_cols, safe_rows = np.where(finite_mask)
            dists = np.hypot(safe_cols - start_col, safe_rows - start_row)
            best  = np.argmin(dists)
            start_col = int(safe_cols[best])
            start_row = int(safe_rows[best])

        c = float(start_col)
        r = float(start_row)
        converge_cells = self.CONVERGE_M / res

        for _ in range(self.MAX_ITER):
            path.append((c, r))
            if np.hypot(c - goal_col, r - goal_row) < converge_cells:
                path.append((float(goal_col), float(goal_row)))
                break

            ci = int(np.clip(round(c), 1, n_cols - 2))
            ri = int(np.clip(round(r), 1, n_rows - 2))

            U_centre = U[ci, ri]
            U_cp1 = U[ci + 1, ri] if np.isfinite(U[ci + 1, ri]) else U_centre
            U_cm1 = U[ci - 1, ri] if np.isfinite(U[ci - 1, ri]) else U_centre
            U_rp1 = U[ci, ri + 1] if np.isfinite(U[ci, ri + 1]) else U_centre
            U_rm1 = U[ci, ri - 1] if np.isfinite(U[ci, ri - 1]) else U_centre

            dU_dc = (U_cp1 - U_cm1) / 2.0
            dU_dr = (U_rp1 - U_rm1) / 2.0
            grad_mag = np.hypot(dU_dc, dU_dr)

            if not np.isfinite(grad_mag) or grad_mag < 1e-9:
                dc = goal_col - c
                dr = goal_row - r
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
        raw_heights   = []
        floor_heights = []

        for (c, r) in path:
            ci = int(np.clip(round(c), 0, n_cols - 1))
            ri = int(np.clip(round(r), 0, n_rows - 1))
            z = opt_z[ci, ri]
            f = floor[ci, ri]
            if np.isnan(z) and raw_heights:
                z = raw_heights[-1]
            elif np.isnan(z):
                z = self.z_min
            raw_heights.append(z)
            floor_heights.append(f if not np.isnan(f) else 0.0)

        raw_heights   = np.array(raw_heights,   dtype=np.float32)
        floor_heights = np.array(floor_heights, dtype=np.float32)
        heights_smooth = gaussian_filter1d(raw_heights, sigma=self.HEIGHT_SIGMA)
        heights_smooth = np.clip(heights_smooth, self.z_min, self.z_max)
        return heights_smooth, floor_heights

    def publish_pointcloud(self, publisher, header, info, values, base_z, mask, is_flat=True):
        if not np.any(mask):
            return
        res    = info.resolution
        cx     = info.pose.position.x
        cy     = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        cols, rows  = np.where(mask)
        world_y     = cy + half_y - (cols + 0.5) * res
        world_x     = cx + half_x - (rows + 0.5) * res
        intensities = values[cols, rows]
        world_z     = np.zeros_like(world_x) if is_flat else base_z[cols, rows] + (intensities / 2.0)

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = pc2.create_cloud(header, fields,
                                     np.column_stack((world_x, world_y, world_z, intensities)))
        publisher.publish(cloud_msg)

    def publish_apf_field(self, header, info, U, safe_mask):
        U_vis     = np.where(safe_mask, np.clip(U, 0, np.percentile(U[safe_mask], 95)), np.nan)
        valid_vis = safe_mask & ~np.isnan(U_vis)
        self.publish_pointcloud(
            self.pub_apf_field, header, info, U_vis,
            np.zeros_like(U_vis), valid_vis, is_flat=True)

    def publish_apf_path(self, header, info, path, heights_smooth, floor_heights):
        if not path:
            return
        res    = info.resolution
        cx     = info.pose.position.x
        cy     = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        pts = []
        for i, (c, r) in enumerate(path):
            wy = cy + half_y - (c + 0.5) * res
            wx = cx + half_x - (r + 0.5) * res
            h  = heights_smooth[i]
            f  = floor_heights[i]
            pts.append((wx, wy, f + h / 2.0, h))

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub_apf_path.publish(
            pc2.create_cloud(header, fields, np.array(pts, dtype=np.float32)))

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            return

        res   = floor_msg.info.resolution
        floor = self.get_layer(floor_msg, 'elevation')
        ceil  = self.get_layer(ceil_msg,  'elevation')

        h_avail, w_avail, f_base, valid = self.compute_clearances(floor, ceil, res)
        opt_z, safe_mask = self.optimize_posture(h_avail, w_avail)

        self.get_logger().info(f'Safe Cells: {np.sum(safe_mask)} / {np.sum(valid)}')

        if np.sum(safe_mask) < 5:
            self.get_logger().warn('APF: fewer than 5 safe cells — skipping path planning')
            return

        goal_wx  = self.get_parameter('goal_x').value
        goal_wy  = self.get_parameter('goal_y').value
        goal_col, goal_row = self.world_to_grid(goal_wx, goal_wy, floor_msg.info)
        goal_col = int(np.clip(goal_col, 0, safe_mask.shape[0] - 1))
        goal_row = int(np.clip(goal_row, 0, safe_mask.shape[1] - 1))

        start_col = safe_mask.shape[0] // 2
        start_row = safe_mask.shape[1] // 2

        U    = self.compute_apf(safe_mask, w_avail, goal_col, goal_row)
        path = self.gradient_descent_path(U, start_col, start_row, goal_col, goal_row, res)

        self.get_logger().info(f'APF path: {len(path)} waypoints')

        heights_smooth, floor_heights = self.sample_smooth_height(path, opt_z, floor)

        self.publish_pointcloud(self.pub_h_avail,   floor_msg.header, floor_msg.info, h_avail,               f_base, valid,     is_flat=True)
        self.publish_pointcloud(self.pub_w_avail,   floor_msg.header, floor_msg.info, w_avail,               f_base, valid,     is_flat=True)
        self.publish_pointcloud(self.pub_z_targets, floor_msg.header, floor_msg.info, opt_z,                 f_base, safe_mask, is_flat=False)
        self.publish_pointcloud(self.pub_safe_mask, floor_msg.header, floor_msg.info, safe_mask.astype(np.float32), f_base, valid, is_flat=True)
        self.publish_apf_field(floor_msg.header, floor_msg.info, U, safe_mask)
        self.publish_apf_path(floor_msg.header, floor_msg.info, path, heights_smooth, floor_heights)


def main(args=None):
    rclpy.init(args=args)
    node = PostureOptimizerDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
