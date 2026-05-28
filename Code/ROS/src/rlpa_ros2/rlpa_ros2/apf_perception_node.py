#!/usr/bin/env python3
"""
APF Perception Node — Advanced perception pipeline based on map2apf.py.

Pipeline:
  PointCloud → ceiling_filter_node → elevation_mapping (floor + ceiling)
             → ceiling_flipper_node → map_flipper_node
             → HERE: GridMap sync → EDT → Posture Sweep → APF → [height, sprawl]

Subscribes to:
  /elevation_map                    (floor GridMap from elevation_mapping)
  /ceiling_elevation_map_flipped    (ceiling GridMap, negated by map_flipper_node)

Publishes:
  /perception/spatial_commands      Float64MultiArray([height, sprawl])
                                    — same format as naive PerceptionPreprocessor

Debug topics (same as original map2apf):
  /debug_h_avail_pc, /debug_w_avail_pc, /debug_z_targets_pc,
  /debug_safe_spaces_pc, /debug_apf_field_pc, /apf_path_pc
"""

import rclpy
import numpy as np
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray
import sensor_msgs_py.point_cloud2 as pc2

from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.ndimage import distance_transform_edt, gaussian_filter1d


class ApfPerceptionNode(Node):
    """
    Faithfully based on map2apf.py (PostureOptimizerDebug), adapted for:
      - THex robot kinematic limits (not the archive test-world values)
      - Goal auto-set to forward edge of the 20cm look-ahead map
      - Command output on /perception/spatial_commands [height, sprawl]
    """

    # ── APF tuning (from map2apf.py) ─────────────────────────────────────────
    K_ATTR      = 1.0
    K_REP       = 0.08
    D0          = 0.20   # influence radius (m) — scaled to 20cm map
    GRAD_STEP   = 0.3
    MAX_ITER    = 200    # reduced: 20cm map needs far fewer steps than 4m map
    CONVERGE_M  = 0.03
    HEIGHT_SIGMA = 1.5   # smaller σ — fewer waypoints on 20cm map

    # ── THex robot kinematics (from existing PerceptionPreprocessor) ──────────
    Z_MIN = 0.06;  Z_MAX = 0.12
    S_MIN = 0.25;  S_MAX = 0.50

    def __init__(self):
        super().__init__('apf_perception_node')

        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil  = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        # Command output — same topic/format as naive PerceptionPreprocessor
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/perception/spatial_commands', 10)

        # Debug publishers (identical to map2apf.py)
        self.pub_h_avail   = self.create_publisher(PointCloud2, '/debug_h_avail_pc',     10)
        self.pub_w_avail   = self.create_publisher(PointCloud2, '/debug_w_avail_pc',     10)
        self.pub_z_targets = self.create_publisher(PointCloud2, '/debug_z_targets_pc',   10)
        self.pub_safe_mask = self.create_publisher(PointCloud2, '/debug_safe_spaces_pc', 10)
        self.pub_apf_field = self.create_publisher(PointCloud2, '/debug_apf_field_pc',   10)
        self.pub_apf_path  = self.create_publisher(PointCloud2, '/apf_path_pc',          10)

        # Fallback in case no safe path found
        self._last_z = self.Z_MAX
        self._last_s = self.S_MIN

        self.get_logger().info('APF Perception Node ready (map2apf pipeline, 20cm look-ahead)')

    # ── Unchanged from map2apf.py ─────────────────────────────────────────────

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        idx    = msg.layers.index(name)
        d      = msg.data[idx]
        n_cols = d.layout.dim[0].size
        n_rows = d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def get_span_from_z(self, z_arr: np.ndarray) -> np.ndarray:
        ratio = (z_arr - self.Z_MIN) / (self.Z_MAX - self.Z_MIN)
        return self.S_MAX - ratio * (self.S_MAX - self.S_MIN)

    def compute_clearances(self, floor: np.ndarray, ceil: np.ndarray, res: float):
        valid         = ~np.isnan(floor) & ~np.isnan(ceil)
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        walls         = valid & (clearance_raw < self.Z_MIN)
        d_xy          = distance_transform_edt(~walls) * res
        h_avail       = np.where(valid, clearance_raw, np.nan)
        w_avail       = np.where(valid, d_xy,          np.nan)
        return h_avail, w_avail, floor, valid

    def optimize_posture(self, h_avail: np.ndarray, w_avail: np.ndarray):
        z_candidates = np.linspace(self.Z_MIN, self.Z_MAX, 15)
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
        cx, cy   = info.pose.position.x, info.pose.position.y
        half_x   = info.length_x / 2.0
        half_y   = info.length_y / 2.0
        res      = info.resolution
        col = int((cy + half_y - wy) / res - 0.5)
        row = int((cx + half_x - wx) / res - 0.5)
        return col, row

    def grid_to_world(self, col: float, row: float, info) -> tuple[float, float]:
        cx, cy   = info.pose.position.x, info.pose.position.y
        half_x   = info.length_x / 2.0
        half_y   = info.length_y / 2.0
        res      = info.resolution
        wy = cy + half_y - (col + 0.5) * res
        wx = cx + half_x - (row + 0.5) * res
        return wx, wy

    def compute_apf(self, safe_mask: np.ndarray, w_avail: np.ndarray,
                    goal_col: int, goal_row: int) -> np.ndarray:
        c_idx, r_idx = np.meshgrid(
            np.arange(safe_mask.shape[0]),
            np.arange(safe_mask.shape[1]),
            indexing='ij'
        )
        U_attr = self.K_ATTR * ((c_idx - goal_col)**2 + (r_idx - goal_row)**2).astype(np.float32)

        w        = np.where(safe_mask, w_avail, np.nan)
        in_range = safe_mask & (w < self.D0) & (w > 1e-6)
        U_rep    = np.zeros_like(U_attr)
        U_rep[in_range] = self.K_REP * (1.0 / w[in_range] - 1.0 / self.D0) ** 2

        return np.where(safe_mask, U_attr + U_rep, np.inf)

    def gradient_descent_path(self, U: np.ndarray, start_col: int, start_row: int,
                               goal_col: int, goal_row: int, res: float) -> list[tuple[float, float]]:
        n_cols, n_rows = U.shape
        path = []

        finite_mask = np.isfinite(U)
        if not finite_mask.any():
            return path
        if not finite_mask[start_col, start_row]:
            safe_cols, safe_rows = np.where(finite_mask)
            dists = np.hypot(safe_cols - start_col, safe_rows - start_row)
            best  = np.argmin(dists)
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
            Uc = U[ci, ri]

            U_cp1 = U[ci+1, ri] if np.isfinite(U[ci+1, ri]) else Uc
            U_cm1 = U[ci-1, ri] if np.isfinite(U[ci-1, ri]) else Uc
            U_rp1 = U[ci, ri+1] if np.isfinite(U[ci, ri+1]) else Uc
            U_rm1 = U[ci, ri-1] if np.isfinite(U[ci, ri-1]) else Uc

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

    def sample_smooth_height(self, path: list[tuple[float, float]],
                              opt_z: np.ndarray, floor: np.ndarray):
        n_cols, n_rows = opt_z.shape
        raw_heights, floor_heights = [], []

        for (c, r) in path:
            ci = int(np.clip(round(c), 0, n_cols - 1))
            ri = int(np.clip(round(r), 0, n_rows - 1))
            z = opt_z[ci, ri]
            f = floor[ci, ri]
            if np.isnan(z):
                z = raw_heights[-1] if raw_heights else self.Z_MIN
            raw_heights.append(z)
            floor_heights.append(f if not np.isnan(f) else 0.0)

        raw_heights   = np.array(raw_heights,   dtype=np.float32)
        floor_heights = np.array(floor_heights, dtype=np.float32)
        heights_smooth = np.clip(
            gaussian_filter1d(raw_heights, sigma=self.HEIGHT_SIGMA),
            self.Z_MIN, self.Z_MAX)
        return heights_smooth, floor_heights

    # ── Publishing (unchanged from map2apf.py) ────────────────────────────────

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
        publisher.publish(pc2.create_cloud(
            header, fields,
            np.column_stack((world_x, world_y, world_z, intensities)).astype(np.float32)))

    def publish_apf_field(self, header, info, U: np.ndarray, safe_mask: np.ndarray):
        U_vis = np.where(safe_mask, np.clip(U, 0, np.percentile(U[safe_mask], 95)), np.nan)
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
            wz = floor_heights[i] + h / 2.0
            pts.append([wx, wy, wz, h])

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub_apf_path.publish(pc2.create_cloud(
            header, fields, np.array(pts, dtype=np.float32)))

    # ── Main callback ─────────────────────────────────────────────────────────

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            return

        res   = floor_msg.info.resolution
        floor = self.get_layer(floor_msg, 'elevation')
        ceil  = self.get_layer(ceil_msg,  'elevation')

        h_avail, w_avail, f_base, valid = self.compute_clearances(floor, ceil, res)
        opt_z, safe_mask = self.optimize_posture(h_avail, w_avail)

        if safe_mask.sum() < 3:
            self.get_logger().warn('APF: fewer than 3 safe cells — holding last command')
            self._publish_command(self._last_z, self._last_s)
            return

        # Goal = forward edge of the 20cm map, center laterally.
        # The map is in camera_link frame: +X is depth (forward), center at (cx, cy).
        # Forward edge in world frame = (cx + length_x/2, cy).
        info = floor_msg.info
        goal_wx = info.pose.position.x + info.length_x / 2.0
        goal_wy = info.pose.position.y
        goal_col, goal_row = self.world_to_grid(goal_wx, goal_wy, info)
        goal_col = int(np.clip(goal_col, 0, safe_mask.shape[0] - 1))
        goal_row = int(np.clip(goal_row, 0, safe_mask.shape[1] - 1))

        # Start = map centre (camera/sensor origin)
        start_col = safe_mask.shape[0] // 2
        start_row = safe_mask.shape[1] // 2

        U    = self.compute_apf(safe_mask, w_avail, goal_col, goal_row)
        path = self.gradient_descent_path(U, start_col, start_row, goal_col, goal_row, res)

        if not path:
            self._publish_command(self._last_z, self._last_s)
            return

        heights_smooth, floor_heights = self.sample_smooth_height(path, opt_z, floor)

        # Map APF scalar output → [height, sprawl]:
        # Take the first step along the path (index 1 avoids the start=robot position)
        # and read the smoothed height there. Derive sprawl via the kinematic coupling.
        step_idx = min(1, len(heights_smooth) - 1)
        z_cmd = float(heights_smooth[step_idx])
        s_cmd = float(self.get_span_from_z(np.array([z_cmd]))[0])

        self._publish_command(z_cmd, s_cmd)
        self._last_z = z_cmd
        self._last_s = s_cmd

        # Debug visualisation
        self.publish_pointcloud(self.pub_h_avail,   floor_msg.header, info, h_avail,              f_base, valid,     is_flat=True)
        self.publish_pointcloud(self.pub_w_avail,   floor_msg.header, info, w_avail,              f_base, valid,     is_flat=True)
        self.publish_pointcloud(self.pub_z_targets, floor_msg.header, info, opt_z,                f_base, safe_mask, is_flat=False)
        self.publish_pointcloud(self.pub_safe_mask, floor_msg.header, info, safe_mask.astype(np.float32), f_base, valid, is_flat=True)
        self.publish_apf_field(floor_msg.header, info, U, safe_mask)
        self.publish_apf_path(floor_msg.header, info, path, heights_smooth, floor_heights)

        self.get_logger().debug(
            f'APF cmd: height={z_cmd:.3f}m sprawl={s_cmd:.3f}m '
            f'path_len={len(path)} safe={safe_mask.sum()}')

    def _publish_command(self, z: float, s: float):
        msg = Float64MultiArray()
        msg.data = [float(z), float(s)]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ApfPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
