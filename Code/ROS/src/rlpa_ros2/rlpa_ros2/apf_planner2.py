#!/usr/bin/env python3
"""
Enhanced APF Planner with Debug Publishers
==========================================
Publishes comprehensive debug topics for RViz visualization:
  - /apf_field_potential         : Full potential field as heatmap
  - /apf_gradient_field          : Gradient magnitude visualization
  - /apf_path_smoothed           : Final planned path (3D with body height)
  - /apf_repulsion_field         : Just the repulsive component
  - /apf_attraction_field        : Just the attractive component
  - /debug_h_avail               : Height availability heatmap
  - /debug_w_avail               : Width availability heatmap
  - /debug_safe_cells            : Binary safe/unsafe mask
  - /debug_obstacles_xy          : Obstacle locations (red markers)
  - /debug_goal_marker           : Goal position (blue sphere)
  - /debug_start_marker          : Start position (green sphere)
  
Parameters (configurable via ros2 param set):
  - k_attractive, k_repulsive, influence_radius
  - goal_x, goal_y
  - publish_debug_topics (toggle all debug output)
"""

import rclpy
import numpy as np
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.ndimage import distance_transform_edt, gaussian_filter1d


class APFPlannerDebug(Node):
    """APF planner with extensive debug publishers for RViz."""

    def __init__(self):
        super().__init__('apf_planner_debug')

        # --- Parameters (with defaults) ---
        self.declare_parameter('k_attractive', 1.0)
        self.declare_parameter('k_repulsive', 0.08)
        self.declare_parameter('influence_radius', 0.6)  # D0
        self.declare_parameter('gradient_step', 0.3)
        self.declare_parameter('max_iterations', 2000)
        self.declare_parameter('convergence_threshold', 0.06)
        self.declare_parameter('height_smoothing_sigma', 2.5)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('publish_debug_topics', True)
        self.declare_parameter('debug_publish_rate', 1.0)

        self.K_ATTR = self.get_parameter('k_attractive').value
        self.K_REP = self.get_parameter('k_repulsive').value
        self.D0 = self.get_parameter('influence_radius').value
        self.GRAD_STEP = self.get_parameter('gradient_step').value
        self.MAX_ITER = self.get_parameter('max_iterations').value
        self.CONVERGE_M = self.get_parameter('convergence_threshold').value
        self.HEIGHT_SIGMA = self.get_parameter('height_smoothing_sigma').value
        self.PUBLISH_DEBUG = self.get_parameter('publish_debug_topics').value

        # --- Subscriptions ---
        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        # --- Publishers (Geometry Layer Debugging) ---
        self.pub_h_avail = self.create_publisher(PointCloud2, '/debug_h_avail', 10)
        self.pub_w_avail = self.create_publisher(PointCloud2, '/debug_w_avail', 10)
        self.pub_safe_cells = self.create_publisher(PointCloud2, '/debug_safe_cells', 10)
        self.pub_opt_z = self.create_publisher(PointCloud2, '/debug_optimal_height', 10)

        # --- Publishers (APF Field Components) ---
        self.pub_apf_field = self.create_publisher(PointCloud2, '/apf_field_potential', 10)
        self.pub_apf_attr = self.create_publisher(PointCloud2, '/apf_attraction_field', 10)
        self.pub_apf_rep = self.create_publisher(PointCloud2, '/apf_repulsion_field', 10)
        self.pub_apf_grad = self.create_publisher(PointCloud2, '/apf_gradient_field', 10)

        # --- Publishers (Path & Plan) ---
        self.pub_apf_path = self.create_publisher(PointCloud2, '/apf_path_smoothed', 10)
        self.pub_waypoints = self.create_publisher(PointCloud2, '/apf_waypoints', 10)

        # --- Publishers (Obstacle/Goal Markers) ---
        self.pub_obstacles = self.create_publisher(PointCloud2, '/debug_obstacles_xy', 10)
        self.pub_goal_marker = self.create_publisher(Marker, '/debug_goal_marker', 10)
        self.pub_start_marker = self.create_publisher(Marker, '/debug_start_marker', 10)

        # --- Robot Kinematics ---
        self.z_min = 0.15
        self.z_max = 0.40
        self.s_min = 0.40
        self.s_max = 0.80
        self.l_rob = 0.50

        self.get_logger().info('APF Planner (Debug) initialized with extensive publishers')

    # ================================================================== #
    #  GEOMETRY COMPUTATION                                              #
    # ================================================================== #

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        """Extract a named layer from a GridMap."""
        idx = msg.layers.index(name)
        d = msg.data[idx]
        n_cols = d.layout.dim[0].size
        n_rows = d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def get_span_from_z(self, z_arr: np.ndarray) -> np.ndarray:
        """Compute leg span from body height."""
        ratio = (z_arr - self.z_min) / (self.z_max - self.z_min)
        return self.s_max - ratio * (self.s_max - self.s_min)

    def compute_clearances(self, floor: np.ndarray, ceil: np.ndarray, res: float):
        """Compute height and width availability."""
        valid = ~np.isnan(floor) & ~np.isnan(ceil)
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        walls = valid & (clearance_raw < self.z_min)
        d_xy = distance_transform_edt(~walls) * res
        h_avail = np.where(valid, clearance_raw, np.nan)
        w_avail = np.where(valid, d_xy, np.nan)
        return h_avail, w_avail, floor, valid

    def optimize_posture(self, h_avail: np.ndarray, w_avail: np.ndarray):
        """Find optimal body height at each cell."""
        z_candidates = np.linspace(self.z_min, self.z_max, 15)
        best_score = np.full_like(h_avail, -np.inf)
        best_z = np.full_like(h_avail, np.nan)

        for z in z_candidates:
            s = self.get_span_from_z(z)
            margin_z = h_avail - z
            margin_w = w_avail - s
            score = np.minimum(margin_z, margin_w)
            score = np.where((z > h_avail) | (s > w_avail), -np.inf, score)
            improved = score > best_score
            best_score = np.where(improved, score, best_score)
            best_z = np.where(improved, z, best_z)

        is_safe = best_score > -np.inf
        return best_z, is_safe

    # ================================================================== #
    #  APF COMPUTATION                                                   #
    # ================================================================== #

    def world_to_grid(self, wx: float, wy: float, info) -> tuple:
        """Convert world (x,y) to grid (col, row)."""
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0
        res = info.resolution

        col = int((cy + half_y - wy) / res - 0.5)
        row = int((cx + half_x - wx) / res - 0.5)
        return col, row

    def grid_to_world(self, col: float, row: float, info) -> tuple:
        """Convert grid (col, row) to world (x, y)."""
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0
        res = info.resolution
        wy = cy + half_y - (col + 0.5) * res
        wx = cx + half_x - (row + 0.5) * res
        return wx, wy

    def compute_apf(self, safe_mask: np.ndarray, w_avail: np.ndarray,
                    goal_col: int, goal_row: int) -> tuple:
        """
        Compute APF: U = U_attr + U_rep
        Returns (U_total, U_attr, U_rep, grad_magnitude)
        """
        rows, cols = safe_mask.shape
        c_idx, r_idx = np.meshgrid(
            np.arange(safe_mask.shape[0]),
            np.arange(safe_mask.shape[1]),
            indexing='ij'
        )

        # Attractive term
        U_attr = self.K_ATTR * ((c_idx - goal_col)**2 + (r_idx - goal_row)**2).astype(np.float32)

        # Repulsive term
        w = np.where(safe_mask, w_avail, np.nan)
        in_range = safe_mask & (w < self.D0) & (w > 1e-6)
        U_rep = np.zeros_like(U_attr)
        U_rep[in_range] = self.K_REP * (1.0 / w[in_range] - 1.0 / self.D0) ** 2

        # Combined
        U = U_attr + U_rep
        U = np.where(safe_mask, U, np.inf)

        # Gradient magnitude (for visualization)
        grad_mag = self._compute_gradient_magnitude(U)

        return U, U_attr, U_rep, grad_mag

    def _compute_gradient_magnitude(self, U: np.ndarray) -> np.ndarray:
        """Compute |∇U| for visualization."""
        n_cols, n_rows = U.shape
        grad_mag = np.zeros_like(U)

        for i in range(1, n_cols - 1):
            for j in range(1, n_rows - 1):
                U_c = U[i, j] if np.isfinite(U[i, j]) else 0
                dU_dc = (U[i + 1, j] if np.isfinite(U[i + 1, j]) else U_c) - \
                        (U[i - 1, j] if np.isfinite(U[i - 1, j]) else U_c)
                dU_dr = (U[i, j + 1] if np.isfinite(U[i, j + 1]) else U_c) - \
                        (U[i, j - 1] if np.isfinite(U[i, j - 1]) else U_c)
                grad_mag[i, j] = np.hypot(dU_dc / 2.0, dU_dr / 2.0)

        return grad_mag

    def gradient_descent_path(self, U: np.ndarray, start_col: int, start_row: int,
                             goal_col: int, goal_row: int, res: float) -> list:
        """Gradient descent to find path."""
        n_cols, n_rows = U.shape
        path = []

        finite_mask = np.isfinite(U)
        if not finite_mask.any():
            self.get_logger().warn('No finite cells in U')
            return path

        if not finite_mask[start_col, start_row]:
            safe_cols, safe_rows = np.where(finite_mask)
            dists = np.hypot(safe_cols - start_col, safe_rows - start_row)
            best = np.argmin(dists)
            start_col = int(safe_cols[best])
            start_row = int(safe_rows[best])

        c, r = float(start_col), float(start_row)
        converge_cells = self.CONVERGE_M / res

        for _ in range(self.MAX_ITER):
            path.append((c, r))

            dist_to_goal = np.hypot(c - goal_col, r - goal_row)
            if dist_to_goal < converge_cells:
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

    def sample_smooth_height(self, path: list, opt_z: np.ndarray,
                            floor: np.ndarray) -> tuple:
        """Sample and smooth body height along path."""
        n_cols, n_rows = opt_z.shape
        raw_heights = []
        floor_heights = []

        for c, r in path:
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

        raw_heights = np.array(raw_heights, dtype=np.float32)
        floor_heights = np.array(floor_heights, dtype=np.float32)

        heights_smooth = gaussian_filter1d(raw_heights, sigma=self.HEIGHT_SIGMA)
        heights_smooth = np.clip(heights_smooth, self.z_min, self.z_max)

        return heights_smooth, floor_heights

    # ================================================================== #
    #  PUBLISHING                                                         #
    # ================================================================== #

    def publish_heatmap(self, publisher, header, info, values: np.ndarray, mask: np.ndarray):
        """Publish a 2D heatmap as PointCloud2."""
        if not np.any(mask):
            return

        res = info.resolution
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        cols, rows = np.where(mask)
        world_y = cy + half_y - (cols + 0.5) * res
        world_x = cx + half_x - (rows + 0.5) * res
        intensities = values[cols, rows]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        world_z = np.zeros_like(world_x)
        points = np.column_stack((world_x, world_y, world_z, intensities))
        cloud_msg = pc2.create_cloud(header, fields, points)
        publisher.publish(cloud_msg)

    def publish_path_3d(self, publisher, header, info, path: list,
                       heights_smooth: np.ndarray, floor_heights: np.ndarray):
        """Publish path as 3D PointCloud2 with body height as intensity."""
        if not path:
            return

        res = info.resolution
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        wx_list, wy_list, wz_list, h_list = [], [], [], []

        for i, (c, r) in enumerate(path):
            wy = cy + half_y - (c + 0.5) * res
            wx = cx + half_x - (r + 0.5) * res
            h = heights_smooth[i]
            f = floor_heights[i]
            wz = f + h / 2.0

            wx_list.append(wx)
            wy_list.append(wy)
            wz_list.append(wz)
            h_list.append(h)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pts = np.column_stack((wx_list, wy_list, wz_list, h_list)).astype(np.float32)
        cloud_msg = pc2.create_cloud(header, fields, pts)
        publisher.publish(cloud_msg)

    def publish_waypoints_as_markers(self, publisher, header, path: list, info):
        """Publish individual waypoints as sphere markers."""
        res = info.resolution
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        for i, (c, r) in enumerate(path[::max(1, len(path)//20)]):  # Sparse for performance
            wy = cy + half_y - (c + 0.5) * res
            wx = cx + half_x - (r + 0.5) * res

            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            publisher.publish(marker)

    def publish_goal_start_markers(self, header, info, goal_col: int, goal_row: int):
        """Publish goal (blue) and start (green) as markers."""
        res = info.resolution
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        start_col, start_row = info.data[0].layout.dim[0].size // 2, info.data[0].layout.dim[1].size // 2
        
        goal_wy = cy + half_y - (goal_col + 0.5) * res
        goal_wx = cx + half_x - (goal_row + 0.5) * res

        start_wy = cy + half_y - (start_col + 0.5) * res
        start_wx = cx + half_x - (start_row + 0.5) * res

        # Goal marker (blue)
        goal_marker = Marker()
        goal_marker.header = header
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal_wx
        goal_marker.pose.position.y = goal_wy
        goal_marker.pose.position.z = 0.0
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color.r = 0.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 1.0
        goal_marker.color.a = 1.0
        self.pub_goal_marker.publish(goal_marker)

        # Start marker (green)
        start_marker = Marker()
        start_marker.header = header
        start_marker.id = 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = start_wx
        start_marker.pose.position.y = start_wy
        start_marker.pose.position.z = 0.0
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        self.pub_start_marker.publish(start_marker)

    # ================================================================== #
    #  MAIN CALLBACK                                                      #
    # ================================================================== #

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            return

        res = floor_msg.info.resolution
        floor = self.get_layer(floor_msg, 'elevation')
        ceil = self.get_layer(ceil_msg, 'elevation')

        # Phase 1 & 2: Geometry
        h_avail, w_avail, f_base, valid = self.compute_clearances(floor, ceil, res)
        opt_z, safe_mask = self.optimize_posture(h_avail, w_avail)

        n_safe = np.sum(safe_mask)
        n_valid = np.sum(valid)
        self.get_logger().info(f'Safe cells: {n_safe} / {n_valid}')

        if n_safe < 5:
            self.get_logger().warn('Fewer than 5 safe cells — aborting path planning')
            return

        # Phase 3: APF Planning
        goal_wx = self.get_parameter('goal_x').value
        goal_wy = self.get_parameter('goal_y').value
        goal_col, goal_row = self.world_to_grid(goal_wx, goal_wy, floor_msg.info)

        goal_col = int(np.clip(goal_col, 0, safe_mask.shape[0] - 1))
        goal_row = int(np.clip(goal_row, 0, safe_mask.shape[1] - 1))

        start_col = safe_mask.shape[0] // 2
        start_row = safe_mask.shape[1] // 2

        # Compute APF with all components
        U, U_attr, U_rep, grad_mag = self.compute_apf(safe_mask, w_avail, goal_col, goal_row)

        # Plan path
        path = self.gradient_descent_path(U, start_col, start_row, goal_col, goal_row, res)
        self.get_logger().info(f'APF path: {len(path)} waypoints')

        # Sample & smooth heights
        heights_smooth, floor_heights = self.sample_smooth_height(path, opt_z, floor)

        if not self.PUBLISH_DEBUG:
            return

        # --- PUBLISH ALL DEBUG TOPICS ---
        # Geometry layers
        self.publish_heatmap(self.pub_h_avail, floor_msg.header, floor_msg.info, h_avail, valid)
        self.publish_heatmap(self.pub_w_avail, floor_msg.header, floor_msg.info, w_avail, valid)
        safe_float = safe_mask.astype(np.float32)
        self.publish_heatmap(self.pub_safe_cells, floor_msg.header, floor_msg.info, safe_float, valid)
        self.publish_heatmap(self.pub_opt_z, floor_msg.header, floor_msg.info, opt_z, safe_mask)

        # APF field components
        U_vis = np.where(safe_mask, np.clip(U, 0, np.percentile(U[safe_mask], 95)), np.nan)
        self.publish_heatmap(self.pub_apf_field, floor_msg.header, floor_msg.info, U_vis, safe_mask)

        U_attr_vis = np.where(safe_mask, np.clip(U_attr, 0, np.percentile(U_attr[safe_mask], 95)), np.nan)
        self.publish_heatmap(self.pub_apf_attr, floor_msg.header, floor_msg.info, U_attr_vis, safe_mask)

        U_rep_vis = np.where(safe_mask, np.clip(U_rep, 0, np.percentile(U_rep[safe_mask], 95)), np.nan)
        self.publish_heatmap(self.pub_apf_rep, floor_msg.header, floor_msg.info, U_rep_vis, safe_mask)

        grad_vis = np.where(safe_mask, np.clip(grad_mag, 0, np.percentile(grad_mag[safe_mask], 95)), np.nan)
        self.publish_heatmap(self.pub_apf_grad, floor_msg.header, floor_msg.info, grad_vis, safe_mask)

        # Path and waypoints
        self.publish_path_3d(self.pub_apf_path, floor_msg.header, floor_msg.info,
                            path, heights_smooth, floor_heights)
        # self.publish_waypoints_as_markers(self.pub_waypoints, floor_msg.header, path, floor_msg.info)

        # Goal and start markers
        # self.publish_goal_start_markers(floor_msg.header, floor_msg.info, goal_col, goal_row)


def main(args=None):
    rclpy.init(args=args)
    node = APFPlannerDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()