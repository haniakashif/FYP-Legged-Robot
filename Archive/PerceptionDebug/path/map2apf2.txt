#!/usr/bin/env python3
import sys
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.ndimage import distance_transform_edt, gaussian_filter1d


class OneShotMapGrabber(Node):
    """
    Briefly connects to ROS 2, grabs one pair of synchronized maps,
    stores them, and flags that it is done.
    """
    def __init__(self):
        super().__init__('apf_one_shot_recorder')
        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil  = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        self.got_data = False
        self.floor_data = None
        self.ceil_data = None
        self.info = None
        self.get_logger().info("Waiting for maps to arrive...")

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        idx    = msg.layers.index(name)
        d      = msg.data[idx]
        n_cols = d.layout.dim[0].size
        n_rows = d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        if self.got_data: return
        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            return

        self.floor_data = self.get_layer(floor_msg, 'elevation')
        self.ceil_data  = self.get_layer(ceil_msg,  'elevation')
        self.info       = floor_msg.info
        self.got_data   = True
        self.get_logger().info("Maps captured! Shutting down ROS subscriber...")


# =====================================================================
# APF MATH ENGINE
# =====================================================================
class APFPlanner:
    # Kinematics
    Z_MIN, Z_MAX = 0.15, 0.40
    S_MIN, S_MAX = 0.40, 0.80

    # APF tuning
    K_ATTR, K_REP = 1.0, 0.1
    D0, GRAD_STEP = 1.0, 0.7
    MAX_ITER = 2000
    HEIGHT_SIGMA = 0.5

    def get_span_from_z(self, z):
        return self.S_MAX - ((z - self.Z_MIN) / (self.Z_MAX - self.Z_MIN)) * (self.S_MAX - self.S_MIN)

    def compute_clearances(self, floor, ceil, res):
        valid = ~np.isnan(floor) & ~np.isnan(ceil)
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        walls = valid & (clearance_raw < self.Z_MIN)
        w_avail = np.where(valid, distance_transform_edt(~walls) * res, np.nan)
        return np.where(valid, clearance_raw, np.nan), w_avail, valid

    def optimize_posture(self, h_avail, w_avail):
        best_score = np.full_like(h_avail, -np.inf)
        best_z = np.full_like(h_avail, np.nan)
        for z in np.linspace(self.Z_MIN, self.Z_MAX, 15):
            s = self.get_span_from_z(z)
            score = np.where((z > h_avail) | (s > w_avail), -np.inf, np.minimum(h_avail - z, w_avail - s))
            improved = score > best_score
            best_score = np.where(improved, score, best_score)
            best_z = np.where(improved, z, best_z)
        return best_z, best_score > -np.inf

    def compute_apf(self, safe_mask, w_avail, goal_col, goal_row):
        c_idx, r_idx = np.meshgrid(np.arange(safe_mask.shape[0]), np.arange(safe_mask.shape[1]), indexing='ij')
        U_attr = self.K_ATTR * ((c_idx - goal_col)**2 + (r_idx - goal_row)**2).astype(np.float32)
        w = np.where(safe_mask, w_avail, np.nan)
        in_range = safe_mask & (w < self.D0) & (w > 1e-6)
        U_rep = np.zeros_like(U_attr)
        U_rep[in_range] = self.K_REP * (1.0 / w[in_range] - 1.0 / self.D0) ** 2
        return np.where(safe_mask, U_attr + U_rep, np.inf)

    def gradient_descent(self, U, start_col, start_row, goal_col, goal_row, res):
        path = []
        c, r = float(start_col), float(start_row)
        n_cols, n_rows = U.shape
        converge = 0.06 / res

        for _ in range(self.MAX_ITER):
            path.append((c, r))
            if np.hypot(c - goal_col, r - goal_row) < converge:
                path.append((float(goal_col), float(goal_row)))
                break

            ci, ri = int(np.clip(round(c), 1, n_cols - 2)), int(np.clip(round(r), 1, n_rows - 2))
            Uc = U[ci, ri]
            dU_dc = ((U[ci+1, ri] if np.isfinite(U[ci+1, ri]) else Uc) - (U[ci-1, ri] if np.isfinite(U[ci-1, ri]) else Uc)) / 2.0
            dU_dr = ((U[ci, ri+1] if np.isfinite(U[ci, ri+1]) else Uc) - (U[ci, ri-1] if np.isfinite(U[ci, ri-1]) else Uc)) / 2.0

            grad_mag = np.hypot(dU_dc, dU_dr)
            if not np.isfinite(grad_mag) or grad_mag < 1e-9:
                dc, dr = goal_col - c, goal_row - r
                norm = np.hypot(dc, dr) + 1e-9
                c += self.GRAD_STEP * dc / norm
                r += self.GRAD_STEP * dr / norm
            else:
                c -= self.GRAD_STEP * dU_dc / grad_mag
                r -= self.GRAD_STEP * dU_dr / grad_mag

            c, r = np.clip(c, 0, n_cols - 1), np.clip(r, 0, n_rows - 1)
        return path
    def adam_optimizer_path(self, U, start_col, start_row, goal_col, goal_row, res):
        path = []
        c, r = float(start_col), float(start_row)
        n_cols, n_rows = U.shape
        converge = 0.06 / res

        # --- Adam Hyperparameters ---
        alpha = self.GRAD_STEP   # Base step size
        beta1 = 0.9              # Momentum (Smooths the jaggedness)
        beta2 = 0.999            # RMSprop (Adapts step size near steep obstacles)
        epsilon = 1e-8

        # Initialize moments
        m_c, m_r = 0.0, 0.0
        v_c, v_r = 0.0, 0.0
        t = 0

        for _ in range(self.MAX_ITER):
            path.append((c, r))
            if np.hypot(c - goal_col, r - goal_row) < converge:
                path.append((float(goal_col), float(goal_row)))
                break

            t += 1

            # 1. Compute discrete gradient
            ci, ri = int(np.clip(round(c), 1, n_cols - 2)), int(np.clip(round(r), 1, n_rows - 2))
            Uc = U[ci, ri]

            dU_dc = ((U[ci+1, ri] if np.isfinite(U[ci+1, ri]) else Uc) - (U[ci-1, ri] if np.isfinite(U[ci-1, ri]) else Uc)) / 2.0
            dU_dr = ((U[ci, ri+1] if np.isfinite(U[ci, ri+1]) else Uc) - (U[ci, ri-1] if np.isfinite(U[ci, ri-1]) else Uc)) / 2.0

            # 2. Normalize raw gradient to prevent massive explosions near obstacles
            grad_mag = np.hypot(dU_dc, dU_dr)
            if not np.isfinite(grad_mag) or grad_mag < 1e-9:
                # Flat/Degenerate: point straight to goal
                dc, dr = goal_col - c, goal_row - r
                norm = np.hypot(dc, dr) + 1e-9
                g_c, g_r = -dc / norm, -dr / norm  # Negative because update rule subtracts
            else:
                g_c, g_r = dU_dc / grad_mag, dU_dr / grad_mag

            # 3. Adam Update Math
            # Update biased first moment estimate (Momentum)
            m_c = beta1 * m_c + (1 - beta1) * g_c
            m_r = beta1 * m_r + (1 - beta1) * g_r

            # Update biased second raw moment estimate (RMSprop)
            v_c = beta2 * v_c + (1 - beta2) * (g_c ** 2)
            v_r = beta2 * v_r + (1 - beta2) * (g_r ** 2)

            # Compute bias-corrected first moment estimate
            m_hat_c = m_c / (1 - beta1 ** t)
            m_hat_r = m_r / (1 - beta1 ** t)

            # Compute bias-corrected second raw moment estimate
            v_hat_c = v_c / (1 - beta2 ** t)
            v_hat_r = v_r / (1 - beta2 ** t)

            # 4. Take the step
            c -= alpha * m_hat_c / (np.sqrt(v_hat_c) + epsilon)
            r -= alpha * m_hat_r / (np.sqrt(v_hat_r) + epsilon)

            # Clamp to grid boundaries
            c, r = np.clip(c, 0, n_cols - 1), np.clip(r, 0, n_rows - 1)

        return path


# =====================================================================
# SCRIPT EXECUTION
# =====================================================================
def main():
    # 1. Start ROS and get one map snapshot
    rclpy.init()
    grabber = OneShotMapGrabber()

    while rclpy.ok() and not grabber.got_data:
        rclpy.spin_once(grabber, timeout_sec=0.1)

    # Clean shutdown of ROS
    floor = grabber.floor_data
    ceil = grabber.ceil_data
    res = grabber.info.resolution
    cx, cy = grabber.info.pose.position.x, grabber.info.pose.position.y
    hx, hy = grabber.info.length_x / 2.0, grabber.info.length_y / 2.0

    grabber.destroy_node()
    rclpy.shutdown()

    print("\n--- ROS disconnected. Processing APF Data ---")
    planner = APFPlanner()

    # 2. Compute Clearances & Obstacles
    h_avail, w_avail, valid = planner.compute_clearances(floor, ceil, res)
    opt_z, safe_mask = planner.optimize_posture(h_avail, w_avail)

    if np.sum(safe_mask) < 5:
        print("Error: Map is entirely blocked. No safe spaces found.")
        sys.exit(0)

    # 3. AUTO-GOAL DETERMINATION (The Furthest Point)
    start_col, start_row = safe_mask.shape[0] // 2, safe_mask.shape[1] // 2

    safe_cols, safe_rows = np.where(safe_mask)
    dists_from_start = np.hypot(safe_cols - start_col, safe_rows - start_row)
    furthest_idx = np.argmax(dists_from_start)

    goal_col = int(safe_cols[furthest_idx])
    goal_row = int(safe_rows[furthest_idx])

    print(f"Robot Start: Grid({start_col}, {start_row})")
    print(f"Auto Goal Detected: Grid({goal_col}, {goal_row})")

    # 4. Generate Path
    U = planner.compute_apf(safe_mask, w_avail, goal_col, goal_row)
    path = planner.adam_optimizer_path(U, start_col, start_row, goal_col, goal_row, res)

    # Sample heights for visualization
    raw_h = [opt_z[int(c), int(r)] for c, r in path]
    raw_h = [h if not np.isnan(h) else planner.Z_MIN for h in raw_h]
    smooth_h = np.clip(gaussian_filter1d(raw_h, sigma=planner.HEIGHT_SIGMA), planner.Z_MIN, planner.Z_MAX)

    # Convert path to world coordinates
    path_x = np.array([cx + hx - (r + 0.5) * res for c, r in path])
    path_y = np.array([cy + hy - (c + 0.5) * res for c, r in path])
    path_z = np.array([floor[int(c), int(r)] + smooth_h[i] / 2.0 for i, (c, r) in enumerate(path)])

    # Fill in nan values in Z just in case
    path_z = np.nan_to_num(path_z, nan=0.0)

    # 5. Build Interactive 3D Visualization
    print("\nOpening Interactive 3D Viewer...")

    fig = plt.figure(figsize=(12, 9))
    # Create the 3D plot, leaving room at the bottom for the slider
    ax = fig.add_axes([0.05, 0.15, 0.9, 0.8], projection='3d')

    # Map index to world coords for plotting meshes
    y_vals = cy + hy - (np.arange(floor.shape[0]) + 0.5) * res
    x_vals = cx + hx - (np.arange(floor.shape[1]) + 0.5) * res
    X, Y = np.meshgrid(x_vals, y_vals)

    # Plot the full Floor and Ceiling
    F_plot = np.copy(floor)
    C_plot = np.copy(ceil)

    ax.plot_surface(X, Y, F_plot, cmap='terrain', alpha=0.7, edgecolor='none')
    ax.plot_surface(X, Y, C_plot, color='lightgray', alpha=0.3, edgecolor='none')

    # Plot Obstacles (The unsafe areas that generate repulsive forces)
    # This mimics the MATLAB output where obstacles are clearly defined
    obstacle_mask = valid & (~safe_mask)
    if np.any(obstacle_mask):
        obs_x = X[obstacle_mask]
        obs_y = Y[obstacle_mask]
        obs_z = F_plot[obstacle_mask]
        # Scatter red hazard points along the floor where the robot cannot go
        ax.scatter(obs_x, obs_y, obs_z, color='red', marker='x', s=15, label='Repulsive Obstacles')

    # Initialize the Path and Robot Marker
    line, = ax.plot([], [], [], '-k', linewidth=3, label='Navigated Path')
    robot_dot, = ax.plot([], [], [], 'wo', markeredgecolor='black', markersize=8, label='Robot Center of Mass')

    # Plot Start and Goal Flags permanently
    ax.scatter(path_x[0], path_y[0], path_z[0], color='green', s=50, label='Start')
    ax.scatter(path_x[-1], path_y[-1], path_z[-1], color='blue', s=50, label='Goal')

    ax.set_title("Interactive Cave APF Explorer", fontsize=14)
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_zlabel("Elevation Z (m)")
    ax.legend(loc='upper left')

    # Adjust the default viewing angle
    ax.view_init(elev=35, azim=135)

    # Add a dynamic text element to show body height
    posture_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes,
                             bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'), fontsize=12)

    # --- Setup the Scrubbing Dial (Slider) ---
    ax_slider = fig.add_axes([0.2, 0.05, 0.6, 0.03])
    frame_slider = Slider(
        ax=ax_slider,
        label='Timeline Frame',
        valmin=0,
        valmax=len(path) - 1,
        valinit=0,
        valstep=1,
        color='blue'
    )

    # The update function called every time the slider moves
    def update_view(val):
        frame = int(frame_slider.val)

        # Draw path strictly up to the current frame
        line.set_data(path_x[:frame+1], path_y[:frame+1])
        line.set_3d_properties(path_z[:frame+1])

        # Move the robot dot to the current frame
        robot_dot.set_data([path_x[frame]], [path_y[frame]])
        robot_dot.set_3d_properties([path_z[frame]])

        # Update UI Text
        h_now = smooth_h[frame]
        state = "Crouching" if h_now < 0.25 else "Standing"
        posture_text.set_text(f"Posture: {h_now:.3f}m ({state})")

        fig.canvas.draw_idle()

    # Link the slider to the update function
    frame_slider.on_changed(update_view)

    # Force the first frame to render
    update_view(0)

    # Blocks execution until the user manually closes the window
    plt.show()

    sys.exit(0)

if __name__ == '__main__':
    main()
