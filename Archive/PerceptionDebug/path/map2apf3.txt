#!/usr/bin/env python3
"""
PostureOptimizerDebug — APF Path Planner with Adam Optimizer
=============================================================
Methodology pipeline (each phase is instrumented for presentation):

  Phase 1 · Map Ingestion          — sync floor/ceiling GridMaps from ROS 2
  Phase 2 · Clearance Computation  — vertical h_avail, lateral w_avail (EDT)
  Phase 3 · Posture Optimisation   — per-cell optimal body height sweep
  Phase 4 · APF Construction       — U = U_attr (parabolic) + U_rep (EDT-driven)
  Phase 5 · Adam Path Descent      — momentum+RMSprop gradient walk to goal
  Phase 6 · Height Smoothing       — 1-D Gaussian filter on body-height profile
  Phase 7 · RViz Publishing        — PointCloud2 topics for every key field
"""

import rclpy
import numpy as np
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.ndimage import distance_transform_edt, gaussian_filter1d


# ══════════════════════════════════════════════════════════════════════════════
#  DEBUG HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _fmt_arr(label: str, arr: np.ndarray, mask: np.ndarray | None = None) -> str:
    """
    One-line statistical summary for any float array.
    If *mask* is supplied, statistics are computed only over mask==True cells.
    NaN cells are always excluded from statistics.
    """
    data = arr[mask] if mask is not None else arr.ravel()
    data = data[np.isfinite(data)]
    if data.size == 0:
        return f"[{label}] — no finite values"
    return (
        f"[{label}] "
        f"shape={arr.shape}  "
        f"finite={data.size}  "
        f"min={data.min():.4f}  "
        f"max={data.max():.4f}  "
        f"mean={data.mean():.4f}  "
        f"std={data.std():.4f}"
    )


def _fmt_matrix_heatmap(label: str, arr: np.ndarray, mask: np.ndarray | None = None,
                         cols: int = 60, rows: int = 20) -> str:
    """
    Renders an ASCII heatmap of *arr* using a 10-level ramp.
    Shows the spatial structure of 2-D fields (great for presentations).
    Masked/non-finite cells are shown as '·'.

    Ramp (low → high): ░░▒▒▓▓██  (unicode block chars)
    Negative cells highlighted with '-' prefix per block column.
    """
    RAMP = [' ', '·', '░', '▒', '▓', '█']

    data = arr.copy().astype(float)
    if mask is not None:
        data[~mask] = np.nan

    finite = data[np.isfinite(data)]
    if finite.size == 0:
        return f"[{label}] heatmap: (no data)\n"

    lo, hi = finite.min(), finite.max()

    # Downsample to display resolution
    r_step = max(1, arr.shape[0] // rows)
    c_step = max(1, arr.shape[1] // cols)
    small  = data[::r_step, ::c_step]

    lines = [f"\n╔══ {label}  [{lo:.3f} … {hi:.3f}] ══╗"]
    for row_idx in range(small.shape[0]):
        row_str = ""
        for val in small[row_idx]:
            if not np.isfinite(val):
                row_str += "·"
            else:
                # Normalise 0-1, then bucket into RAMP
                norm  = (val - lo) / (hi - lo + 1e-12)
                level = int(np.clip(norm * (len(RAMP) - 1), 0, len(RAMP) - 1))
                row_str += RAMP[level]
        lines.append("║ " + row_str + " ║")
    lines.append("╚" + "═" * (small.shape[1] + 4) + "╝")
    return "\n".join(lines)


def _fmt_sign_map(label: str, arr: np.ndarray, mask: np.ndarray | None = None,
                   cols: int = 60, rows: int = 20) -> str:
    """
    Sign map: '+' = positive, '-' = negative, '0' = zero, '·' = masked/nan.
    Useful for gradient fields and potential residuals.
    """
    data = arr.copy().astype(float)
    if mask is not None:
        data[~mask] = np.nan

    r_step = max(1, arr.shape[0] // rows)
    c_step = max(1, arr.shape[1] // cols)
    small  = data[::r_step, ::c_step]

    lines = [f"\n╔══ {label}  sign map ══╗"]
    for row_idx in range(small.shape[0]):
        row_str = ""
        for val in small[row_idx]:
            if not np.isfinite(val):
                row_str += "·"
            elif val > 1e-6:
                row_str += "+"
            elif val < -1e-6:
                row_str += "-"
            else:
                row_str += "0"
        lines.append("║ " + row_str + " ║")
    lines.append("╚" + "═" * (small.shape[1] + 4) + "╝")
    return "\n".join(lines)


def _fmt_bool_map(label: str, mask: np.ndarray,
                   cols: int = 60, rows: int = 20) -> str:
    """
    Boolean map: '█' = True, ' ' = False.
    Used for safe_mask, valid, obstacle geometry.
    """
    r_step = max(1, mask.shape[0] // rows)
    c_step = max(1, mask.shape[1] // cols)
    small  = mask[::r_step, ::c_step]

    pct = 100.0 * mask.sum() / max(mask.size, 1)
    lines = [f"\n╔══ {label}  ({mask.sum()} / {mask.size} cells = {pct:.1f}%) ══╗"]
    for row_idx in range(small.shape[0]):
        lines.append("║ " + "".join("█" if v else " " for v in small[row_idx]) + " ║")
    lines.append("╚" + "═" * (small.shape[1] + 4) + "╝")
    return "\n".join(lines)


def _section(title: str) -> str:
    bar = "═" * 70
    return f"\n{bar}\n  ▶  {title}\n{bar}"


# ══════════════════════════════════════════════════════════════════════════════
#  NODE
# ══════════════════════════════════════════════════════════════════════════════

class PostureOptimizerDebug(Node):
    """
    Extends the geometric feasibility tester with an Artificial Potential Field
    (APF) planner using an Adam optimiser for gradient descent.

    Pipeline
    --------
    1. Sync floor/ceiling GridMaps via ApproximateTimeSynchronizer.
    2. Compute per-cell vertical clearance h_avail and lateral EDT clearance w_avail.
    3. Sweep body-height candidates; select optimal z per cell (posture optimiser).
    4. Build APF:  U = K_ATTR·d²_goal  +  K_REP·(1/w − 1/D0)²
    5. Walk downhill with Adam (momentum + RMSprop) to produce a smooth path.
    6. Sample opt_z along path; apply 1-D Gaussian smoothing for gradual posture.
    7. Publish all intermediate fields and the final path as PointCloud2 topics.
    """

    # ── APF tuning ────────────────────────────────────────────────────────────
    K_ATTR       = 1.0    # Attractive gain
    K_REP        = 0.08   # Repulsive gain
    D0           = 0.6    # Repulsion influence radius (m)
    GRAD_STEP    = 0.7    # Base step size for Adam (cells per iteration)
    MAX_ITER     = 2000   # Safety cap on descent iterations
    CONVERGE_M   = 0.06   # Stop when within this distance of goal (m)
    HEIGHT_SIGMA = 2.5    # Gaussian σ (cells) for height-profile smoothing

    # ── Adam hyperparameters ──────────────────────────────────────────────────
    ADAM_BETA1   = 0.9    # Momentum decay (smooths direction jitter)
    ADAM_BETA2   = 0.999  # RMSprop decay  (adapts step near steep obstacles)
    ADAM_EPS     = 1e-8   # Numerical stability epsilon

    # ── Robot kinematics ──────────────────────────────────────────────────────
    Z_MIN = 0.15;  Z_MAX = 0.40   # Body height range (m)
    S_MIN = 0.40;  S_MAX = 0.80   # Leg span range (m)

    # ── Debug verbosity ───────────────────────────────────────────────────────
    # Set to False to silence heatmaps (keeps stats printouts)
    SHOW_HEATMAPS = True
    # Print Adam state every N iterations (0 = silent during descent)
    ADAM_LOG_EVERY = 200

    def __init__(self):
        super().__init__('posture_optimizer_debug')

        # Subscriptions
        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil  = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        # Existing debug publishers
        self.pub_h_avail   = self.create_publisher(PointCloud2, '/debug_h_avail_pc',     10)
        self.pub_w_avail   = self.create_publisher(PointCloud2, '/debug_w_avail_pc',     10)
        self.pub_z_targets = self.create_publisher(PointCloud2, '/debug_z_targets_pc',   10)
        self.pub_safe_mask = self.create_publisher(PointCloud2, '/debug_safe_spaces_pc', 10)

        # APF publishers
        self.pub_apf_field = self.create_publisher(PointCloud2, '/debug_apf_field_pc',   10)
        self.pub_apf_path  = self.create_publisher(PointCloud2, '/apf_path_pc',          10)

        # Goal parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)

        self.get_logger().info(
            f'PostureOptimizerDebug (APF+Adam) ready. '
            f'Goal: ({self.get_parameter("goal_x").value}, '
            f'{self.get_parameter("goal_y").value})'
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  UTILITY
    # ══════════════════════════════════════════════════════════════════════════

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        idx    = msg.layers.index(name)
        d      = msg.data[idx]
        n_cols = d.layout.dim[0].size
        n_rows = d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def get_span_from_z(self, z_arr: np.ndarray) -> np.ndarray:
        ratio = (z_arr - self.Z_MIN) / (self.Z_MAX - self.Z_MIN)
        return self.S_MAX - ratio * (self.S_MAX - self.S_MIN)

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

    # ══════════════════════════════════════════════════════════════════════════
    #  PHASE 2 — CLEARANCE COMPUTATION
    # ══════════════════════════════════════════════════════════════════════════

    def compute_clearances(self, floor: np.ndarray, ceil: np.ndarray, res: float):
        """
        Compute vertical and lateral clearance at every grid cell.

        h_avail[c,r] = ceil[c,r] − floor[c,r]   (vertical headroom, metres)
        w_avail[c,r] = EDT distance to nearest cell with h_avail < Z_MIN  (metres)
                       — this is the "wall clearance" used directly in U_rep.

        The EDT is applied to the binary obstacle mask (cells too low for robot)
        so w_avail is a smooth, everywhere-defined distance field rather than
        a step function.
        """
        log = self.get_logger()
        log.info(_section("PHASE 2 · Clearance Computation"))

        valid         = ~np.isnan(floor) & ~np.isnan(ceil)
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        walls         = valid & (clearance_raw < self.Z_MIN)
        d_xy          = distance_transform_edt(~walls) * res
        h_avail       = np.where(valid, clearance_raw, np.nan)
        w_avail       = np.where(valid, d_xy,          np.nan)

        # ── Debug: raw input maps ─────────────────────────────────────
        log.info(_fmt_arr("floor elevation", floor, valid))
        log.info(_fmt_arr("ceil  elevation", ceil,  valid))

        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("floor elevation", floor, valid))
            log.info(_fmt_matrix_heatmap("ceil  elevation", ceil,  valid))

        # ── Debug: valid region ───────────────────────────────────────
        log.info(_fmt_bool_map("valid cells (floor & ceil observed)", valid))

        # ── Debug: wall obstacle mask ─────────────────────────────────
        log.info(f"[walls] cells with h_avail < Z_MIN({self.Z_MIN}m): {walls.sum()}")
        if self.SHOW_HEATMAPS:
            log.info(_fmt_bool_map("wall obstacle mask", walls))

        # ── Debug: h_avail ────────────────────────────────────────────
        log.info(_fmt_arr("h_avail (vertical clearance, m)", h_avail, valid))
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("h_avail", h_avail, valid))

        # ── Debug: w_avail (EDT) ──────────────────────────────────────
        log.info(_fmt_arr("w_avail (EDT lateral clearance, m)", w_avail, valid))
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("w_avail", w_avail, valid))

        return h_avail, w_avail, floor, valid

    # ══════════════════════════════════════════════════════════════════════════
    #  PHASE 3 — POSTURE OPTIMISATION
    # ══════════════════════════════════════════════════════════════════════════

    def optimize_posture(self, h_avail: np.ndarray, w_avail: np.ndarray):
        """
        For each cell, sweep z ∈ [Z_MIN, Z_MAX] (15 candidates) and choose
        the body height that maximises  min(h_avail−z, w_avail−s(z))  — the
        tightest clearance margin in both axes simultaneously.

        A cell is 'safe' iff at least one candidate z fits.
        """
        log = self.get_logger()
        log.info(_section("PHASE 3 · Posture Optimisation"))

        z_candidates = np.linspace(self.Z_MIN, self.Z_MAX, 15)
        log.info(f"[posture sweep] z candidates: {np.round(z_candidates, 3).tolist()}")

        best_score = np.full_like(h_avail, -np.inf)
        best_z     = np.full_like(h_avail,  np.nan)

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

        # ── Debug: best_score distribution ───────────────────────────
        log.info(_fmt_arr("best_score (min-margin across both axes)", best_score, is_safe))
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("best_score", best_score, is_safe))
            # Sign map highlights cells that barely pass (score ≈ 0) vs roomy cells
            log.info(_fmt_sign_map("best_score sign", best_score, is_safe))

        # ── Debug: opt_z distribution ─────────────────────────────────
        log.info(_fmt_arr("opt_z (optimal body height per cell, m)", best_z, is_safe))
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("opt_z", best_z, is_safe))

        # ── Debug: safe_mask coverage ─────────────────────────────────
        log.info(_fmt_bool_map("safe_mask (navigable cells)", is_safe))
        safe_pct = 100.0 * is_safe.sum() / max(np.isfinite(h_avail).sum(), 1)
        log.info(f"[posture] {is_safe.sum()} safe cells  ({safe_pct:.1f}% of observed area)")

        return best_z, is_safe

    # ══════════════════════════════════════════════════════════════════════════
    #  PHASE 4 — APF CONSTRUCTION
    # ══════════════════════════════════════════════════════════════════════════

    def compute_apf(
        self,
        safe_mask: np.ndarray,
        w_avail:   np.ndarray,
        goal_col:  int,
        goal_row:  int,
    ) -> np.ndarray:
        """
        Build the combined scalar potential field:

            U_attr(c,r) = K_ATTR · [(c−goal_c)² + (r−goal_r)²]
                          — parabolic bowl; minimum exactly at goal

            U_rep(c,r)  = K_REP · (1/w − 1/D0)²   if w < D0
                        = 0                          otherwise
                          — diverges as w→0 (robot approaching wall)

            U = U_attr + U_rep     (safe cells only; unsafe = +∞)

        The gradient of U drives the Adam descent in Phase 5.
        """
        log = self.get_logger()
        log.info(_section("PHASE 4 · APF Construction"))
        log.info(f"[APF] goal cell: ({goal_col}, {goal_row})  "
                 f"K_ATTR={self.K_ATTR}  K_REP={self.K_REP}  D0={self.D0}m")

        c_idx, r_idx = np.meshgrid(
            np.arange(safe_mask.shape[0]),
            np.arange(safe_mask.shape[1]),
            indexing='ij'
        )

        # ── Attractive term ───────────────────────────────────────────
        U_attr = self.K_ATTR * (
            (c_idx - goal_col)**2 + (r_idx - goal_row)**2
        ).astype(np.float32)

        log.info(_fmt_arr("U_attr (attractive potential)", U_attr, safe_mask))
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("U_attr", U_attr, safe_mask))

        # ── Repulsive term ────────────────────────────────────────────
        w        = np.where(safe_mask, w_avail, np.nan)
        in_range = safe_mask & (w < self.D0) & (w > 1e-6)
        U_rep    = np.zeros_like(U_attr)
        U_rep[in_range] = self.K_REP * (1.0 / w[in_range] - 1.0 / self.D0) ** 2

        log.info(_fmt_arr("U_rep (repulsive potential)", U_rep, safe_mask))
        log.info(f"[U_rep] cells within D0={self.D0}m of wall: {in_range.sum()} "
                 f"({100.0*in_range.sum()/max(safe_mask.sum(),1):.1f}% of safe area)")
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("U_rep", U_rep, safe_mask))

        # ── Combined field ────────────────────────────────────────────
        U = U_attr + U_rep
        U = np.where(safe_mask, U, np.inf)

        finite_U = U[np.isfinite(U)]
        log.info(_fmt_arr("U = U_attr + U_rep (combined APF)", U, safe_mask))
        log.info(f"[U] finite cells: {np.isfinite(U).sum()}  "
                 f"median={np.median(finite_U):.4f}  "
                 f"95th-pct={np.percentile(finite_U, 95):.4f}")
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("U (combined APF)", U, safe_mask))

        # ── Residual: U_rep / U_attr ratio (where both finite) ────────
        ratio = np.where(
            safe_mask & (U_attr > 1e-6),
            U_rep / U_attr,
            np.nan
        )
        log.info(_fmt_arr("U_rep/U_attr ratio (repulsion dominance)", ratio, safe_mask))
        if self.SHOW_HEATMAPS:
            log.info(_fmt_matrix_heatmap("U_rep/U_attr ratio", ratio, safe_mask))

        return U

    # ══════════════════════════════════════════════════════════════════════════
    #  PHASE 5 — ADAM GRADIENT DESCENT
    # ══════════════════════════════════════════════════════════════════════════

    def adam_optimizer_path(
        self,
        U:         np.ndarray,
        start_col: int,
        start_row: int,
        goal_col:  int,
        goal_row:  int,
        res:       float,
    ) -> list[tuple[float, float]]:
        """
        Walk downhill on U using the Adam update rule.

        Why Adam over vanilla gradient descent?
        ────────────────────────────────────────
        • The APF gradient is computed via finite differences on a discrete grid,
          so the raw gradient direction oscillates as the agent crosses cell
          boundaries — vanilla GD accumulates these directional errors and produces
          a jagged path.
        • Adam's first-moment (m) acts as exponential momentum, smoothing the
          direction over multiple steps.
        • Adam's second-moment (v) scales the step size *per axis* — near steep
          obstacle gradients the large v dampens the step, preventing overshoot.

        Update equations (per axis, shown for col; identical for row):
            g_t  = ∇U / |∇U|                (normalised gradient)
            m_t  = β₁·m_{t-1} + (1−β₁)·g_t  (1st moment / momentum)
            v_t  = β₂·v_{t-1} + (1−β₂)·g_t² (2nd moment / RMSprop)
            m̂_t = m_t / (1−β₁^t)            (bias correction)
            v̂_t = v_t / (1−β₂^t)            (bias correction)
            Δ    = −α · m̂_t / (√v̂_t + ε)   (Adam step)
        """
        log = self.get_logger()
        log.info(_section("PHASE 5 · Adam Gradient Descent"))
        log.info(
            f"[Adam] start=({start_col},{start_row})  goal=({goal_col},{goal_row})  "
            f"α={self.GRAD_STEP}  β₁={self.ADAM_BETA1}  β₂={self.ADAM_BETA2}  "
            f"ε={self.ADAM_EPS}  max_iter={self.MAX_ITER}"
        )

        n_cols, n_rows = U.shape
        path = []

        # ── Snap start to nearest finite cell ─────────────────────────
        finite_mask = np.isfinite(U)
        if not finite_mask.any():
            log.warn('[Adam] U has no finite cells — cannot plan path')
            return path

        if not finite_mask[start_col, start_row]:
            safe_cols, safe_rows = np.where(finite_mask)
            dists = np.hypot(safe_cols - start_col, safe_rows - start_row)
            best  = np.argmin(dists)
            start_col = int(safe_cols[best])
            start_row = int(safe_rows[best])
            log.info(f'[Adam] start snapped to nearest safe cell: ({start_col},{start_row})')

        c = float(start_col)
        r = float(start_row)
        converge_cells = self.CONVERGE_M / res

        # Adam state
        m_c = m_r = 0.0
        v_c = v_r = 0.0
        t = 0

        # Logging accumulators for presentation
        grad_mags    = []
        step_sizes   = []
        m_hat_norms  = []
        local_minima = 0

        for iteration in range(self.MAX_ITER):
            path.append((c, r))

            dist_to_goal = np.hypot(c - goal_col, r - goal_row)
            if dist_to_goal < converge_cells:
                path.append((float(goal_col), float(goal_row)))
                log.info(
                    f"[Adam] ✓ Converged at iteration {iteration}  "
                    f"dist={dist_to_goal*res:.4f}m  path_len={len(path)}"
                )
                break

            t += 1

            # ── Finite-difference gradient ────────────────────────────
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
            grad_mags.append(grad_mag if np.isfinite(grad_mag) else 0.0)

            # ── Normalise gradient (or nudge toward goal if flat) ──────
            if not np.isfinite(grad_mag) or grad_mag < 1e-9:
                # Local minimum / flat region — point directly at goal
                dc, dr = goal_col - c, goal_row - r
                norm   = np.hypot(dc, dr) + 1e-9
                g_c, g_r = -dc / norm, -dr / norm
                local_minima += 1
            else:
                g_c, g_r = dU_dc / grad_mag, dU_dr / grad_mag

            # ── Adam moment updates ────────────────────────────────────
            m_c = self.ADAM_BETA1 * m_c + (1 - self.ADAM_BETA1) * g_c
            m_r = self.ADAM_BETA1 * m_r + (1 - self.ADAM_BETA1) * g_r

            v_c = self.ADAM_BETA2 * v_c + (1 - self.ADAM_BETA2) * g_c**2
            v_r = self.ADAM_BETA2 * v_r + (1 - self.ADAM_BETA2) * g_r**2

            # Bias correction
            beta1_t = self.ADAM_BETA1 ** t
            beta2_t = self.ADAM_BETA2 ** t
            m_hat_c = m_c / (1 - beta1_t)
            m_hat_r = m_r / (1 - beta1_t)
            v_hat_c = v_c / (1 - beta2_t)
            v_hat_r = v_r / (1 - beta2_t)

            # ── Adam step ─────────────────────────────────────────────
            dc_step = self.GRAD_STEP * m_hat_c / (np.sqrt(v_hat_c) + self.ADAM_EPS)
            dr_step = self.GRAD_STEP * m_hat_r / (np.sqrt(v_hat_r) + self.ADAM_EPS)

            step_size = np.hypot(dc_step, dr_step)
            step_sizes.append(step_size)
            m_hat_norms.append(np.hypot(m_hat_c, m_hat_r))

            c -= dc_step
            r -= dr_step
            c = float(np.clip(c, 0, n_cols - 1))
            r = float(np.clip(r, 0, n_rows - 1))

            # ── Periodic iteration log (for presentation) ─────────────
            if self.ADAM_LOG_EVERY > 0 and (iteration % self.ADAM_LOG_EVERY == 0):
                log.info(
                    f"  [Adam iter {iteration:4d}]  "
                    f"pos=({c:.2f},{r:.2f})  "
                    f"dist_goal={dist_to_goal*res:.3f}m  "
                    f"|∇U|={grad_mag:.5f}  "
                    f"|step|={step_size:.4f}  "
                    f"m=({m_c:.4f},{m_r:.4f})  "
                    f"v=({v_c:.6f},{v_r:.6f})  "
                    f"m̂=({m_hat_c:.4f},{m_hat_r:.4f})  "
                    f"local_minima_so_far={local_minima}"
                )
        else:
            log.warn(f'[Adam] ✗ MAX_ITER={self.MAX_ITER} reached without convergence')

        # ── Post-descent diagnostics ───────────────────────────────────
        grad_mags  = np.array(grad_mags)
        step_sizes = np.array(step_sizes)
        m_hat_norms = np.array(m_hat_norms)

        log.info("\n" + "─"*60)
        log.info(f"[Adam summary] total waypoints  : {len(path)}")
        log.info(f"[Adam summary] local_minima hits: {local_minima}")
        log.info(f"[Adam summary] |∇U|   — mean={grad_mags.mean():.5f}  "
                 f"min={grad_mags.min():.5f}  max={grad_mags.max():.5f}  "
                 f"zeros={np.sum(grad_mags < 1e-9)}")
        log.info(f"[Adam summary] |step| — mean={step_sizes.mean():.4f}  "
                 f"min={step_sizes.min():.4f}  max={step_sizes.max():.4f}")
        log.info(f"[Adam summary] |m̂|   — mean={m_hat_norms.mean():.4f}  "
                 f"(momentum magnitude over path)")

        # Path bounding box in grid space
        if path:
            pc = [p[0] for p in path]; pr = [p[1] for p in path]
            log.info(
                f"[Adam summary] path bounding box: "
                f"col=[{min(pc):.1f},{max(pc):.1f}]  "
                f"row=[{min(pr):.1f},{max(pr):.1f}]"
            )

        return path

    # ══════════════════════════════════════════════════════════════════════════
    #  PHASE 6 — HEIGHT SMOOTHING
    # ══════════════════════════════════════════════════════════════════════════

    def sample_smooth_height(
        self,
        path:     list[tuple[float, float]],
        opt_z:    np.ndarray,
        floor:    np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Sample opt_z at each waypoint (nearest-cell), then apply a 1-D Gaussian
        with σ=HEIGHT_SIGMA cells to yield a smooth body-height profile.

        Why smooth?  The per-cell opt_z field is piecewise-constant; following it
        directly would cause abrupt posture jumps.  The Gaussian low-pass filter
        ensures the robot's height changes gradually between waypoints.
        """
        log = self.get_logger()
        log.info(_section("PHASE 6 · Height Trajectory Smoothing"))
        log.info(f"[height] σ={self.HEIGHT_SIGMA} cells  path_len={len(path)}")

        n_cols, n_rows = opt_z.shape
        raw_heights   = []
        floor_heights = []

        for (c, r) in path:
            ci = int(np.clip(round(c), 0, n_cols - 1))
            ri = int(np.clip(round(r), 0, n_rows - 1))
            z  = opt_z[ci, ri]
            f  = floor[ci, ri]
            if np.isnan(z) and raw_heights:
                z = raw_heights[-1]
            elif np.isnan(z):
                z = self.Z_MIN
            raw_heights.append(z)
            floor_heights.append(f if not np.isnan(f) else 0.0)

        raw_heights   = np.array(raw_heights,   dtype=np.float32)
        floor_heights = np.array(floor_heights, dtype=np.float32)

        # ── Debug: raw profile ─────────────────────────────────────────
        log.info(f"[height raw]    min={raw_heights.min():.4f}  "
                 f"max={raw_heights.max():.4f}  "
                 f"mean={raw_heights.mean():.4f}  "
                 f"std={raw_heights.std():.4f}  "
                 f"NaN_count={np.isnan(raw_heights).sum()}")

        # Count posture-state transitions (crouching ↔ standing)
        CROUCH_THRESH = 0.25
        states     = (raw_heights >= CROUCH_THRESH).astype(int)
        transitions = int(np.sum(np.abs(np.diff(states))))
        log.info(f"[height raw]    crouch(<{CROUCH_THRESH}m): "
                 f"{(raw_heights < CROUCH_THRESH).sum()} waypoints  "
                 f"stand(≥{CROUCH_THRESH}m): {(raw_heights >= CROUCH_THRESH).sum()} waypoints  "
                 f"transitions: {transitions}")

        # Gaussian smooth
        heights_smooth = gaussian_filter1d(raw_heights, sigma=self.HEIGHT_SIGMA)
        heights_smooth = np.clip(heights_smooth, self.Z_MIN, self.Z_MAX)

        # ── Debug: smoothed profile ────────────────────────────────────
        delta = heights_smooth - raw_heights
        log.info(f"[height smooth] min={heights_smooth.min():.4f}  "
                 f"max={heights_smooth.max():.4f}  "
                 f"mean={heights_smooth.mean():.4f}  "
                 f"std={heights_smooth.std():.4f}")
        log.info(f"[height smooth] smoothing Δ — "
                 f"mean_abs={np.abs(delta).mean():.4f}  "
                 f"max_abs={np.abs(delta).max():.4f}  "
                 f"(how much the filter moved the profile)")

        # ── ASCII height profile (side view of posture along path) ────
        if self.SHOW_HEATMAPS and len(heights_smooth) > 1:
            bar_width = min(len(heights_smooth), 70)
            step_w    = max(1, len(heights_smooth) // bar_width)
            sampled   = heights_smooth[::step_w]
            lo_h, hi_h = self.Z_MIN, self.Z_MAX
            H_ROWS = 8
            log.info("\n╔══ Body Height Profile (along path, raw=· smooth=█) ══╗")
            for lvl in range(H_ROWS, -1, -1):
                thresh = lo_h + (hi_h - lo_h) * lvl / H_ROWS
                raw_s  = raw_heights[::step_w]
                row_s  = "".join("█" if v >= thresh else ("·" if r >= thresh else " ")
                                 for v, r in zip(sampled, raw_s))
                label = f"{thresh:.2f}m"
                log.info(f"║ {label:6s} {row_s} ║")
            log.info("╚" + "═" * (len(sampled) + 11) + "╝")

        # ── Debug: floor heights along path ───────────────────────────
        log.info(f"[floor@path]    min={floor_heights.min():.4f}  "
                 f"max={floor_heights.max():.4f}  "
                 f"mean={floor_heights.mean():.4f}  "
                 f"(terrain elevation under robot CoM)")

        return heights_smooth, floor_heights

    # ══════════════════════════════════════════════════════════════════════════
    #  PUBLISHING
    # ══════════════════════════════════════════════════════════════════════════

    def publish_pointcloud(self, publisher, header, info, values, base_z, mask, is_flat=True):
        if not np.any(mask):
            return
        res    = info.resolution
        cx, cy = info.pose.position.x, info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0
        cols, rows   = np.where(mask)
        world_y      = cy + half_y - (cols + 0.5) * res
        world_x      = cx + half_x - (rows + 0.5) * res
        intensities  = values[cols, rows]
        world_z = np.zeros_like(world_x) if is_flat else base_z[cols, rows] + (intensities / 2.0)
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        points    = np.column_stack((world_x, world_y, world_z, intensities))
        cloud_msg = pc2.create_cloud(header, fields, points)
        publisher.publish(cloud_msg)

    def publish_apf_field(self, header, info, U: np.ndarray, safe_mask: np.ndarray):
        U_vis     = np.where(safe_mask, np.clip(U, 0, np.percentile(U[safe_mask], 95)), np.nan)
        valid_vis = safe_mask & ~np.isnan(U_vis)
        dummy_base = np.zeros_like(U_vis)
        self.publish_pointcloud(
            self.pub_apf_field, header, info, U_vis, dummy_base, valid_vis, is_flat=True)

    def publish_apf_path(self, header, info, path, heights_smooth, floor_heights):
        if not path:
            return
        res    = info.resolution
        cx, cy = info.pose.position.x, info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0
        wx_list, wy_list, wz_list, h_list = [], [], [], []
        for i, (c, r) in enumerate(path):
            wy = cy + half_y - (c + 0.5) * res
            wx = cx + half_x - (r + 0.5) * res
            h  = heights_smooth[i]
            f  = floor_heights[i]
            wx_list.append(wx); wy_list.append(wy)
            wz_list.append(f + h / 2.0); h_list.append(h)
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pts = np.column_stack((wx_list, wy_list, wz_list, h_list)).astype(np.float32)
        self.pub_apf_path.publish(pc2.create_cloud(header, fields, pts))

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN CALLBACK
    # ══════════════════════════════════════════════════════════════════════════

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        log = self.get_logger()
        log.info(_section("PHASE 1 · Map Ingestion"))

        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            log.warn('[Phase 1] Missing elevation layer — skipping')
            return

        res   = floor_msg.info.resolution
        floor = self.get_layer(floor_msg, 'elevation')
        ceil  = self.get_layer(ceil_msg,  'elevation')

        log.info(
            f"[Phase 1] floor shape={floor.shape}  "
            f"ceil shape={ceil.shape}  "
            f"resolution={res:.4f}m/cell  "
            f"map_size=({floor_msg.info.length_x:.2f}m × {floor_msg.info.length_y:.2f}m)  "
            f"origin=({floor_msg.info.pose.position.x:.2f}, "
            f"{floor_msg.info.pose.position.y:.2f})"
        )
        log.info(f"[Phase 1] floor NaN={np.isnan(floor).sum()}  "
                 f"ceil NaN={np.isnan(ceil).sum()}")

        # ── Phase 2 & 3 ───────────────────────────────────────────────
        h_avail, w_avail, f_base, valid = self.compute_clearances(floor, ceil, res)
        opt_z, safe_mask = self.optimize_posture(h_avail, w_avail)

        if np.sum(safe_mask) < 5:
            log.warn('[Callback] Fewer than 5 safe cells — skipping APF planning')
            return

        # ── Goal & start in grid coords ───────────────────────────────
        log.info(_section("PHASE 4-5 · Planning Setup"))

        goal_wx = self.get_parameter('goal_x').value
        goal_wy = self.get_parameter('goal_y').value
        goal_col, goal_row = self.world_to_grid(goal_wx, goal_wy, floor_msg.info)
        goal_col = int(np.clip(goal_col, 0, safe_mask.shape[0] - 1))
        goal_row = int(np.clip(goal_row, 0, safe_mask.shape[1] - 1))

        start_col = safe_mask.shape[0] // 2
        start_row = safe_mask.shape[1] // 2

        log.info(f"[planning] world goal: ({goal_wx:.2f},{goal_wy:.2f})m  "
                 f"→ grid: ({goal_col},{goal_row})")
        log.info(f"[planning] world start: map centre  "
                 f"→ grid: ({start_col},{start_row})")

        goal_in_safe = safe_mask[goal_col, goal_row] if (
            0 <= goal_col < safe_mask.shape[0] and
            0 <= goal_row < safe_mask.shape[1]) else False
        log.info(f"[planning] goal cell is safe: {goal_in_safe}"
                 + ("" if goal_in_safe else "  ← WARNING: goal lands in obstacle!"))

        dist_start_goal_m = np.hypot(start_col - goal_col, start_row - goal_row) * res
        log.info(f"[planning] Euclidean start→goal: {dist_start_goal_m:.3f}m "
                 f"({np.hypot(start_col-goal_col, start_row-goal_row):.1f} cells)")

        # ── Phase 4 ───────────────────────────────────────────────────
        U = self.compute_apf(safe_mask, w_avail, goal_col, goal_row)

        # ── Phase 5 (Adam) ────────────────────────────────────────────
        path = self.adam_optimizer_path(
            U, start_col, start_row, goal_col, goal_row, res)

        # ── Phase 6 ───────────────────────────────────────────────────
        heights_smooth, floor_heights = self.sample_smooth_height(path, opt_z, floor)

        # ── Final summary ─────────────────────────────────────────────
        log.info(_section("PIPELINE COMPLETE"))
        path_len_m = sum(
            np.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) * res
            for i in range(len(path)-1)
        )
        log.info(f"[result] waypoints         : {len(path)}")
        log.info(f"[result] path length       : {path_len_m:.3f}m")
        log.info(f"[result] path/straight ratio: {path_len_m / max(dist_start_goal_m, 1e-6):.3f}  "
                 f"(1.0 = perfectly straight)")
        log.info(f"[result] height range      : "
                 f"{heights_smooth.min():.3f}m – {heights_smooth.max():.3f}m")
        log.info(f"[result] mean body height  : {heights_smooth.mean():.3f}m")

        # ── Publish ───────────────────────────────────────────────────
        self.publish_pointcloud(self.pub_h_avail,   floor_msg.header, floor_msg.info,
                                h_avail,   f_base,    valid,     is_flat=True)
        self.publish_pointcloud(self.pub_w_avail,   floor_msg.header, floor_msg.info,
                                w_avail,   f_base,    valid,     is_flat=True)
        self.publish_pointcloud(self.pub_z_targets, floor_msg.header, floor_msg.info,
                                opt_z,     f_base,    safe_mask, is_flat=False)
        safe_float = safe_mask.astype(np.float32)
        self.publish_pointcloud(self.pub_safe_mask, floor_msg.header, floor_msg.info,
                                safe_float, f_base,   valid,     is_flat=True)
        self.publish_apf_field(floor_msg.header, floor_msg.info, U, safe_mask)
        self.publish_apf_path(floor_msg.header, floor_msg.info, path,
                              heights_smooth, floor_heights)


def main(args=None):
    rclpy.init(args=args)
    node = PostureOptimizerDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
