"""2D Lidar Scan Matcher for NIDAR AirMouse.

Aligns planar 2D lidar scans (LidarScan) against the current occupancy grid map
(OccupancyGrid) using a hybrid correlative coarse search and continuous
point-to-grid Levenberg-Marquardt (LM) optimization with robust Huber loss on
the Euclidean distance field.

Returns:
- Pose correction (dx, dy, dyaw) relative to initial pose
- Match score in [0.0, 1.0]
- 3x3 Covariance matrix of the matched pose
- Geometric degeneracy detection (corridors / featureless environments)
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple, Union
import numpy as np
import scipy.ndimage

from airmouse.core.hal import LidarScan
from airmouse.core.mapping.occupancy_grid import OccupancyGrid


def _wrap_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class ScanMatchResult:
    """Result of 2D scan matching against an occupancy grid."""
    dx: float               # Estimated x correction (x_matched - init_x) [m]
    dy: float               # Estimated y correction (y_matched - init_y) [m]
    dyaw: float             # Estimated yaw correction (yaw_matched - init_yaw) [rad]
    score: float            # Match score in [0.0, 1.0]
    cov: np.ndarray         # 3x3 covariance matrix of pose [x, y, yaw]
    valid: bool             # True if match passed score and geometry criteria
    matched_x: float        # Absolute matched x position [m]
    matched_y: float        # Absolute matched y position [m]
    matched_yaw: float      # Absolute matched yaw heading [rad]
    reject_reason: Optional[str] = None  # Reason if rejected
    iterations: int = 0     # Optimization iterations performed
    inlier_ratio: float = 0.0 # Fraction of scan points landing near obstacles

    @property
    def correction(self) -> Tuple[float, float, float]:
        """Returns (dx, dy, dyaw)."""
        return (self.dx, self.dy, self.dyaw)

    @property
    def pose(self) -> Tuple[float, float, float]:
        """Returns (matched_x, matched_y, matched_yaw)."""
        return (self.matched_x, self.matched_y, self.matched_yaw)


class ScanMatcher:
    """2D Scan Matcher using Correlative Search + Point-to-Grid LM Optimization.

    Features:
    1. Distance Transform of OccupancyGrid obstacle boundary.
    2. Multi-hypothesis coarse correlative search in (x, y, yaw) window when needed.
    3. Continuous bilinear interpolation Levenberg-Marquardt with Huber robust loss.
    4. Fisher Information Matrix Hessian for 3x3 covariance estimation.
    5. Eigenvalue degeneracy detection for featureless corridors.
    """

    def __init__(
        self,
        min_score: float = 0.65,
        min_inliers: int = 15,
        inlier_dist_m: float = 0.25,
        min_trans_eig: float = 0.5,
        max_condition_number: float = 6.0,
        range_noise_sigma: float = 0.02,
        score_sigma_m: float = 0.15,
        max_iterations: int = 20,
        coarse_search_radius_m: float = 0.80,
        coarse_search_yaw_rad: float = 0.15,
        min_valid_points: int = 15,
    ):
        self.min_score = min_score
        self.min_inliers = min_inliers
        self.inlier_dist_m = inlier_dist_m
        self.min_trans_eig = min_trans_eig
        self.max_condition_number = max_condition_number
        self.range_noise_sigma = range_noise_sigma
        self.score_sigma_m = score_sigma_m
        self.max_iterations = max_iterations
        self.coarse_search_radius_m = coarse_search_radius_m
        self.coarse_search_yaw_rad = coarse_search_yaw_rad
        self.min_valid_points = min_valid_points

        # Cache for Distance Transform
        self._cached_grid_ref = None
        self._cached_grid_version = None
        self._cached_dist = None
        self._cached_gx = None
        self._cached_gy = None

    def _prepare_distance_field(self, grid_map: OccupancyGrid):
        """Compute or retrieve cached Euclidean distance transform and gradients."""
        grid_version = (grid_map.rows, grid_map.cols, float(np.sum(grid_map.log_odds)))
        if (
            self._cached_grid_ref is grid_map
            and self._cached_grid_version == grid_version
            and self._cached_dist is not None
        ):
            return self._cached_dist, self._cached_gx, self._cached_gy

        occ_mask = (grid_map.log_odds > 0.0)
        num_occ = int(np.count_nonzero(occ_mask))
        if num_occ < self.min_valid_points:
            return None, None, None

        res = grid_map.res
        dist = scipy.ndimage.distance_transform_edt(~occ_mask) * res
        gy, gx = np.gradient(dist, res)

        self._cached_grid_ref = grid_map
        self._cached_grid_version = grid_version
        self._cached_dist = dist
        self._cached_gx = gx
        self._cached_gy = gy

        return dist, gx, gy

    def match(
        self,
        scan: LidarScan,
        grid_map: OccupancyGrid,
        initial_pose: Optional[Union[Tuple[float, float, float], np.ndarray]] = None,
        init_x: Optional[float] = None,
        init_y: Optional[float] = None,
        init_yaw: Optional[float] = None,
    ) -> ScanMatchResult:
        """Match scan to grid map starting from initial pose."""
        if initial_pose is not None:
            x0, y0, yaw0 = float(initial_pose[0]), float(initial_pose[1]), float(initial_pose[2])
        elif init_x is not None and init_y is not None and init_yaw is not None:
            x0, y0, yaw0 = float(init_x), float(init_y), float(init_yaw)
        else:
            raise ValueError("Must provide either initial_pose or (init_x, init_y, init_yaw)")

        default_cov = np.diag([10.0, 10.0, 1.0]).astype(np.float64)

        # 1. Extract and decimate valid lidar points
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        r_min = float(scan.range_min)
        r_max = float(scan.range_max)
        valid = (ranges >= r_min) & (ranges < r_max - 0.05) & ~np.isnan(ranges)
        valid_indices = np.where(valid)[0]

        if len(valid_indices) < self.min_valid_points:
            return ScanMatchResult(
                dx=0.0, dy=0.0, dyaw=0.0,
                score=0.0, cov=default_cov, valid=False,
                matched_x=x0, matched_y=y0, matched_yaw=yaw0,
                reject_reason="insufficient_scan_points",
            )

        # Stride to ~90-120 rays for fast, high-accuracy optimization
        stride = max(1, len(valid_indices) // 100)
        selected_idx = valid_indices[::stride]
        angles = scan.angle_min + selected_idx * scan.angle_increment
        sel_ranges = ranges[selected_idx]
        pts_b = np.column_stack([
            sel_ranges * np.cos(angles),
            sel_ranges * np.sin(angles),
        ])

        # 2. Get distance field and gradients
        dist, gx, gy = self._prepare_distance_field(grid_map)
        if dist is None:
            return ScanMatchResult(
                dx=0.0, dy=0.0, dyaw=0.0,
                score=0.0, cov=default_cov, valid=False,
                matched_x=x0, matched_y=y0, matched_yaw=yaw0,
                reject_reason="empty_or_sparse_map",
            )

        res = grid_map.res
        origin_x = grid_map.origin_x
        origin_y = grid_map.origin_y
        cols = grid_map.cols
        rows = grid_map.rows

        # 3. Robust residual & Jacobian function with Huber weighting
        def compute_residuals_and_jacobian(x: float, y: float, yaw: float):
            c = math.cos(yaw)
            s = math.sin(yaw)
            px_w = x + c * pts_b[:, 0] - s * pts_b[:, 1]
            py_w = y + s * pts_b[:, 0] + c * pts_b[:, 1]

            r_coords = ((py_w - origin_y) / res) - 0.5
            c_coords = ((px_w - origin_x) / res) - 0.5
            coords = np.vstack([r_coords, c_coords])

            r_raw = scipy.ndimage.map_coordinates(dist, coords, order=1, mode="nearest")
            dx_vals = scipy.ndimage.map_coordinates(gx, coords, order=1, mode="nearest")
            dy_vals = scipy.ndimage.map_coordinates(gy, coords, order=1, mode="nearest")

            d_rot_x = -s * pts_b[:, 0] - c * pts_b[:, 1]
            d_rot_y =  c * pts_b[:, 0] - s * pts_b[:, 1]
            d_yaw_vals = dx_vals * d_rot_x + dy_vals * d_rot_y

            J_raw = np.column_stack([dx_vals, dy_vals, d_yaw_vals])

            # Huber robust weights: points in open space / unmapped territory (>0.40m) get 0 weight
            abs_r = np.abs(r_raw)
            k_huber = 0.20
            weights = np.ones_like(r_raw)
            soft_mask = (abs_r > k_huber) & (abs_r <= 0.40)
            weights[soft_mask] = k_huber / abs_r[soft_mask]
            weights[abs_r > 0.40] = 0.0

            sqrt_w = np.sqrt(weights)
            r_weighted = r_raw * sqrt_w
            J_weighted = J_raw * sqrt_w[:, None]

            return r_raw, r_weighted, J_weighted

        # 4. LM Optimization and Validation routine
        def run_lm(start_x: float, start_y: float, start_yaw: float) -> ScanMatchResult:
            cur_x, cur_y, cur_yaw = start_x, start_y, start_yaw
            r_raw_cur, r_cur, J_cur = compute_residuals_and_jacobian(cur_x, cur_y, cur_yaw)
            cur_cost = float(np.sum(r_cur ** 2))
            lam = 1.0
            iters_done = 0

            for it in range(self.max_iterations):
                iters_done = it + 1
                H = J_cur.T @ J_cur
                g = J_cur.T @ r_cur

                try:
                    step = np.linalg.solve(H + lam * np.eye(3), -g)
                except np.linalg.LinAlgError:
                    break

                cand_x = cur_x + float(step[0])
                cand_y = cur_y + float(step[1])
                cand_yaw = _wrap_angle(cur_yaw + float(step[2]))

                cand_r_raw, cand_r, cand_J = compute_residuals_and_jacobian(cand_x, cand_y, cand_yaw)
                cand_cost = float(np.sum(cand_r ** 2))

                if cand_cost < cur_cost:
                    lam = max(lam * 0.5, 1e-4)
                    cur_x, cur_y, cur_yaw = cand_x, cand_y, cand_yaw
                    r_raw_cur, r_cur, J_cur = cand_r_raw, cand_r, cand_J
                    cur_cost = cand_cost
                    if np.linalg.norm(step[:2]) < 1e-4 and abs(step[2]) < 1e-4:
                        break
                else:
                    lam = min(lam * 2.0, 1e4)

            # Metrics & Validation
            point_distances = np.abs(r_raw_cur)
            inliers = point_distances <= self.inlier_dist_m
            num_inliers = int(np.count_nonzero(inliers))
            inlier_ratio = float(num_inliers) / len(pts_b)

            if num_inliers < self.min_inliers:
                return ScanMatchResult(
                    dx=0.0, dy=0.0, dyaw=0.0,
                    score=0.0, cov=default_cov, valid=False,
                    matched_x=x0, matched_y=y0, matched_yaw=yaw0,
                    reject_reason=f"low_inlier_count ({num_inliers} < {self.min_inliers})",
                    iterations=iters_done,
                    inlier_ratio=inlier_ratio,
                )

            # Match score evaluated on inliers
            inlier_dists = point_distances[inliers]
            scores = np.exp(-(inlier_dists ** 2) / (2.0 * (self.score_sigma_m ** 2)))
            match_score = float(np.mean(scores))

            # Hessian computed from inliers
            J_inliers = J_cur[inliers]
            H_final = J_inliers.T @ J_inliers
            J_xy = J_inliers[:, :2]
            H_xy = J_xy.T @ J_xy
            trans_eigvals = np.sort(np.linalg.eigvalsh(H_xy))

            eps = 1e-3
            cov = (self.range_noise_sigma ** 2) * np.linalg.inv(H_final + eps * np.eye(3))

            min_eig = float(trans_eigvals[0])
            max_eig = float(trans_eigvals[1])
            condition_number = max_eig / max(min_eig, 1e-6)

            dx = cur_x - x0
            dy = cur_y - y0
            dyaw = _wrap_angle(cur_yaw - yaw0)

            valid = True
            reject_reason = None

            if match_score < self.min_score:
                valid = False
                reject_reason = f"low_match_score ({match_score:.2f} < {self.min_score:.2f})"
            elif min_eig < self.min_trans_eig or condition_number > self.max_condition_number:
                valid = False
                reject_reason = f"degenerate_geometry (min_eig={min_eig:.1f}, cond={condition_number:.1f})"

            return ScanMatchResult(
                dx=dx,
                dy=dy,
                dyaw=dyaw,
                score=match_score,
                cov=cov,
                valid=valid,
                matched_x=cur_x,
                matched_y=cur_y,
                matched_yaw=cur_yaw,
                reject_reason=reject_reason,
                iterations=iters_done,
                inlier_ratio=inlier_ratio,
            )

        # Step A: Local continuous LM directly from initial pose
        local_res = run_lm(x0, y0, yaw0)
        if local_res.valid:
            return local_res

        if local_res.reject_reason and ("degenerate" in local_res.reject_reason or "low_inlier" in local_res.reject_reason):
            return local_res

        if local_res.inlier_ratio < 0.25:
            return local_res

        # Step B: If local LM failed or had low score, try coarse correlative search (5x5x3)
        dx_candidates = np.linspace(-self.coarse_search_radius_m, self.coarse_search_radius_m, 5)
        dy_candidates = np.linspace(-self.coarse_search_radius_m, self.coarse_search_radius_m, 5)
        dyaw_candidates = np.linspace(-self.coarse_search_yaw_rad, self.coarse_search_yaw_rad, 3)

        best_coarse_cost = float("inf")
        best_coarse_pose = (x0, y0, yaw0)

        for dyaw_c in dyaw_candidates:
            yaw_c = _wrap_angle(yaw0 + dyaw_c)
            cos_y = math.cos(yaw_c)
            sin_y = math.sin(yaw_c)
            rot_x = cos_y * pts_b[:, 0] - sin_y * pts_b[:, 1]
            rot_y = sin_y * pts_b[:, 0] + cos_y * pts_b[:, 1]

            for dx_c in dx_candidates:
                cand_px = x0 + dx_c + rot_x
                col_indices = np.clip(np.round((cand_px - origin_x) / res - 0.5).astype(int), 0, cols - 1)

                for dy_c in dy_candidates:
                    cand_py = y0 + dy_c + rot_y
                    row_indices = np.clip(np.round((cand_py - origin_y) / res - 0.5).astype(int), 0, rows - 1)

                    pt_dists = dist[row_indices, col_indices]
                    trunc_dists = np.minimum(pt_dists, 0.40)
                    cost = float(np.sum(trunc_dists ** 2))
                    if cost < best_coarse_cost:
                        best_coarse_cost = cost
                        best_coarse_pose = (x0 + dx_c, y0 + dy_c, yaw_c)

        coarse_res = run_lm(best_coarse_pose[0], best_coarse_pose[1], best_coarse_pose[2])
        if coarse_res.valid:
            return coarse_res
        return local_res
