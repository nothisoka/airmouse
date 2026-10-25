"""Log-Odds Occupancy Grid Mapping for NIDAR AirMouse.

Updates map probabilities solely using ultrasonic range measurements and
the drone's estimated pose. Unknown cells start at log-odds 0.0 (prob 0.5).
Supports beam-cone updates, obstacle inflation, and map save/load.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Tuple
import cv2
import numpy as np


class OccupancyGrid:
    def __init__(
        self,
        width_m: float = 30.0,
        height_m: float = 22.0,
        resolution_m: float = 0.25,
        origin_m: Tuple[float, float] = (0.0, 0.0),
        p_occ: float = 0.85,
        p_free: float = 0.35,
        l_min: float = -4.0,
        l_max: float = 4.0,
        occ_thresh: float = 0.65,
        free_thresh: float = 0.35,
    ):
        self.res = resolution_m
        self.origin_x, self.origin_y = origin_m
        self.cols = int(math.ceil(width_m / self.res))
        self.rows = int(math.ceil(height_m / self.res))

        # Log-odds array: 0.0 corresponds to prior probability 0.5 (unknown)
        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)

        self.l_occ = math.log(p_occ / (1.0 - p_occ))
        self.l_free = math.log(p_free / (1.0 - p_free))
        self.l_min = l_min
        self.l_max = l_max
        self.occ_thresh = occ_thresh
        self.free_thresh = free_thresh

        # Precompute circular inflation structuring element (0.30m allows 1-cell doorway clearance)
        self.inflation_radius_m = 0.30

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin_x) / self.res)
        gy = int((wy - self.origin_y) / self.res)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = self.origin_x + (gx + 0.5) * self.res
        wy = self.origin_y + (gy + 0.5) * self.res
        return wx, wy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.cols and 0 <= gy < self.rows

    def update_beam(
        self,
        start_wx: float,
        start_wy: float,
        angle_rad: float,
        range_m: float,
        valid_hit: bool,
        cone_half_angle_rad: float = 0.13,
        num_cone_rays: int = 5,
    ):
        """Update occupancy grid with an ultrasonic cone beam reading."""
        # Cast rays across the beam cone
        for ray_idx in range(num_cone_rays):
            frac = (ray_idx / max(1, num_cone_rays - 1)) - 0.5
            ray_angle = angle_rad + frac * (2 * cone_half_angle_rad)
            self._update_single_ray(start_wx, start_wy, ray_angle, range_m, valid_hit)

    def _update_single_ray(
        self,
        x0_w: float,
        y0_w: float,
        angle: float,
        range_m: float,
        valid_hit: bool,
    ):
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        step = self.res * 0.5
        max_dist = min(range_m, 4.0)

        # Free space along beam
        dist = step
        while dist < max_dist - (self.res * 0.75):
            wx = x0_w + dist * cos_a
            wy = y0_w + dist * sin_a
            gx, gy = self.world_to_grid(wx, wy)
            if not self.in_bounds(gx, gy):
                break
            nval = self.log_odds[gy, gx] + self.l_free
            if nval < self.l_min:
                nval = self.l_min
            elif nval > self.l_max:
                nval = self.l_max
            self.log_odds[gy, gx] = nval
            dist += step

        # Occupied hit cell at beam end (only if valid reflection)
        if valid_hit and range_m < 3.95:
            hit_wx = x0_w + range_m * cos_a
            hit_wy = y0_w + range_m * sin_a
            gx, gy = self.world_to_grid(hit_wx, hit_wy)
            if self.in_bounds(gx, gy):
                nval = self.log_odds[gy, gx] + self.l_occ
                if nval < self.l_min:
                    nval = self.l_min
                elif nval > self.l_max:
                    nval = self.l_max
                self.log_odds[gy, gx] = nval

    def get_probability(self, gx: int, gy: int) -> float:
        if not self.in_bounds(gx, gy):
            return 0.5
        lo = self.log_odds[gy, gx]
        return 1.0 / (1.0 + math.exp(-lo))

    def is_free(self, gx: int, gy: int) -> bool:
        if not self.in_bounds(gx, gy):
            return False
        return self.get_probability(gx, gy) < self.free_thresh

    def is_occupied(self, gx: int, gy: int) -> bool:
        if not self.in_bounds(gx, gy):
            return True
        return self.get_probability(gx, gy) > self.occ_thresh

    def is_unknown(self, gx: int, gy: int) -> bool:
        if not self.in_bounds(gx, gy):
            return True
        p = self.get_probability(gx, gy)
        return self.free_thresh <= p <= self.occ_thresh

    def get_inflated_costmap(self, inflation_radius_m: Optional[float] = None) -> np.ndarray:
        """Returns binary costmap: 0 = traversable free, 1 = obstacle or lethal inflated."""
        if inflation_radius_m is None:
            inflation_radius_m = self.inflation_radius_m

        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        # Raw occupied mask
        occ_mask = (prob > self.occ_thresh).astype(np.uint8)

        # 1. Lethal obstacle dilation: physical drone clearance (radius ~ 0.20m, 1 grid cell)
        lethal_k = int(math.ceil((0.22 / self.res) * 2)) | 1
        lethal_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (lethal_k, lethal_k))
        lethal = cv2.dilate(occ_mask, lethal_kernel)

        # 2. Soft inflation buffer: preferred clearance around walls (0.45m)
        soft_k = int(math.ceil((inflation_radius_m / self.res) * 2)) | 1
        soft_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (soft_k, soft_k))
        soft = cv2.dilate(occ_mask, soft_kernel)

        # Multi-tier costmap: 0 = known free, 25 = unknown (explorable), 50 = soft buffer, 254 = lethal (impassable)
        costmap = np.zeros_like(occ_mask, dtype=np.uint8)
        unknown_mask = (prob >= self.free_thresh) & (prob <= self.occ_thresh)
        costmap[unknown_mask] = 25
        costmap[soft != 0] = 50
        costmap[lethal != 0] = 254

        return costmap

    def save_map(self, pgm_path: Path | str, yaml_path: Optional[Path | str] = None):
        p = Path(pgm_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        # Standard ROS: 254 = free, 0 = occupied, 205 = unknown
        img = np.full((self.rows, self.cols), 205, dtype=np.uint8)
        img[prob < self.free_thresh] = 254
        img[prob > self.occ_thresh] = 0
        # Flip vertically so row 0 is top
        img_flip = np.flipud(img)
        cv2.imwrite(str(p), img_flip)

        if yaml_path:
            y_p = Path(yaml_path)
            y_p.write_text(
                f"image: {p.name}\nresolution: {self.res:.4f}\norigin: [{self.origin_x:.2f}, {self.origin_y:.2f}, 0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.35\n"
            )

