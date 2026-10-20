"""Monocular Geometric Ray Back-Projection onto Ground Plane."""
from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np


class GroundRayProjector:
    def __init__(
        self,
        focal_length_px: float = 228.5,
        img_w: int = 320,
        img_h: int = 240,
        camera_pitch_down_deg: float = 12.0,
    ):
        self.fx = focal_length_px
        self.fy = focal_length_px
        self.cx = img_w / 2.0
        self.cy = img_h / 2.0
        self.cam_pitch_down = math.radians(camera_pitch_down_deg)

    def project_pixel_to_ground(
        self,
        pixel_u: float,
        pixel_v: float,
        drone_pos_world: np.ndarray,
        drone_yaw: float,
        drone_pitch: float = 0.0,
        drone_roll: float = 0.0,
    ) -> Optional[Tuple[float, float]]:
        """Back-project image pixel (u, v) to world ground coordinate (X, Y, 0).
        
        Returns:
            (wx, wy) on ground, or None if ray points above horizon or too far.
        """
        zd = float(drone_pos_world[2])
        if zd <= 0.05:
            return None

        # Normalized ray in camera frame (Z forward, X right, Y down)
        x_c = (pixel_u - self.cx) / self.fx
        y_c = (pixel_v - self.cy) / self.fy
        z_c = 1.0

        # Rotate from camera frame to drone body frame:
        # Camera tilted down by total_pitch around body Y axis
        # Body frame: +X forward, +Y left, +Z up
        # In Camera frame: +Z is forward, +X is right (-Y body), +Y is down (-Z body)
        total_pitch = drone_pitch + self.cam_pitch_down
        cp, sp = math.cos(total_pitch), math.sin(total_pitch)

        # Un-tilt ray into body frame:
        # forward axis in body (+X)
        r_body_x = z_c * cp - y_c * sp
        # left axis in body (+Y)
        r_body_y = -x_c
        # up axis in body (+Z)
        r_body_z = -z_c * sp - y_c * cp

        # Rotate from body frame to world frame using yaw and roll
        cy, sy = math.cos(drone_yaw), math.sin(drone_yaw)
        r_world_x = r_body_x * cy - r_body_y * sy
        r_world_y = r_body_x * sy + r_body_y * cy
        r_world_z = r_body_z

        # Intersect ray with ground z=0:
        # z(t) = zd + t * r_world_z = 0 -> t = -zd / r_world_z
        if r_world_z >= -0.05:
            # Ray points parallel to ground or upward; no ground intersection
            return None

        t = -zd / r_world_z
        if t <= 0.2 or t > 10.0:  # Far clip threshold
            return None

        wx = float(drone_pos_world[0] + t * r_world_x)
        wy = float(drone_pos_world[1] + t * r_world_y)

        return wx, wy

