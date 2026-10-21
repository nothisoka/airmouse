"""Optical Flow Visual Odometry for NIDAR AirMouse.

Estimates 2D body velocities from sparse Lucas-Kanade optical flow, scaled
by downward ultrasonic altitude and compensated for camera rotation.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple
import cv2
import numpy as np


class VisualOdometry:
    def __init__(
        self,
        focal_length_px: float = 228.5,
        img_w: int = 320,
        img_h: int = 240,
        camera_pitch_down_deg: float = 12.0,
        max_corners: int = 60,
        quality_level: float = 0.03,
        min_distance: float = 6.0,
    ):
        self.focal_length = focal_length_px
        self.w = img_w
        self.h = img_h
        self.cam_pitch = math.radians(camera_pitch_down_deg)
        self.cos_p = math.cos(self.cam_pitch)
        self.sin_p = math.sin(self.cam_pitch)
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance

        self.K = np.array([
            [self.focal_length, 0.0, self.w / 2.0],
            [0.0, self.focal_length, self.h / 2.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)

        p = self.cam_pitch
        self.R_c_b = np.array([
            [0.0, -1.0, 0.0],
            [-math.sin(p), 0.0, -math.cos(p)],
            [math.cos(p), 0.0, -math.sin(p)]
        ], dtype=np.float64)

        self.prev_gray: Optional[np.ndarray] = None
        self.prev_pts: Optional[np.ndarray] = None
        self.prev_time: float = 0.0
        self.prev_rpy: Optional[np.ndarray] = None

    def reset(self):
        """Reset optical flow tracking history."""
        self.prev_gray = None
        self.prev_pts = None
        self.prev_time = 0.0
        self.prev_rpy = None

    def process_frame(
        self,
        image_bgr: np.ndarray,
        timestamp_s: float,
        altitude_m: float,
        gyro_radps: Optional[np.ndarray] = None,
        current_rpy: Optional[np.ndarray] = None,
    ) -> Tuple[Optional[np.ndarray], bool]:
        """Compute estimated body velocity [vx_body, vy_body] in m/s.
        
        Returns:
            (vel_body, is_valid): vel_body is shape (2,) [vx_forward, vy_left]
        """
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        dt = timestamp_s - self.prev_time

        if self.prev_gray is None or self.prev_pts is None or dt <= 0.005 or dt > 0.5:
            self._detect_features(gray)
            self.prev_gray = gray
            self.prev_time = timestamp_s
            if current_rpy is not None:
                self.prev_rpy = current_rpy.copy()
            return None, False

        # If too few points, refresh features
        if len(self.prev_pts) < 10:
            self._detect_features(self.prev_gray)
            if self.prev_pts is None or len(self.prev_pts) < 8:
                self.prev_gray = gray
                self.prev_time = timestamp_s
                if current_rpy is not None:
                    self.prev_rpy = current_rpy.copy()
                return None, False

        # Track features with Lucas-Kanade
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_pts,
            None,
            winSize=(21, 21),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
        )

        good_prev = self.prev_pts[status == 1]
        good_next = next_pts[status == 1]

        if len(good_prev) < 6:
            self._detect_features(gray)
            self.prev_gray = gray
            self.prev_time = timestamp_s
            if current_rpy is not None:
                self.prev_rpy = current_rpy.copy()
            return None, False

        # Pixel displacement vectors: dx = next_x - prev_x (right), dy = next_y - prev_y (down)
        displacements = good_next - good_prev

        # Relative body attitude change between camera frames
        # Prioritize integrated filtered attitude (current_rpy - prev_rpy) to eliminate single-sample gyro white noise
        if current_rpy is not None and self.prev_rpy is not None:
            delta_rpy = (current_rpy - self.prev_rpy + math.pi) % (2.0 * math.pi) - math.pi
        elif gyro_radps is not None:
            delta_rpy = gyro_radps * dt
        else:
            delta_rpy = np.zeros(3)

        # Exact relative body rotation matrix (Z-Y-X Euler angles)
        d_roll, d_pitch, d_yaw = delta_rpy[0], delta_rpy[1], delta_rpy[2]
        cr, sr = math.cos(d_roll), math.sin(d_roll)
        cp, sp = math.cos(d_pitch), math.sin(d_pitch)
        cy, sy = math.cos(d_yaw), math.sin(d_yaw)
        R_b_rel = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ], dtype=np.float64)

        # Exact camera relative rotation homography: H_rot = K @ R_cam_rel @ K_inv
        R_cam_rel = self.R_c_b @ R_b_rel.T @ self.R_c_b.T
        H_rot = self.K @ R_cam_rel @ self.K_inv

        # De-rotate previous points under camera rotation
        pts_prev_h = np.column_stack([good_prev[:, 0], good_prev[:, 1], np.ones(len(good_prev))])
        pts_rot_h = (H_rot @ pts_prev_h.T).T
        pts_rot_2d = pts_rot_h[:, :2] / np.maximum(1e-6, pts_rot_h[:, 2:3])

        # Exact translational pixel displacement
        pure_trans_disp = good_next - pts_rot_2d
        pure_trans_dx = pure_trans_disp[:, 0]
        pure_trans_dy = pure_trans_disp[:, 1]

        # Filter points to ground region in central floor corridor (v in 140..225, u in 80..w-80)
        # to avoid corridor walls with aperture problem
        u_coords = good_prev[:, 0]
        v_coords = good_prev[:, 1]
        ground_mask = (v_coords >= 140) & (v_coords <= 225) & (u_coords >= 80) & (u_coords <= self.w - 80)
        if np.sum(ground_mask) < 4:
            ground_mask = (v_coords >= 125) & (v_coords <= 235)
            if np.sum(ground_mask) < 4:
                ground_mask = np.ones(len(v_coords), dtype=bool)

        v_sel = v_coords[ground_mask]
        du_sel = pure_trans_dx[ground_mask]
        dv_sel = pure_trans_dy[ground_mask]

        # Analytical perspective projection Jacobian:
        effective_alt = max(0.2, min(altitude_m, 4.0))

        tan_v = (v_sel - (self.h / 2.0)) / self.focal_length
        denom = np.maximum(0.1, self.sin_p + self.cos_p * tan_v)

        # dv relates to body forward motion dx: dv = f * (denom^2) / h * dx
        # du relates to body left motion dy: du = f * denom / h * dy
        vx_estimates = (dv_sel / (self.focal_length * (denom**2) / effective_alt)) / dt
        vy_estimates = (du_sel / (self.focal_length * denom / effective_alt)) / dt
        # Calibrated subpixel Lucas-Kanade response factors with center corridor mask
        calib_factor_x = 1.035
        calib_factor_y = 1.000
        vx_body = float(np.median(vx_estimates)) * calib_factor_x
        vy_body = float(np.median(vy_estimates)) * calib_factor_y

        # Sanity clamp to physical drone velocity limits
        vx_body = float(np.clip(vx_body, -0.85, 0.85))
        vy_body = float(np.clip(vy_body, -0.85, 0.85))

        # Re-detect points for next step if points depleted
        if len(good_next) < 25:
            self._detect_features(gray)
        else:
            self.prev_pts = good_next.reshape(-1, 1, 2)
        self.prev_gray = gray

        self.prev_time = timestamp_s
        if current_rpy is not None:
            self.prev_rpy = current_rpy.copy()
        return np.array([vx_body, vy_body]), True

    def _detect_features(self, gray: np.ndarray):
        # Focus feature detection on the central ground plane (v in 140..225, u in 80..w-80)
        # to avoid horizon and corridor walls (which cause aperture effect on forward motion)
        mask = np.zeros_like(gray)
        mask[140:225, 80:self.w - 80] = 255
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            blockSize=5,
            mask=mask,
        )
        self.prev_pts = pts
