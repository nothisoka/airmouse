"""15-State Extended Kalman Filter for Sensor Fusion on NIDAR AirMouse.

Fuses:
1. MPU6050 Accelerometer + Gyroscope (100 Hz prediction)
2. Downward Ultrasonic Altitude (20 Hz update)
3. Camera Optical Flow Velocity (20 Hz update)
4. Accelerometer Gravity Reference for Pitch/Roll (100 Hz update)

State vector x (15x1):
[x, y, z, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bgx, bgy, bgz]^T
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple
import numpy as np

from airmouse.core.hal import ImuData, RangeData, CameraFrame, DroneState, PoseQuality
from airmouse.core.estimation.altitude_filter import AltitudeFilter
from airmouse.core.estimation.visual_odometry import VisualOdometry


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz])


class SensorFusionEKF:
    def __init__(self, init_pos: np.ndarray, init_yaw: float = 0.0):
        # 15-state vector:
        # [0:3] pos (x,y,z), [3:6] vel (vx,vy,vz), [6:9] rpy, [9:12] accel_bias, [12:15] gyro_bias
        self.x = np.zeros(15, dtype=np.float64)
        self.x[0:3] = init_pos
        self.x[8] = init_yaw

        # Covariance matrix P (15x15)
        self.P = np.diag([
            0.05, 0.05, 0.05,    # pos
            0.1, 0.1, 0.1,       # vel
            0.02, 0.02, 0.05,    # rpy
            0.05, 0.05, 0.05,    # accel bias
            0.01, 0.01, 0.01,    # gyro bias
        ]).astype(np.float64)

        # Process noise Q
        self.q_pos = 0.001
        self.q_vel = 0.08
        self.q_att = 0.005
        self.q_bax = 0.0001
        self.q_bgx = 0.00002

        self.last_predict_time = -1.0
        self.last_cam_time = -1.0
        self.alt_filter = AltitudeFilter(init_z=float(init_pos[2]))
        self.vo = VisualOdometry()
        self.last_vo_meas: Optional[np.ndarray] = None

        self.g_const = 9.80665

        # Stationary preflight gyroscope bias calibration buffer
        self.calibrated = False
        self.calib_gyro_samples: list[np.ndarray] = []

    def predict(self, imu: ImuData):
        # Auto-calibrate gyroscope bias during initial stationary/takeoff phase
        if not self.calibrated:
            self.calib_gyro_samples.append(imu.gyro_radps.copy())
            if len(self.calib_gyro_samples) >= 100:
                self.x[12:15] = np.mean(self.calib_gyro_samples, axis=0)
                self.calibrated = True

        if self.last_predict_time < 0:
            self.last_predict_time = imu.timestamp_s
            return

        dt = imu.timestamp_s - self.last_predict_time
        self.last_predict_time = imu.timestamp_s
        if dt <= 0.0 or dt > 0.1:
            return

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]
        ba = self.x[9:12]
        bg = self.x[12:15]

        # Corrected gyro rates
        omega = imu.gyro_radps - bg
        # Update attitude
        self.x[6] += omega[0] * dt
        self.x[7] += omega[1] * dt
        self.x[8] = (self.x[8] + omega[2] * dt + math.pi) % (2 * math.pi) - math.pi

        # Body to World rotation matrix
        cy, sy = math.cos(self.x[8]), math.sin(self.x[8])
        cp, sp = math.cos(self.x[7]), math.sin(self.x[7])
        cr, sr = math.cos(self.x[6]), math.sin(self.x[6])

        R_wb = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])

        # Corrected accelerometer in world frame
        accel_body = imu.accel_mps2 - ba
        accel_world = R_wb @ accel_body
        accel_world[2] -= self.g_const  # Subtract gravity

        # Update position: integrate velocity
        self.x[0:3] += self.x[3:6] * dt

        # Vertical velocity integrated from vertical acceleration
        self.x[5] += accel_world[2] * dt
        # Altitude filter prediction
        self.alt_filter.predict(az_world=float(accel_world[2]), dt=dt)

        # Damp horizontal velocity predictions gently between VO updates
        self.x[3] *= 0.995
        self.x[4] *= 0.995

        # Accelerometer gravity reference update for Roll and Pitch
        # When dynamic acceleration is low and hovering, specific force aligns with gravity
        a_mag = np.linalg.norm(imu.accel_mps2)
        v_xy = math.hypot(self.x[3], self.x[4])
        if abs(a_mag - self.g_const) < 0.40 and v_xy < 0.25:
            # Measured pitch & roll from gravity vector in body frame:
            # f_b = R_bw * [0, 0, g]^T = [-g*sp, g*cp*sr, g*cp*cr]
            meas_pitch = math.atan2(-imu.accel_mps2[0], math.sqrt(imu.accel_mps2[1]**2 + imu.accel_mps2[2]**2))
            meas_roll = math.atan2(imu.accel_mps2[1], imu.accel_mps2[2])
            # Gentle complementary update
            alpha = 0.02
            self.x[6] = (1 - alpha) * self.x[6] + alpha * meas_roll
            self.x[7] = (1 - alpha) * self.x[7] + alpha * meas_pitch

        # Covariance propagation (simplified block diagonal)
        Q = np.diag([
            self.q_pos, self.q_pos, self.q_pos,
            self.q_vel, self.q_vel, self.q_vel,
            self.q_att, self.q_att, self.q_att,
            self.q_bax, self.q_bax, self.q_bax,
            self.q_bgx, self.q_bgx, self.q_bgx,
        ]) * dt
        self.P += Q

    def update_sonar(self, ranges: Dict[str, RangeData]):
        if "down" not in ranges or not ranges["down"].valid:
            return

        r_down = ranges["down"].range_m
        roll, pitch = self.x[6], self.x[7]
        self.alt_filter.update_sonar(r_down, roll, pitch)

        # Directly update EKF z and vz from altitude filter
        z_filtered = self.alt_filter.altitude
        vz_filtered = self.alt_filter.vertical_velocity

        H = np.zeros((2, 15))
        H[0, 2] = 1.0  # z
        H[1, 5] = 1.0  # vz
        z_meas = np.array([z_filtered, vz_filtered])
        R = np.diag([0.02**2, 0.05**2])

        y = z_meas - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(15) - K @ H) @ self.P

    def update_camera(self, cam: CameraFrame, gyro_radps: np.ndarray):
        # Inhibit horizontal optical flow during vertical takeoff climb (zoom scale artifact)
        vz = self.alt_filter.vertical_velocity
        alt = float(self.x[2])
        if vz > 0.10 and alt < 1.05:
            self.x[3] = 0.0
            self.x[4] = 0.0
            self.vo.reset()
            return

        # Ignore horizontal VO only during extreme vertical maneuvers or crashes
        if abs(vz) > 0.8:
            self.vo.reset()
            return

        alt = max(0.2, float(self.x[2]))
        current_rpy = self.x[6:9].copy()
        corrected_gyro = (gyro_radps - self.x[12:15]) if gyro_radps is not None else None
        vel_body, valid = self.vo.process_frame(
            cam.image_bgr, cam.timestamp_s, alt, gyro_radps=corrected_gyro, current_rpy=current_rpy
        )
        if not valid or vel_body is None:
            return

        self.last_vo_meas = vel_body.copy()

        # Transform body velocity [vx_body, vy_body] to world frame [vx_world, vy_world]
        yaw = self.x[8]
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Body +X is forward, body +Y is left
        v_world_x = vel_body[0] * cos_y - vel_body[1] * sin_y
        v_world_y = vel_body[0] * sin_y + vel_body[1] * cos_y

        # Velocity innovation: difference between optical flow and predicted velocity
        # Physical acceleration limit gating: drone max accel is 1.5 m/s^2.
        # Limit frame-to-frame velocity jump to prevent camera glitches/outliers from throwing EKF
        dt_cam = cam.timestamp_s - self.last_cam_time if self.last_cam_time > 0 else 0.05
        self.last_cam_time = cam.timestamp_s
        max_dvel = max(0.35, 4.0 * dt_cam)

        vel_innov_x = float(np.clip(v_world_x - self.x[3], -max_dvel, max_dvel))
        vel_innov_y = float(np.clip(v_world_y - self.x[4], -max_dvel, max_dvel))

        # Tightly update velocity from visual odometry
        self.x[3] += 0.85 * vel_innov_x
        self.x[4] += 0.85 * vel_innov_y

        # Stable accelerometer bias compensation from smoothed velocity residual
        ba_world_x = 0.01 * vel_innov_x
        ba_world_y = 0.01 * vel_innov_y
        self.x[9] -= (ba_world_x * cos_y + ba_world_y * sin_y)
        self.x[10] -= (-ba_world_x * sin_y + ba_world_y * cos_y)

    def update_scan_to_map(self, ranges: Dict[str, RangeData], grid_map):
        """Scan-to-map EKF measurement update using sonar ranges against the online occupancy grid."""
        est_x, est_y = float(self.x[0]), float(self.x[1])
        est_yaw = float(self.x[8])

        yaw_offsets = {
            "front": 0.0,
            "left": math.pi / 2.0,
            "right": -math.pi / 2.0,
            "rear": math.pi,
        }

        H_rows = []
        innovations = []

        res = grid_map.res
        search_radius_cells = 3  # +/- 0.75m search window

        for sid, offset in yaw_offsets.items():
            if sid not in ranges or not ranges[sid].valid:
                continue

            r_meas = ranges[sid].range_m
            # Only use reliable reflections from walls within range
            if r_meas >= 3.6 or r_meas <= 0.35:
                continue

            beam_angle = est_yaw + offset
            cos_b = math.cos(beam_angle)
            sin_b = math.sin(beam_angle)

            # Expected hit point in world frame if pose was exact
            x_hit = est_x + r_meas * cos_b
            y_hit = est_y + r_meas * sin_b

            gx0, gy0 = grid_map.world_to_grid(x_hit, y_hit)

            # Search in a local neighborhood for established occupied wall cells
            best_dist = 1e9
            best_wx, best_wy = None, None

            for dy in range(-search_radius_cells, search_radius_cells + 1):
                gy = gy0 + dy
                for dx in range(-search_radius_cells, search_radius_cells + 1):
                    gx = gx0 + dx
                    if grid_map.in_bounds(gx, gy) and grid_map.is_occupied(gx, gy):
                        wx = (gx + 0.5) * res + grid_map.origin_x
                        wy = (gy + 0.5) * res + grid_map.origin_y
                        d_pt = math.hypot(wx - x_hit, wy - y_hit)
                        if d_pt < best_dist:
                            best_dist = d_pt
                            best_wx, best_wy = wx, wy

            # If a corresponding map wall was found within search gate
            if best_wx is not None and best_dist <= 0.85:
                # Radial distance error along beam
                d_radial = (best_wx - x_hit) * cos_b + (best_wy - y_hit) * sin_b
                innov = -d_radial

                # Measurement Jacobian for 15-state vector:
                h_row = np.zeros(15, dtype=np.float64)
                h_row[0] = -cos_b
                h_row[1] = -sin_b
                h_row[8] = -r_meas * math.sin(offset) * 0.1  # mild yaw leverage

                H_rows.append(h_row)
                innovations.append(innov)

        if not H_rows:
            return

        H = np.array(H_rows, dtype=np.float64)
        y = np.array(innovations, dtype=np.float64)
        m = len(innovations)

        # Measurement noise covariance R (independent sonar beams)
        R = np.eye(m, dtype=np.float64) * (0.04**2)

        # Kalman innovation covariance S = H P H^T + R
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        # Kalman gain K = P H^T S^-1 (15 x m)
        K = self.P @ H.T @ S_inv

        # State correction dx = K y
        dx = K @ y

        # Clamp max single-step correction for physical trajectory continuity
        dx[0] = np.clip(dx[0], -0.15, 0.15)
        dx[1] = np.clip(dx[1], -0.15, 0.15)
        dx[8] = np.clip(dx[8], -0.05, 0.05)

        self.x += dx
        # Keep yaw in [-pi, pi]
        self.x[8] = (self.x[8] + math.pi) % (2.0 * math.pi) - math.pi

        # Joseph form covariance update: P = (I - K H) P (I - K H)^T + K R K^T
        I_15 = np.eye(15, dtype=np.float64)
        IKH = I_15 - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def _raycast_map(self, x0: float, y0: float, angle: float, grid_map, max_d: float = 4.0):
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        step = 0.05
        d = step
        while d < max_d:
            rx = x0 + d * cos_a
            ry = y0 + d * sin_a
            gx, gy = grid_map.world_to_grid(rx, ry)
            if not grid_map.in_bounds(gx, gy):
                return None
            if grid_map.is_occupied(gx, gy):
                return d
            d += step
        return None

    def get_state(self, timestamp_s: float) -> DroneState:
        # Determine pose quality from covariance trace of position
        pos_cov_trace = float(np.sum(np.diag(self.P)[0:3]))
        if pos_cov_trace < 0.35:
            quality = PoseQuality.EXCELLENT
        elif pos_cov_trace < 1.0:
            quality = PoseQuality.DEGRADED
        else:
            quality = PoseQuality.CRITICAL

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]
        quat = euler_to_quat(roll, pitch, yaw)

        return DroneState(
            timestamp_s=timestamp_s,
            position=self.x[0:3].copy(),
            velocity=self.x[3:6].copy(),
            attitude_rpy=self.x[6:9].copy(),
            quaternion=quat,
            accel_bias=self.x[9:12].copy(),
            gyro_bias=self.x[12:15].copy(),
            covariance_diag=np.diag(self.P).copy(),
            quality=quality,
        )
