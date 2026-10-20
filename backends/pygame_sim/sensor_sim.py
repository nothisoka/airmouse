"""Realistic Sensor Simulation for MPU6050, Ultrasonic Array, and Camera.

Implements the SensorHub abstract base class using continuous physical state
from DroneDynamics and geometric raycasting against the true world.
Autonomy reads sensors via SensorHub without knowing ground-truth positions.
"""
from __future__ import annotations

import math
import random
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np

from airmouse.core.hal import SensorHub, ImuData, RangeData, CameraFrame
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer


class SimulatedSensorHub(SensorHub):
    def __init__(
        self,
        drone: DroneDynamics,
        renderer: WorldRenderer,
        grid: np.ndarray,
        cell_size_m: float = 1.0,
        imu_rate_hz: float = 100.0,
        range_rate_hz: float = 20.0,
        camera_rate_hz: float = 20.0,
    ):
        self.drone = drone
        self.renderer = renderer
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.cell_size = cell_size_m

        self.imu_period = 1.0 / imu_rate_hz
        self.range_period = 1.0 / range_rate_hz
        self.camera_period = 1.0 / camera_rate_hz

        self.sim_time = 0.0
        self.last_imu_time = -1.0
        self.last_range_time = -1.0
        self.last_camera_time = -1.0

        # MPU6050 noise & bias model
        self.accel_bias = np.random.uniform(-0.04, 0.04, size=3)
        self.gyro_bias = np.random.uniform(-0.015, 0.015, size=3)
        self.accel_noise_density = 0.015       # m/s^2 / sqrt(Hz)
        self.accel_random_walk = 0.0005        # m/s^3 / sqrt(Hz)
        self.gyro_noise_density = 0.005        # rad/s / sqrt(Hz)
        self.gyro_random_walk = 0.0001         # rad/s^2 / sqrt(Hz)

        # Ultrasonic configuration
        self.min_range = 0.02
        self.max_range = 4.0
        self.sonar_noise_std = 0.015
        self.dropout_prob = 0.03
        self.cone_half_angle = math.radians(7.5)  # 15 deg full cone
        self.cone_rays = 5

        # Sensor orientations relative to drone forward: (sensor_id, yaw_offset_rad)
        self.horizontal_sensors = [
            ("front", 0.0),
            ("left", math.pi / 2.0),
            ("right", -math.pi / 2.0),
            ("rear", math.pi),
        ]

        # Cache latest readings
        self.cached_ranges: Dict[str, RangeData] = {}
        self.cached_camera: Optional[CameraFrame] = None

    def advance_time(self, dt: float):
        self.sim_time += dt

    def get_time(self) -> float:
        return self.sim_time

    def read_imu(self) -> Optional[ImuData]:
        if self.sim_time - self.last_imu_time < self.imu_period - 1e-4:
            return None

        dt = self.imu_period if self.last_imu_time > 0 else 0.01
        self.last_imu_time = self.sim_time

        state = self.drone.get_state()
        r, p, y = state.attitude_rpy

        # Update random walk biases
        self.accel_bias += np.random.normal(0.0, self.accel_random_walk * math.sqrt(dt), size=3)
        self.gyro_bias += np.random.normal(0.0, self.gyro_random_walk * math.sqrt(dt), size=3)

        # Rotation matrix: World to Body
        cy, sy = math.cos(y), math.sin(y)
        cp, sp = math.cos(p), math.sin(p)
        cr, sr = math.cos(r), math.sin(r)

        # R_wb (body to world)
        R_wb = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])
        R_bw = R_wb.T

        # Specific force measured by accelerometer in body frame:
        # a_meas = R_bw * (a_world - g_world) + bias + noise
        g_world = np.array([0.0, 0.0, -9.80665])
        true_specific_force = R_bw @ (state.acceleration - g_world)

        accel_noise = np.random.normal(0.0, self.accel_noise_density / math.sqrt(dt), size=3)
        gyro_noise = np.random.normal(0.0, self.gyro_noise_density / math.sqrt(dt), size=3)

        measured_accel = true_specific_force + self.accel_bias + accel_noise
        measured_gyro = state.angular_velocity + self.gyro_bias + gyro_noise

        return ImuData(
            timestamp_s=self.sim_time,
            accel_mps2=measured_accel,
            gyro_radps=measured_gyro,
        )

    def read_ranges(self) -> Dict[str, RangeData]:
        if self.sim_time - self.last_range_time < self.range_period - 1e-4:
            return {}

        self.last_range_time = self.sim_time
        state = self.drone.get_state()
        pos = state.position
        yaw = state.attitude_rpy[2]

        ranges: Dict[str, RangeData] = {}

        # 4 Horizontal sensors with 15 deg beam cone
        for sensor_id, yaw_offset in self.horizontal_sensors:
            center_angle = yaw + yaw_offset
            # Cast multiple rays across cone and take minimum distance (first echo return)
            min_dist = self.max_range
            for ray_idx in range(self.cone_rays):
                frac = (ray_idx / max(1, self.cone_rays - 1)) - 0.5
                ray_angle = center_angle + frac * (2 * self.cone_half_angle)
                dist = self._cast_ray_2d(pos[0], pos[1], ray_angle, self.max_range)
                if dist < min_dist:
                    min_dist = dist

            # Apply dropout and sensor noise
            if random.random() < self.dropout_prob:
                measured = self.max_range
                valid = False
            else:
                noise = np.random.normal(0.0, self.sonar_noise_std)
                measured = float(np.clip(min_dist + noise, self.min_range, self.max_range))
                valid = (measured < self.max_range - 0.01)

            ranges[sensor_id] = RangeData(
                sensor_id=sensor_id,
                range_m=measured,
                valid=valid,
                timestamp_s=self.sim_time,
                cone_half_angle_rad=self.cone_half_angle,
            )

        # 1 Downward ultrasonic sensor (altitude)
        # Returns distance to floor z=0 taking drone pitch/roll into account
        cos_tilt = math.cos(state.attitude_rpy[0]) * math.cos(state.attitude_rpy[1])
        true_alt_range = pos[2] / max(0.2, cos_tilt)
        if random.random() < self.dropout_prob or true_alt_range > self.max_range:
            down_meas = self.max_range
            down_valid = False
        else:
            down_noise = np.random.normal(0.0, self.sonar_noise_std)
            down_meas = float(np.clip(true_alt_range + down_noise, self.min_range, self.max_range))
            down_valid = True

        ranges["down"] = RangeData(
            sensor_id="down",
            range_m=down_meas,
            valid=down_valid,
            timestamp_s=self.sim_time,
            cone_half_angle_rad=self.cone_half_angle,
        )

        self.cached_ranges = ranges
        return ranges

    def read_camera(self) -> Optional[CameraFrame]:
        if self.sim_time - self.last_camera_time < self.camera_period - 1e-4:
            return None

        self.last_camera_time = self.sim_time
        state = self.drone.get_state()
        survivors = self.renderer.boxes  # renderer knows survivor positions internally

        # Get survivors from map metadata via renderer
        frame_bgr = self.renderer.render_camera_frame(
            drone_pos=state.position,
            drone_yaw=state.attitude_rpy[2],
            drone_pitch=state.attitude_rpy[1],
            drone_roll=state.attitude_rpy[0],
            survivors=[s for s in getattr(self.renderer, 'true_survivors', [])],
        )

        self.cached_camera = CameraFrame(
            timestamp_s=self.sim_time,
            image_bgr=frame_bgr,
            width=self.renderer.w,
            height=self.renderer.h,
            fov_h_rad=self.renderer.fov_h,
            fov_v_rad=math.radians(55.0),
        )
        return self.cached_camera

    def _cast_ray_2d(self, x0: float, y0: float, angle: float, max_dist: float) -> float:
        """Raymarch against grid cells to find nearest obstacle distance."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        step = 0.05
        d = 0.0
        while d < max_dist:
            d += step
            rx = x0 + d * cos_a
            ry = y0 + d * sin_a
            cx = int(rx / self.cell_size)
            cy = int(self.rows - (ry / self.cell_size))
            if cx < 0 or cx >= self.cols or cy < 0 or cy >= self.rows:
                return d
            if self.grid[cy, cx] != 0:
                return d
        return max_dist
