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

from airmouse.core.hal import (
    SensorHub,
    ImuData,
    RangeData,
    CameraFrame,
    Camera,
    Lidar2D,
    LidarScan,
)
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer


class SimulatedCamera(Camera):
    """Simulated camera sensor implementing HAL Camera interface."""

    def __init__(
        self,
        name: str,
        renderer: WorldRenderer,
        drone: DroneDynamics,
        get_time_fn,
        rate_hz: float = 20.0,
        fov_v_deg: float = 55.0,
    ):
        self._name = name
        self.renderer = renderer
        self.drone = drone
        self.get_time_fn = get_time_fn
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.fov_v_rad = math.radians(fov_v_deg)
        self.last_read_time = -1.0
        self.cached_frame: Optional[CameraFrame] = None

    @property
    def name(self) -> str:
        return self._name

    def read(self) -> Optional[CameraFrame]:
        sim_time = self.get_time_fn()
        if sim_time - self.last_read_time < self.period - 1e-4:
            return None

        self.last_read_time = sim_time
        state = self.drone.get_state()

        if self._name == "front":
            frame_bgr = self.renderer.render_camera_frame(
                drone_pos=state.position,
                drone_yaw=state.attitude_rpy[2],
                drone_pitch=state.attitude_rpy[1],
                drone_roll=state.attitude_rpy[0],
                survivors=[s for s in getattr(self.renderer, "true_survivors", [])],
            )
            self.cached_frame = CameraFrame(
                timestamp_s=sim_time,
                image_bgr=frame_bgr,
                width=self.renderer.w,
                height=self.renderer.h,
                fov_h_rad=self.renderer.fov_h,
                fov_v_rad=self.fov_v_rad,
                name="front",
            )
            return self.cached_frame
        elif self._name == "down":
            # Downward camera simulation will be implemented in D14
            return None
        return None


class SimulatedLidar2D(Lidar2D):
    """2D 360-degree planar lidar sensor simulation.

    Emits 360 rays (1 degree resolution) spanning 0.15m to 12.0m.
    Raycasts against the true simulation grid from the true drone pose at flight altitude.
    Applies Gaussian range noise (sigma=0.02m) and random dropouts (2% return range_max).

    Physical Reality Note:
    A real planar 2D lidar mounted horizontally on a drone flying at ~1.2m altitude
    only scans obstacles in its horizontal scan plane and would miss low floor obstacles,
    carpets, or low furniture below its plane. In this simulator, obstacle grid cells
    (walls, furniture) are treated as obstacles matching the simulator's obstacle model.
    """

    def __init__(
        self,
        drone: DroneDynamics,
        grid: np.ndarray,
        cell_size_m: float = 1.0,
        get_time_fn=None,
        rate_hz: float = 8.0,
        num_rays: int = 360,
        range_min: float = 0.15,
        range_max: float = 12.0,
        noise_sigma: float = 0.02,
        dropout_prob: float = 0.02,
        sees_furniture: bool = True,
        angle_min: float = 0.0,
    ):
        self.drone = drone
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.cell_size = cell_size_m
        self.get_time_fn = get_time_fn
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.num_rays = num_rays
        self.range_min = range_min
        self.range_max = range_max
        self.noise_sigma = noise_sigma
        self.dropout_prob = dropout_prob
        self.sees_furniture = sees_furniture
        self.angle_min = angle_min
        self.angle_increment = 2.0 * math.pi / float(num_rays)

        self.last_scan_time = -1.0
        self.cached_scan: Optional[LidarScan] = None

    def read(self) -> Optional[LidarScan]:
        sim_time = self.get_time_fn() if self.get_time_fn else 0.0
        if self.last_scan_time >= 0 and (sim_time - self.last_scan_time < self.period - 1e-4):
            return None

        self.last_scan_time = sim_time
        state = self.drone.get_state()
        x0, y0 = float(state.position[0]), float(state.position[1])
        yaw = float(state.attitude_rpy[2])

        ranges = np.zeros(self.num_rays, dtype=np.float32)
        for i in range(self.num_rays):
            angle_body = self.angle_min + i * self.angle_increment
            angle_world = yaw + angle_body

            # Raycast against grid
            dist = self._cast_ray(x0, y0, angle_world, self.range_max)

            # 2% dropout: returns range_max
            if random.random() < self.dropout_prob:
                ranges[i] = self.range_max
            else:
                noise = np.random.normal(0.0, self.noise_sigma)
                noisy_dist = float(np.clip(dist + noise, self.range_min, self.range_max))
                ranges[i] = noisy_dist

        self.cached_scan = LidarScan(
            ranges=ranges,
            angle_min=self.angle_min,
            angle_increment=self.angle_increment,
            range_min=self.range_min,
            range_max=self.range_max,
            timestamp=sim_time,
        )
        return self.cached_scan

    def _cast_ray(self, x0: float, y0: float, angle: float, max_dist: float) -> float:
        """Raycast against grid cells with sub-millimeter bisection refinement."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        step = 0.10
        d = 0.0
        hit = False
        while d < max_dist:
            d += step
            rx = x0 + d * cos_a
            ry = y0 + d * sin_a
            cx = int(rx / self.cell_size)
            cy = int(self.rows - (ry / self.cell_size))
            if cx < 0 or cx >= self.cols or cy < 0 or cy >= self.rows:
                hit = True
                break
            if self.grid[cy, cx] != 0:
                hit = True
                break

        if not hit:
            return max_dist

        # Refine intersection boundary using binary search bisection
        d_low = max(0.0, d - step)
        d_high = min(max_dist, d)
        for _ in range(8):
            d_mid = 0.5 * (d_low + d_high)
            rx = x0 + d_mid * cos_a
            ry = y0 + d_mid * sin_a
            cx = int(rx / self.cell_size)
            cy = int(self.rows - (ry / self.cell_size))
            if cx < 0 or cx >= self.cols or cy < 0 or cy >= self.rows or self.grid[cy, cx] != 0:
                d_high = d_mid
            else:
                d_low = d_mid

        return d_high


class SimulatedSensorHub(SensorHub):
    PROFILE_DEFAULTS = {
        "baseline": {
            "mpu6050": True,
            "ultrasonic": True,
            "camera_front": True,
            "camera_down": False,
            "lidar2d": False,
        },
        "flow": {
            "mpu6050": True,
            "ultrasonic": True,
            "camera_front": True,
            "camera_down": True,
            "lidar2d": False,
        },
        "full": {
            "mpu6050": True,
            "ultrasonic": True,
            "camera_front": True,
            "camera_down": True,
            "lidar2d": True,
        },
    }

    def __init__(
        self,
        drone: DroneDynamics,
        renderer: WorldRenderer,
        grid: np.ndarray,
        cell_size_m: float = 1.0,
        imu_rate_hz: float = 100.0,
        range_rate_hz: float = 20.0,
        camera_rate_hz: float = 20.0,
        profile: str = "baseline",
        sensors_config: Optional[dict] = None,
    ):
        self.drone = drone
        self.renderer = renderer
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.cell_size = cell_size_m
        self.profile = profile

        # Resolve enabled sensors
        prof_map = self.PROFILE_DEFAULTS.get(profile, self.PROFILE_DEFAULTS["baseline"])
        if sensors_config and "profiles" in sensors_config and profile in sensors_config["profiles"]:
            prof_map = sensors_config["profiles"][profile]
        self.enabled_sensors = dict(prof_map)
        if sensors_config:
            for sname in ("mpu6050", "ultrasonic", "camera_front", "camera_down", "lidar2d"):
                s_sec = sensors_config.get(sname)
                if isinstance(s_sec, dict) and "enabled" in s_sec:
                    self.enabled_sensors[sname] = bool(self.enabled_sensors.get(sname, False) and s_sec["enabled"])

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

        # Camera sensors
        self.camera_front: Optional[SimulatedCamera] = None
        if self.has_sensor("camera_front"):
            self.camera_front = SimulatedCamera(
                name="front",
                renderer=renderer,
                drone=drone,
                get_time_fn=self.get_time,
                rate_hz=camera_rate_hz,
            )

        self.camera_down: Optional[SimulatedCamera] = None
        if self.has_sensor("camera_down"):
            self.camera_down = SimulatedCamera(
                name="down",
                renderer=renderer,
                drone=drone,
                get_time_fn=self.get_time,
                rate_hz=camera_rate_hz,
            )

        # 2D Planar Lidar
        lidar_cfg = {}
        if sensors_config and "lidar2d" in sensors_config and isinstance(sensors_config["lidar2d"], dict):
            lidar_cfg = sensors_config["lidar2d"]

        lidar_rate_hz = float(lidar_cfg.get("lidar_rate_hz", lidar_cfg.get("rate_hz", 8.0)))
        lidar_range_max = float(lidar_cfg.get("lidar_range_max", lidar_cfg.get("max_range_m", 12.0)))
        lidar_noise_sigma = float(lidar_cfg.get("lidar_noise_sigma", lidar_cfg.get("noise_sigma", 0.02)))
        lidar_dropout = float(lidar_cfg.get("lidar_dropout", lidar_cfg.get("dropout_prob", 0.02)))
        lidar_sees_furniture = bool(lidar_cfg.get("lidar_sees_furniture", lidar_cfg.get("sees_furniture", True)))

        self.lidar: Optional[SimulatedLidar2D] = None
        if self.has_sensor("lidar2d"):
            self.lidar = SimulatedLidar2D(
                drone=drone,
                grid=grid,
                cell_size_m=cell_size_m,
                get_time_fn=self.get_time,
                rate_hz=lidar_rate_hz,
                num_rays=360,
                range_min=0.15,
                range_max=lidar_range_max,
                noise_sigma=lidar_noise_sigma,
                dropout_prob=lidar_dropout,
                sees_furniture=lidar_sees_furniture,
            )

        # Cache latest readings
        self.cached_ranges: Dict[str, RangeData] = {}
        self.cached_camera: Optional[CameraFrame] = None

    def advance_time(self, dt: float):
        self.sim_time += dt

    def get_time(self) -> float:
        return self.sim_time

    def has_sensor(self, name: str) -> bool:
        alias_map = {
            "imu": "mpu6050",
            "ranges": "ultrasonic",
            "sonar": "ultrasonic",
            "camera": "camera_front",
            "lidar": "lidar2d",
        }
        canonical = alias_map.get(name, name)
        return bool(self.enabled_sensors.get(canonical, False))

    def get_camera(self, name: str = "front") -> Optional[Camera]:
        if name == "front" and self.has_sensor("camera_front"):
            return self.camera_front
        elif name == "down" and self.has_sensor("camera_down"):
            return self.camera_down
        return None

    def get_lidar(self) -> Optional[Lidar2D]:
        if self.has_sensor("lidar2d"):
            return self.lidar
        return None

    def read_lidar(self) -> Optional[LidarScan]:
        if self.lidar is not None:
            return self.lidar.read()
        return None

    def read_imu(self) -> Optional[ImuData]:
        if not self.has_sensor("mpu6050"):
            return None
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
        if not self.has_sensor("ultrasonic"):
            return {}
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
        if self.camera_front is not None:
            self.cached_camera = self.camera_front.read()
            return self.cached_camera
        return None

    def _cast_ray_2d(self, x0: float, y0: float, angle: float, max_dist: float) -> float:
        """Raymarch against grid cells to find nearest obstacle distance."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        step = 0.12
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
