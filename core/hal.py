"""Hardware Abstraction Layer (HAL) for NIDAR AirMouse.

Defines the contract between the platform (pygame simulator, Gazebo ROS2 bridge,
or real companion hardware) and the pure Python autonomy core.
No simulator or platform-specific libraries are imported here.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional, Tuple
import numpy as np


class PoseQuality(Enum):
    EXCELLENT = auto()  # Covariance low, all sensors nominal
    DEGRADED = auto()   # Feature tracking sparse or sonar dropout
    CRITICAL = auto()   # High covariance, IMU divergence risk, hold/RTH required


@dataclass
class ImuData:
    """Raw 6-DOF IMU data (MPU6050)."""
    timestamp_s: float
    accel_mps2: np.ndarray       # shape (3,), body frame [ax, ay, az]
    gyro_radps: np.ndarray       # shape (3,), body frame [wx, wy, wz]


@dataclass
class RangeData:
    """Single ultrasonic ranger reading (HC-SR04)."""
    sensor_id: str               # "front", "left", "right", "rear", "down"
    range_m: float               # measured distance
    valid: bool                  # True if echo received within [min_range, max_range]
    timestamp_s: float
    cone_half_angle_rad: float = 0.1309  # ~7.5 degrees (15 deg full cone)


@dataclass
class CameraFrame:
    """Monocular camera frame."""
    timestamp_s: float
    image_bgr: np.ndarray        # shape (H, W, 3), uint8
    width: int
    height: int
    fov_h_rad: float
    fov_v_rad: float


@dataclass
class DroneState:
    """Estimated state vector produced by state estimator."""
    timestamp_s: float
    position: np.ndarray         # [x, y, z] in world metres
    velocity: np.ndarray         # [vx, vy, vz] in world m/s
    attitude_rpy: np.ndarray     # [roll, pitch, yaw] in radians
    quaternion: np.ndarray       # [qw, qx, qy, qz]
    accel_bias: np.ndarray       # [bax, bay, baz]
    gyro_bias: np.ndarray        # [bgx, bgy, bgz]
    covariance_diag: np.ndarray  # diag of P for [x, y, z, vx, vy, vz, roll, pitch, yaw]
    quality: PoseQuality = PoseQuality.EXCELLENT


@dataclass
class VelocityCommand:
    """Body or world-frame velocity setpoint sent to flight controller."""
    vx: float                    # forward/longitudinal m/s
    vy: float                    # lateral m/s
    vz: float                    # vertical m/s (positive up)
    yaw_rate_radps: float        # yaw rate rad/s


class SensorHub(ABC):
    """Abstract interface to read sensors on any platform."""

    @abstractmethod
    def read_imu(self) -> Optional[ImuData]:
        """Poll the latest IMU sample."""
        raise NotImplementedError

    @abstractmethod
    def read_ranges(self) -> Dict[str, RangeData]:
        """Poll the latest ultrasonic ranges: keys 'front', 'left', 'right', 'rear', 'down'."""
        raise NotImplementedError

    @abstractmethod
    def read_camera(self) -> Optional[CameraFrame]:
        """Poll the latest camera frame."""
        raise NotImplementedError

    @abstractmethod
    def get_time(self) -> float:
        """Current monotonic simulation / system time in seconds."""
        raise NotImplementedError


class FlightInterface(ABC):
    """Abstract interface to command drone motion and query platform status."""

    @abstractmethod
    def send_velocity(self, cmd: VelocityCommand) -> None:
        """Send velocity / yaw-rate setpoint to flight controller."""
        raise NotImplementedError

    @abstractmethod
    def takeoff(self, target_altitude_m: float = 1.2) -> bool:
        """Command vertical takeoff to target hover altitude."""
        raise NotImplementedError

    @abstractmethod
    def land(self) -> bool:
        """Command controlled landing."""
        raise NotImplementedError

    @abstractmethod
    def get_battery_percentage(self) -> float:
        """Return remaining battery percentage (0.0 to 100.0)."""
        raise NotImplementedError

