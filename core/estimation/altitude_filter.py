"""1D Altitude Kalman Filter fusing downward ultrasonic range with vertical acceleration."""
from __future__ import annotations

import math
import numpy as np


class AltitudeFilter:
    def __init__(self, init_z: float = 0.0, sigma_accel: float = 0.2, sigma_sonar: float = 0.03):
        # State: [z, vz]
        self.x = np.array([init_z, 0.0], dtype=np.float64)
        self.P = np.diag([0.05, 0.1]).astype(np.float64)
        self.sigma_accel = sigma_accel
        self.sigma_sonar = sigma_sonar

    def predict(self, az_world: float, dt: float):
        """az_world is vertical acceleration in world frame (excluding gravity)."""
        if dt <= 0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        G = np.array([0.5 * dt * dt, dt], dtype=np.float64)
        Q = np.outer(G, G) * (self.sigma_accel**2)

        self.x = F @ self.x + G * az_world
        self.P = F @ self.P @ F.T + Q

    def update_sonar(self, measured_range: float, roll: float = 0.0, pitch: float = 0.0):
        """Correct state with downward ultrasonic measurement."""
        # Convert range to vertical altitude z
        cos_tilt = math.cos(roll) * math.cos(pitch)
        if cos_tilt <= 0.2 or measured_range <= 0.02 or measured_range >= 3.95:
            return  # invalid or extreme tilt

        z_meas = measured_range * cos_tilt
        H = np.array([[1.0, 0.0]], dtype=np.float64)
        R = np.array([[self.sigma_sonar**2]], dtype=np.float64)

        y = z_meas - (H @ self.x)[0]
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K * y).flatten()
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P

    @property
    def altitude(self) -> float:
        return float(self.x[0])

    @property
    def vertical_velocity(self) -> float:
        return float(self.x[1])

