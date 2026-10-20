"""Continuous 3D Drone Flight Dynamics for the Simulator Backend.

Maintains continuous physical state (position, velocity, attitude), applies
motor response lag, acceleration limits, aerodynamic drag, and disturbances.
True physical state is held strictly inside this class and never exposed to autonomy.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
import numpy as np
from airmouse.core.hal import VelocityCommand


@dataclass
class DronePhysicalState:
    position: np.ndarray      # [x, y, z] in world metres
    velocity: np.ndarray      # [vx, vy, vz] in world m/s
    attitude_rpy: np.ndarray  # [roll, pitch, yaw] in radians
    angular_velocity: np.ndarray # [p, q, r] in body rad/s
    acceleration: np.ndarray  # [ax, ay, az] in world m/s^2 (excluding gravity)
    battery_pct: float = 100.0


class DroneDynamics:
    def __init__(
        self,
        initial_position: np.ndarray,
        initial_yaw: float = 0.0,
        mass_kg: float = 0.85,
        max_vel_xy: float = 1.2,
        max_vel_z: float = 0.8,
        max_accel: float = 3.0,
        max_yaw_rate: float = 2.0,
        drag_coeff: float = 0.45,
        battery_capacity_mah: float = 2200.0,
        hover_current_a: float = 11.5,
        grid: Optional[np.ndarray] = None,
        cell_size_m: float = 1.0,
    ):
        self.mass = mass_kg
        self.max_vel_xy = max_vel_xy
        self.max_vel_z = max_vel_z
        self.max_accel = max_accel
        self.max_yaw_rate = max_yaw_rate
        self.drag_coeff = drag_coeff
        self.battery_capacity_mah = battery_capacity_mah
        self.hover_current_a = hover_current_a
        self.grid = grid
        self.cell_size = cell_size_m
        if grid is not None:
            self.rows, self.cols = grid.shape
        else:
            self.rows, self.cols = 0, 0
        self.had_collision = False

        self.pos = np.array(initial_position, dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.accel = np.zeros(3, dtype=np.float64)
        self.yaw = float(initial_yaw)
        self.yaw_rate = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.battery_mah_remaining = battery_capacity_mah

        # Target command buffer
        self.cmd_vel_world = np.zeros(3, dtype=np.float64)
        self.cmd_yaw_rate = 0.0

    def set_command(self, cmd: VelocityCommand):
        """Transform body-frame command [forward, left, up] into world frame."""
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        # cmd.vx is forward (body +X), cmd.vy is left (body +Y)
        world_vx = cmd.vx * cos_y - cmd.vy * sin_y
        world_vy = cmd.vx * sin_y + cmd.vy * cos_y
        world_vz = cmd.vz

        self.cmd_vel_world = np.array([world_vx, world_vy, world_vz], dtype=np.float64)
        self.cmd_yaw_rate = np.clip(cmd.yaw_rate_radps, -self.max_yaw_rate, self.max_yaw_rate)

    def step(self, dt: float, disturbance_std: float = 0.02):
        """Advance physical simulation by dt seconds."""
        if dt <= 0:
            return

        # Simple autopilot velocity tracking with acceleration limit
        vel_err = self.cmd_vel_world - self.vel
        desired_accel = vel_err / max(dt, 0.05)
        accel_mag = np.linalg.norm(desired_accel)
        if accel_mag > self.max_accel:
            self.accel = desired_accel * (self.max_accel / accel_mag)
        else:
            self.accel = desired_accel

        # Add small random aerodynamic / motor disturbance
        if disturbance_std > 0:
            noise = np.random.normal(0.0, disturbance_std, size=3)
            self.accel += noise

        # Update velocity and apply drag
        self.vel += self.accel * dt
        self.vel -= self.drag_coeff * self.vel * dt

        # Clamp speed limits
        speed_xy = np.linalg.norm(self.vel[:2])
        if speed_xy > self.max_vel_xy:
            self.vel[:2] *= (self.max_vel_xy / speed_xy)
        self.vel[2] = np.clip(self.vel[2], -self.max_vel_z, self.max_vel_z)

        # Update position with physical wall collision checks
        cand_x = self.pos[0] + self.vel[0] * dt
        cand_y = self.pos[1] + self.vel[1] * dt
        cand_z = self.pos[2] + self.vel[2] * dt

        if self.grid is not None:
            r_drone = 0.20
            # Test X movement
            if abs(self.vel[0]) > 1e-3:
                cx = int((cand_x + math.copysign(r_drone, self.vel[0])) / self.cell_size)
                cy = int(self.rows - (self.pos[1] / self.cell_size))
                if 0 <= cx < self.cols and 0 <= cy < self.rows and self.grid[cy, cx] == 0:
                    self.pos[0] = cand_x
                else:
                    self.had_collision = True
                    self.pos[0] -= math.copysign(0.04, self.vel[0])
                    self.vel[0] = -0.3 * self.vel[0]
            else:
                self.pos[0] = cand_x

            # Test Y movement
            if abs(self.vel[1]) > 1e-3:
                cx = int(self.pos[0] / self.cell_size)
                cy = int(self.rows - ((cand_y + math.copysign(r_drone, self.vel[1])) / self.cell_size))
                if 0 <= cx < self.cols and 0 <= cy < self.rows and self.grid[cy, cx] == 0:
                    self.pos[1] = cand_y
                else:
                    self.had_collision = True
                    self.pos[1] -= math.copysign(0.04, self.vel[1])
                    self.vel[1] = -0.3 * self.vel[1]
            else:
                self.pos[1] = cand_y
        else:
            self.pos[0] = cand_x
            self.pos[1] = cand_y

        self.pos[2] = cand_z

        # Prevent underground penetration
        if self.pos[2] < 0.0:
            self.pos[2] = 0.0
            self.vel[2] = max(0.0, self.vel[2])

        # Update yaw
        self.yaw_rate = self.cmd_yaw_rate
        self.yaw = (self.yaw + self.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi

        # Tilt angles proportional to lateral accelerations (quadrotor flight dynamics)
        g = 9.80665
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        # Body frame accelerations
        body_ax = self.accel[0] * cos_y + self.accel[1] * sin_y
        body_ay = -self.accel[0] * sin_y + self.accel[1] * cos_y
        self.pitch = float(np.clip(-body_ax / g, -0.4, 0.4))
        self.roll = float(np.clip(body_ay / g, -0.4, 0.4))

        # Battery discharge: hover current + extra draw for velocity
        current_draw_a = self.hover_current_a * (1.0 + 0.2 * np.linalg.norm(self.vel))
        mah_used = (current_draw_a * 1000.0) * (dt / 3600.0)
        self.battery_mah_remaining = max(0.0, self.battery_mah_remaining - mah_used)

    def get_state(self) -> DronePhysicalState:
        p_roll = self.roll
        p_pitch = self.pitch
        p_yaw = self.yaw
        rpy = np.array([p_roll, p_pitch, p_yaw])
        ang_vel = np.array([0.0, 0.0, self.yaw_rate])
        battery_pct = max(0.0, (self.battery_mah_remaining / self.battery_capacity_mah) * 100.0)

        return DronePhysicalState(
            position=self.pos.copy(),
            velocity=self.vel.copy(),
            attitude_rpy=rpy,
            angular_velocity=ang_vel,
            acceleration=self.accel.copy(),
            battery_pct=battery_pct,
        )
