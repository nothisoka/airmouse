"""Reactive Collision Avoidance Layer using Live Ultrasonic Readings.

Functions as an instantaneous safety override above the global path planner.
Applies virtual repulsive force vectors if obstacles breach safety margins.
"""
from __future__ import annotations

import math
from typing import Dict, Tuple
import numpy as np
from airmouse.core.hal import RangeData, VelocityCommand


class ReactiveAvoidance:
    def __init__(
        self,
        danger_dist_m: float = 0.35,
        slowdown_dist_m: float = 0.65,
        repulsion_gain: float = 0.8,
    ):
        self.danger_dist = danger_dist_m
        self.slowdown_dist = slowdown_dist_m
        self.repulsion_gain = repulsion_gain

        # Sensor angle relative to drone body forward (+X is forward, +Y is left)
        self.sensor_directions = {
            "front": (1.0, 0.0),
            "left": (0.0, 1.0),
            "right": (0.0, -1.0),
            "rear": (-1.0, 0.0),
        }

    def filter_command(
        self,
        cmd: VelocityCommand,
        ranges: Dict[str, RangeData],
    ) -> Tuple[VelocityCommand, bool]:
        """Modulate velocity command with repulsive vectors if obstacles are near.
        
        Returns:
            (safe_cmd, override_active)
        """
        safe_vx = cmd.vx
        safe_vy = cmd.vy
        override_active = False

        for sensor_id, (dir_x, dir_y) in self.sensor_directions.items():
            if sensor_id not in ranges or not ranges[sensor_id].valid:
                continue

            dist = ranges[sensor_id].range_m

            # Slowdown zone: attenuate any velocity moving towards this sensor
            if dist < self.slowdown_dist:
                proj_vel = safe_vx * dir_x + safe_vy * dir_y
                if proj_vel > 0:
                    scale = max(0.0, (dist - self.danger_dist) / (self.slowdown_dist - self.danger_dist))
                    # Reduce velocity component towards obstacle
                    safe_vx -= dir_x * proj_vel * (1.0 - scale)
                    safe_vy -= dir_y * proj_vel * (1.0 - scale)

            # Danger zone: strictly block inward velocity and push outward
            if dist < self.danger_dist:
                override_active = True
                strength = self.repulsion_gain * (self.danger_dist - dist) / self.danger_dist
                # Zero out any remaining inward velocity
                proj_vel = safe_vx * dir_x + safe_vy * dir_y
                if proj_vel > 0:
                    safe_vx -= dir_x * proj_vel
                    safe_vy -= dir_y * proj_vel
                # Push actively away from obstacle
                safe_vx -= dir_x * strength
                safe_vy -= dir_y * strength

        front_r = ranges["front"].range_m if ("front" in ranges and ranges["front"].valid) else 4.0
        rear_r = ranges["rear"].range_m if ("rear" in ranges and ranges["rear"].valid) else 4.0
        left_r = ranges["left"].range_m if ("left" in ranges and ranges["left"].valid) else 4.0
        right_r = ranges["right"].range_m if ("right" in ranges and ranges["right"].valid) else 4.0

        # Corridor centering: if flanked by walls on both sides, center smoothly
        if left_r < 0.75 and right_r < 0.75:
            lat_error = (left_r - right_r) * 0.5
            safe_vy = float(np.clip(lat_error * 0.8, -0.25, 0.25))

        safe_yaw_rate = cmd.yaw_rate_radps
        if override_active:
            # Active escape maneuvers:
            # 1. Front obstacle: reverse and steer towards open clearance
            if front_r < self.danger_dist:
                steer_dir = 1.0 if left_r >= right_r else -1.0
                safe_yaw_rate = steer_dir * 0.8
                safe_vx = min(safe_vx, -0.25) if rear_r > 0.38 else 0.0

            # 2. Rear obstacle: advance forward
            if rear_r < self.danger_dist and front_r > 0.45:
                safe_vx = max(safe_vx, 0.20)

            # Sanity clamp repulsive speed
            safe_vx = float(np.clip(safe_vx, -0.5, 0.5))
            safe_vy = float(np.clip(safe_vy, -0.5, 0.5))
            return VelocityCommand(vx=safe_vx, vy=safe_vy, vz=cmd.vz, yaw_rate_radps=float(safe_yaw_rate)), True

        return VelocityCommand(vx=safe_vx, vy=safe_vy, vz=cmd.vz, yaw_rate_radps=float(safe_yaw_rate)), False

