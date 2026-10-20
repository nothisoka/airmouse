"""Position and Heading Tracking Controller for NIDAR AirMouse."""
from __future__ import annotations

import math
from typing import Tuple
import numpy as np
from airmouse.core.hal import DroneState, VelocityCommand


class WaypointController:
    def __init__(
        self,
        kp_xy: float = 1.0,
        max_vel_xy: float = 0.6,
        kp_z: float = 1.2,
        max_vel_z: float = 0.5,
        kp_yaw: float = 1.5,
        max_yaw_rate: float = 1.2,
        waypoint_reached_tol_m: float = 0.35,
    ):
        self.kp_xy = kp_xy
        self.max_vel_xy = max_vel_xy
        self.kp_z = kp_z
        self.max_vel_z = max_vel_z
        self.kp_yaw = kp_yaw
        self.max_yaw_rate = max_yaw_rate
        self.tolerance = waypoint_reached_tol_m

    def compute_command(
        self,
        state: DroneState,
        target_pos_w: Tuple[float, float, float],
    ) -> Tuple[VelocityCommand, bool]:
        """Generate VelocityCommand to steer drone toward target_pos_w.
        
        Returns:
            (cmd, is_reached)
        """
        dx = target_pos_w[0] - state.position[0]
        dy = target_pos_w[1] - state.position[1]
        dz = target_pos_w[2] - state.position[2]
        dist_xy = math.hypot(dx, dy)

        if dist_xy < self.tolerance and abs(dz) < 0.2:
            return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0), True

        # Transform world error into drone body frame
        yaw = state.attitude_rpy[2]
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Body forward (+X) and left (+Y)
        e_body_x = dx * cos_y + dy * sin_y
        e_body_y = -dx * sin_y + dy * cos_y

        # Proportional velocity command
        cmd_vx = np.clip(self.kp_xy * e_body_x, -self.max_vel_xy, self.max_vel_xy)
        cmd_vy = np.clip(self.kp_xy * e_body_y, -self.max_vel_xy, self.max_vel_xy)
        cmd_vz = np.clip(self.kp_z * dz, -self.max_vel_z, self.max_vel_z)

        # Heading alignment: turn toward target waypoint if distance is significant
        if dist_xy > 0.4:
            target_yaw = math.atan2(dy, dx)
            yaw_err = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
            cmd_yaw_rate = np.clip(self.kp_yaw * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)
        else:
            cmd_yaw_rate = 0.0

        return VelocityCommand(
            vx=float(cmd_vx),
            vy=float(cmd_vy),
            vz=float(cmd_vz),
            yaw_rate_radps=float(cmd_yaw_rate),
        ), False

