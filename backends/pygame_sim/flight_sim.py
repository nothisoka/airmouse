"""Simulated Flight Interface implementing FlightInterface ABC."""
from __future__ import annotations

from airmouse.core.hal import FlightInterface, VelocityCommand
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics


class SimulatedFlightInterface(FlightInterface):
    def __init__(self, drone: DroneDynamics):
        self.drone = drone
        self.is_armed = False
        self.in_flight = False

    def send_velocity(self, cmd: VelocityCommand) -> None:
        if self.is_armed:
            self.drone.set_command(cmd)

    def takeoff(self, target_altitude_m: float = 1.2) -> bool:
        self.is_armed = True
        self.in_flight = True
        # Climb at 0.5 m/s until target reached
        if self.drone.pos[2] < target_altitude_m - 0.05:
            self.drone.set_command(VelocityCommand(vx=0.0, vy=0.0, vz=0.5, yaw_rate_radps=0.0))
            return False
        else:
            self.drone.set_command(VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0))
            return True

    def land(self) -> bool:
        if self.drone.pos[2] > 0.1:
            self.drone.set_command(VelocityCommand(vx=0.0, vy=0.0, vz=-0.4, yaw_rate_radps=0.0))
            return False
        else:
            self.drone.set_command(VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0))
            self.is_armed = False
            self.in_flight = False
            return True

    def get_battery_percentage(self) -> float:
        return self.drone.get_state().battery_pct

