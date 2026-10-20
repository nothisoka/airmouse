"""Test mission state machine transitions and return-to-home."""
import numpy as np
import pytest
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.hal import DroneState, PoseQuality, FlightInterface, VelocityCommand


class DummyFlight(FlightInterface):
    def __init__(self):
        self.battery = 100.0
        self.alt = 0.0

    def send_velocity(self, cmd: VelocityCommand):
        pass

    def takeoff(self, target_altitude_m: float = 1.2) -> bool:
        self.alt = target_altitude_m
        return True

    def land(self) -> bool:
        self.alt = 0.0
        return True

    def get_battery_percentage(self) -> float:
        return self.battery


def test_mission_state_machine():
    flight = DummyFlight()
    grid = OccupancyGrid(width_m=10.0, height_m=10.0)
    coord = MissionCoordinator(start_world=(2.0, 2.0), grid_map=grid, flight=flight)

    assert coord.state == MissionState.PREFLIGHT

    # Step 1: Preflight -> Takeoff
    dummy_state = DroneState(
        timestamp_s=0.1,
        position=np.array([2.0, 2.0, 0.0]),
        velocity=np.zeros(3),
        attitude_rpy=np.zeros(3),
        quaternion=np.array([1, 0, 0, 0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3),
        covariance_diag=np.ones(9)*0.01,
        quality=PoseQuality.EXCELLENT,
    )
    cmd = coord.update(current_time_s=0.1, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.TAKEOFF

    # Step 2: In Takeoff, once reached target alt -> Explore
    dummy_state.position[2] = 1.2
    cmd = coord.update(current_time_s=0.5, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.EXPLORE

    # Move drone out into the building away from home (2.0, 2.0)
    dummy_state.position[0] = 6.0
    dummy_state.position[1] = 7.0

    # Step 3: Low battery trigger -> Return Home
    flight.battery = 15.0  # low battery
    cmd = coord.update(current_time_s=1.0, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.RETURN_HOME
