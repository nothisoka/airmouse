"""Unit tests for the smarter return-home trigger calculation and state machine transition."""
import math
import numpy as np
import pytest

from airmouse.core.hal import DroneState, PoseQuality, VelocityCommand, FlightInterface
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState


class DummyFlight(FlightInterface):
    def __init__(self, battery: float = 100.0):
        self.battery = battery
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


def test_should_trigger_return_home_synthetic_numbers():
    """Verify should_trigger_return_home formula: remaining_time < est_time_home * 1.5 + 30."""
    cruise_speed = 0.6  # m/s
    path_len = 30.0     # m
    # est_time_home = 30.0 / 0.6 = 50.0 s
    # required_margin = 50.0 * 1.5 + 30.0 = 105.0 s

    # 1. remaining_time well above margin -> No trigger
    triggered, est_time, margin = MissionCoordinator.should_trigger_return_home(
        remaining_time_s=120.0,
        path_length_home_m=path_len,
        cruise_speed_mps=cruise_speed,
    )
    assert not triggered
    assert pytest.approx(est_time, rel=1e-3) == 50.0
    assert pytest.approx(margin, rel=1e-3) == 105.0

    # 2. remaining_time exactly on boundary -> No trigger (< strictly)
    triggered, _, _ = MissionCoordinator.should_trigger_return_home(
        remaining_time_s=105.0,
        path_length_home_m=path_len,
        cruise_speed_mps=cruise_speed,
    )
    assert not triggered

    # 3. remaining_time below margin -> Trigger
    triggered, _, _ = MissionCoordinator.should_trigger_return_home(
        remaining_time_s=104.9,
        path_length_home_m=path_len,
        cruise_speed_mps=cruise_speed,
    )
    assert triggered

    # 4. Zero path length edge case: est_time = 0, margin = 30.0 s
    triggered, est_time, margin = MissionCoordinator.should_trigger_return_home(
        remaining_time_s=31.0,
        path_length_home_m=0.0,
        cruise_speed_mps=cruise_speed,
    )
    assert not triggered
    assert pytest.approx(est_time, rel=1e-3) == 0.0
    assert pytest.approx(margin, rel=1e-3) == 30.0

    triggered, _, _ = MissionCoordinator.should_trigger_return_home(
        remaining_time_s=29.0,
        path_length_home_m=0.0,
        cruise_speed_mps=cruise_speed,
    )
    assert triggered


def test_compute_path_length():
    """Verify compute_path_length helper function."""
    assert MissionCoordinator.compute_path_length([]) == 0.0

    path = [(0.0, 0.0), (3.0, 4.0), (3.0, 8.0)]
    # (0,0) -> (3,4) is 5.0, (3,4) -> (3,8) is 4.0, total = 9.0
    length = MissionCoordinator.compute_path_length(path)
    assert pytest.approx(length, rel=1e-3) == 9.0

    # With start_pos
    length_with_start = MissionCoordinator.compute_path_length(path, start_pos=(-3.0, 0.0))
    # (-3,0) -> (0,0) is 3.0 + 9.0 = 12.0
    assert pytest.approx(length_with_start, rel=1e-3) == 12.0


def test_mission_coordinator_rth_trigger_transition():
    """Test MissionCoordinator transitions from EXPLORE to RETURN_HOME when budget triggers."""
    flight = DummyFlight(battery=100.0)
    grid = OccupancyGrid(width_m=20.0, height_m=20.0)
    # Start at (2.0, 2.0), max flight time = 100.0s, cruise_speed = 1.0 m/s
    coord = MissionCoordinator(
        start_world=(2.0, 2.0),
        grid_map=grid,
        flight=flight,
        max_flight_time_s=100.0,
        cruise_speed_mps=1.0,
    )

    dummy_state = DroneState(
        timestamp_s=0.0,
        position=np.array([2.0, 2.0, 0.0]),
        velocity=np.zeros(3),
        attitude_rpy=np.zeros(3),
        quaternion=np.array([1, 0, 0, 0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3),
        covariance_diag=np.ones(9) * 0.01,
        quality=PoseQuality.EXCELLENT,
    )

    # 1. Preflight -> Takeoff at t=0.0
    coord.update(current_time_s=0.0, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.TAKEOFF

    # 2. Takeoff -> Explore at t=1.0
    dummy_state.position[2] = 1.2
    coord.update(current_time_s=1.0, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.EXPLORE

    # Move drone to (12.0, 2.0) -> distance to home (2.0, 2.0) is 10.0m
    dummy_state.position[0] = 12.0
    dummy_state.position[1] = 2.0
    # Add breadcrumbs from home to current position
    coord.breadcrumbs = [(2.0, 2.0), (7.0, 2.0), (12.0, 2.0)]
    # Path length home ~ 10.0m. At 1.0 m/s:
    # est_time_home = 10.0s
    # margin = 10.0 * 1.5 + 30.0 = 45.0s

    # At t = 20.0s: elapsed = 20.0 - 0.0 = 20.0s, remaining = 80.0s > 45.0s -> Still EXPLORE
    coord.update(current_time_s=20.0, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.EXPLORE

    # At t = 60.0s: elapsed = 60.0s, remaining = 40.0s < 45.0s -> Triggers RETURN_HOME
    coord.update(current_time_s=60.0, drone_state=dummy_state, ranges={})
    assert coord.state == MissionState.RETURN_HOME

