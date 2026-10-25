"""Unit tests for stall and oscillation watchdog mechanisms in MissionCoordinator."""
import numpy as np
import pytest
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.hal import DroneState, PoseQuality, FlightInterface, VelocityCommand


class MockFlight(FlightInterface):
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


def _make_drone_state(t: float, x: float = 2.0, y: float = 2.0, z: float = 1.2) -> DroneState:
    return DroneState(
        timestamp_s=t,
        position=np.array([x, y, z]),
        velocity=np.zeros(3),
        attitude_rpy=np.zeros(3),
        quaternion=np.array([1, 0, 0, 0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3),
        covariance_diag=np.ones(9) * 0.01,
        quality=PoseQuality.EXCELLENT,
    )


def test_scripted_stall_watchdog():
    """Verify watchdog blacklists current goal and replans when staying within 0.5m for >= 15s."""
    flight = MockFlight()
    grid = OccupancyGrid(width_m=12.0, height_m=12.0, resolution_m=0.25)
    # Clear space around start to create valid frontiers ahead
    for x in np.arange(1.0, 5.0, 0.5):
        grid.update_beam(start_wx=x, start_wy=2.0, angle_rad=0.0, range_m=2.0, valid_hit=False)

    coord = MissionCoordinator(start_world=(2.0, 2.0), grid_map=grid, flight=flight)

    # Step to EXPLORE state
    coord.update(current_time_s=0.1, drone_state=_make_drone_state(0.1, z=0.0), ranges={})
    coord.update(current_time_s=0.5, drone_state=_make_drone_state(0.5, z=1.2), ranges={})
    assert coord.state == MissionState.EXPLORE

    # Initial explore update establishes stall anchor
    coord.update(current_time_s=1.0, drone_state=_make_drone_state(1.0, x=2.0, y=2.0), ranges={})
    assert coord.stall_anchor_pos is not None
    assert coord.stall_start_time == 1.0
    initial_target = coord.current_target

    # Simulate drone staying at (2.0, 2.0) for 14.0 seconds (up to t=15.0)
    coord.update(current_time_s=15.0, drone_state=_make_drone_state(15.0, x=2.1, y=2.0), ranges={})
    # Watchdog should NOT have triggered yet (14s < 15s)
    if initial_target is not None:
        target_cell = grid.world_to_grid(initial_target[0], initial_target[1])
        assert coord.frontier_detector.blacklist_counts.get(target_cell, 0) == 0

    # At t=16.5s (15.5s after anchor time 1.0s), watchdog MUST fire
    coord.update(current_time_s=16.5, drone_state=_make_drone_state(16.5, x=2.1, y=2.0), ranges={})

    # Verify that initial target was blacklisted
    if initial_target is not None:
        target_cell = grid.world_to_grid(initial_target[0], initial_target[1])
        assert coord.frontier_detector.blacklist_counts.get(target_cell, 0) >= 1

    # Verify anchor and timer reset to current time
    assert coord.stall_start_time == 16.5


def test_stall_watchdog_resets_on_motion():
    """Verify watchdog resets its timer when the drone moves > 0.5m."""
    flight = MockFlight()
    grid = OccupancyGrid(width_m=12.0, height_m=12.0, resolution_m=0.25)
    coord = MissionCoordinator(start_world=(2.0, 2.0), grid_map=grid, flight=flight)

    # Reach EXPLORE state
    coord.update(0.1, _make_drone_state(0.1, z=0.0), {})
    coord.update(0.5, _make_drone_state(0.5, z=1.2), {})

    # Start anchor at (2.0, 2.0) at t=1.0
    coord.update(1.0, _make_drone_state(1.0, x=2.0, y=2.0), {})
    assert coord.stall_start_time == 1.0

    # Drone moves 1.0m to (3.0, 2.0) at t=10.0
    coord.update(10.0, _make_drone_state(10.0, x=3.0, y=2.0), {})
    assert coord.stall_start_time == 10.0
    assert coord.stall_anchor_pos == (3.0, 2.0)

    # At t=20.0 (10s later), only 10s have elapsed from new anchor, so no stall trigger
    coord.update(20.0, _make_drone_state(20.0, x=3.1, y=2.0), {})
    assert coord.stall_start_time == 10.0


def test_goal_oscillation_watchdog(capsys):
    """Verify alternating between two goals 3 times triggers oscillation watchdog and blacklists."""
    flight = MockFlight()
    grid = OccupancyGrid(width_m=12.0, height_m=12.0, resolution_m=0.25)
    # Clear line
    for x in np.arange(1.0, 6.0, 0.5):
        grid.update_beam(start_wx=x, start_wy=2.0, angle_rad=0.0, range_m=2.0, valid_hit=False)

    coord = MissionCoordinator(start_world=(2.0, 2.0), grid_map=grid, flight=flight)

    goal_a = (4.0, 2.0)
    goal_b = (1.5, 2.0)

    # Feed alternating history: A, B, A, B
    coord.recent_goals = [goal_a, goal_b, goal_a, goal_b]

    # Mock candidate search to return goal_a as the top candidate
    coord.frontier_detector.find_candidate_frontiers = lambda g, xy, top_k=15: [goal_a]

    state = _make_drone_state(1.0, x=2.5, y=2.0)
    coord._plan_next_frontier(state)

    # Should detect oscillation and blacklist goal_a
    captured = capsys.readouterr()
    assert "Goal oscillation detected" in captured.out

    cell_a = grid.world_to_grid(goal_a[0], goal_a[1])
    assert coord.frontier_detector.blacklist_counts.get(cell_a, 0) >= 1
