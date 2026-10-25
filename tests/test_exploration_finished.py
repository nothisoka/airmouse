"""Unit tests for exploration-finished detection in MissionCoordinator."""
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


def test_exploration_finished_on_fully_known_map():
    """Verify that exploration immediately finishes and returns home on a fully known map."""
    flight = MockFlight()
    grid = OccupancyGrid(width_m=8.0, height_m=8.0, resolution_m=0.25)

    # Populate grid as fully known:
    # All interior cells are known free, perimeter is known wall
    grid.log_odds[:] = grid.l_free * 3.0  # known free space
    grid.log_odds[0, :] = grid.l_occ * 3.0
    grid.log_odds[-1, :] = grid.l_occ * 3.0
    grid.log_odds[:, 0] = grid.l_occ * 3.0
    grid.log_odds[:, -1] = grid.l_occ * 3.0

    # Ensure no frontier clusters exist
    assert grid.is_free(10, 10)
    assert not grid.is_unknown(10, 10)

    coord = MissionCoordinator(start_world=(2.0, 2.0), grid_map=grid, flight=flight)

    # Step into TAKEOFF then EXPLORE
    coord.update(current_time_s=0.1, drone_state=_make_drone_state(0.1, z=0.0), ranges={})
    coord.update(current_time_s=0.5, drone_state=_make_drone_state(0.5, z=1.2), ranges={})
    assert coord.state == MissionState.EXPLORE

    # On next update in EXPLORE, 0 clusters exist -> must transition to RETURN_HOME
    coord.update(current_time_s=1.0, drone_state=_make_drone_state(1.0, z=1.2), ranges={})
    assert coord.state == MissionState.RETURN_HOME
    assert "EXPLORATION_FINISHED" in coord.end_cause


def test_configurable_min_cluster_size():
    """Verify exploration honors configurable minimum cluster size."""
    flight = MockFlight()

    def make_map_with_small_unknown_spot():
        grid = OccupancyGrid(width_m=8.0, height_m=8.0, resolution_m=0.25)
        # All free
        grid.log_odds[:] = grid.l_free * 3.0
        # Perimeter walls
        grid.log_odds[0, :] = grid.l_occ * 3.0
        grid.log_odds[-1, :] = grid.l_occ * 3.0
        grid.log_odds[:, 0] = grid.l_occ * 3.0
        grid.log_odds[:, -1] = grid.l_occ * 3.0
        # Exactly 2 unknown cells in the corner: (gy=2, gx=2) and (gy=2, gx=3)
        grid.log_odds[2, 2] = 0.0  # unknown
        grid.log_odds[2, 3] = 0.0  # unknown
        return grid

    # Map with 2 unknown cells
    grid_small = make_map_with_small_unknown_spot()

    # Case 1: min_cluster_size = 5 (larger than the 2-cell spot)
    # The spot does not meet the minimum cluster threshold -> 0 clusters remain -> finishes exploration
    coord_large_thresh = MissionCoordinator(
        start_world=(4.0, 4.0),
        grid_map=grid_small,
        flight=flight,
        frontier_min_cluster_size=5,
    )
    coord_large_thresh.update(0.1, _make_drone_state(0.1, x=4.0, y=4.0, z=0.0), {})
    coord_large_thresh.update(0.5, _make_drone_state(0.5, x=4.0, y=4.0, z=1.2), {})
    coord_large_thresh.update(1.0, _make_drone_state(1.0, x=4.0, y=4.0, z=1.2), {})
    assert coord_large_thresh.state == MissionState.RETURN_HOME
    assert "EXPLORATION_FINISHED" in coord_large_thresh.end_cause

    # Case 2: min_cluster_size = 2 (matches the spot)
    grid_small2 = make_map_with_small_unknown_spot()
    coord_small_thresh = MissionCoordinator(
        start_world=(4.0, 4.0),
        grid_map=grid_small2,
        flight=flight,
        frontier_min_cluster_size=2,
    )
    coord_small_thresh.update(0.1, _make_drone_state(0.1, x=4.0, y=4.0, z=0.0), {})
    coord_small_thresh.update(0.5, _make_drone_state(0.5, x=4.0, y=4.0, z=1.2), {})
    coord_small_thresh.update(1.0, _make_drone_state(1.0, x=4.0, y=4.0, z=1.2), {})
    # Should stay in EXPLORE and pursue the valid cluster
    assert coord_small_thresh.state == MissionState.EXPLORE
