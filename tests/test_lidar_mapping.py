"""Unit test for Lidar Occupancy Mapping (Task D4).

Validates:
- Lidar scan updates occupancy grid log-odds correctly.
- Free space marked along each ray.
- Occupied cell marked at the hit point.
- Max-range returns skip the occupied update (no false walls in open space).
- Known scene with true pose achieves >= 95% classification accuracy.
"""
import math
import numpy as np
import pytest

from airmouse.core.hal import LidarScan
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.sensor_sim import SimulatedLidar2D


def test_lidar_mapping_known_scene_accuracy():
    """CHECK requirement: unit test with a known scene and true pose: >= 95% of cells classified correctly."""
    # 10x10 m room with outer perimeter walls
    sim_grid = np.zeros((10, 10), dtype=np.uint8)
    sim_grid[0, :] = sim_grid[-1, :] = 1
    sim_grid[:, 0] = sim_grid[:, -1] = 1

    # Place drone at center (5.0, 5.0, 1.2m)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)

    lidar = SimulatedLidar2D(
        drone=drone,
        grid=sim_grid,
        cell_size_m=1.0,
        get_time_fn=lambda: 0.0,
        rate_hz=8.0,
        num_rays=360,
        range_min=0.15,
        range_max=12.0,
        noise_sigma=0.01,
        dropout_prob=0.0,
    )
    scan = lidar.read()
    assert scan is not None

    # Map with resolution 0.25m: 40x40 cells
    grid_map = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)
    grid_map.update_lidar_scan(
        start_wx=5.0,
        start_wy=5.0,
        yaw_rad=0.0,
        scan=scan,
    )

    # Evaluate classification against the known simulation ground truth
    total_evaluated = 0
    correct_classified = 0

    for gy in range(grid_map.rows):
        for gx in range(grid_map.cols):
            # Evaluate all observed cells (cells updated by lidar scan)
            if grid_map.log_odds[gy, gx] == 0.0:
                continue

            total_evaluated += 1
            wx, wy = grid_map.grid_to_world(gx, gy)
            cx = int(wx / 1.0)
            cy = int(10.0 - (wy / 1.0))
            is_true_occ = (0 <= cx < 10 and 0 <= cy < 10 and sim_grid[cy, cx] != 0)
            is_map_occ = (grid_map.log_odds[gy, gx] > 0.0)

            if is_true_occ == is_map_occ:
                correct_classified += 1

    assert total_evaluated > 1000, f"Expected >1000 observed cells, got {total_evaluated}"
    accuracy = correct_classified / total_evaluated
    assert accuracy >= 0.95, f"Lidar classification accuracy {accuracy*100:.1f}% below 95% threshold ({correct_classified}/{total_evaluated})"


def test_lidar_mapping_skips_max_range_occupied_update():
    """Verify max-range returns (open space/dropouts) mark free space but NEVER add occupied cells."""
    grid_map = OccupancyGrid(width_m=20.0, height_m=20.0, resolution_m=0.25)

    # Synthetic scan: ray 0 has a valid hit at 5.0m; ray 1 is max-range (12.0m)
    ranges = np.array([5.0, 12.0], dtype=np.float32)
    scan = LidarScan(
        ranges=ranges,
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.15,
        range_max=12.0,
        timestamp=0.0,
    )

    grid_map.update_lidar_scan(start_wx=10.0, start_wy=10.0, yaw_rad=0.0, scan=scan)

    # Ray 0 (angle 0, East): hit at wx=15.0, wy=10.0
    hit_gx, hit_gy = grid_map.world_to_grid(15.05, 10.0)
    assert grid_map.is_occupied(hit_gx, hit_gy), "Ray 0 hit point must be occupied"

    # Free space along Ray 0: wx=12.0, wy=10.0
    free_gx, free_gy = grid_map.world_to_grid(12.0, 10.0)
    assert grid_map.is_free(free_gx, free_gy), "Ray 0 midpoint must be free"

    # Ray 1 (angle 0.1, ~12.0m): end point at wx=10 + 12*cos(0.1) ~ 21.9m
    # Any cell along Ray 1 within bounds should be free, but NO cell along Ray 1 should be occupied
    cos_1 = math.cos(0.1)
    sin_1 = math.sin(0.1)
    for d in [2.0, 4.0, 6.0, 8.0]:
        gx, gy = grid_map.world_to_grid(10.0 + d * cos_1, 10.0 + d * sin_1)
        if grid_map.in_bounds(gx, gy):
            assert grid_map.is_free(gx, gy), f"Midpoint at d={d} along max-range ray must be free"

    # Verify no cell in the grid has log_odds > 0 other than the hit point of Ray 0
    occ_cells = np.argwhere(grid_map.log_odds > 0.0)
    assert len(occ_cells) >= 1
    for gy, gx in occ_cells:
        wx, wy = grid_map.grid_to_world(gx, gy)
        # All occupied cells must be near Ray 0's hit (wx=15.0, wy=10.0)
        assert abs(wx - 15.05) <= 0.35 and abs(wy - 10.0) <= 0.35
