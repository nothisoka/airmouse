"""Test occupancy grid log-odds updates and obstacle inflation."""
import numpy as np
import pytest
from airmouse.core.mapping.occupancy_grid import OccupancyGrid


def test_occupancy_grid():
    grid = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)

    # Initial state: all cells unknown (p=0.5)
    gx, gy = grid.world_to_grid(5.0, 5.0)
    assert abs(grid.get_probability(gx, gy) - 0.5) < 0.01
    assert grid.is_unknown(gx, gy)

    # Update beam from (1.0, 5.0) pointing East (angle 0.0) hitting obstacle at 3.0m
    grid.update_beam(start_wx=1.0, start_wy=5.0, angle_rad=0.0, range_m=3.0, valid_hit=True)

    # Cell along the beam at x=2.5m should be marked FREE (prob < free_thresh)
    fx, fy = grid.world_to_grid(2.5, 5.0)
    assert grid.get_probability(fx, fy) < 0.35
    assert grid.is_free(fx, fy)

    # Cell at hit location x=4.0m should be marked OCCUPIED (prob > occ_thresh)
    hx, hy = grid.world_to_grid(4.0, 5.0)
    assert grid.get_probability(hx, hy) > 0.65
    assert grid.is_occupied(hx, hy)

    # Test obstacle inflation
    costmap = grid.get_inflated_costmap(inflation_radius_m=0.35)
    # The cell at the obstacle and adjacent should be inflated (> 0)
    assert costmap[hy, hx] >= 200
    adj_x, adj_y = grid.world_to_grid(3.8, 5.0)
    assert costmap[adj_y, adj_x] > 0

