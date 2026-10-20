"""Test frontier detection and A* planning."""
import numpy as np
import pytest
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.planning.astar import AStarPlanner
from airmouse.core.exploration.frontier_detector import FrontierDetector


def test_frontier_and_astar():
    grid = OccupancyGrid(width_m=12.0, height_m=12.0, resolution_m=0.25)

    # Clear a corridor from x=2.0 to x=6.0, y=5.0
    for x in np.arange(2.0, 6.0, 0.5):
        grid.update_beam(start_wx=x, start_wy=5.0, angle_rad=0.0, range_m=2.0, valid_hit=False)

    # Detect frontiers
    detector = FrontierDetector(min_cluster_size=2)
    frontier = detector.find_best_frontier(grid, drone_pos_world=(2.0, 5.0))
    assert frontier is not None
    # Frontier should be ahead of drone near the edge of explored space (x > 3.0)
    assert frontier[0] > 3.0

    # Plan path to frontier
    planner = AStarPlanner()
    path = planner.plan(grid, (2.0, 5.0), frontier)
    assert len(path) > 0
    # First point should start near (2.0, 5.0) and end near the frontier
    assert abs(path[-1][0] - frontier[0]) < 0.6
    assert abs(path[-1][1] - frontier[1]) < 0.6

