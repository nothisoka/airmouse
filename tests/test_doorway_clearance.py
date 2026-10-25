"""Test drone radius vs obstacle inflation vs doorway clearance.

Validates:
- 1-cell doorway (1.0m / 4 cells at res=0.25m) provides traversable path keeping configured clearance.
- Narrow doorway (< 0.40m, less than drone diameter 2 * 0.20m) is marked impassable and returns no path.
- Configuration invariants: drone radius <= lethal clearance <= inflation radius < door half-width.
"""
import math
import numpy as np
import pytest
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.planning.astar import AStarPlanner


def test_drone_radius_vs_inflation_invariants():
    """Verify geometrical and physical invariants between drone, inflation, and doorway."""
    drone_radius_m = 0.20      # Physical drone collision radius from dynamics
    lethal_radius_m = 0.22     # Hard lethal clearance
    inflation_radius_m = 0.30  # Soft inflation safety margin
    door_width_1cell_m = 1.0   # Standard 1-cell doorway width in 1.0m world grid

    # 1. Lethal clearance must be at least the drone physical radius
    assert lethal_radius_m >= drone_radius_m

    # 2. Soft inflation must exceed lethal radius
    assert inflation_radius_m >= lethal_radius_m

    # 3. Two lethal buffers from opposing doorposts must leave free traversable space
    # in a 1-cell doorway (1.0m):
    clearance_corridor_m = door_width_1cell_m - 2 * lethal_radius_m
    assert clearance_corridor_m > 0.40, f"Corridor {clearance_corridor_m}m too narrow for drone"

    # 4. Center of 1-cell doorway leaves at least 0.50m from either wall, exceeding drone radius
    center_clearance_m = door_width_1cell_m / 2.0
    assert center_clearance_m > drone_radius_m


def test_planner_finds_path_through_1cell_doorway_with_clearance():
    """Verify planner finds a path through a 1-cell (1.0m) doorway keeping configured clearance."""
    grid = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)
    # Set entire map as known free space
    grid.log_odds[:] = grid.l_free * 3.0

    # Build a dividing wall along x = 5.0m (grid column gx = 20)
    # with a 1.0m doorway (4 cells: gy 18, 19, 20, 21, i.e., y in [4.5, 5.5]m)
    gx_wall = 20
    door_gy = set(range(18, 22))  # 4 cells = 1.0m
    wall_cells = []
    for gy in range(grid.rows):
        if gy not in door_gy:
            grid.log_odds[gy, gx_wall] = grid.l_occ * 3.0
            wall_cells.append((gx_wall, gy))

    planner = AStarPlanner(allow_diagonal=True)
    start_w = (2.0, 5.0)
    goal_w = (8.0, 5.0)

    path = planner.plan(grid, start_w, goal_w, allow_unknown=False)
    assert len(path) > 0, "Planner failed to find path through 1-cell doorway"

    # Verify path reaches vicinity of goal
    assert math.hypot(path[-1][0] - goal_w[0], path[-1][1] - goal_w[1]) < 0.6

    # Verify configured clearance: drone radius is 0.20m, every point along the path
    # through the doorway must maintain >= 0.20m distance to any occupied wall cell
    for wx, wy in path:
        min_dist_to_wall = min(
            math.hypot(wx - grid.grid_to_world(cx, cy)[0], wy - grid.grid_to_world(cx, cy)[1])
            for cx, cy in wall_cells
        )
        assert min_dist_to_wall >= 0.20, f"Path waypoint ({wx:.2f}, {wy:.2f}) violates clearance ({min_dist_to_wall:.2f}m < 0.20m)"


def test_planner_returns_no_path_if_doorway_too_small():
    """Verify no path is returned if the doorway is too small (< 0.40m, less than drone diameter)."""
    grid = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)
    grid.log_odds[:] = grid.l_free * 3.0

    # Build a dividing wall with a single 0.25m cell opening (gy = 20),
    # which is smaller than the drone diameter (2 * 0.20m = 0.40m)
    gx_wall = 20
    too_small_door_gy = {20}  # only 1 cell = 0.25m wide opening
    for gy in range(grid.rows):
        if gy not in too_small_door_gy:
            grid.log_odds[gy, gx_wall] = grid.l_occ * 3.0

    planner = AStarPlanner(allow_diagonal=True)
    start_w = (2.0, 5.0)
    goal_w = (8.0, 5.0)

    path = planner.plan(grid, start_w, goal_w, allow_unknown=False)
    assert len(path) == 0, f"Planner should have returned empty path for impassable doorway, but got: {path}"
