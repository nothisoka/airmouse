"""8-Connected A* Path Planner on Inflated Grid Maps."""
from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple
import numpy as np
from airmouse.core.mapping.occupancy_grid import OccupancyGrid


class AStarPlanner:
    def __init__(self, allow_diagonal: bool = True):
        self.allow_diagonal = allow_diagonal
        # Neighbor offsets and movement costs
        if self.allow_diagonal:
            self.neighbors = [
                (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)
            ]
        else:
            self.neighbors = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]

    def plan(
        self,
        grid_map: OccupancyGrid,
        start_w: Tuple[float, float],
        goal_w: Tuple[float, float],
        costmap: Optional[np.ndarray] = None,
        max_iterations: int = 15000,
        allow_unknown: bool = True,
    ) -> List[Tuple[float, float]]:
        """Plan a path from start_w to goal_w in metric world coordinates."""
        if costmap is None:
            costmap = grid_map.get_inflated_costmap()

        sx, sy = grid_map.world_to_grid(start_w[0], start_w[1])
        gx, gy = grid_map.world_to_grid(goal_w[0], goal_w[1])

        if not grid_map.in_bounds(sx, sy) or not grid_map.in_bounds(gx, gy):
            return []

        start = (sx, sy)
        goal = (gx, gy)

        lethal_cost = 200 if allow_unknown else 25

        # If goal or start is on an obstacle, search nearby free cell
        if costmap[gy, gx] >= 200:
            goal = self._find_nearest_free(costmap, goal, grid_map)
            if goal is None:
                return []
        if costmap[sy, sx] >= 200:
            start = self._find_nearest_free(costmap, start, grid_map)
            if start is None:
                return []

        if start == goal:
            return [goal_w]

        # Priority queue stores (f_score, h_score, current_node)
        open_set = []
        heapq.heappush(open_set, (self._heuristic(start, goal), 0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
            _, cur_g, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path_grid = [current]
                while current in came_from:
                    current = came_from[current]
                    path_grid.append(current)
                path_grid.reverse()

                # Convert to metric waypoints and prune collinear points
                waypoints = [grid_map.grid_to_world(cx, cy) for cx, cy in path_grid]
                return self._smooth_path(waypoints)

            cx, cy = current
            for dx, dy, move_cost in self.neighbors:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)

                if not grid_map.in_bounds(nx, ny):
                    continue
                if costmap[ny, nx] >= 200:
                    continue  # Lethal obstacle (impassable)
                if not allow_unknown and costmap[ny, nx] == 25:
                    continue  # Unknown unmapped cell (cannot risk hitting unmapped wall)

                # Step cost + soft buffer inflation penalty (e.g. 50 * 0.04 = 2.0m extra cost to keep clear of walls)
                buffer_penalty = costmap[ny, nx] * 0.04
                tentative_g = cur_g + move_cost * grid_map.res + buffer_penalty
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal) * grid_map.res
                    heapq.heappush(open_set, (f, tentative_g, neighbor))

        return []  # No path found

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Octile / Euclidean distance
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return math.hypot(dx, dy)

    def _find_nearest_free(
        self, costmap: np.ndarray, cell: Tuple[int, int], grid_map: OccupancyGrid
    ) -> Optional[Tuple[int, int]]:
        cx, cy = cell
        for radius in range(1, 10):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = cx + dx, cy + dy
                    if grid_map.in_bounds(nx, ny) and costmap[ny, nx] < 200:
                        return (nx, ny)
        return None

    def _smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Downsample redundant close waypoints while keeping path fidelity."""
        if len(waypoints) <= 2:
            return waypoints

        smoothed = [waypoints[0]]
        for i in range(1, len(waypoints) - 1):
            prev = smoothed[-1]
            curr = waypoints[i]
            dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
            if dist >= 0.35:  # sample every ~0.35m (keeps path clear of wall corners)
                smoothed.append(curr)

        smoothed.append(waypoints[-1])
        return smoothed

