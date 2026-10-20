"""Frontier-Based Exploration Algorithm for Unknown Indoor Environments.

Detects boundary cells between explored FREE space and UNKNOWN space,
clusters frontiers, evaluates information gain vs travel distance, and
manages blacklists for unreachable frontiers.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
from airmouse.core.mapping.occupancy_grid import OccupancyGrid


@dataclass
class FrontierCluster:
    centroid_world: Tuple[float, float]
    cells: List[Tuple[int, int]]
    size: int
    score: float = 0.0


class FrontierDetector:
    def __init__(
        self,
        min_cluster_size: int = 2,
        dist_weight: float = 0.8,
        info_weight: float = 1.0,
        blacklist_max_attempts: int = 3,
    ):
        self.min_cluster_size = min_cluster_size
        self.dist_weight = dist_weight
        self.info_weight = info_weight
        self.blacklist_max_attempts = blacklist_max_attempts

        # Failure count per grid cell target: (gx, gy) -> fail_count
        self.blacklist_counts: Dict[Tuple[int, int], int] = {}

    def report_failure(self, target_world: Tuple[float, float], grid_map: OccupancyGrid):
        gx, gy = grid_map.world_to_grid(target_world[0], target_world[1])
        self.blacklist_counts[(gx, gy)] = self.blacklist_counts.get((gx, gy), 0) + 1

    def find_candidate_frontiers(
        self,
        grid_map: OccupancyGrid,
        drone_pos_world: Tuple[float, float],
        top_k: int = 5,
    ) -> List[Tuple[float, float]]:
        """Return top-K highest-utility reachable frontier centroids."""
        frontier_cells = self._extract_frontier_cells(grid_map)
        if not frontier_cells:
            return []

        clusters = self._cluster_frontiers(frontier_cells, grid_map)
        if not clusters:
            return []

        dx_w, dy_w = drone_pos_world
        scored = []

        for cluster in clusters:
            cx_w, cy_w = cluster.centroid_world
            gx, gy = grid_map.world_to_grid(cx_w, cy_w)

            # Check blacklist
            if self.blacklist_counts.get((gx, gy), 0) >= self.blacklist_max_attempts:
                continue

            dist = math.hypot(cx_w - dx_w, cy_w - dy_w)
            if dist < 0.3:
                continue

            # Super-linear size scaling rewards entering large unexplored rooms over wall alcoves
            score = self.info_weight * (cluster.size ** 1.3) - self.dist_weight * (dist * 2.5)
            scored.append((score, cluster.centroid_world))

        if not scored and self.blacklist_counts:
            # If all frontiers were blocked or failed from previous poses, clear blacklist so newly mapped space is retried
            self.blacklist_counts.clear()
            for cluster in clusters:
                cx_w, cy_w = cluster.centroid_world
                dist = math.hypot(cx_w - dx_w, cy_w - dy_w)
                if dist >= 0.3:
                    score = self.info_weight * (cluster.size ** 1.3) - self.dist_weight * (dist * 2.5)
                    scored.append((score, cluster.centroid_world))

        scored.sort(key=lambda item: item[0], reverse=True)
        return [centroid for _, centroid in scored[:top_k]]

    def find_best_frontier(
        self,
        grid_map: OccupancyGrid,
        drone_pos_world: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        """Identify and return the highest-utility reachable frontier centroid."""
        candidates = self.find_candidate_frontiers(grid_map, drone_pos_world, top_k=1)
        return candidates[0] if candidates else None

    def _extract_frontier_cells(self, grid_map: OccupancyGrid) -> List[Tuple[int, int]]:
        """Find FREE cells that have at least one UNKNOWN 8-neighbor."""
        frontiers = []
        costmap = grid_map.get_inflated_costmap()

        # Step through map cells
        for gy in range(1, grid_map.rows - 1):
            for gx in range(1, grid_map.cols - 1):
                # Must be free and outside lethal obstacle zone
                if grid_map.is_free(gx, gy) and costmap[gy, gx] < 200:
                    # Check 8 neighbors for unknown space
                    has_unknown_neighbor = False
                    for dx, dy in ((1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)):
                        nx, ny = gx + dx, gy + dy
                        if grid_map.is_unknown(nx, ny):
                            has_unknown_neighbor = True
                            break
                    if has_unknown_neighbor:
                        frontiers.append((gx, gy))

        return frontiers

    def _cluster_frontiers(
        self,
        frontier_cells: List[Tuple[int, int]],
        grid_map: OccupancyGrid,
    ) -> List[FrontierCluster]:
        """Cluster adjacent frontier cells using BFS."""
        cell_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            q = deque([cell])
            visited.add(cell)
            cluster_cells = []

            while q:
                cur = q.popleft()
                cluster_cells.append(cur)

                cx, cy = cur
                for dx, dy in ((1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,-1), (1,-1), (-1,1)):
                    nbr = (cx + dx, cy + dy)
                    if nbr in cell_set and nbr not in visited:
                        visited.add(nbr)
                        q.append(nbr)

            if len(cluster_cells) >= self.min_cluster_size:
                # Compute centroid
                mean_gx = sum(c[0] for c in cluster_cells) / len(cluster_cells)
                mean_gy = sum(c[1] for c in cluster_cells) / len(cluster_cells)
                centroid_w = grid_map.grid_to_world(int(round(mean_gx)), int(round(mean_gy)))

                clusters.append(FrontierCluster(
                    centroid_world=centroid_w,
                    cells=cluster_cells,
                    size=len(cluster_cells),
                ))

        return clusters

