"""Mission Logger for tracking exploration statistics and benchmarking."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np


@dataclass
class MissionMetrics:
    total_time_s: float = 0.0
    distance_traveled_m: float = 0.0
    coverage_percentage: float = 0.0
    survivors_found: int = 0
    total_survivors: int = 0
    false_positives: int = 0
    return_home_error_m: float = 0.0
    mean_drift_m: float = 0.0
    max_drift_m: float = 0.0
    collisions: int = 0


class MissionLogger:
    def __init__(self, total_free_cells: int, true_survivors: List[Tuple[float, float]]):
        self.total_free_cells = max(1, total_free_cells)
        self.true_survivors = true_survivors

        self.drifts: List[float] = []
        self.traj_est: List[Tuple[float, float]] = []
        self.traj_true: List[Tuple[float, float]] = []
        self.collisions: int = 0
        self.in_collision: bool = False

    def record_step(
        self,
        est_pos: np.ndarray,
        true_pos: np.ndarray,
        is_collision: bool = False,
    ):
        drift = float(np.linalg.norm(est_pos[:2] - true_pos[:2]))
        self.drifts.append(drift)
        self.traj_est.append((float(est_pos[0]), float(est_pos[1])))
        self.traj_true.append((float(true_pos[0]), float(true_pos[1])))
        if is_collision:
            if not self.in_collision:
                self.collisions += 1
                self.in_collision = True
        else:
            self.in_collision = False

    def compute_metrics(
        self,
        flight_time_s: float,
        explored_free_cells: int,
        confirmed_survivors: List[Tuple[float, float]],
        start_world: Tuple[float, float],
        final_true_pos: np.ndarray,
    ) -> MissionMetrics:
        # Distance traveled
        dist = 0.0
        for i in range(1, len(self.traj_true)):
            p1, p2 = self.traj_true[i - 1], self.traj_true[i]
            dist += math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        # Coverage
        cov_pct = min(100.0, (explored_free_cells / self.total_free_cells) * 100.0)

        # Survivor matching
        found = 0
        matched_true = set()
        for s_est in confirmed_survivors:
            for idx, s_true in enumerate(self.true_survivors):
                if idx not in matched_true and math.hypot(s_est[0] - s_true[0], s_est[1] - s_true[1]) < 1.5:
                    found += 1
                    matched_true.add(idx)
                    break

        false_positives = max(0, len(confirmed_survivors) - found)

        # Return home error
        rth_err = math.hypot(final_true_pos[0] - start_world[0], final_true_pos[1] - start_world[1])

        mean_drift = float(np.mean(self.drifts)) if self.drifts else 0.0
        max_drift = float(np.max(self.drifts)) if self.drifts else 0.0

        return MissionMetrics(
            total_time_s=flight_time_s,
            distance_traveled_m=dist,
            coverage_percentage=cov_pct,
            survivors_found=found,
            total_survivors=len(self.true_survivors),
            false_positives=false_positives,
            return_home_error_m=rth_err,
            mean_drift_m=mean_drift,
            max_drift_m=max_drift,
            collisions=self.collisions,
        )
