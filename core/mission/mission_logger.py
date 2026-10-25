"""Mission Logger for tracking exploration statistics and benchmarking."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple
import numpy as np


def determine_exit_reason(
    base_exit_reason: str,
    landed: bool,
    coverage_pct: float,
    frontier_clusters_remaining: int,
    home_error_m: float,
    coverage_target_pct: float = 90.0,
) -> str:
    """Determine final exit reason according to Task A7 rules:
    - COMPLETE only if (coverage target met OR no reachable frontiers) AND home_error_m < 1.0.
    - Landed but not meeting that = INCOMPLETE.
    - Others: TIMEOUT, FAILSAFE, CRASH.
    """
    if base_exit_reason in ("TIMEOUT", "FAILSAFE", "CRASH"):
        return base_exit_reason

    coverage_met = coverage_pct >= coverage_target_pct
    no_frontiers = frontier_clusters_remaining == 0
    home_ok = home_error_m < 1.0

    if landed and (coverage_met or no_frontiers) and home_ok:
        return "COMPLETE"
    else:
        return "INCOMPLETE"


@dataclass
class MissionMetrics:
    profile: str = "baseline"
    flight_time_s: float = 0.0
    total_time_s: float = 0.0
    path_length_m: float = 0.0
    distance_traveled_m: float = 0.0
    coverage_pct: float = 0.0
    coverage_percentage: float = 0.0
    survivors_found: int = 0
    survivors_total: int = 0
    total_survivors: int = 0
    false_positives: int = 0
    home_error_m: float = 0.0
    return_home_error_m: float = 0.0
    mean_drift_m: float = 0.0
    max_drift_m: float = 0.0
    collision_events: int = 0
    collisions: int = 0
    took_off: bool = True
    pose_valid: bool = True
    pose_estimate_valid: bool = True
    frontier_clusters_remaining: int = 0
    frontiers_seen: int = 0
    exit_reason: str = "COMPLETE"
    exit_code: int = 0
    success: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def save_json(self, path: str | Path):
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2))


class MissionLogger:
    def __init__(self, total_free_cells: int, true_survivors: List[Tuple[float, float]]):
        self.total_free_cells = max(1, total_free_cells)
        self.true_survivors = true_survivors

        self.drifts: List[float] = []
        self.traj_est: List[Tuple[float, float]] = []
        self.traj_true: List[Tuple[float, float]] = []
        self.collisions: int = 0
        self.in_collision: bool = False
        self.last_collision_time: float = -10.0
        self.max_alt_true: float = 0.0

    def record_step(
        self,
        est_pos: np.ndarray,
        true_pos: np.ndarray,
        is_collision: bool = False,
        sim_time: float = 0.0,
    ):
        drift = float(np.linalg.norm(est_pos[:2] - true_pos[:2]))
        self.drifts.append(drift)
        self.traj_est.append((float(est_pos[0]), float(est_pos[1])))
        self.traj_true.append((float(true_pos[0]), float(true_pos[1])))
        if len(true_pos) >= 3:
            self.max_alt_true = max(self.max_alt_true, float(true_pos[2]))
        if is_collision:
            if not self.in_collision:
                self.collisions += 1
                self.last_collision_time = sim_time
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
        frontier_clusters_remaining: int = 0,
        frontiers_seen: int = 0,
        exit_reason: str = "COMPLETE",
        landed: Optional[bool] = None,
        coverage_target_pct: float = 90.0,
        profile: str = "baseline",
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

        # Assess takeoff and pose validity (protocol: mean drift < 1.0 m)
        took_off = (dist > 0.5) or (self.max_alt_true >= 0.3) or (len(final_true_pos) >= 3 and final_true_pos[2] >= 0.3)
        pose_valid = (mean_drift < 1.0) and not any(
            math.isnan(p[0]) or math.isnan(p[1]) for p in self.traj_est
        )

        # Determine exit reason per protocol A7 rules
        is_landed = landed if landed is not None else (final_true_pos[2] <= 0.25 and took_off)
        final_exit_reason = determine_exit_reason(
            base_exit_reason=exit_reason,
            landed=is_landed,
            coverage_pct=cov_pct,
            frontier_clusters_remaining=frontier_clusters_remaining,
            home_error_m=rth_err,
            coverage_target_pct=coverage_target_pct,
        )

        exit_code = 0 if final_exit_reason == "COMPLETE" else 1
        success = (final_exit_reason == "COMPLETE")

        return MissionMetrics(
            profile=profile,
            flight_time_s=flight_time_s,
            total_time_s=flight_time_s,
            path_length_m=dist,
            distance_traveled_m=dist,
            coverage_pct=cov_pct,
            coverage_percentage=cov_pct,
            survivors_found=found,
            survivors_total=len(self.true_survivors),
            total_survivors=len(self.true_survivors),
            false_positives=false_positives,
            home_error_m=rth_err,
            return_home_error_m=rth_err,
            mean_drift_m=mean_drift,
            max_drift_m=max_drift,
            collision_events=self.collisions,
            collisions=self.collisions,
            took_off=took_off,
            pose_valid=pose_valid,
            pose_estimate_valid=pose_valid,
            frontier_clusters_remaining=frontier_clusters_remaining,
            frontiers_seen=frontiers_seen,
            exit_reason=final_exit_reason,
            exit_code=exit_code,
            success=success,
        )
