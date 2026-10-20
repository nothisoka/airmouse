"""Multi-View Survivor Tracking, Distance Gating, and Deduplication."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class SurvivorTrack:
    id: int
    confirmed: bool
    detection_count: int
    positions: List[Tuple[float, float]] = field(default_factory=list)
    confidences: List[float] = field(default_factory=list)

    @property
    def estimated_pos(self) -> Tuple[float, float]:
        mean_x = sum(p[0] for p in self.positions) / len(self.positions)
        mean_y = sum(p[1] for p in self.positions) / len(self.positions)
        return mean_x, mean_y

    @property
    def max_confidence(self) -> float:
        return max(self.confidences) if self.confidences else 0.0


class SurvivorTracker:
    def __init__(
        self,
        gate_distance_m: float = 1.5,
        confirmation_count: int = 2,
        max_survivors: int = 6,
    ):
        self.gate_dist = gate_distance_m
        self.confirm_count = confirmation_count
        self.max_survivors = max_survivors
        self.tracks: List[SurvivorTrack] = []
        self._next_id = 1

    def update(self, projected_ground_pos: Tuple[float, float], confidence: float):
        """Update tracker with a projected ground contact detection."""
        wx, wy = projected_ground_pos

        # Find closest existing track within gating distance
        best_track: Optional[SurvivorTrack] = None
        min_dist = float('inf')

        for track in self.tracks:
            tx, ty = track.estimated_pos
            dist = math.hypot(wx - tx, wy - ty)
            if dist < self.gate_dist and dist < min_dist:
                min_dist = dist
                best_track = track

        if best_track is not None:
            # Associate with existing track
            best_track.positions.append((wx, wy))
            best_track.confidences.append(confidence)
            best_track.detection_count += 1
            if not best_track.confirmed and best_track.detection_count >= self.confirm_count:
                best_track.confirmed = True
        else:
            # Create new track if under limit
            if len(self.tracks) < self.max_survivors * 2:  # allow candidate pool
                new_track = SurvivorTrack(
                    id=self._next_id,
                    confirmed=(self.confirm_count <= 1),
                    detection_count=1,
                    positions=[(wx, wy)],
                    confidences=[confidence],
                )
                self._next_id += 1
                self.tracks.append(new_track)

    @property
    def confirmed_survivors(self) -> List[SurvivorTrack]:
        return [t for t in self.tracks if t.confirmed][:self.max_survivors]

    @property
    def all_found(self) -> bool:
        return len(self.confirmed_survivors) >= self.max_survivors

