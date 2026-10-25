"""Test 3D ground projection, survivor deduplication, confirmation count, capping, and wall rejection."""
import numpy as np
import pytest
from airmouse.core.perception.ray_projector import GroundRayProjector
from airmouse.core.perception.survivor_tracker import SurvivorTracker
from airmouse.core.mapping.occupancy_grid import OccupancyGrid


def test_ray_projection():
    projector = GroundRayProjector(focal_length_px=228.5, img_w=320, img_h=240, camera_pitch_down_deg=15.0)
    drone_pos = np.array([5.0, 5.0, 1.2])

    # Center pixel (160, 120): should project onto ground forward along drone heading
    ground_pt = projector.project_pixel_to_ground(
        pixel_u=160.0,
        pixel_v=120.0,
        drone_pos_world=drone_pos,
        drone_yaw=0.0,
    )
    assert ground_pt is not None
    # Looking East (yaw=0), so ground_pt x should be > 5.0 and y should be ~ 5.0
    assert ground_pt[0] > 5.0
    assert abs(ground_pt[1] - 5.0) < 0.2


def test_survivor_dedupe():
    tracker = SurvivorTracker(gate_distance_m=1.0, confirmation_count=3, max_survivors=6)

    # 3 sightings of survivor #1 near (7.0, 5.0)
    tracker.update((7.05, 5.02), 0.75)
    assert len(tracker.confirmed_survivors) == 0  # not yet confirmed (requires 3)
    tracker.update((6.98, 4.95), 0.80)
    tracker.update((7.02, 5.00), 0.85)

    assert len(tracker.confirmed_survivors) == 1
    s1 = tracker.confirmed_survivors[0]
    assert s1.id == 1
    assert abs(s1.estimated_pos[0] - 7.0) < 0.1
    assert abs(s1.estimated_pos[1] - 5.0) < 0.1

    # Sighting of another survivor far away (15.0, 10.0)
    tracker.update((15.0, 10.0), 0.70)
    tracker.update((15.1, 9.9), 0.72)
    tracker.update((14.9, 10.1), 0.74)

    assert len(tracker.confirmed_survivors) == 2


def test_confirmation_requires_n3_consistent_detections():
    """Verify survivor is confirmed ONLY after N=3 detections."""
    tracker = SurvivorTracker(gate_distance_m=1.0, confirmation_count=3, max_survivors=6)

    # 1st detection
    tracker.update((5.0, 5.0), 0.70)
    assert len(tracker.confirmed_survivors) == 0
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].detection_count == 1
    assert not tracker.tracks[0].confirmed

    # 2nd detection
    tracker.update((5.1, 5.0), 0.75)
    assert len(tracker.confirmed_survivors) == 0
    assert tracker.tracks[0].detection_count == 2
    assert not tracker.tracks[0].confirmed

    # 3rd detection -> Confirmation!
    tracker.update((4.9, 5.1), 0.80)
    assert len(tracker.confirmed_survivors) == 1
    assert tracker.tracks[0].detection_count == 3
    assert tracker.tracks[0].confirmed

    # 4th detection updates track without creating duplicate
    tracker.update((5.0, 5.05), 0.85)
    assert len(tracker.confirmed_survivors) == 1
    assert tracker.tracks[0].detection_count == 4


def test_dedupe_within_1m():
    """Verify detections within 1.0m merge, while detections > 1.0m form separate tracks."""
    tracker = SurvivorTracker(gate_distance_m=1.0, confirmation_count=3, max_survivors=6)

    # Sighting at (10.0, 10.0)
    tracker.update((10.0, 10.0), 0.80)

    # Sighting at (10.6, 10.0) -> distance 0.6m <= 1.0m -> merged into Track 1
    tracker.update((10.6, 10.0), 0.82)
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].detection_count == 2

    # Sighting at (11.8, 10.0) -> distance ~1.5m > 1.0m -> creates Track 2
    tracker.update((11.8, 10.0), 0.85)
    assert len(tracker.tracks) == 2
    assert tracker.tracks[1].detection_count == 1


def test_cap_at_6_survivors():
    """Verify tracker strictly caps confirmed survivors at 6."""
    tracker = SurvivorTracker(gate_distance_m=1.0, confirmation_count=3, max_survivors=6)

    # Sighting of 8 distinct survivors, 3 detections each
    for i in range(8):
        pos = (float(i * 5.0), 20.0)
        for _ in range(3):
            tracker.update(pos, 0.90)

    # Total tracks may be stored, but confirmed survivors must strictly cap at 6
    assert len(tracker.confirmed_survivors) == 6
    assert tracker.all_found is True
    # Verify IDs are 1..6
    confirmed_ids = [s.id for s in tracker.confirmed_survivors]
    assert confirmed_ids == [1, 2, 3, 4, 5, 6]


def test_ignore_detections_inside_walls():
    """Verify detections that project inside wall obstacles are rejected."""
    grid_map = OccupancyGrid(width_m=20.0, height_m=20.0, resolution_m=0.25)
    tracker = SurvivorTracker(gate_distance_m=1.0, confirmation_count=3, max_survivors=6, grid_map=grid_map)

    # Mark a 1.0m wall block centered at (4.5, 4.5)
    wall_gx, wall_gy = grid_map.world_to_grid(4.5, 4.5)
    for dgx in range(-1, 2):
        for dgy in range(-1, 2):
            grid_map.log_odds[wall_gy + dgy, wall_gx + dgx] = 5.0  # high log-odds -> p > 0.99

    assert grid_map.is_occupied(wall_gx, wall_gy)

    # Detection inside wall cell -> should be ignored
    tracker.update((4.5, 4.5), 0.95)
    assert len(tracker.tracks) == 0

    # Detection in free space -> should be accepted
    tracker.update((2.0, 2.0), 0.95)
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].positions[0] == (2.0, 2.0)
