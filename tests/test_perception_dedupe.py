"""Test 3D ground projection and survivor deduplication."""
import numpy as np
import pytest
from airmouse.core.perception.ray_projector import GroundRayProjector
from airmouse.core.perception.survivor_tracker import SurvivorTracker


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
    tracker = SurvivorTracker(gate_distance_m=1.2, confirmation_count=3, max_survivors=6)

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

