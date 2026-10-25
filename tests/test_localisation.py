"""Unit tests for survivor 3D localization from camera detections."""
from __future__ import annotations

import math
from typing import Tuple
import numpy as np
import pytest

from airmouse.core.perception.ray_projector import GroundRayProjector
from airmouse.core.perception.yolo_detector import DetectionBox


def forward_project_to_pixel(
    target_world: Tuple[float, float, float],
    drone_pos: np.ndarray,
    drone_yaw: float,
    drone_pitch: float,
    cam_pitch_down_rad: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float] | None:
    """Analytical forward projection of a 3D point into camera image coordinates."""
    dx = target_world[0] - drone_pos[0]
    dy = target_world[1] - drone_pos[1]
    dz = target_world[2] - drone_pos[2]

    # Rotate by yaw around world Z
    cos_y = math.cos(-drone_yaw)
    sin_y = math.sin(-drone_yaw)
    x_rot = dx * cos_y - dy * sin_y
    y_rot = dx * sin_y + dy * cos_y

    total_pitch = drone_pitch + cam_pitch_down_rad
    cos_p = math.cos(total_pitch)
    sin_p = math.sin(total_pitch)

    z_cam = x_rot * cos_p - dz * sin_p
    x_cam = -y_rot
    y_cam = -x_rot * sin_p - dz * cos_p

    if z_cam <= 0.1:
        return None

    u = cx + (fx * x_cam) / z_cam
    v = cy + (fy * y_cam) / z_cam
    return (u, v)


def localize_detection_to_ground(
    box: DetectionBox,
    projector: GroundRayProjector,
    drone_pos: np.ndarray,
    drone_yaw: float,
    drone_pitch: float = 0.0,
    drone_roll: float = 0.0,
) -> Tuple[float, float] | None:
    """Localize person detection box on ground plane (Z=0) from drone state and camera intrinsics."""
    u_b, v_b = box.bottom_center
    return projector.project_pixel_to_ground(
        pixel_u=u_b,
        pixel_v=v_b,
        drone_pos_world=drone_pos,
        drone_yaw=drone_yaw,
        drone_pitch=drone_pitch,
        drone_roll=drone_roll,
    )


class TestSurvivorLocalisation:
    @pytest.fixture
    def setup_projector(self):
        fx = 228.5
        fy = 228.5
        cx = 160.0
        cy = 120.0
        cam_pitch_deg = 12.0
        projector = GroundRayProjector(
            focal_length_px=fx,
            img_w=320,
            img_h=240,
            camera_pitch_down_deg=cam_pitch_deg,
        )
        return projector, fx, fy, cx, cy, math.radians(cam_pitch_deg)

    @pytest.mark.parametrize(
        "drone_pos,drone_yaw,drone_pitch,drone_roll,target_ground,description",
        [
            (
                np.array([0.0, 0.0, 1.2]),
                0.0,
                0.0,
                0.0,
                (3.0, 0.0),
                "Straight East, 3.0m distance",
            ),
            (
                np.array([5.0, 5.0, 1.5]),
                math.pi / 2.0,
                0.0,
                0.0,
                (5.0, 8.5),
                "Straight North, 3.5m distance",
            ),
            (
                np.array([10.0, 12.0, 1.0]),
                math.pi,
                0.0,
                0.0,
                (7.5, 12.0),
                "Straight West, 2.5m distance",
            ),
            (
                np.array([8.0, 4.0, 1.3]),
                -math.pi / 2.0,
                0.0,
                0.0,
                (8.0, 1.0),
                "Straight South, 3.0m distance",
            ),
            (
                np.array([2.0, 3.0, 1.2]),
                math.pi / 4.0,
                0.0,
                0.0,
                (4.5, 5.5),
                "Diagonal North-East 45 deg, 3.54m distance",
            ),
            (
                np.array([5.0, 5.0, 1.2]),
                0.0,
                0.0,
                0.0,
                (7.0, 6.0),
                "Off-center left viewing angle",
            ),
            (
                np.array([5.0, 5.0, 1.2]),
                0.0,
                0.0,
                0.0,
                (7.0, 4.0),
                "Off-center right viewing angle",
            ),
            (
                np.array([6.0, 8.0, 1.8]),
                0.8,
                0.04,
                -0.02,
                (8.5, 10.2),
                "Flight dynamics: non-zero pitch and roll",
            ),
            (
                np.array([12.0, 6.0, 0.7]),
                -1.2,
                0.0,
                0.0,
                (13.2, 3.5),
                "Low altitude flight: 0.7m",
            ),
            (
                np.array([12.0, 6.0, 2.2]),
                -1.2,
                0.0,
                0.0,
                (14.0, 2.5),
                "High altitude flight: 2.2m",
            ),
        ],
    )
    def test_localisation_accuracy_under_threshold(
        self, setup_projector, drone_pos, drone_yaw, drone_pitch, drone_roll, target_ground, description
    ):
        projector, fx, fy, cx, cy, cam_pitch_rad = setup_projector
        tx, ty = target_ground

        # Compute ground contact point pixel projection
        pixel_pt = forward_project_to_pixel(
            target_world=(tx, ty, 0.0),
            drone_pos=drone_pos,
            drone_yaw=drone_yaw,
            drone_pitch=drone_pitch,
            cam_pitch_down_rad=cam_pitch_rad,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
        assert pixel_pt is not None, f"Target ({tx}, {ty}) projected behind camera for {description}"
        u_b, v_b = pixel_pt

        # Synthesize a realistic YOLO bounding box around this ground contact point
        # Height typically 40-100 pixels depending on distance
        box_h = 60.0
        box_w = 24.0
        synthetic_box = DetectionBox(
            x1=u_b - box_w / 2.0,
            y1=v_b - box_h,
            x2=u_b + box_w / 2.0,
            y2=v_b,
            confidence=0.85,
            class_name="person",
        )

        # Run localization pipeline
        est_ground = localize_detection_to_ground(
            box=synthetic_box,
            projector=projector,
            drone_pos=drone_pos,
            drone_yaw=drone_yaw,
            drone_pitch=drone_pitch,
            drone_roll=drone_roll,
        )
        assert est_ground is not None, f"Localization failed to produce ground point for {description}"

        est_x, est_y = est_ground
        error = math.hypot(est_x - tx, est_y - ty)

        print(f"[{description}] Target: ({tx:.2f}, {ty:.2f}) -> Est: ({est_x:.2f}, {est_y:.2f}) | Error: {error:.4f}m")
        # Specification threshold: error < 0.5 m on all test cases
        assert error < 0.5, f"Localization error {error:.4f}m exceeds 0.5m limit for {description}"
