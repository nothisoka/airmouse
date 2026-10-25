#!/usr/bin/env python3
"""Perception Check: YOLO Person Detection on Simulated Camera Frames.

Runs a headless mission, saves 10 camera frames in which a person is in view
to generated_maps/frames/, runs YOLO (weights path from config/mission.yaml)
on them, and prints per-frame detections, confidence scores, and overall detection rate.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from airmouse.core.mapping.map_io import load_map_file
from airmouse.core.hal import DroneState, PoseQuality
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub
from airmouse.backends.pygame_sim.flight_sim import SimulatedFlightInterface
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState
from airmouse.core.mapping.occupancy_grid import OccupancyGrid


def load_config() -> dict:
    cfg_path = PROJECT_ROOT / "config" / "mission.yaml"
    if cfg_path.exists():
        with open(cfg_path, "r") as f:
            return yaml.safe_load(f) or {}
    return {}


def is_person_in_view(
    renderer: WorldRenderer,
    sensor_hub: SimulatedSensorHub,
    pos: np.ndarray,
    yaw: float,
    pitch: float,
    survivors: List[Tuple[float, float]],
) -> Tuple[bool, Optional[Tuple[float, float]], float]:
    """Check if any survivor is currently in view of the camera frustum with clear line of sight."""
    total_pitch = pitch + renderer.cam_pitch_down
    for sx, sy in survivors:
        dist = math.hypot(sx - pos[0], sy - pos[1])
        if dist < 0.6 or dist > 5.5:
            continue
        p_feet = renderer._project_world_to_cam(sx, sy, 0.0, pos, yaw, total_pitch)
        p_head = renderer._project_world_to_cam(sx, sy, 1.7, pos, yaw, total_pitch)
        if p_feet is None or p_head is None:
            continue
        uf, vf = p_feet
        uh, vh = p_head
        if 0 <= uf < renderer.w and 0 <= uh < renderer.w:
            height_px = vf - vh
            if height_px >= 20:
                angle_to_person = math.atan2(sy - pos[1], sx - pos[0])
                los = sensor_hub._cast_ray_2d(pos[0], pos[1], angle_to_person, dist)
                if los >= dist - 0.35:
                    return True, (sx, sy), dist
    return False, None, 0.0


def run_headless_frame_capture(
    map_path: str,
    frames_dir: Path,
    target_frame_count: int = 10,
    seed: int = 42,
) -> List[Path]:
    """Run a headless mission and capture frames containing a person in view."""
    frames_dir.mkdir(parents=True, exist_ok=True)
    for p in frames_dir.glob("frame_*.png"):
        try:
            p.unlink()
        except OSError:
            pass

    true_grid, metadata = load_map_file(map_path)
    start_wx, start_wy = metadata.start_world
    survivors = metadata.true_survivors

    if not survivors:
        raise ValueError(f"Map {map_path} does not contain any survivor annotations.")

    drone = DroneDynamics(
        initial_position=np.array([start_wx, start_wy, 0.0]),
        initial_yaw=0.0,
        grid=true_grid,
        cell_size_m=metadata.cell_size_m,
    )
    renderer = WorldRenderer(
        grid=true_grid,
        cell_size_m=metadata.cell_size_m,
        img_w=320,
        img_h=240,
    )
    renderer.true_survivors = survivors

    sensor_hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=true_grid,
        cell_size_m=metadata.cell_size_m,
    )
    flight = SimulatedFlightInterface(drone=drone)
    grid_map = OccupancyGrid(
        width_m=metadata.cols * metadata.cell_size_m,
        height_m=metadata.rows * metadata.cell_size_m,
        resolution_m=0.25,
    )
    mission = MissionCoordinator(
        start_world=(start_wx, start_wy),
        grid_map=grid_map,
        flight=flight,
        max_flight_time_s=600.0,
    )

    saved_frame_paths: List[Path] = []
    last_saved_time = -10.0
    sim_dt = 0.01

    print(f"Running headless mission on {Path(map_path).name} with {len(survivors)} hidden survivors...")

    # Fly headless exploration mission, checking for persons in view
    step = 0
    max_steps = 4000  # Up to 40 seconds of mission flight
    while step < max_steps and len(saved_frame_paths) < target_frame_count:
        sim_time = step * sim_dt
        step += 1
        sensor_hub.advance_time(sim_dt)

        state_phys = drone.get_state()
        drone_state = DroneState(
            timestamp_s=sim_time,
            position=state_phys.position.copy(),
            velocity=state_phys.velocity.copy(),
            attitude_rpy=state_phys.attitude_rpy.copy(),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
            accel_bias=np.zeros(3),
            gyro_bias=np.zeros(3),
            covariance_diag=np.zeros(9),
            quality=PoseQuality.EXCELLENT,
        )

        ranges = sensor_hub.read_ranges()
        map_x, map_y = drone_state.position[0], drone_state.position[1]
        map_yaw = drone_state.attitude_rpy[2]
        yaw_offsets = {
            "front": 0.0,
            "right": -math.pi / 2.0,
            "rear": math.pi,
            "left": math.pi / 2.0,
        }
        for sid, offset in yaw_offsets.items():
            if sid in ranges:
                grid_map.update_beam(
                    start_wx=map_x,
                    start_wy=map_y,
                    angle_rad=map_yaw + offset,
                    range_m=ranges[sid].range_m,
                    valid_hit=ranges[sid].valid,
                )

        cam = sensor_hub.read_camera()
        if cam is not None and sim_time - last_saved_time >= 0.5:
            pos = state_phys.position
            yaw = state_phys.attitude_rpy[2]
            pitch = state_phys.attitude_rpy[1]
            in_view, person_pos, dist = is_person_in_view(
                renderer, sensor_hub, pos, yaw, pitch, survivors
            )
            if in_view:
                idx = len(saved_frame_paths) + 1
                frame_path = frames_dir / f"frame_{idx:02d}.png"
                cv2.imwrite(str(frame_path), cam.image_bgr)
                saved_frame_paths.append(frame_path)
                last_saved_time = sim_time
                print(f"  [Capture] Frame {idx:02d}: survivor at ({person_pos[0]:.1f}, {person_pos[1]:.1f}), dist={dist:.2f}m, drone=({pos[0]:.2f}, {pos[1]:.2f})")

        cmd = mission.update(
            current_time_s=sim_time,
            drone_state=drone_state,
            ranges=ranges,
        )
        drone.set_command(cmd)
        drone.step(sim_dt)

        if mission.state == MissionState.COMPLETE:
            break

    # If autonomous path did not visit all survivors within time limit, visit survivor vantage points
    if len(saved_frame_paths) < target_frame_count:
        print(f"Sampling remaining viewpoints from survivor vantage points ({len(saved_frame_paths)}/{target_frame_count} collected)...")
        for s_idx, (sx, sy) in enumerate(survivors):
            if len(saved_frame_paths) >= target_frame_count:
                break
            # Sample vantage points in free space around the survivor
            offsets = [
                (0.0, 1.8),
                (0.0, 2.4),
                (-1.2, 1.8),
                (1.2, 1.8),
                (0.0, 3.0),
            ]
            for dx, dy in offsets:
                if len(saved_frame_paths) >= target_frame_count:
                    break
                vx, vy = sx + dx, sy + dy
                gx, gy = int(vx / metadata.cell_size_m), int(metadata.rows - 1 - vy / metadata.cell_size_m)
                if 0 <= gy < true_grid.shape[0] and 0 <= gx < true_grid.shape[1]:
                    if true_grid[gy, gx] != 0:
                        continue
                pos = np.array([vx, vy, 1.2])
                yaw = math.atan2(sy - vy, sx - vx)
                pitch = 0.0
                in_view, person_pos, dist = is_person_in_view(
                    renderer, sensor_hub, pos, yaw, pitch, survivors
                )
                if in_view:
                    frame = renderer.render_camera_frame(
                        drone_pos=pos,
                        drone_yaw=yaw,
                        drone_pitch=pitch,
                        drone_roll=0.0,
                        survivors=survivors,
                    )
                    idx = len(saved_frame_paths) + 1
                    frame_path = frames_dir / f"frame_{idx:02d}.png"
                    cv2.imwrite(str(frame_path), frame)
                    saved_frame_paths.append(frame_path)
                    print(f"  [Capture] Frame {idx:02d}: survivor at ({person_pos[0]:.1f}, {person_pos[1]:.1f}), dist={dist:.2f}m, vantage=({pos[0]:.2f}, {pos[1]:.2f})")

    return saved_frame_paths


def main():
    parser = argparse.ArgumentParser(description="Check YOLO person detection on simulated camera frames.")
    parser.add_argument("--map", default="maps/house_00042.json", help="Path to layout JSON map.")
    parser.add_argument("--frames-dir", default="generated_maps/frames", help="Directory to save frames.")
    parser.add_argument("--count", type=int, default=10, help="Number of frames with person in view.")
    args = parser.parse_args()

    cfg = load_config()
    perception_cfg = cfg.get("perception", {})
    weights_path = perception_cfg.get("model_path", "yolo11n.pt")
    target_class = perception_cfg.get("target_class", "person")
    conf_thresh = perception_cfg.get("confidence_threshold", 0.25)

    print("========================================================")
    print("  NIDAR AIRMOUSE: YOLO Person Detection Verification")
    print(f"  Model Weights: {weights_path}")
    print(f"  Target Class:  {target_class} (conf >= {conf_thresh})")
    print(f"  Map:           {args.map}")
    print("========================================================\n")

    # 1. Verify YOLO loading
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics package is not installed.")
        sys.exit(1)

    weights_file = Path(weights_path)
    if not weights_file.exists():
        # Check relative to project root
        alt_path = PROJECT_ROOT / weights_path
        if alt_path.exists():
            weights_path = str(alt_path)
        else:
            print(f"ERROR: YOLO weights file not found at '{weights_path}'.")
            sys.exit(1)

    try:
        model = YOLO(weights_path)
    except Exception as e:
        print(f"ERROR: Failed to load YOLO weights ({weights_path}): {e}")
        sys.exit(1)

    print(f"YOLO model successfully loaded: {weights_path}\n")

    # 2. Capture frames with person in view
    frames_dir = Path(args.frames_dir)
    frame_paths = run_headless_frame_capture(
        map_path=args.map,
        frames_dir=frames_dir,
        target_frame_count=args.count,
    )

    if len(frame_paths) < args.count:
        print(f"Warning: Only collected {len(frame_paths)}/{args.count} frames.")

    print(f"\nSaved {len(frame_paths)} camera frames to {frames_dir}/\n")

    # 3. Run YOLO inference on each frame and report detections
    print("--- Per-Frame Detection Results ---")
    detections_count = 0
    results_summary = []

    for idx, fpath in enumerate(frame_paths, 1):
        img_bgr = cv2.imread(str(fpath))
        if img_bgr is None:
            print(f"Frame {idx:02d} ({fpath.name}): ERROR loading image")
            continue

        results = model.predict(
            img_bgr,
            imgsz=320,
            conf=conf_thresh,
            verbose=False,
        )

        frame_detections = []
        for r in results:
            names = r.names
            for box in r.boxes:
                cls_id = int(box.cls[0])
                name = names.get(cls_id, str(cls_id))
                conf = float(box.conf[0])
                if name == target_class:
                    b = box.xyxy[0].cpu().numpy()
                    frame_detections.append((conf, (float(b[0]), float(b[1]), float(b[2]), float(b[3]))))

        if frame_detections:
            detections_count += 1
            # Sort by confidence descending
            frame_detections.sort(key=lambda d: d[0], reverse=True)
            best_conf, best_box = frame_detections[0]
            print(f"Frame {idx:02d} ({fpath.name}): DETECTED | conf={best_conf:.3f} | box=({best_box[0]:.1f}, {best_box[1]:.1f}, {best_box[2]:.1f}, {best_box[3]:.1f})")
            results_summary.append((idx, fpath.name, True, best_conf, best_box))
        else:
            print(f"Frame {idx:02d} ({fpath.name}): NOT DETECTED")
            results_summary.append((idx, fpath.name, False, 0.0, None))

    detection_rate = (detections_count / len(frame_paths)) * 100.0 if frame_paths else 0.0

    print("\n========================================================")
    print(f"  Frames Evaluated:   {len(frame_paths)}")
    print(f"  Frames with Person: {len(frame_paths)}")
    print(f"  Successful Detections: {detections_count}")
    print(f"  Detection Rate:     {detection_rate:.1f}% ({detections_count}/{len(frame_paths)})")
    print("========================================================")

    # Return 0 if detection rate >= 80%, else 1
    sys.exit(0 if detection_rate >= 80.0 else 1)


if __name__ == "__main__":
    main()
