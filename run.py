#!/usr/bin/env python3
"""Unified Simulation Runner for NIDAR AirMouse.

Executes autonomous search, sensor fusion, mapping, survivor localization,
and return-to-home across both Pygame and Gazebo backends.

Usage:
    python run.py --map maps/house_00042.json --backend pygame
    python run.py --map maps/house_00042.json --headless
"""
from __future__ import annotations

import argparse
import math
import random
import sys
import time
from pathlib import Path
from typing import Optional
import numpy as np
import yaml

try:
    from airmouse.tools import generate_maps
except ImportError:
    import generate_maps
from airmouse.core.mapping.map_io import load_map_file, cell_to_world
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.perception.yolo_detector import YoloDetector
from airmouse.core.perception.ray_projector import GroundRayProjector
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState
from airmouse.core.mission.mission_logger import MissionLogger
from airmouse.core.hal import DroneState, PoseQuality

from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub
from airmouse.backends.pygame_sim.flight_sim import SimulatedFlightInterface
from airmouse.backends.pygame_sim.pygame_viewer import PygameViewer


def load_mission_config() -> dict:
    cfg_path = Path(__file__).resolve().parent / "config" / "mission.yaml"
    if cfg_path.exists():
        with open(cfg_path, "r") as f:
            return yaml.safe_load(f) or {}
    return {}


def make_drone_state_from_physical(phys, sim_time: float) -> DroneState:
    return DroneState(
        timestamp_s=sim_time,
        position=phys.position.copy(),
        velocity=phys.velocity.copy(),
        attitude_rpy=phys.attitude_rpy.copy(),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3),
        covariance_diag=np.zeros(9),
        quality=PoseQuality.EXCELLENT,
    )


def compute_true_room_coverage(true_grid: np.ndarray, grid_map: OccupancyGrid, cell_size_m: float = 1.0) -> float:
    """Compute percentage of true free floor cells observed as free in the occupancy map."""
    rows, cols = true_grid.shape
    total_free = 0
    explored_free = 0
    for y in range(rows):
        for x in range(cols):
            if true_grid[y, x] == 0:  # free room cell
                total_free += 1
                # Center in world coordinates (row 0 = top/max Y)
                wx = (x + 0.5) * cell_size_m
                wy = (rows - 1 - y + 0.5) * cell_size_m
                gx, gy = grid_map.world_to_grid(wx, wy)
                if 0 <= gx < grid_map.cols and 0 <= gy < grid_map.rows:
                    if grid_map.log_odds[gy, gx] < 0.0:  # observed as free space
                        explored_free += 1
    if total_free == 0:
        return 0.0
    return float(explored_free / total_free) * 100.0


def resolve_or_generate_map(map_path: str, seed: int = 42, mode: Optional[str] = None) -> Path:
    """Return map file Path, generating it at runtime if missing or requested."""
    p = Path(map_path)
    if p.exists():
        return p

    name = p.stem
    layout_mode = mode or ("house" if "house" in name else "maze")
    out_dir = p.parent if str(p.parent) not in (".", "") else Path("generated_maps")
    out_dir.mkdir(parents=True, exist_ok=True)

    rng = random.Random(seed)
    if layout_mode == "house":
        grid = generate_maps.gen_house(28, 20, rng)
        start = generate_maps.pick_start(grid, (2, 2))
        generate_maps.add_obstacles(grid, rng, 5, avoid=start)
    else:
        grid = generate_maps.gen_maze(28, 20, rng)
        start = generate_maps.pick_start(grid, (1, 1))
        generate_maps.add_obstacles(grid, rng, 3, avoid=start)

    people = generate_maps.place_people(grid, start, min(6, 1 + (seed % 6)), rng)
    generate_maps.export_map(grid, start, people, seed, layout_mode, 1.0, out_dir, name)
    print(f"Generated layout at runtime: {out_dir / f'{name}.json'}")
    return out_dir / f"{name}.json"


def run_pygame_sim(
    map_path: str,
    headless: bool = False,
    max_flight_time_s: Optional[float] = None,
    hard_time_limit_s: Optional[float] = None,
    max_sim_steps: Optional[int] = None,
    enable_yolo: bool = True,
    seed: int = 42,
    mode: Optional[str] = None,
    output_json: Optional[str] = None,
    debug_true_pose: bool = False,
):
    resolved_map = resolve_or_generate_map(map_path, seed=seed, mode=mode)

    cfg = load_mission_config()
    mission_cfg = cfg.get("mission", {})
    if max_flight_time_s is None:
        max_flight_time_s = float(mission_cfg.get("max_flight_time_s", 240.0))
    if hard_time_limit_s is None:
        hard_time_limit_s = float(mission_cfg.get("hard_time_limit_s", max_flight_time_s + 40.0))
    if max_sim_steps is None:
        max_sim_steps = int(mission_cfg.get("max_sim_steps", int(hard_time_limit_s * 100)))

    print(f"\n========================================================")
    print(f"  NIDAR AIRMOUSE: Autonomous Search & Sensor Fusion")
    print(f"  Map: {resolved_map} | Headless: {headless} | Seed: {seed}")
    print(f"  Planned Budget: {max_flight_time_s:.1f}s | Hard Limit: {hard_time_limit_s:.1f}s ({max_sim_steps} steps)")
    print(f"========================================================\n")

    np.random.seed(seed)

    # 1. Load Map Assets
    true_grid, metadata = load_map_file(str(resolved_map))
    start_wx, start_wy = metadata.start_world
    print(f"Loaded layout: {metadata.name} ({metadata.cols}x{metadata.rows} cells)")
    print(f"Start world position: ({start_wx:.2f}, {start_wy:.2f})")
    print(f"Ground-truth survivors hidden in building: {len(metadata.true_survivors)}")

    # 2. Initialize Simulator Backend (Ground Truth Physics & Sensors)
    init_drone_pos = np.array([start_wx, start_wy, 0.0])
    drone = DroneDynamics(
        initial_position=init_drone_pos,
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
    renderer.true_survivors = metadata.true_survivors

    sensor_hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=true_grid,
        cell_size_m=metadata.cell_size_m,
    )
    flight = SimulatedFlightInterface(drone=drone)
    viewer = PygameViewer(
        grid_rows=metadata.rows,
        grid_cols=metadata.cols,
        headless=headless,
    )

    # 3. Initialize Pure Autonomy Stack (Zero Ground Truth Access)
    ekf = SensorFusionEKF(init_pos=init_drone_pos, init_yaw=0.0)
    grid_map = OccupancyGrid(
        width_m=metadata.cols * metadata.cell_size_m,
        height_m=metadata.rows * metadata.cell_size_m,
        resolution_m=0.25,
    )
    yolo = YoloDetector() if enable_yolo else None
    ray_projector = GroundRayProjector()
    mission = MissionCoordinator(
        start_world=(start_wx, start_wy),
        grid_map=grid_map,
        flight=flight,
        max_flight_time_s=max_flight_time_s,
    )

    # Count reachable free cells for coverage metrics
    total_free_cells = int(np.count_nonzero(true_grid == 0))
    logger = MissionLogger(total_free_cells=total_free_cells, true_survivors=metadata.true_survivors)

    # 4. Simulation Execution Loop
    sim_dt = 0.01  # 100 Hz simulation clock
    wall_start = time.time()
    sim_time = 0.0
    paused = False
    running = True

    step_count = 0
    exit_reason = "COMPLETE"
    while running and mission.state != MissionState.COMPLETE:
        # Check GUI events
        gui_running, pause_toggled, reset_req = viewer.process_events()
        if not gui_running:
            exit_reason = "INTERRUPTED"
            break
        if pause_toggled:
            paused = not paused
        if reset_req:
            print("Restarting mission...")
            return run_pygame_sim(
                map_path=map_path,
                headless=headless,
                max_flight_time_s=max_flight_time_s,
                hard_time_limit_s=hard_time_limit_s,
                max_sim_steps=max_sim_steps,
                enable_yolo=enable_yolo,
                seed=seed,
                mode=mode,
                output_json=output_json,
            )

        if paused:
            time.sleep(0.05)
            continue

        sim_time += sim_dt
        step_count += 1
        sensor_hub.advance_time(sim_dt)

        # Check hard simulation limits
        if sim_time >= hard_time_limit_s or step_count >= max_sim_steps:
            print(f"[LIMIT REACHED] Hard mission limit reached (sim_time={sim_time:.1f}s / {hard_time_limit_s:.1f}s, steps={step_count} / {max_sim_steps}). Ending mission.")
            exit_reason = "TIMEOUT"
            break

        # Step 4a: Poll IMU & Predict EKF
        imu = sensor_hub.read_imu()
        if imu is not None:
            ekf.predict(imu)

        # Step 4b: Poll Sonar Ranges (20 Hz) & Update Map + EKF Altitude
        ranges = sensor_hub.read_ranges()
        if ranges:
            ekf.update_sonar(ranges)
            drone_state = ekf.get_state(sim_time)
            # Scan-to-map drift correction against established map walls before updating beams
            if step_count % 10 == 0:
                ekf.update_scan_to_map(ranges, grid_map)
                drone_state = ekf.get_state(sim_time)

            # Update occupancy grid with horizontal sensor beams
            true_phys = drone.get_state()
            active_pose = make_drone_state_from_physical(true_phys, sim_time) if debug_true_pose else drone_state
            map_yaw = active_pose.attitude_rpy[2]
            map_x, map_y = active_pose.position[0], active_pose.position[1]
            yaw_offsets = {"front": 0.0, "left": math.pi / 2.0, "right": -math.pi / 2.0, "rear": math.pi}
            for sid, offset in yaw_offsets.items():
                if sid in ranges:
                    grid_map.update_beam(
                        start_wx=map_x,
                        start_wy=map_y,
                        angle_rad=map_yaw + offset,
                        range_m=ranges[sid].range_m,
                        valid_hit=ranges[sid].valid,
                    )

        # Step 4c: Poll Camera (20 Hz) & Run Optical Flow + YOLO
        cam = sensor_hub.read_camera()
        latest_detections = []
        if cam is not None:
            drone_state = ekf.get_state(sim_time)
            # Optical Flow velocity update
            gyro_rates = imu.gyro_radps if imu is not None else np.zeros(3)
            ekf.update_camera(cam, gyro_rates)

            # YOLO inference (run at ~5 Hz on CPU to meet real-time budget)
            if yolo is not None and step_count % 20 == 0:
                latest_detections = yolo.detect(cam.image_bgr)
                for det in latest_detections:
                    u_b, v_b = det.bottom_center
                    true_phys = drone.get_state()
                    active_pose = make_drone_state_from_physical(true_phys, sim_time) if debug_true_pose else drone_state
                    ground_coords = ray_projector.project_pixel_to_ground(
                        pixel_u=u_b,
                        pixel_v=v_b,
                        drone_pos_world=active_pose.position,
                        drone_yaw=active_pose.attitude_rpy[2],
                        drone_pitch=active_pose.attitude_rpy[1],
                        drone_roll=active_pose.attitude_rpy[0],
                    )
                    if ground_coords is not None:
                        mission.tracker.update(ground_coords, det.confidence)

        # Periodic status logging in terminal
        if step_count % 200 == 0:  # every 2.0 simulation seconds
            cov_p = compute_true_room_coverage(true_grid, grid_map, metadata.cell_size_m)
            print(f"[t={sim_time:5.1f}s] State: {mission.state.name:<11} | "
                  f"Pos: ({drone_state.position[0]:.2f}, {drone_state.position[1]:.2f}) | "
                  f"Alt: {drone_state.position[2]:.2f}m | "
                  f"Drift: {drift:.2f}m | "
                  f"Cov: {cov_p:4.1f}% | "
                  f"Survivors: {len(mission.tracker.confirmed_survivors)}/6")

        # Step 4d: Autonomy Mission Coordinator Update
        drone_state = ekf.get_state(sim_time)
        true_phys = drone.get_state()
        active_pose = make_drone_state_from_physical(true_phys, sim_time) if debug_true_pose else drone_state
        cmd = mission.update(
            current_time_s=sim_time,
            drone_state=active_pose,
            ranges=ranges,
        )

        # Step 4e: Physical Simulation Step
        drone.set_command(cmd)
        drone.step(sim_dt)

        # Record telemetry & drift metrics
        true_phys = drone.get_state()
        drift = float(np.linalg.norm(drone_state.position[:2] - true_phys.position[:2]))
        is_coll = getattr(drone, 'had_collision', False)
        drone.had_collision = False
        logger.record_step(
            est_pos=drone_state.position,
            true_pos=true_phys.position,
            is_collision=is_coll,
            sim_time=sim_time,
        )

        # Failsafe checks
        if true_phys.battery_pct <= 0.0:
            print("[FAILSAFE] Battery depleted.")
            exit_reason = "FAILSAFE"
            break

        # Step 4f: Render UI
        if not headless and step_count % 2 == 0:
            cov_pct = compute_true_room_coverage(true_grid, grid_map, metadata.cell_size_m)

            cam_img = cam.image_bgr if cam is not None else np.zeros((240, 320, 3), dtype=np.uint8)
            viewer.render(
                true_grid=true_grid,
                occupancy_grid=grid_map,
                drone_est_state=drone_state,
                drone_true_pos=true_phys.position,
                drone_true_yaw=true_phys.attitude_rpy[2],
                planned_path=mission.current_path,
                confirmed_survivors=mission.tracker.confirmed_survivors,
                camera_bgr=cam_img,
                detections=latest_detections,
                mission_state=mission.state,
                flight_time_s=sim_time,
                battery_pct=true_phys.battery_pct,
                ranges=ranges,
                coverage_pct=cov_pct,
                drift_m=drift,
            )

    # 5. Final Report & Performance Evaluation
    if mission.state == MissionState.COMPLETE:
        exit_reason = "COMPLETE"
    elif exit_reason == "COMPLETE":
        exit_reason = "TIMEOUT"

    final_true = drone.get_state()
    confirmed = [s.estimated_pos for s in mission.tracker.confirmed_survivors]
    cov_pct = compute_true_room_coverage(true_grid, grid_map, metadata.cell_size_m)
    explored_cells = int((cov_pct / 100.0) * total_free_cells)
    frontiers_seen = mission.frontier_detector.total_frontiers_seen
    metrics = logger.compute_metrics(
        flight_time_s=sim_time,
        explored_free_cells=explored_cells,
        confirmed_survivors=confirmed,
        start_world=(start_wx, start_wy),
        final_true_pos=final_true.position,
        frontiers_seen=frontiers_seen,
        exit_reason=exit_reason,
    )

    if output_json:
        metrics.save_json(output_json)
        print(f"Metrics saved to: {output_json}")

    print("\n=================== MISSION REPORT ===================")
    print(f"Final State:             {mission.state.name}")
    print(f"Exit Reason:             {metrics.exit_reason} (Exit Code: {metrics.exit_code})")
    print(f"Mission Success:         {'YES' if metrics.success else 'NO'}")
    print(f"Took Off:                {'YES' if metrics.took_off else 'NO'}")
    print(f"Pose Estimate Valid:     {'YES' if metrics.pose_estimate_valid else 'NO'}")
    print(f"Frontiers Discovered:    {metrics.frontiers_seen}")
    print(f"Total Flight Time:       {metrics.total_time_s:.1f} s")
    print(f"Distance Traveled:       {metrics.distance_traveled_m:.2f} m")
    print(f"Room Coverage:           {metrics.coverage_percentage:.1f}%")
    print(f"Survivors Located:       {metrics.survivors_found} / {metrics.total_survivors}")
    print(f"False Positives:         {metrics.false_positives}")
    print(f"Return-to-Home Error:    {metrics.return_home_error_m:.2f} m (Target: < 1.0 m)")
    print(f"Mean Localization Drift: {metrics.mean_drift_m:.2f} m")
    print(f"Peak Localization Drift: {metrics.max_drift_m:.2f} m")
    print(f"Collisions:              {metrics.collisions}")
    print("======================================================\n")

    return metrics


def main():
    parser = argparse.ArgumentParser(description="NIDAR AirMouse Autonomous Sim Runner")
    parser.add_argument("--map", default="maps/house_00042.json", help="Path to map JSON file")
    parser.add_argument("--backend", choices=["pygame", "gazebo"], default="pygame", help="Simulation backend")
    parser.add_argument("--headless", action="store_true", help="Run without graphical display")
    parser.add_argument("--max-time", type=float, default=None, help="Maximum flight time in seconds")
    parser.add_argument("--hard-limit", type=float, default=None, help="Hard simulation time ceiling in seconds")
    parser.add_argument("--max-steps", type=int, default=None, help="Maximum simulation steps")
    parser.add_argument("--mode", choices=["house", "maze"], default=None, help="Map type if generating on the fly")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--no-yolo", action="store_true", help="Disable YOLO object detection")
    parser.add_argument("--output-json", default=None, help="Path to save metrics JSON")
    parser.add_argument("--debug-true-pose", action="store_true", help="Debug-only: feed true pose to mapper and planner")
    args = parser.parse_args()

    if args.backend == "pygame":
        metrics = run_pygame_sim(
            map_path=args.map,
            headless=args.headless,
            max_flight_time_s=args.max_time,
            hard_time_limit_s=args.hard_limit,
            max_sim_steps=args.max_steps,
            enable_yolo=not args.no_yolo,
            seed=args.seed,
            mode=args.mode,
            output_json=args.output_json,
            debug_true_pose=args.debug_true_pose,
        )
        sys.exit(metrics.exit_code)
    elif args.backend == "gazebo":
        print("Launching Gazebo Harmonic ROS 2 mission bridge...")
        # Gazebo ROS2 bridge launch invocation
        from airmouse.backends.gazebo_ros2.airmouse_node import launch_ros2_node
        launch_ros2_node(args.map)


if __name__ == "__main__":
    main()
