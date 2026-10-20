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
import sys
import time
from pathlib import Path
import numpy as np

from airmouse.core.mapping.map_io import load_map_file, cell_to_world
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.perception.yolo_detector import YoloDetector
from airmouse.core.perception.ray_projector import GroundRayProjector
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState
from airmouse.core.mission.mission_logger import MissionLogger

from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub
from airmouse.backends.pygame_sim.flight_sim import SimulatedFlightInterface
from airmouse.backends.pygame_sim.pygame_viewer import PygameViewer


def run_pygame_sim(
    map_path: str,
    headless: bool = False,
    max_flight_time_s: float = 300.0,
    enable_yolo: bool = True,
    seed: int = 42,
):
    print(f"\n========================================================")
    print(f"  NIDAR AIRMOUSE: Autonomous Search & Sensor Fusion")
    print(f"  Map: {map_path} | Headless: {headless} | Seed: {seed}")
    print(f"========================================================\n")

    np.random.seed(seed)

    # 1. Load Map Assets
    true_grid, metadata = load_map_file(map_path)
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
    while running and mission.state != MissionState.COMPLETE:
        # Check GUI events
        gui_running, pause_toggled, reset_req = viewer.process_events()
        if not gui_running:
            break
        if pause_toggled:
            paused = not paused
        if reset_req:
            print("Restarting mission...")
            return run_pygame_sim(map_path, headless, max_flight_time_s, enable_yolo, seed)

        if paused:
            time.sleep(0.05)
            continue

        sim_time += sim_dt
        step_count += 1
        sensor_hub.advance_time(sim_dt)

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
            est_yaw = drone_state.attitude_rpy[2]
            est_x, est_y = drone_state.position[0], drone_state.position[1]
            yaw_offsets = {"front": 0.0, "left": math.pi / 2.0, "right": -math.pi / 2.0, "rear": math.pi}
            for sid, offset in yaw_offsets.items():
                if sid in ranges:
                    grid_map.update_beam(
                        start_wx=est_x,
                        start_wy=est_y,
                        angle_rad=est_yaw + offset,
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
                    ground_coords = ray_projector.project_pixel_to_ground(
                        pixel_u=u_b,
                        pixel_v=v_b,
                        drone_pos_world=drone_state.position,
                        drone_yaw=drone_state.attitude_rpy[2],
                        drone_pitch=drone_state.attitude_rpy[1],
                        drone_roll=drone_state.attitude_rpy[0],
                    )
                    if ground_coords is not None:
                        mission.tracker.update(ground_coords, det.confidence)

        # Periodic status logging in terminal
        if step_count % 200 == 0:  # every 2.0 simulation seconds
            cov_cells = int(np.sum((grid_map.log_odds < grid_map.l_free * 0.5)))
            cov_p = min(100.0, (cov_cells / (total_free_cells * 16)) * 100.0)
            print(f"[t={sim_time:5.1f}s] State: {mission.state.name:<11} | "
                  f"Pos: ({drone_state.position[0]:.2f}, {drone_state.position[1]:.2f}) | "
                  f"Alt: {drone_state.position[2]:.2f}m | "
                  f"Drift: {drift:.2f}m | "
                  f"Cov: {cov_p:4.1f}% | "
                  f"Survivors: {len(mission.tracker.confirmed_survivors)}/6")

        if sim_time >= max_flight_time_s + 30.0:
            print("Flight time ceiling reached, finalizing mission.")
            break

        # Step 4d: Autonomy Mission Coordinator Update
        drone_state = ekf.get_state(sim_time)
        cmd = mission.update(
            current_time_s=sim_time,
            drone_state=drone_state,
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
        )

        # Step 4f: Render UI
        if not headless and step_count % 2 == 0:
            # Count explored free cells
            explored_cells = int(np.sum((grid_map.log_odds < grid_map.l_free * 0.5)))
            cov_pct = min(100.0, (explored_cells / (total_free_cells * 16)) * 100.0)

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
    final_true = drone.get_state()
    confirmed = [s.estimated_pos for s in mission.tracker.confirmed_survivors]
    explored_cells = int(np.sum((grid_map.log_odds < grid_map.l_free * 0.5)))
    metrics = logger.compute_metrics(
        flight_time_s=sim_time,
        explored_free_cells=int(explored_cells / 16),
        confirmed_survivors=confirmed,
        start_world=(start_wx, start_wy),
        final_true_pos=final_true.position,
    )

    print("\n=================== MISSION REPORT ===================")
    print(f"Final State:          {mission.state.name}")
    print(f"Total Flight Time:    {metrics.total_time_s:.1f} s")
    print(f"Distance Traveled:    {metrics.distance_traveled_m:.2f} m")
    print(f"Estimated Coverage:   {metrics.coverage_percentage:.1f}%")
    print(f"Survivors Located:    {metrics.survivors_found} / {metrics.total_survivors}")
    print(f"False Positives:      {metrics.false_positives}")
    print(f"Return-to-Home Error: {metrics.return_home_error_m:.2f} m (Target: < 1.0 m)")
    print(f"Mean Localization Drift: {metrics.mean_drift_m:.2f} m")
    print(f"Peak Localization Drift: {metrics.max_drift_m:.2f} m")
    print(f"Collisions:           {metrics.collisions}")
    print("======================================================\n")

    return metrics


def main():
    parser = argparse.ArgumentParser(description="NIDAR AirMouse Autonomous Sim Runner")
    parser.add_argument("--map", default="maps/house_00042.json", help="Path to map JSON file")
    parser.add_argument("--backend", choices=["pygame", "gazebo"], default="pygame", help="Simulation backend")
    parser.add_argument("--headless", action="store_true", help="Run without graphical display")
    parser.add_argument("--max-time", type=float, default=240.0, help="Maximum flight time in seconds")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--no-yolo", action="store_true", help="Disable YOLO object detection")
    args = parser.parse_args()

    if args.backend == "pygame":
        run_pygame_sim(
            map_path=args.map,
            headless=args.headless,
            max_flight_time_s=args.max_time,
            enable_yolo=not args.no_yolo,
            seed=args.seed,
        )
    elif args.backend == "gazebo":
        print("Launching Gazebo Harmonic ROS 2 mission bridge...")
        # Gazebo ROS2 bridge launch invocation
        from airmouse.backends.gazebo_ros2.airmouse_node import launch_ros2_node
        launch_ros2_node(args.map)


if __name__ == "__main__":
    main()
