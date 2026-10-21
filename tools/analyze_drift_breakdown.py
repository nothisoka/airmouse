#!/usr/bin/env python3
"""Detailed State Estimation Error Breakdown Analysis.

Analyzes IMU integration, optical flow velocity/scale errors, and heading drift
over time, saving a structured CSV and a multi-panel diagnostic plot.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from airmouse.core.mapping.map_io import load_map_file
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub
from airmouse.backends.pygame_sim.flight_sim import SimulatedFlightInterface
from airmouse.tools.generate_maps import gen_house, pick_start, add_obstacles, place_people, export_map
import random


def run_drift_analysis(
    map_path: str = "generated_maps/house_00101.json",
    csv_out: str = "generated_maps/drift_breakdown_house_00101.csv",
    plot_out: str = "generated_maps/drift_breakdown_house_00101.png",
    seed: int = 101,
    max_flight_time_s: float = 150.0,
):
    map_p = Path(map_path)
    if not map_p.exists():
        map_p.parent.mkdir(parents=True, exist_ok=True)
        rng = random.Random(seed)
        grid = gen_house(28, 20, rng)
        start = pick_start(grid, (2, 2))
        add_obstacles(grid, rng, 5, avoid=start)
        people = place_people(grid, start, 2, rng)
        export_map(grid, start, people, seed, "house", 1.0, map_p.parent, map_p.stem)

    true_grid, metadata = load_map_file(str(map_p))
    start_wx, start_wy = metadata.start_world

    init_drone_pos = np.array([start_wx, start_wy, 0.0])
    drone = DroneDynamics(
        initial_position=init_drone_pos,
        initial_yaw=0.0,
        grid=true_grid,
        cell_size_m=metadata.cell_size_m,
    )
    renderer = WorldRenderer(grid=true_grid, cell_size_m=metadata.cell_size_m, img_w=320, img_h=240)
    renderer.true_survivors = metadata.true_survivors
    sensor_hub = SimulatedSensorHub(drone=drone, renderer=renderer, grid=true_grid, cell_size_m=metadata.cell_size_m)
    flight = SimulatedFlightInterface(drone=drone)

    ekf = SensorFusionEKF(init_pos=init_drone_pos, init_yaw=0.0)
    grid_map = OccupancyGrid(
        width_m=metadata.cols * metadata.cell_size_m,
        height_m=metadata.rows * metadata.cell_size_m,
        resolution_m=0.25,
    )
    mission = MissionCoordinator(
        start_world=(start_wx, start_wy),
        grid_map=grid_map,
        flight=flight,
        max_flight_time_s=max_flight_time_s,
    )

    sim_dt = 0.01
    sim_time = 0.0
    step_count = 0

    records = []
    last_vo_vx = 0.0
    last_vo_vy = 0.0

    print(f"Running drift breakdown analysis on {map_path} (seed={seed})...")

    while sim_time < max_flight_time_s and mission.state != MissionState.COMPLETE:
        sim_time += sim_dt
        step_count += 1
        sensor_hub.advance_time(sim_dt)

        imu = sensor_hub.read_imu()
        if imu is not None:
            ekf.predict(imu)

        ranges = sensor_hub.read_ranges()
        if ranges:
            ekf.update_sonar(ranges)
            drone_state = ekf.get_state(sim_time)
            est_yaw = drone_state.attitude_rpy[2]
            est_x, est_y = drone_state.position[0], drone_state.position[1]
            yaw_offsets = {"front": 0.0, "left": math.pi / 2.0, "right": -math.pi / 2.0, "rear": math.pi}
            for sid, offset in yaw_offsets.items():
                if sid in ranges:
                    grid_map.update_beam(est_x, est_y, est_yaw + offset, ranges[sid].range_m, ranges[sid].valid)

        cam = sensor_hub.read_camera()
        if cam is not None:
            gyro_rates = imu.gyro_radps if imu is not None else np.zeros(3)
            ekf.update_camera(cam, gyro_rates)
            if ekf.last_vo_meas is not None:
                last_vo_vx = float(ekf.last_vo_meas[0])
                last_vo_vy = float(ekf.last_vo_meas[1])

        drone_state = ekf.get_state(sim_time)
        cmd = mission.update(sim_time, drone_state, ranges)
        drone.set_command(cmd)
        drone.step(sim_dt)

        # Log at 20 Hz (step_count % 5 == 0)
        if step_count % 5 == 0:
            true_phys = drone.get_state()
            p_est = drone_state.position
            p_true = true_phys.position
            v_est = drone_state.velocity
            v_true = true_phys.velocity

            # Yaw error wrapped [-pi, pi]
            yaw_err = math.atan2(math.sin(drone_state.attitude_rpy[2] - true_phys.attitude_rpy[2]),
                                 math.cos(drone_state.attitude_rpy[2] - true_phys.attitude_rpy[2]))

            pos_err_xy = math.hypot(p_est[0] - p_true[0], p_est[1] - p_true[1])
            z_err = abs(p_est[2] - p_true[2])
            vel_err_xy = math.hypot(v_est[0] - v_true[0], v_est[1] - v_true[1])

            # Body frame true velocity
            psi_true = true_phys.attitude_rpy[2]
            c_y, s_y = math.cos(psi_true), math.sin(psi_true)
            true_vx_body = c_y * v_true[0] + s_y * v_true[1]
            true_vy_body = -s_y * v_true[0] + c_y * v_true[1]

            # Along-track vs cross-track error decomposition
            true_speed = math.hypot(v_true[0], v_true[1])
            if true_speed > 0.05:
                track_dir = (v_true[0] / true_speed, v_true[1] / true_speed)
                along_track = (p_est[0] - p_true[0]) * track_dir[0] + (p_est[1] - p_true[1]) * track_dir[1]
                cross_track = -(p_est[0] - p_true[0]) * track_dir[1] + (p_est[1] - p_true[1]) * track_dir[0]
            else:
                along_track = 0.0
                cross_track = 0.0

            records.append({
                "time_s": sim_time,
                "x_true": p_true[0], "y_true": p_true[1], "z_true": p_true[2], "yaw_true_deg": math.degrees(psi_true),
                "x_est": p_est[0], "y_est": p_est[1], "z_est": p_est[2], "yaw_est_deg": math.degrees(drone_state.attitude_rpy[2]),
                "pos_err_xy": pos_err_xy,
                "along_track_err": along_track,
                "cross_track_err": cross_track,
                "z_err": z_err,
                "yaw_err_deg": math.degrees(yaw_err),
                "vel_err_xy": vel_err_xy,
                "vo_meas_vx": last_vo_vx, "vo_meas_vy": last_vo_vy,
                "true_vx_body": true_vx_body, "true_vy_body": true_vy_body,
                "state": mission.state.name,
            })

    # 1. Save CSV
    p_csv = Path(csv_out)
    p_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(p_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    print(f"Drift telemetry logged to {csv_out} ({len(records)} samples).")

    # 2. Extract arrays for summary metrics & plotting
    times = [r["time_s"] for r in records]
    pos_err = [r["pos_err_xy"] for r in records]
    along_err = [r["along_track_err"] for r in records]
    cross_err = [r["cross_track_err"] for r in records]
    yaw_err_deg = [r["yaw_err_deg"] for r in records]
    z_err = [r["z_err"] for r in records]
    x_true = [r["x_true"] for r in records]
    y_true = [r["y_true"] for r in records]
    x_est = [r["x_est"] for r in records]
    y_est = [r["y_est"] for r in records]
    vo_vx = [r["vo_meas_vx"] for r in records]
    true_vx = [r["true_vx_body"] for r in records]

    # Summary Statistics
    mean_pos_err = np.mean(pos_err)
    max_pos_err = np.max(pos_err)
    mean_yaw_err = np.mean(np.abs(yaw_err_deg))
    max_yaw_err = np.max(np.abs(yaw_err_deg))
    mean_z_err = np.mean(z_err)

    moving_mask = np.abs(true_vx) > 0.1
    if np.any(moving_mask):
        vo_scale = np.mean(np.array(vo_vx)[moving_mask] / np.array(true_vx)[moving_mask])
    else:
        vo_scale = 1.0

    print("\n--- DRIFT BREAKDOWN SUMMARY ---")
    print(f"Mean Horizontal Error:   {mean_pos_err:.2f} m (Peak: {max_pos_err:.2f} m)")
    print(f"Mean Altitude Error:     {mean_z_err:.3f} m (Strictly bounded by sonar)")
    print(f"Mean Heading Error:      {mean_yaw_err:.2f} deg (Peak: {max_yaw_err:.2f} deg)")
    print(f"Average Along-Track Err: {np.mean(np.abs(along_err)):.2f} m (Longitudinal scale drift)")
    print(f"Average Cross-Track Err: {np.mean(np.abs(cross_err)):.2f} m (Heading rotation drift)")
    print(f"Mean VO Scale Ratio:     {vo_scale:.2f} (1.0 = perfect scale)")
    print("--------------------------------\n")

    # 3. Generate Diagnostic Plot
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"NIDAR AirMouse: Estimation Drift Breakdown ({Path(map_path).stem})", fontsize=14, fontweight="bold")

    # Panel 1: Position Error & Along/Cross-Track decomposition
    axs[0, 0].plot(times, pos_err, label="Total Horizontal Error ||p_est - p_true||", color="crimson", linewidth=2)
    axs[0, 0].plot(times, along_err, label="Along-Track Error (Scale)", color="orange", linestyle="--")
    axs[0, 0].plot(times, cross_err, label="Cross-Track Error (Heading)", color="purple", linestyle=":")
    axs[0, 0].set_title("Horizontal Position Drift vs Time")
    axs[0, 0].set_xlabel("Flight Time (s)")
    axs[0, 0].set_ylabel("Error (m)")
    axs[0, 0].grid(True, alpha=0.3)
    axs[0, 0].legend()

    # Panel 2: Heading Error
    axs[0, 1].plot(times, yaw_err_deg, label="Heading Error (yaw_est - yaw_true)", color="royalblue", linewidth=1.8)
    axs[0, 1].axhline(0, color="gray", linestyle="--", alpha=0.5)
    axs[0, 1].set_title("Heading / Yaw Error vs Time")
    axs[0, 1].set_xlabel("Flight Time (s)")
    axs[0, 1].set_ylabel("Yaw Error (degrees)")
    axs[0, 1].grid(True, alpha=0.3)
    axs[0, 1].legend()

    # Panel 3: Optical Flow vs True Body Velocity
    axs[1, 0].plot(times, true_vx, label="True Body Vx (forward)", color="black", alpha=0.7)
    axs[1, 0].plot(times, vo_vx, label="Measured VO Vx", color="forestgreen", alpha=0.8, linestyle="--")
    axs[1, 0].set_title(f"Optical Flow Velocity Fidelity (Mean Scale Ratio: {vo_scale:.2f})")
    axs[1, 0].set_xlabel("Flight Time (s)")
    axs[1, 0].set_ylabel("Forward Speed (m/s)")
    axs[1, 0].grid(True, alpha=0.3)
    axs[1, 0].legend()

    # Panel 4: Spatial Trajectory (True vs Estimated)
    axs[1, 1].plot(x_true, y_true, label="Ground Truth Path", color="darkgreen", linewidth=2)
    axs[1, 1].plot(x_est, y_est, label="Estimated EKF Path", color="red", linestyle="--", linewidth=1.8)
    axs[1, 1].plot([start_wx], [start_wy], "k*", markersize=12, label="Launch Point")
    axs[1, 1].set_title("2D Spatial Trajectory: Ground Truth vs Estimated")
    axs[1, 1].set_xlabel("X (m)")
    axs[1, 1].set_ylabel("Y (m)")
    axs[1, 1].grid(True, alpha=0.3)
    axs[1, 1].axis("equal")
    axs[1, 1].legend()

    plt.tight_layout()
    plt.savefig(plot_out, dpi=150)
    plt.close()
    print(f"Diagnostic plot saved to {plot_out}")


if __name__ == "__main__":
    run_drift_analysis()

