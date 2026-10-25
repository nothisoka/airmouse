"""Unit tests for Fusing 2D Lidar Scan Matching into the EKF (Task D6).

Validates:
1. Synthetic test: IMU with injected bias, straight 20 m flight.
   - Drift stays below 0.3 m with the scan matcher.
   - Drift is clearly above 0.3 m without the scan matcher.
2. Gating: low score or invalid matches are rejected and do not corrupt the EKF state.
"""
import math
import numpy as np
import pytest

from airmouse.core.hal import ImuData, LidarScan
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.estimation.scan_matcher import ScanMatcher, ScanMatchResult


def create_synthetic_world_and_map():
    """Create a structured 26x8 m corridor with perimeter walls and pillars."""
    sim_grid = np.zeros((8, 26), dtype=np.uint8)
    sim_grid[0, :] = sim_grid[-1, :] = 1  # North and South walls
    sim_grid[:, 0] = sim_grid[:, -1] = 1  # East and West walls
    # Pillars at x=6, 12, 18 to provide longitudinal (x) constraints
    for px in [6, 12, 18]:
        sim_grid[2, px] = 1
        sim_grid[5, px] = 1

    grid_map = OccupancyGrid(width_m=26.0, height_m=8.0, resolution_m=0.25)
    for gy in range(grid_map.rows):
        for gx in range(grid_map.cols):
            wx, wy = grid_map.grid_to_world(gx, gy)
            cx = int(wx / 1.0)
            cy = int(8.0 - wy / 1.0)
            if 0 <= cx < 26 and 0 <= cy < 8 and sim_grid[cy, cx] != 0:
                grid_map.log_odds[gy, gx] = 3.0

    return sim_grid, grid_map


def raycast_lidar_scan(sim_grid: np.ndarray, true_x: float, true_y: float, true_yaw: float, timestamp: float) -> LidarScan:
    """Simulate 2D lidar scan from ground-truth pose."""
    num_rays = 180  # 2 degree resolution for fast test execution
    angles = np.linspace(0, 2.0 * math.pi, num_rays, endpoint=False)
    ranges = []

    for a in angles:
        ray_ang = true_yaw + a
        cos_a = math.cos(ray_ang)
        sin_a = math.sin(ray_ang)
        d = 12.0
        # Step along ray
        for dist in np.arange(0.15, 12.0, 0.05):
            rx = true_x + dist * cos_a
            ry = true_y + dist * sin_a
            cx = int(rx / 1.0)
            cy = int(8.0 - ry / 1.0)
            if 0 <= cx < 26 and 0 <= cy < 8 and sim_grid[cy, cx] != 0:
                d = dist
                break
        ranges.append(d)

    return LidarScan(
        ranges=np.array(ranges, dtype=np.float32),
        angle_min=0.0,
        angle_increment=float(angles[1] - angles[0]),
        range_min=0.15,
        range_max=12.0,
        timestamp=timestamp,
    )


def test_ekf_lidar_fusion_straight_20m_flight_with_bias():
    """CHECK requirement: IMU with injected bias, straight 20 m flight;

    drift stays below 0.3 m with the matcher and clearly above that without it.
    """
    sim_grid, grid_map = create_synthetic_world_and_map()
    matcher = ScanMatcher()

    # Flight parameters: 20 seconds at 1.0 m/s = 20 m straight flight along x
    dt_imu = 0.01   # 100 Hz IMU
    total_time = 20.0
    num_steps = int(total_time / dt_imu)
    start_pos = np.array([2.0, 4.0, 1.2])
    v_true = 1.0

    # Injected IMU biases
    bias_ay = 0.06   # Accel bias on y [m/s^2]
    bias_gz = 0.005  # Gyro bias on yaw [rad/s]

    # --- Run 1: WITHOUT scan matcher ---
    ekf_no_match = SensorFusionEKF(init_pos=start_pos.copy(), init_yaw=0.0)
    ekf_no_match.calibrated = True

    t = 0.0
    for step in range(num_steps):
        t += dt_imu
        imu = ImuData(
            accel_mps2=np.array([0.0, bias_ay, 9.80665]),
            gyro_radps=np.array([0.0, 0.0, bias_gz]),
            timestamp_s=t,
        )
        ekf_no_match.predict(imu)

    final_true_pos = np.array([start_pos[0] + v_true * total_time, start_pos[1]])
    state_no_match = ekf_no_match.get_state(t)
    drift_without_matcher = math.hypot(
        state_no_match.position[0] - final_true_pos[0],
        state_no_match.position[1] - final_true_pos[1],
    )

    # --- Run 2: WITH scan matcher ---
    ekf_with_match = SensorFusionEKF(init_pos=start_pos.copy(), init_yaw=0.0)
    ekf_with_match.calibrated = True

    max_drift_with_matcher = 0.0
    t = 0.0
    lidar_period = 0.125  # 8 Hz
    last_lidar_time = 0.0

    for step in range(num_steps):
        t += dt_imu
        true_x = start_pos[0] + v_true * t
        true_y = start_pos[1]
        true_yaw = 0.0

        # IMU update with injected bias
        imu = ImuData(
            accel_mps2=np.array([0.0, bias_ay, 9.80665]),
            gyro_radps=np.array([0.0, 0.0, bias_gz]),
            timestamp_s=t,
        )
        ekf_with_match.predict(imu)

        # Lidar scan at 8 Hz
        if t - last_lidar_time >= lidar_period:
            last_lidar_time = t
            scan = raycast_lidar_scan(sim_grid, true_x, true_y, true_yaw, t)
            state = ekf_with_match.get_state(t)

            match_res = matcher.match(
                scan=scan,
                grid_map=grid_map,
                initial_pose=(state.position[0], state.position[1], state.attitude_rpy[2]),
            )

            if match_res.valid:
                updated = ekf_with_match.update_lidar_match(match_res, timestamp_s=t)
                assert updated is True

        cur_drift = math.hypot(
            ekf_with_match.x[0] - true_x,
            ekf_with_match.x[1] - true_y,
        )
        if cur_drift > max_drift_with_matcher:
            max_drift_with_matcher = cur_drift

    final_state_with_match = ekf_with_match.get_state(t)
    final_drift_with_matcher = math.hypot(
        final_state_with_match.position[0] - final_true_pos[0],
        final_state_with_match.position[1] - final_true_pos[1],
    )

    print(f"\nDrift without matcher: {drift_without_matcher:.3f} m")
    print(f"Max drift with matcher: {max_drift_with_matcher:.3f} m")
    print(f"Final drift with matcher: {final_drift_with_matcher:.3f} m")

    # CHECK assertions:
    # 1. Drift stays below 0.3 m with the matcher
    assert max_drift_with_matcher < 0.30, f"Max drift {max_drift_with_matcher:.3f} m >= 0.30 m threshold"
    assert final_drift_with_matcher < 0.15, f"Final drift {final_drift_with_matcher:.3f} m >= 0.15 m"

    # 2. Drift is clearly above 0.3 m without the matcher
    assert drift_without_matcher > 1.0, f"Drift without matcher {drift_without_matcher:.3f} m is not clearly above 0.3 m"


def test_ekf_lidar_fusion_rejects_low_score():
    """Verify that update_lidar_match gates low score and invalid matches."""
    ekf = SensorFusionEKF(init_pos=np.array([5.0, 5.0, 1.2]), init_yaw=0.0)
    orig_pos = ekf.x[0:3].copy()

    # Result with score below threshold
    low_score_res = ScanMatchResult(
        dx=2.0, dy=2.0, dyaw=0.5,
        score=0.40,  # Below default min_score 0.65
        cov=np.eye(3) * 0.01,
        valid=True,
        matched_x=7.0, matched_y=7.0, matched_yaw=0.5,
    )

    updated = ekf.update_lidar_match(low_score_res, min_score=0.65)
    assert updated is False
    assert np.allclose(ekf.x[0:3], orig_pos), "EKF state must not change when score is low"

    # Result with valid=False
    invalid_res = ScanMatchResult(
        dx=2.0, dy=2.0, dyaw=0.5,
        score=0.90,
        cov=np.eye(3) * 0.01,
        valid=False,
        matched_x=7.0, matched_y=7.0, matched_yaw=0.5,
        reject_reason="degenerate_geometry",
    )

    updated_inv = ekf.update_lidar_match(invalid_res)
    assert updated_inv is False
    assert np.allclose(ekf.x[0:3], orig_pos), "EKF state must not change when valid=False"

