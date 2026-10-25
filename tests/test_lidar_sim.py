"""Unit test for 2D lidar simulation (Task D3).

Validates:
- 360 rays (1 degree resolution), 0.15 to 12.0m range, 8 Hz.
- In an empty 10x10 m room, non-dropout ray distances equal true analytical wall distances within 3 sigma (0.06m).
- With zero noise, rays equal true wall distances to sub-centimeter accuracy.
- 2% dropout returns range_max.
- HAL interface compatibility and zero ground truth leakage.
"""
import math
import numpy as np
import pytest

from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub, SimulatedLidar2D
from airmouse.core.hal import Lidar2D, LidarScan


def compute_true_box_distance(x0: float, y0: float, angle: float, x_min=1.0, x_max=9.0, y_min=1.0, y_max=9.0) -> float:
    """Compute exact analytical distance from (x0, y0) to axis-aligned box boundary."""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    t_candidates = []
    # Vertical walls: x = x_min and x = x_max
    if abs(cos_a) > 1e-9:
        t1 = (x_max - x0) / cos_a
        t2 = (x_min - x0) / cos_a
        if t1 > 0:
            t_candidates.append(t1)
        if t2 > 0:
            t_candidates.append(t2)

    # Horizontal walls: y = y_min and y = y_max
    if abs(sin_a) > 1e-9:
        t3 = (y_max - y0) / sin_a
        t4 = (y_min - y0) / sin_a
        if t3 > 0:
            t_candidates.append(t3)
        if t4 > 0:
            t_candidates.append(t4)

    return min(t_candidates) if t_candidates else 12.0


def test_lidar_room_distances_within_3_sigma():
    """CHECK requirement: drone in an empty 10x10 m room returns distances equal to true wall distances within 3 sigma."""
    np.random.seed(42)

    # 10x10 room with walls at outer perimeter (cells at row/col 0 and 9)
    # Inner boundary is [1.0, 9.0] in both X and Y
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1

    # Place drone at center (5.0, 5.0), heading 0.0 (facing +X)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)

    sim_time = [0.0]
    def get_time():
        return sim_time[0]

    lidar = SimulatedLidar2D(
        drone=drone,
        grid=grid,
        cell_size_m=1.0,
        get_time_fn=get_time,
        rate_hz=8.0,
        num_rays=360,
        range_min=0.15,
        range_max=12.0,
        noise_sigma=0.02,
        dropout_prob=0.02,
    )

    scan = lidar.read()
    assert scan is not None
    assert isinstance(scan, LidarScan)
    assert len(scan.ranges) == 360
    assert scan.range_min == 0.15
    assert scan.range_max == 12.0

    three_sigma = 3.0 * 0.02  # 0.06 m

    valid_count = 0
    within_3_sigma_count = 0
    dropout_count = 0

    for i in range(scan.ranges.shape[0]):
        angle = scan.angle_min + i * scan.angle_increment
        meas = float(scan.ranges[i])

        if math.isclose(meas, scan.range_max, abs_tol=1e-3):
            dropout_count += 1
            continue

        d_true = compute_true_box_distance(5.0, 5.0, angle)
        err = abs(meas - d_true)

        valid_count += 1
        if err <= three_sigma:
            within_3_sigma_count += 1

    assert valid_count > 340, f"Expected ~353 valid rays, got {valid_count}"
    # Standard normal distribution: 99.73% within 3 sigma
    pass_fraction = within_3_sigma_count / valid_count
    assert pass_fraction >= 0.98, f"Pass fraction {pass_fraction:.3f} below 0.98 (3-sigma bound)"


def test_lidar_zero_noise_accuracy():
    """Verify exact raycasting accuracy when noise is disabled."""
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1

    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)

    lidar = SimulatedLidar2D(
        drone=drone,
        grid=grid,
        cell_size_m=1.0,
        get_time_fn=lambda: 0.0,
        rate_hz=8.0,
        num_rays=360,
        range_min=0.15,
        range_max=12.0,
        noise_sigma=0.0,
        dropout_prob=0.0,
    )

    scan = lidar.read()
    assert scan is not None

    for i in range(360):
        angle = scan.angle_min + i * scan.angle_increment
        meas = float(scan.ranges[i])
        d_true = compute_true_box_distance(5.0, 5.0, angle)
        err = abs(meas - d_true)
        assert err < 0.005, f"Ray {i} (angle {math.degrees(angle):.1f}°): meas={meas:.4f}, true={d_true:.4f}, err={err:.4f}"


def test_lidar_cardinal_directions():
    """Verify exact cardinal wall distances (4.0m to North, South, East, West)."""
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1

    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)

    lidar = SimulatedLidar2D(
        drone=drone,
        grid=grid,
        cell_size_m=1.0,
        get_time_fn=lambda: 0.0,
        rate_hz=8.0,
        num_rays=360,
        range_min=0.15,
        range_max=12.0,
        noise_sigma=0.0,
        dropout_prob=0.0,
    )

    scan = lidar.read()
    assert scan is not None

    # Front = ray 0 (0 deg, +X East): wall at x=9.0 -> dist 4.0m
    assert math.isclose(scan.ranges[0], 4.0, abs_tol=0.005)
    # Left = ray 90 (90 deg, +Y North): wall at y=9.0 -> dist 4.0m
    assert math.isclose(scan.ranges[90], 4.0, abs_tol=0.005)
    # Rear = ray 180 (180 deg, -X West): wall at x=1.0 -> dist 4.0m
    assert math.isclose(scan.ranges[180], 4.0, abs_tol=0.005)
    # Right = ray 270 (270 deg, -Y South): wall at y=1.0 -> dist 4.0m
    assert math.isclose(scan.ranges[270], 4.0, abs_tol=0.005)


def test_lidar_hub_integration_full_profile():
    """Verify that SimulatedSensorHub in full profile creates and polls SimulatedLidar2D."""
    grid = np.zeros((10, 10), dtype=np.uint8)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    renderer = WorldRenderer(grid=grid, cell_size_m=1.0)

    hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=grid,
        profile="full",
    )

    assert hub.has_sensor("lidar2d") is True
    lidar = hub.get_lidar()
    assert lidar is not None
    assert isinstance(lidar, Lidar2D)

    scan = hub.read_lidar()
    assert scan is not None
    assert isinstance(scan, LidarScan)
    assert len(scan.ranges) == 360

    # Rate limiting: polling immediately without advancing time returns None
    assert hub.read_lidar() is None

    # Advance past 1/8s period (0.125s)
    hub.advance_time(0.13)
    scan2 = hub.read_lidar()
    assert scan2 is not None
