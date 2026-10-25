"""Unit tests for 2D Lidar Scan Matcher (Task D5).

Validates:
1. Recovers true pose from known map and scan with a 0.30 m / 5 degree offset to within 0.05 m / 1 degree.
2. Rejects bad scans:
   a) Uncorrelated/random scan (low match score).
   b) Featureless corridor (degenerate geometry / unconstrained translation).
   c) Empty/max-range scan (insufficient points).
3. Produces a valid 3x3 covariance matrix.
"""
import math
import numpy as np
import pytest

from airmouse.core.hal import LidarScan
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.scan_matcher import ScanMatcher, ScanMatchResult, _wrap_angle
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.sensor_sim import SimulatedLidar2D


@pytest.fixture
def room_map_and_scan():
    """Create a known 10x10 m room and take a lidar scan at center (5.0, 5.0, 0.0)."""
    # 10x10 m sim grid with perimeter walls
    sim_grid = np.zeros((10, 10), dtype=np.uint8)
    sim_grid[0, :] = sim_grid[-1, :] = sim_grid[:, 0] = sim_grid[:, -1] = 1

    # Place drone at center
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    lidar = SimulatedLidar2D(
        drone=drone,
        grid=sim_grid,
        cell_size_m=1.0,
        get_time_fn=lambda: 0.0,
        rate_hz=8.0,
        num_rays=360,
        range_min=0.15,
        range_max=12.0,
        noise_sigma=0.01,
        dropout_prob=0.0,
    )
    scan = lidar.read()
    assert scan is not None

    grid_map = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)
    grid_map.update_lidar_scan(5.0, 5.0, 0.0, scan)

    return grid_map, scan, (5.0, 5.0, 0.0)


def test_scan_matcher_recovers_offset_pose(room_map_and_scan):
    """CHECK requirement: start with known scan and map, add 0.30 m / 5 deg offset;

    the matcher recovers it to within 0.05 m / 1 degree.
    """
    grid_map, scan, (x0, y0, yaw0) = room_map_and_scan
    matcher = ScanMatcher()

    # Test multiple offset headings at 0.30 m distance and 5 degree yaw offset
    test_offsets = [
        (0.30, 0.00, math.radians(5.0)),
        (0.00, 0.30, math.radians(-5.0)),
        (0.30 * math.cos(math.radians(35.0)), 0.30 * math.sin(math.radians(35.0)), math.radians(5.0)),
        (-0.30 * math.cos(math.radians(45.0)), -0.30 * math.sin(math.radians(45.0)), math.radians(-5.0)),
    ]

    for dx_off, dy_off, dyaw_off in test_offsets:
        init_x = x0 + dx_off
        init_y = y0 + dy_off
        init_yaw = yaw0 + dyaw_off

        # Initial offset verification
        init_pos_err = math.hypot(init_x - x0, init_y - y0)
        init_yaw_err_deg = math.degrees(abs(_wrap_angle(init_yaw - yaw0)))
        assert abs(init_pos_err - 0.30) < 1e-3
        assert abs(init_yaw_err_deg - 5.0) < 1e-3

        # Match scan against map
        result = matcher.match(
            scan=scan,
            grid_map=grid_map,
            initial_pose=(init_x, init_y, init_yaw),
        )

        assert result.valid is True, f"Match should be valid, rejected: {result.reject_reason}"
        assert result.score >= 0.70, f"Match score {result.score:.3f} below 0.70"

        # Recovered pose
        rec_x = init_x + result.dx
        rec_y = init_y + result.dy
        rec_yaw = _wrap_angle(init_yaw + result.dyaw)

        pos_err = math.hypot(rec_x - x0, rec_y - y0)
        yaw_err_deg = math.degrees(abs(_wrap_angle(rec_yaw - yaw0)))

        # Verification: within 0.05 m and 1 degree
        assert pos_err <= 0.05, f"Recovered position error {pos_err:.4f} m > 0.05 m threshold"
        assert yaw_err_deg <= 1.0, f"Recovered yaw error {yaw_err_deg:.4f} deg > 1.0 deg threshold"

        # Covariance verification
        assert result.cov.shape == (3, 3)
        assert np.all(np.diag(result.cov) > 0.0), "Covariance diagonal must be strictly positive"
        assert np.allclose(result.cov, result.cov.T), "Covariance must be symmetric"


def test_scan_matcher_rejects_bad_scan(room_map_and_scan):
    """CHECK requirement: A second test with a bad scan must be rejected."""
    grid_map, _, (x0, y0, yaw0) = room_map_and_scan
    matcher = ScanMatcher()

    # 1. Random noise scan (completely uncorrelated with the map)
    rng = np.random.default_rng(42)
    bad_ranges = rng.uniform(0.5, 3.0, 360).astype(np.float32)
    bad_scan = LidarScan(
        ranges=bad_ranges,
        angle_min=0.0,
        angle_increment=2.0 * math.pi / 360,
        range_min=0.15,
        range_max=12.0,
        timestamp=0.0,
    )

    result_bad = matcher.match(
        scan=bad_scan,
        grid_map=grid_map,
        initial_pose=(x0, y0, yaw0),
    )

    assert result_bad.valid is False, "Random uncorrelated scan must be rejected"
    assert result_bad.score < matcher.min_score, f"Expected low score, got {result_bad.score}"
    assert "low" in (result_bad.reject_reason or "").lower()

    # 2. All max-range / dropout scan (empty space, no obstacle hits)
    empty_ranges = np.full(360, 12.0, dtype=np.float32)
    empty_scan = LidarScan(
        ranges=empty_ranges,
        angle_min=0.0,
        angle_increment=2.0 * math.pi / 360,
        range_min=0.15,
        range_max=12.0,
        timestamp=0.0,
    )

    result_empty = matcher.match(
        scan=empty_scan,
        grid_map=grid_map,
        initial_pose=(x0, y0, yaw0),
    )

    assert result_empty.valid is False, "Empty / max-range scan must be rejected"
    assert result_empty.reject_reason == "insufficient_scan_points"


def test_scan_matcher_rejects_degenerate_corridor():
    """Verify matcher detects and rejects featureless corridor geometry (degenerate geometry)."""
    # 20m x 4m corridor aligned along x (walls at y=1 and y=3)
    corridor_grid = np.zeros((4, 20), dtype=np.uint8)
    corridor_grid[0, :] = corridor_grid[-1, :] = 1

    drone_c = DroneDynamics(initial_position=np.array([10.0, 2.0, 1.2]), initial_yaw=0.0)
    lidar_c = SimulatedLidar2D(
        drone=drone_c,
        grid=corridor_grid,
        cell_size_m=1.0,
        get_time_fn=lambda: 0.0,
        rate_hz=8.0,
        num_rays=360,
        range_min=0.15,
        range_max=12.0,
        noise_sigma=0.01,
        dropout_prob=0.0,
    )
    scan_c = lidar_c.read()
    assert scan_c is not None

    grid_map_c = OccupancyGrid(width_m=20.0, height_m=4.0, resolution_m=0.25)
    grid_map_c.update_lidar_scan(10.0, 2.0, 0.0, scan_c)

    matcher = ScanMatcher()
    result = matcher.match(
        scan=scan_c,
        grid_map=grid_map_c,
        initial_pose=(10.0, 2.0, 0.0),
    )

    assert result.valid is False, "Scan matcher must reject featureless corridor geometry"
    assert "degenerate_geometry" in (result.reject_reason or "")

