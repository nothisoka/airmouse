"""Unit test verifying sensor profiles, HAL interfaces, and core autonomy execution.

Validates Task D2:
- Baseline profile has no lidar object and core still runs.
- HAL SensorHub.has_sensor, get_camera, and get_lidar interfaces.
- Core components operate gracefully with any sensor subset.
"""
import math
import numpy as np
import pytest

from airmouse.core.hal import (
    SensorHub,
    Camera,
    CameraFrame,
    Lidar2D,
    LidarScan,
    DroneState,
    VelocityCommand,
)
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub
from airmouse.backends.pygame_sim.flight_sim import SimulatedFlightInterface
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState


def test_baseline_profile_has_no_lidar():
    grid = np.zeros((10, 10), dtype=np.uint8)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    renderer = WorldRenderer(grid=grid, cell_size_m=1.0)

    hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=grid,
        cell_size_m=1.0,
        profile="baseline",
    )

    # Baseline must NOT have lidar
    assert hub.get_lidar() is None, "Baseline profile must not have a lidar object"
    assert hub.has_sensor("lidar2d") is False
    assert hub.has_sensor("lidar") is False

    # Baseline must have front camera, but NOT downward camera
    assert hub.has_sensor("camera_front") is True
    assert hub.has_sensor("camera") is True
    assert hub.has_sensor("camera_down") is False

    cam_front = hub.get_camera("front")
    assert cam_front is not None
    assert isinstance(cam_front, Camera)
    assert cam_front.name == "front"

    cam_down = hub.get_camera("down")
    assert cam_down is None

    # Baseline has IMU and ultrasonic
    assert hub.has_sensor("mpu6050") is True
    assert hub.has_sensor("imu") is True
    assert hub.has_sensor("ultrasonic") is True
    assert hub.has_sensor("sonar") is True


def test_flow_and_full_profiles():
    grid = np.zeros((10, 10), dtype=np.uint8)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    renderer = WorldRenderer(grid=grid, cell_size_m=1.0)

    # Flow profile
    flow_hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=grid,
        profile="flow",
    )
    assert flow_hub.has_sensor("camera_front") is True
    assert flow_hub.has_sensor("camera_down") is True
    assert flow_hub.has_sensor("lidar2d") is False
    assert flow_hub.get_lidar() is None
    assert flow_hub.get_camera("down") is not None

    # Full profile
    full_hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=grid,
        profile="full",
    )
    assert full_hub.has_sensor("camera_front") is True
    assert full_hub.has_sensor("camera_down") is True
    assert full_hub.has_sensor("lidar2d") is True


def test_core_runs_with_baseline_profile():
    grid = np.zeros((10, 10), dtype=np.uint8)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    renderer = WorldRenderer(grid=grid, cell_size_m=1.0)

    hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=grid,
        cell_size_m=1.0,
        profile="baseline",
    )
    flight = SimulatedFlightInterface(drone=drone)

    grid_map = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)
    ekf = SensorFusionEKF(init_pos=np.array([5.0, 5.0, 1.2]), init_yaw=0.0)
    mission = MissionCoordinator(
        start_world=(5.0, 5.0),
        grid_map=grid_map,
        flight=flight,
        sensor_hub=hub,
    )

    assert mission.sensor_hub is hub
    assert mission.sensor_hub.get_lidar() is None

    # Step simulation and sensors
    hub.advance_time(0.05)
    imu = hub.read_imu()
    ranges = hub.read_ranges()
    cam = hub.read_camera()

    assert imu is not None
    assert ranges is not None
    # cam may be ready or None depending on period

    ekf.predict(imu)
    if ranges:
        ekf.update_sonar(ranges)
    if cam:
        ekf.update_camera(cam, imu.gyro_radps)

    state = ekf.get_state(hub.get_time())
    assert not math.isnan(state.position[0])
    assert not math.isnan(state.position[1])
    assert not math.isnan(state.position[2])

    # Run one step of mission coordinator
    cmd = mission.update(current_time_s=hub.get_time(), drone_state=state, ranges=ranges)
    assert isinstance(cmd, VelocityCommand)


def test_core_runs_with_empty_sensor_subset():
    """Verify core autonomy does not crash if sensors are missing or disabled."""
    grid = np.zeros((10, 10), dtype=np.uint8)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    renderer = WorldRenderer(grid=grid, cell_size_m=1.0)

    # Disable all sensors via config
    empty_cfg = {
        "mpu6050": {"enabled": False},
        "ultrasonic": {"enabled": False},
        "camera_front": {"enabled": False},
        "camera_down": {"enabled": False},
        "lidar2d": {"enabled": False},
    }
    hub = SimulatedSensorHub(
        drone=drone,
        renderer=renderer,
        grid=grid,
        cell_size_m=1.0,
        profile="baseline",
        sensors_config=empty_cfg,
    )
    flight = SimulatedFlightInterface(drone=drone)
    grid_map = OccupancyGrid(width_m=10.0, height_m=10.0, resolution_m=0.25)
    mission = MissionCoordinator(
        start_world=(5.0, 5.0),
        grid_map=grid_map,
        flight=flight,
        sensor_hub=hub,
    )

    hub.advance_time(0.05)
    assert hub.read_imu() is None
    assert hub.read_ranges() == {}
    assert hub.read_camera() is None
    assert hub.get_lidar() is None
    assert hub.get_camera("front") is None

    # Dummy state to verify mission coordinator still steps cleanly
    dummy_state = DroneState(
        timestamp_s=hub.get_time(),
        position=np.array([5.0, 5.0, 1.2]),
        velocity=np.zeros(3),
        attitude_rpy=np.zeros(3),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        accel_bias=np.zeros(3),
        gyro_bias=np.zeros(3),
        covariance_diag=np.zeros(9),
    )
    cmd = mission.update(current_time_s=hub.get_time(), drone_state=dummy_state, ranges={})
    assert isinstance(cmd, VelocityCommand)
