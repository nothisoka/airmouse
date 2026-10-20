"""Test simulated sensors and flight dynamics."""
import math
import numpy as np
import pytest
from airmouse.backends.pygame_sim.drone_dynamics import DroneDynamics
from airmouse.backends.pygame_sim.world_renderer import WorldRenderer
from airmouse.backends.pygame_sim.sensor_sim import SimulatedSensorHub
from airmouse.backends.pygame_sim.flight_sim import SimulatedFlightInterface
from airmouse.core.hal import VelocityCommand


def test_sensor_sim():
    np.random.seed(42)
    # 10x10 empty room with walls
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1

    # Spawn drone at center: (5.0, 5.0, 1.2m altitude)
    drone = DroneDynamics(initial_position=np.array([5.0, 5.0, 1.2]), initial_yaw=0.0)
    renderer = WorldRenderer(grid=grid, cell_size_m=1.0)
    renderer.true_survivors = [(7.0, 5.0)]

    sensor_hub = SimulatedSensorHub(drone=drone, renderer=renderer, grid=grid, cell_size_m=1.0)
    flight = SimulatedFlightInterface(drone=drone)

    # 1. Test IMU reading
    sensor_hub.advance_time(0.02)
    imu = sensor_hub.read_imu()
    assert imu is not None
    # Accelerometer z should measure ~ +9.8m/s^2 (specific force supporting gravity in hover)
    accel_mag = np.linalg.norm(imu.accel_mps2)
    assert 8.5 < accel_mag < 11.5
    assert np.linalg.norm(imu.gyro_radps) < 0.25

    # 2. Test Range readings (4 horizontal + 1 downward)
    sensor_hub.advance_time(0.06)
    ranges = sensor_hub.read_ranges()
    assert len(ranges) == 5
    assert "front" in ranges and "left" in ranges and "right" in ranges and "rear" in ranges and "down" in ranges
    # Distance to wall in front from (5, 5) heading +X (east): walls are at x=9, so distance is ~4m
    assert ranges["front"].range_m > 2.0
    # Downward distance should be ~1.2m
    assert 1.0 < ranges["down"].range_m < 1.4

    # 3. Test Camera reading
    sensor_hub.advance_time(0.06)
    cam = sensor_hub.read_camera()
    assert cam is not None
    assert cam.image_bgr.shape == (240, 320, 3)

    # 4. Test Flight Interface
    flight.send_velocity(VelocityCommand(vx=0.5, vy=0.0, vz=0.0, yaw_rate_radps=0.0))
    drone.step(0.1)
    state = drone.get_state()
    assert state.battery_pct > 99.0

