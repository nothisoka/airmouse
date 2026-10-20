"""Test EKF sensor fusion and convergence."""
import numpy as np
import pytest
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.hal import ImuData, RangeData, CameraFrame, PoseQuality


def test_ekf_prediction_and_sonar_update():
    init_pos = np.array([2.5, 3.5, 1.2])
    ekf = SensorFusionEKF(init_pos=init_pos, init_yaw=0.0)

    # Simulate 1 second of hovering at (2.5, 3.5, 1.2)
    t = 0.0
    dt = 0.01  # 100 Hz IMU
    for step in range(100):
        t += dt
        # IMU measuring gravity in body Z: [0, 0, 9.80665] + small noise
        imu = ImuData(
            timestamp_s=t,
            accel_mps2=np.array([0.01, -0.01, 9.80665 + 0.02]),
            gyro_radps=np.array([0.001, -0.001, 0.0]),
        )
        ekf.predict(imu)

        # 20 Hz sonar update
        if step % 5 == 0:
            ranges = {
                "down": RangeData(
                    sensor_id="down",
                    range_m=1.21,
                    valid=True,
                    timestamp_s=t,
                )
            }
            ekf.update_sonar(ranges)

    state = ekf.get_state(timestamp_s=t)

    # Verify altitude converged close to 1.21m
    assert abs(state.position[2] - 1.21) < 0.05
    # Verify horizontal position drift is small
    assert abs(state.position[0] - 2.5) < 0.2
    assert abs(state.position[1] - 3.5) < 0.2
    # Verify pose quality is EXCELLENT or DEGRADED (not CRITICAL)
    assert state.quality in (PoseQuality.EXCELLENT, PoseQuality.DEGRADED)

