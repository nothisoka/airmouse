"""Unit test for collision event counting on contact start."""
import numpy as np
from airmouse.core.mission.mission_logger import MissionLogger


def test_collision_event_counting_on_contact_transition():
    """Test: 100 consecutive in-contact steps = 1 event; leave and re-enter = 2 events."""
    logger = MissionLogger(total_free_cells=100, true_survivors=[])
    pos = np.zeros(3)

    # 1. 100 consecutive in-contact steps -> exactly 1 collision event
    for step in range(100):
        logger.record_step(est_pos=pos, true_pos=pos, is_collision=True, sim_time=step * 0.01)
    assert logger.collisions == 1

    # 2. Leave contact (free space for 10 steps) -> still 1 event
    for step in range(100, 110):
        logger.record_step(est_pos=pos, true_pos=pos, is_collision=False, sim_time=step * 0.01)
    assert logger.collisions == 1

    # 3. Re-enter contact for 50 steps -> exactly 2 events
    for step in range(110, 160):
        logger.record_step(est_pos=pos, true_pos=pos, is_collision=True, sim_time=step * 0.01)
    assert logger.collisions == 2

    # Check metrics
    metrics = logger.compute_metrics(
        flight_time_s=1.6,
        explored_free_cells=10,
        confirmed_survivors=[],
        start_world=(0.0, 0.0),
        final_true_pos=pos,
    )
    assert metrics.collision_events == 2
