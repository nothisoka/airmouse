"""Unit tests for exit_reason rules (Task A7).

COMPLETE only if (coverage target met OR no reachable frontiers) AND home_error_m < 1.0.
Landed but not meeting that = INCOMPLETE.
Others: TIMEOUT, FAILSAFE, CRASH.
"""
import numpy as np
import pytest
from airmouse.core.mission.mission_logger import MissionLogger, determine_exit_reason


def test_exit_reason_complete_on_coverage_target():
    """Outcome 1a: COMPLETE when landed, coverage met (>= 90%), home_error < 1.0m."""
    reason = determine_exit_reason(
        base_exit_reason="COMPLETE",
        landed=True,
        coverage_pct=92.5,
        frontier_clusters_remaining=3,
        home_error_m=0.45,
        coverage_target_pct=90.0,
    )
    assert reason == "COMPLETE"


def test_exit_reason_complete_on_no_frontiers():
    """Outcome 1b: COMPLETE when landed, no frontiers remaining, home_error < 1.0m."""
    reason = determine_exit_reason(
        base_exit_reason="COMPLETE",
        landed=True,
        coverage_pct=65.0,
        frontier_clusters_remaining=0,
        home_error_m=0.72,
        coverage_target_pct=90.0,
    )
    assert reason == "COMPLETE"


def test_exit_reason_incomplete_when_not_meeting_conditions():
    """Outcome 2: INCOMPLETE when landed but neither coverage met nor no frontiers, or home_error >= 1.0m."""
    # Case A: coverage low and frontiers remain
    reason_low_cov = determine_exit_reason(
        base_exit_reason="COMPLETE",
        landed=True,
        coverage_pct=50.0,
        frontier_clusters_remaining=4,
        home_error_m=0.3,
        coverage_target_pct=90.0,
    )
    assert reason_low_cov == "INCOMPLETE"

    # Case B: coverage high, but landed too far from launch (home_error >= 1.0m)
    reason_bad_rth = determine_exit_reason(
        base_exit_reason="COMPLETE",
        landed=True,
        coverage_pct=95.0,
        frontier_clusters_remaining=0,
        home_error_m=1.85,
        coverage_target_pct=90.0,
    )
    assert reason_bad_rth == "INCOMPLETE"

    # Case C: did not land
    reason_unlanded = determine_exit_reason(
        base_exit_reason="COMPLETE",
        landed=False,
        coverage_pct=95.0,
        frontier_clusters_remaining=0,
        home_error_m=0.2,
        coverage_target_pct=90.0,
    )
    assert reason_unlanded == "INCOMPLETE"


def test_exit_reason_timeout():
    """Outcome 3: TIMEOUT preserves TIMEOUT reason."""
    reason = determine_exit_reason(
        base_exit_reason="TIMEOUT",
        landed=False,
        coverage_pct=30.0,
        frontier_clusters_remaining=12,
        home_error_m=8.5,
    )
    assert reason == "TIMEOUT"


def test_exit_reason_failsafe():
    """Outcome 4: FAILSAFE preserves FAILSAFE reason."""
    reason = determine_exit_reason(
        base_exit_reason="FAILSAFE",
        landed=False,
        coverage_pct=15.0,
        frontier_clusters_remaining=8,
        home_error_m=4.2,
    )
    assert reason == "FAILSAFE"


def test_exit_reason_crash():
    """Outcome 5: CRASH preserves CRASH reason."""
    reason = determine_exit_reason(
        base_exit_reason="CRASH",
        landed=False,
        coverage_pct=10.0,
        frontier_clusters_remaining=15,
        home_error_m=6.0,
    )
    assert reason == "CRASH"


def test_mission_logger_finalize_with_exit_reason_rules():
    """Verify MissionLogger.compute_metrics integrates determine_exit_reason."""
    logger = MissionLogger(total_free_cells=100, true_survivors=[])
    logger.record_step(np.zeros(3), np.array([0.0, 0.0, 1.2]), False, sim_time=1.0)
    logger.record_step(np.array([0.2, 0.1, 0.05]), np.array([0.2, 0.1, 0.05]), False, sim_time=100.0)

    # 1. Complete run
    metrics_complete = logger.compute_metrics(
        flight_time_s=100.0,
        explored_free_cells=95,
        confirmed_survivors=[],
        start_world=(0.0, 0.0),
        final_true_pos=np.array([0.2, 0.1, 0.05]),  # landed near start
        frontier_clusters_remaining=0,
        exit_reason="COMPLETE",
        coverage_target_pct=90.0,
    )
    assert metrics_complete.exit_reason == "COMPLETE"
    assert metrics_complete.exit_code == 0
    assert metrics_complete.success is True

    # 2. Incomplete run (landed, but coverage not met and frontiers left)
    metrics_incomplete = logger.compute_metrics(
        flight_time_s=100.0,
        explored_free_cells=40,
        confirmed_survivors=[],
        start_world=(0.0, 0.0),
        final_true_pos=np.array([0.2, 0.1, 0.05]),
        frontier_clusters_remaining=5,
        exit_reason="COMPLETE",
        coverage_target_pct=90.0,
    )
    assert metrics_incomplete.exit_reason == "INCOMPLETE"
    assert metrics_incomplete.exit_code == 1
    assert metrics_incomplete.success is False

    # 3. Timeout run
    metrics_timeout = logger.compute_metrics(
        flight_time_s=280.0,
        explored_free_cells=30,
        confirmed_survivors=[],
        start_world=(0.0, 0.0),
        final_true_pos=np.array([5.0, 5.0, 1.2]),
        frontier_clusters_remaining=8,
        exit_reason="TIMEOUT",
    )
    assert metrics_timeout.exit_reason == "TIMEOUT"
    assert metrics_timeout.exit_code == 1

    # 4. Failsafe run
    metrics_failsafe = logger.compute_metrics(
        flight_time_s=50.0,
        explored_free_cells=20,
        confirmed_survivors=[],
        start_world=(0.0, 0.0),
        final_true_pos=np.array([3.0, 3.0, 1.2]),
        frontier_clusters_remaining=10,
        exit_reason="FAILSAFE",
    )
    assert metrics_failsafe.exit_reason == "FAILSAFE"
    assert metrics_failsafe.exit_code == 1

    # 5. Crash run
    metrics_crash = logger.compute_metrics(
        flight_time_s=20.0,
        explored_free_cells=5,
        confirmed_survivors=[],
        start_world=(0.0, 0.0),
        final_true_pos=np.array([1.0, 1.0, 0.0]),
        frontier_clusters_remaining=12,
        exit_reason="CRASH",
    )
    assert metrics_crash.exit_reason == "CRASH"
    assert metrics_crash.exit_code == 1
