"""Motion-evidence diagnostics: why the stationary interval restarted, with the odometry
stamp, this node's clock and their difference; a real motion fault told apart from the
restart allowance expiring. Diagnostics only: what counts as stationary is unchanged."""

import json

import pytest

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.motion_settlement import MotionSettlement
from test_mission_manager import manager_factory  # noqa: F401 (fixture)
from test_parking_feasibility import stopped
from test_parking_mission_navigation import _delayed_sample, feasible, hold_still


def test_a_completed_stop_still_confirms_and_reports_no_restarts():
    m = MotionSettlement()
    ns, wall = stopped(m)
    assert m.ready(ns, wall) and m.reason == 'motion_settled'
    assert m.restarts == {} and m.summary() == '' and m.explain() == 'motion_settled'


def test_genuine_motion_is_still_a_fault_and_is_recorded_with_its_speed():
    m = MotionSettlement()
    ns, wall = stopped(m)
    m.observe((0.,) * 5 + (0.007,), ns + 100_000_000, ns + 100_000_000, wall + 0.1)
    assert m.faults == 1 and m.reason == 'robot_moving' and not m.ready(ns + 100_000_000, wall + 0.1)
    fault = m.last_fault
    assert fault['reason'] == 'robot_moving' and fault['fault'] and fault['angular_radps'] == 0.007
    assert 'angular_radps 0.007' in m.describe(fault)


@pytest.mark.parametrize('case', ['missing', 'stale'])
def test_missing_or_stale_odometry_still_blocks_and_is_not_called_motion(case):
    m = MotionSettlement()
    if case == 'missing':
        assert not m.ready(1_000_000_000, 1.0) and m.reason == 'odometry_missing'
        assert m.restarts == {}
        return
    ns, wall = stopped(m)
    assert not m.ready(ns, wall + 0.6) and m.reason == 'odometry_stale'
    restart = m.last_restart
    assert restart['reason'] == 'odometry_stale' and not restart['fault'] and m.faults == 0
    assert restart['wall_age_s'] == pytest.approx(0.6)


def test_a_future_stamp_still_restarts_and_records_the_clock_difference():
    m = MotionSettlement()
    ns, wall = stopped(m)
    m.observe((0.,) * 6, ns + 400_000_000, ns + 100_000_000, wall + 0.1)   # 0.3 s ahead
    assert m.reason == 'odometry_timestamp' and m.faults == 0
    assert not m.ready(ns + 100_000_000, wall + 0.1)
    restart = m.last_restart
    assert restart['stamp_minus_clock_s'] == pytest.approx(0.3) and not restart['fault']
    assert m.skew_range_s[1] == pytest.approx(0.3)
    assert 'stamp-clock +0.300 s' in m.summary()


def test_gaps_are_named_and_a_new_wait_clears_the_diagnostics():
    m = MotionSettlement()
    ns, wall = stopped(m)
    m.observe((0.,) * 6, ns + 100_000_000, ns + 100_000_000, wall + 0.7)      # late callback
    m.observe((0.,) * 6, ns + 800_000_000, ns + 800_000_000, wall + 0.8)      # stamp jump
    assert m.restarts == {'callback_gap': 1, 'odometry_gap': 1}
    assert m.reason == 'collecting_stationary_interval'
    faults = m.faults
    m.reset()
    assert m.restarts == {} and m.last_restart is None and m.faults == faults


def test_restarts_never_change_when_the_stop_is_confirmed():
    """The restart records are bookkeeping: after them, a full interval confirms as before."""
    m = MotionSettlement()
    ns, wall = stopped(m)
    m.observe((0.,) * 6, ns + 400_000_000, ns + 100_000_000, wall + 0.1)
    ns, wall = stopped(m, first=ns * 1e-9 + 0.2, wall=wall + 0.2)
    assert m.ready(ns, wall) and m.restarts == {'odometry_timestamp': 1}


def test_a_plan_lost_to_motion_names_the_fault_not_the_later_samples(manager_factory, monkeypatch):
    """Before, the reason was read at failure time, after later still samples, and the
    Gazebo runs of 2026-11-14 and 2026-11-19 read only collecting_stationary_interval."""
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    _delayed_sample(m, velocity=0.05, wall_gap=0.1)
    m.test_ros_ns += 100_000_000                           # a still sample before the tick
    m._monotonic.value += 0.1
    m.motion.observe((0.,) * 6, m.test_ros_ns, m.test_ros_ns, m._monotonic())
    assert m.motion.reason == 'collecting_stationary_interval'
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    assert m.last_issue.reason.startswith('motion_evidence_lost: motion fault robot_moving')
    assert 'linear_mps 0.05' in m.last_issue.reason


def test_an_expired_restart_allowance_says_so_and_why(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    _delayed_sample(m)                                     # a callback delayed 0.7 s
    m.tick()
    assert m.state == MissionState.PARKING_PATH_FEASIBLE
    m._monotonic.value += 10.5                             # odometry stops entirely
    m.tick()
    assert m.state == MissionState.PARKING_PLANNING_FAILED
    reason = m.last_issue.reason
    assert reason.startswith('motion_evidence_lost: restart allowance of 10.0 s expired without motion')
    assert 'motion fault' not in reason
    assert 'callback_gap' in reason and 'odometry_stale' in reason


def test_the_state_carries_the_diagnostics_and_the_log_is_rate_limited(manager_factory, monkeypatch):
    m, _ = feasible(manager_factory, monkeypatch, enabled=False)
    published, lines = [], []
    m.state_publisher.publish = published.append

    class Log:
        def info(self, text):
            lines.append(text)

        def __getattr__(self, name):
            return lambda *a, **k: None

    monkeypatch.setattr(m, 'get_logger', lambda: Log())
    for _ in range(3):                                     # three restarts within 2 s
        ns = m.motion.last_ns + 100_000_000 if m.motion.last_ns else m.test_ros_ns + 100_000_000
        m.test_ros_ns, m._monotonic.value = ns - 300_000_000, m._monotonic.value + 0.2
        m.motion.observe((0.,) * 6, ns, m.test_ros_ns, m._monotonic())
        hold_still(m, seconds=0.3)
        m.tf_buffer.stamp_ns = m.test_ros_ns
        m.tick()
        assert m.state == MissionState.PARKING_PATH_FEASIBLE
    restarted = [line for line in lines if line.startswith('Stationary evidence restarted')]
    assert len(restarted) == 1 and 'odometry_timestamp' in restarted[0]
    m._publish_state('probe', event='safety_probe')
    evidence = json.loads(published[-1].data)['motion_evidence']
    assert evidence['restarts']['odometry_timestamp'] == 3
    assert evidence['last_restart']['stamp_minus_clock_s'] == pytest.approx(0.3)
