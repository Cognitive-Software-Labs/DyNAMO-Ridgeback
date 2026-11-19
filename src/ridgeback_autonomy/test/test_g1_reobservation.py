"""One bounded step back before planning when G1's heading is unknown from too close a view.

The final Isaac demonstration of 2026-11-19 localized G1 from 0.76-0.86 m, where its lower
legs are out of the camera image: no heading, a position biased toward the far leg, and
parking ranked by the approach side. The mission now steps back once to put the camera
about 1.6 m from G1 and localizes it again; the step is checked in the map, needs fresh
LiDAR and odometry, and is watched by both LiDARs like the parking legs.
"""

import json
import math

import numpy as np
import pytest
from action_msgs.msg import GoalStatus

from ridgeback_autonomy.mission import mission_manager_node as manager_module
from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.target_orientation import (
    HEADING_ESTIMATED, HEADING_VIEW_TOO_CLOSE, HeadingAssessment, OrientationEstimate,
)
from test_g1_exploration import aim_camera, depth_frame
from test_mission_manager import (  # noqa: F401 (fixture)
    FakePauseClient, FakeTfBuffer, PLAN_NOW_NS, detection, enter_localization, manager_factory,
    measurement, occupancy_map, trigger_response,
)
from test_parking_clearance import feed
from test_parking_feasibility import Client
from test_parking_mission_navigation import ParamClient, answer, hold_still, navigation_result, release

WEST = math.pi
ROBOT = (3.0, 2.0, 0.25)


def headings(monkeypatch, m, per_localization):
    """Each localization's five depth frames give ``yaw`` (None: a view too close for one)."""
    queue = iter([yaw for yaw in per_localization for _ in range(5)])

    def assess(points, center, lowest):
        yaw = next(queue)
        if yaw is None:
            return HeadingAssessment(None, HEADING_VIEW_TOO_CLOSE, 'lowest visible height at G1 0.72 m')
        return HeadingAssessment(OrientationEstimate(yaw, 200, 0.95), HEADING_ESTIMATED)
    monkeypatch.setattr(manager_module, 'assess_heading', assess)
    m._camera_to_map = lambda: (np.eye(3), np.zeros(3))
    m._camera_intrinsics = (100.0, 100.0, 1.0, 1.0)


def localize_from(m, forward, first_stamp_ns):
    """Five depth frames, then five detections measuring G1 ``forward`` m along the camera axis."""
    for _ in range(5):
        m._on_depth(depth_frame())
    for offset in range(5):
        stamp_ns = first_stamp_ns + offset
        m.process_detection(detection(stamp_ns))
        msg = measurement(stamp_ns)
        msg.euclidean_reconstruction_forward_m = [forward]
        msg.euclidean_reconstruction_lateral_m = [0.0]
        msg.euclidean_reconstruction_distance_m = [forward]
        m.process_measurement(msg, now_ns=stamp_ns)


def localized(manager_factory, monkeypatch, *, yaws=(None, None), forward=0.8, lidar=True, map_msg=None,
              returns=(), scan_age_s=0.0):
    """G1 localized ``forward`` m ahead of the camera, after LiDAR scans (``scan_age_s`` old,
    with base-frame ``returns``) and odometry; ``lidar=False`` feeds none."""
    m = manager_factory(tf_buffer=FakeTfBuffer(x=ROBOT[0], y=ROBOT[1], yaw=ROBOT[2], stamp_ns=PLAN_NOW_NS),
                        pause_client=FakePauseClient(trigger_response(True, 'safe')))
    m.resume_client = FakePauseClient(trigger_response(True, 'resumed'))
    m.navigation_client = Client()
    m._goal_checker_set_client = ParamClient()
    m.parking_navigation_enabled = True
    published = []
    m.state_publisher.publish = published.append
    m._on_map(map_msg or occupancy_map(size=160))        # 8 m, free
    aim_camera(m)                                         # the camera at the robot, toward G1
    enter_localization(m)
    headings(monkeypatch, m, yaws)
    if lidar:
        feed(m, returns, stamp_ns=m.test_ros_ns - int(scan_age_s * 1e9))
    localize_from(m, forward, 4_000_000_000)
    return m, published


def steps(m):
    return [t for t in m.transition_history if t.current == MissionState.RETURNING_TO_OBSERVATION]


def g1_range(m, x, y):
    return math.hypot(m.target_position.map_x_m - x, m.target_position.map_y_m - y)


def stepping(manager_factory, monkeypatch, **kwargs):
    """The step back started, its goal accepted by Nav2: (manager, goal, handle, published)."""
    m, published = localized(manager_factory, monkeypatch, **kwargs)
    assert m.state == MissionState.RETURNING_TO_OBSERVATION and len(steps(m)) == 1
    answer(m._goal_checker_set_client)
    request, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    p = request.pose.pose
    goal = (p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))
    return m, goal, handle, published


def relocalize_after_step(m, at, forward):
    """The robot stands at ``at`` and localizes G1 again from there."""
    m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw = at
    m.test_ros_ns = m.tf_buffer.stamp_ns = m.test_ros_ns + 10
    fresh = m.latest_map                              # SLAM keeps publishing the map
    fresh.header.stamp.sec, fresh.header.stamp.nanosec = divmod(m.test_ros_ns, 1_000_000_000)
    m._on_map(fresh)
    feed(m)
    localize_from(m, forward, m.test_ros_ns - 4)


# The measurement's range is from the camera's front plane, 0.25 m short of the camera here:
# forward 0.8 places G1 1.05 m from the camera, 1.35 places it 1.6 m away.
CLOSE, AFTER_STEP, FAR = 0.8, 1.35, 1.6
PLANNING = (MissionState.PARKING_CANDIDATES_READY, MissionState.EVALUATING_PARKING_PATHS)


def state_payload(published):
    return json.loads(published[-1].data)


def test_a_known_heading_needs_no_step_back(manager_factory, monkeypatch):
    m, _ = localized(manager_factory, monkeypatch, yaws=(WEST,))
    assert g1_range(m, *ROBOT[:2]) < 1.5
    assert m.state in PLANNING and not steps(m) and m.navigation_client.calls == []
    assert m.g1_reobservation is None and m.parking_plan.target.facing_yaw_rad == pytest.approx(WEST)


def test_an_unknown_heading_from_far_enough_does_not_step_back(manager_factory, monkeypatch):
    m, published = localized(manager_factory, monkeypatch, yaws=(None,), forward=FAR)
    assert g1_range(m, *ROBOT[:2]) > 1.5
    assert m.state in PLANNING and not steps(m) and m.navigation_client.calls == []
    labels = {c.preference_label for c in m.parking_plan.candidates if c.status == 'valid'}
    assert labels and all(label.startswith('approach') for label in labels)   # never a G1 side
    assert state_payload(published)['g1_heading_quality']['status'] == HEADING_VIEW_TOO_CLOSE


def test_an_unknown_heading_from_close_steps_back_once_and_uses_the_new_heading(
        manager_factory, monkeypatch):
    m, goal, handle, published = stepping(manager_factory, monkeypatch, yaws=(None, WEST))
    # The camera (at the base in this fixture) 1.6 m from G1, on G1's line through the
    # robot, facing G1; the move is bounded by its own deadline.
    assert g1_range(m, *goal[:2]) == pytest.approx(1.6, abs=0.01)
    bearing = math.atan2(m.target_position.map_y_m - goal[1], m.target_position.map_x_m - goal[0])
    assert abs(math.remainder(goal[2] - bearing, 2 * math.pi)) < 1e-6
    assert m._nav_deadline_s == pytest.approx(m._monotonic() + m.g1_reobservation_timeout_s)
    m.tick()                                         # clear scans: the step goes on
    assert m.state == MissionState.RETURNING_TO_OBSERVATION and handle.cancels == 0
    navigation_result(m.navigation_client)
    assert m.state == MissionState.SETTLING_MOTION   # stopped first, then a fresh localization
    hold_still(m)
    m.tick()
    assert m.state == MissionState.LOCALIZE_G1
    relocalize_after_step(m, goal, AFTER_STEP)
    assert m.state in PLANNING and len(steps(m)) == 1 and len(m.navigation_client.calls) == 1
    assert m.target_heading.yaw_rad == pytest.approx(WEST)
    assert m.parking_plan.target.facing_yaw_rad == pytest.approx(WEST)
    record = state_payload(published)['g1_reobservation']
    assert record['outcome'] == 'reached' and record['heading_status_before'] == HEADING_VIEW_TOO_CLOSE
    assert record['heading_status_after'] == HEADING_ESTIMATED
    assert record['camera_range_after_m'] is not None


def test_a_reobservation_without_a_heading_plans_undirected_and_never_repeats(
        manager_factory, monkeypatch):
    m, goal, handle, published = stepping(manager_factory, monkeypatch, yaws=(None, None))
    navigation_result(m.navigation_client)
    hold_still(m)
    m.tick()
    relocalize_after_step(m, goal, CLOSE)            # still unknown, and still close
    assert m.state in PLANNING and len(steps(m)) == 1 and len(m.navigation_client.calls) == 1
    assert m.target_heading is None and m.parking_plan.target.facing_yaw_rad is None
    labels = {c.preference_label for c in m.parking_plan.candidates if c.status == 'valid'}
    assert labels and all(label.startswith('approach') for label in labels)
    assert state_payload(published)['g1_reobservation']['heading_status_after'] == HEADING_VIEW_TOO_CLOSE


def test_a_blocked_observation_pose_never_moves(manager_factory, monkeypatch):
    blocked = occupancy_map(size=160)
    data = np.array(blocked.data).reshape(160, 160)
    data[30:50, 36:48] = 100                         # x 1.8-2.4 m, y 1.5-2.5 m: behind the robot
    blocked.data = data.ravel().tolist()
    m, published = localized(manager_factory, monkeypatch, map_msg=blocked)
    assert not steps(m) and m.navigation_client.calls == []
    assert m.g1_reobservation['outcome'].startswith('not started')
    assert m.state in PLANNING + (MissionState.EXPLORING_AROUND_G1, MissionState.PARKING_PLANNING_FAILED,
                                  MissionState.RETURNING_TO_START)   # the existing fallback decides


@pytest.mark.parametrize('problem', ['missing', 'stale', 'obstacle'])
def test_no_step_back_without_fresh_clear_lidar(manager_factory, monkeypatch, problem):
    if problem == 'missing':
        m, _ = localized(manager_factory, monkeypatch, lidar=False)
        expected = 'lidar_unavailable'
    elif problem == 'stale':
        m, _ = localized(manager_factory, monkeypatch, scan_age_s=2.0)
        expected = 'lidar_stale'
    else:
        m, _ = localized(manager_factory, monkeypatch, returns=[(-0.85, 0.0)])   # someone behind
        expected = 'lidar_obstacle_on_route'
    assert not steps(m) and m.navigation_client.calls == []
    assert expected in m.g1_reobservation['outcome']
    assert m.state in PLANNING


def ended_early_then_relocalized(m, handle, *, cancelled=True):
    """Nothing drives before Nav2 releases the step and the robot is seen stopped; then G1 is
    localized from where the robot stands, and the step is not repeated."""
    calls, params = len(m.navigation_client.calls), len(m._goal_checker_set_client.calls)
    assert m.state == MissionState.RETURNING_TO_OBSERVATION and m._step_back_stopping
    if cancelled:
        assert handle.cancels == 1
        hold_still(m)
        m.tick()
        assert m.state == MissionState.RETURNING_TO_OBSERVATION    # no release yet: no localization
        release(handle)
        m.tick()
    assert m.state == MissionState.RETURNING_TO_OBSERVATION        # released, stop not yet seen
    hold_still(m)
    m.tick()
    assert m.state == MissionState.SETTLING_MOTION
    hold_still(m)
    m.tick()
    assert m.state == MissionState.LOCALIZE_G1
    assert len(m.navigation_client.calls) == calls                 # no new goal of any kind
    assert len(m._goal_checker_set_client.calls) == params + 1     # only the shared checker back
    relocalize_after_step(m, ROBOT, CLOSE)
    assert m.state in PLANNING and len(steps(m)) == 1 and len(m.navigation_client.calls) == calls


def test_an_obstacle_on_the_step_cancels_it_and_g1_is_localized_from_where_it_stopped(
        manager_factory, monkeypatch):
    m, _, handle, published = stepping(manager_factory, monkeypatch)
    feed(m)                                          # clear: the step goes on
    assert handle.cancels == 0
    feed(m, [(-0.85, 0.0)])                          # someone steps behind the robot
    assert 'lidar_obstacle_on_route' in m.g1_reobservation['outcome']
    ended_early_then_relocalized(m, handle)
    leg = state_payload(published)['parking_clearance']['legs']['step_back']
    assert leg['checks'] >= 2


def test_an_obstacle_in_the_stopping_path_cancels_the_step(manager_factory, monkeypatch):
    m, _, handle, _ = stepping(manager_factory, monkeypatch)
    feed(m, [(-0.53, 0.0)])                          # within the margin of the rear, basket included
    assert 'lidar_obstacle_in_stopping_path' in m.g1_reobservation['outcome']
    ended_early_then_relocalized(m, handle)


@pytest.mark.parametrize('problem', ['missing', 'stale'])
def test_lidar_lost_during_the_step_cancels_it(manager_factory, monkeypatch, problem):
    m, _, handle, _ = stepping(manager_factory, monkeypatch)
    if problem == 'missing':
        m._scans.clear()
    else:
        m.test_ros_ns += 2_000_000_000               # no scan for 2 s: the steady tick notices
    m.tick()
    assert f'lidar_{"unavailable" if problem == "missing" else "stale"}' in m.g1_reobservation['outcome']
    ended_early_then_relocalized(m, handle)


def test_the_step_back_deadline_ends_it_once(manager_factory, monkeypatch):
    m, _, handle, _ = stepping(manager_factory, monkeypatch)
    m._monotonic.value += m.g1_reobservation_timeout_s + 1.0
    feed(m)                                          # LiDAR still fresh: only the deadline expires
    m.tick()
    assert 'deadline expired' in m.g1_reobservation['outcome']
    ended_early_then_relocalized(m, handle)


def test_an_aborted_step_is_not_retried(manager_factory, monkeypatch):
    m, _, handle, _ = stepping(manager_factory, monkeypatch)
    navigation_result(m.navigation_client, status=GoalStatus.STATUS_ABORTED)
    assert 'ended early' in m.g1_reobservation['outcome']
    ended_early_then_relocalized(m, handle, cancelled=False)


def test_no_confirmed_stop_after_a_cancelled_step_fails_without_driving(manager_factory, monkeypatch):
    m, _, handle, _ = stepping(manager_factory, monkeypatch)
    feed(m, [(-0.85, 0.0)])
    release(handle)
    calls = len(m.navigation_client.calls)
    m._monotonic.value += m.motion_settlement_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'stop_not_confirmed'
    assert len(m.navigation_client.calls) == calls


def test_reobservation_can_be_switched_off(manager_factory, monkeypatch):
    from rclpy.parameter import Parameter
    original = manager_factory

    def factory(**kwargs):
        return original(overrides=[Parameter('g1_reobservation_enabled', value=False)], **kwargs)
    m, _ = localized(factory, monkeypatch)
    assert not steps(m) and m.state in PLANNING and m.g1_reobservation is None
