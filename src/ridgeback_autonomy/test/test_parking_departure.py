"""The departure after the parked wait: a straight, checked move out to the pre-parking pose,
watched by the parking legs' LiDAR check, before the unchanged return to the start."""

import json
import math

import pytest
from action_msgs.msg import GoalStatus

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.parking_geometry import preparking_pose
from test_mission_manager import manager_factory  # noqa: F401 (fixture)
from test_parking_mission_navigation import (
    answer, feasible, hold_still, navigation_result, release)
from test_parking_clearance import feed


def to_base(m, x, y):
    """Map point -> base-frame point of the robot's current pose."""
    b = m.tf_buffer
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    return (c * (x - b.x) + s * (y - b.y), -s * (x - b.x) + c * (y - b.y))


def parked(manager_factory, monkeypatch):
    """Parked at the selected pose (the robot pose follows), stopped, scans clear."""
    m, published = feasible(manager_factory, monkeypatch)
    sel, params, nav = m.parking_plan.selected, m._goal_checker_set_client, m.navigation_client
    answer(params)
    navigation_result(nav)
    answer(params)
    navigation_result(nav)
    assert m.state == MissionState.PARKED
    m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw = sel.base_x, sel.base_y, sel.yaw
    hold_still(m)
    m.tick()                                       # stop confirmed: the wait starts
    return m, published


def fresh(m, points=(), stamp_ns=None):
    """Scans and a new stationary odometry sample, 50 ms after the last stop evidence."""
    m.test_ros_ns += 50_000_000
    feed(m, points, stamp_ns)


def end_wait(m, points=()):
    """The wait runs out with fresh stop evidence and scans showing ``points`` (base frame)."""
    hold_still(m, m.parking_wait_s + 0.2)
    fresh(m, points)
    m.tick()


def departing(manager_factory, monkeypatch):
    m, published = parked(manager_factory, monkeypatch)
    end_wait(m)
    assert m.state == MissionState.RETURNING_TO_START and m._departure_route is not None
    assert answer(m._goal_checker_set_client) == {'general_goal_checker.xy_goal_tolerance': 0.08,
                                                  'general_goal_checker.yaw_goal_tolerance': 0.12}
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    return m, handle, published


def ahead(m, distance=0.75):
    """A base-frame point straight ahead on the departure route, beyond the stopping distance."""
    return (distance, 0.0)


def test_a_clear_departure_drives_out_then_returns_home(manager_factory, monkeypatch):
    m, handle, published = departing(manager_factory, monkeypatch)
    sel = m.parking_plan.selected
    request = m.navigation_client.calls[-1][0].pose.pose
    goal = (request.position.x, request.position.y, 2 * math.atan2(request.orientation.z, request.orientation.w))
    assert goal == pytest.approx(preparking_pose(sel.base_x, sel.base_y, sel.yaw, 0.5))
    feed(m)                                         # monitored while driving: clear
    m.tick()
    assert m.state == MissionState.RETURNING_TO_START and handle.cancels == 0
    navigation_result(m.navigation_client)          # pre-parking pose reached
    assert m._departure_route is None and m.departure['outcome'] == 'reached'
    answer(m._goal_checker_set_client)              # the shared checker for the return
    assert navigation_result(m.navigation_client) == pytest.approx((1.0, 2.0, 0.25))
    assert m.state == MissionState.MISSION_COMPLETE and m.last_issue is None
    payload = json.loads(published['state'][-1].data)
    assert payload['departure']['outcome'] == 'reached'
    assert payload['parking_clearance']['legs']['departure']['checks'] >= 2
    assert 'departure' in payload['parking_mission_navigation']['arrival_errors']


def test_an_obstacle_on_the_departure_route_keeps_the_robot_stopped(manager_factory, monkeypatch):
    m, _ = parked(manager_factory, monkeypatch)
    calls = len(m.navigation_client.calls)
    end_wait(m, [ahead(m)])
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'departure_not_safe'
    assert 'lidar_obstacle' in m.departure['outcome'] and m.departure['outcome'].startswith('not started')
    assert len(m.navigation_client.calls) == calls          # nothing was driven


def test_a_return_beside_the_basket_corner_blocks_the_departure(manager_factory, monkeypatch):
    # 0.03 m behind the basket's rear-wall corner, 0.06 m from the body's angled rear end:
    # only the basket-aware outline puts it inside the 0.05 m stopping-path margin.
    m, _ = parked(manager_factory, monkeypatch)
    end_wait(m, [(-0.51, 0.17)])
    assert m.state == MissionState.MISSION_FAILED
    assert 'lidar_obstacle_in_stopping_path' in m.departure['outcome']


def test_an_obstacle_appearing_during_the_departure_stops_and_fails(manager_factory, monkeypatch):
    m, handle, _ = departing(manager_factory, monkeypatch)
    calls = len(m.navigation_client.calls)
    feed(m, [ahead(m, 0.6)])
    assert handle.cancels == 1 and m._departure_stopping
    assert 'lidar_obstacle' in m.departure['outcome']
    hold_still(m)
    m.tick()
    assert m.state == MissionState.RETURNING_TO_START       # not before Nav2 released the goal
    release(handle)
    hold_still(m)
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'departure_failed'
    assert len(m.navigation_client.calls) == calls          # no return goal near G1


@pytest.mark.parametrize('problem,reason', [('missing', 'lidar_unavailable'), ('stale', 'lidar_stale'),
                                            ('odometry', 'odometry_stale')])
def test_missing_or_stale_sensors_keep_the_robot_stopped(manager_factory, monkeypatch, problem, reason):
    m, _ = parked(manager_factory, monkeypatch)
    hold_still(m, m.parking_wait_s + 0.2)
    fresh(m)
    if problem == 'missing':
        m._scans.clear()
    elif problem == 'stale':
        fresh(m, stamp_ns=m.test_ros_ns - 2_000_000_000)
    else:
        m._execution_odom_received_s -= 5.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'departure_not_safe'
    assert reason in m.departure['outcome']


def test_stale_sensors_during_the_departure_stop_it(manager_factory, monkeypatch):
    m, handle, _ = departing(manager_factory, monkeypatch)
    m.test_ros_ns += 2_000_000_000                 # no scan for 2 s
    m.tick()
    assert handle.cancels == 1 and 'lidar_stale' in m.departure['outcome']


def test_no_confirmed_stop_after_the_wait_never_departs(manager_factory, monkeypatch):
    m, _ = parked(manager_factory, monkeypatch)
    m.test_ros_ns += int((m.parking_wait_s + 0.2) * 1e9)    # the wait ends without stop evidence
    feed(m)
    m.tick()
    assert m.state == MissionState.PARKED                    # waits for evidence first
    m._monotonic.value += m.motion_settlement_timeout_s + 0.1
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'departure_not_safe'
    assert 'stop not confirmed' in m.departure['outcome']


def test_failed_departure_navigation_stops_and_fails(manager_factory, monkeypatch):
    m, handle, _ = departing(manager_factory, monkeypatch)
    calls = len(m.navigation_client.calls)
    navigation_result(m.navigation_client, status=GoalStatus.STATUS_ABORTED, error=1)
    assert m._departure_stopping
    hold_still(m)
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'departure_failed'
    assert len(m.navigation_client.calls) == calls


def test_an_unconfirmed_stop_after_a_veto_fails_explicitly(manager_factory, monkeypatch):
    m, handle, _ = departing(manager_factory, monkeypatch)
    feed(m, [ahead(m, 0.6)])
    release(handle)
    m.tick()
    m._monotonic.value += m.motion_settlement_timeout_s + 0.1
    m.test_ros_ns += 100_000_000
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'stop_not_confirmed'


def test_no_free_departure_route_keeps_the_robot_stopped(manager_factory, monkeypatch):
    m, _ = parked(manager_factory, monkeypatch)
    sel = m.parking_plan.selected
    gx, gy, _ = preparking_pose(sel.base_x, sel.base_y, sel.yaw, 0.5)
    grid = m.latest_map
    col = int((gx - grid.info.origin.position.x) / grid.info.resolution)
    row = int((gy - grid.info.origin.position.y) / grid.info.resolution)
    data = list(grid.data)
    data[row * grid.info.width + col] = 100                  # mapped obstacle at the goal
    grid.data = data
    m._on_map(grid)
    end_wait(m)
    assert m.state == MissionState.MISSION_FAILED
    assert m.departure['outcome'].startswith('not started: route_footprint_occupied')
