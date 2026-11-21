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


# The corridor: Nav2's default tree may replan or recover; the departure ends if the robot
# leaves its checked straight route by more than the goal's own tolerance.

def off_route(m, along, aside_away_from_g1=0.0, turn=0.0):
    """Move the robot ``along`` its heading and sideways away from G1, turned by ``turn``."""
    b, t = m.tf_buffer, m.target_position
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    gy = -s * (t.map_x_m - b.x) + c * (t.map_y_m - b.y)
    side = -math.copysign(aside_away_from_g1, gy)
    b.x, b.y = b.x + c * along - s * side, b.y + s * along + c * side
    b.yaw += turn


@pytest.mark.parametrize('aside,turn,vetoed', [(0.05, 0.0, False), (0.10, 0.0, True), (0.0, 0.15, True)])
def test_leaving_the_checked_route_ends_the_departure(manager_factory, monkeypatch, aside, turn, vetoed):
    m, handle, _ = departing(manager_factory, monkeypatch)
    off_route(m, 0.2, aside, turn)
    feed(m)
    assert handle.cancels == int(vetoed)
    if vetoed:
        assert m.departure['outcome'].startswith('ended early: route_deviation')
        assert m._departure_stopping


# A failed final leg leaves the robot near G1: the same departure runs once it has stopped.

def final_leg_failed(manager_factory, monkeypatch, attempts):
    """Vetoed on the final leg 0.1 m short of the parking pose."""
    from test_parking_mission_navigation import aim_camera
    m, published = feasible(manager_factory, monkeypatch, attempts=attempts)
    aim_camera(m)
    sel, params, nav = m.parking_plan.selected, m._goal_checker_set_client, m.navigation_client
    answer(params)
    navigation_result(nav)                                   # pre-parking pose reached
    assert m.state == MissionState.NAVIGATING_TO_PARKING
    answer(params)
    _, accepted, handle = nav.calls[-1]
    accepted.set_result(handle)
    pre = preparking_pose(sel.base_x, sel.base_y, sel.yaw, 0.5)
    m.tf_buffer.x, m.tf_buffer.y = sel.base_x + 0.2 * (pre[0] - sel.base_x), sel.base_y + 0.2 * (pre[1] - sel.base_y)
    m.tf_buffer.yaw = sel.yaw
    m._parking_navigation_failed('lidar_clearance: lidar_obstacle_in_stopping_path', stop_first=True)
    assert handle.cancels == 1 and m._depart_after_stop
    return m, handle, pre, published


def stopped_near_g1(m, handle, points=()):
    release(handle)
    hold_still(m)
    fresh(m, points)
    m.tick()


def depart(m, pre):
    assert answer(m._goal_checker_set_client) == {'general_goal_checker.xy_goal_tolerance': 0.08,
                                                  'general_goal_checker.yaw_goal_tolerance': 0.12}
    feed(m)
    assert navigation_result(m.navigation_client) == pytest.approx(pre)
    assert m.departure['outcome'] == 'reached'


def test_a_failed_final_leg_departs_before_the_retry(manager_factory, monkeypatch):
    m, handle, pre, _ = final_leg_failed(manager_factory, monkeypatch, attempts=3)
    assert m.state == MissionState.RETURNING_TO_OBSERVATION
    stopped_near_g1(m, handle)
    assert m._departure_route is not None and m.state == MissionState.RETURNING_TO_OBSERVATION
    depart(m, pre)
    answer(m._goal_checker_set_client)                       # the observation tolerance
    assert navigation_result(m.navigation_client) == pytest.approx(m._observation_goal[:3])


def test_a_failed_last_final_leg_departs_before_the_return(manager_factory, monkeypatch):
    m, handle, pre, _ = final_leg_failed(manager_factory, monkeypatch, attempts=1)
    assert m.state == MissionState.RETURNING_TO_START
    stopped_near_g1(m, handle)
    depart(m, pre)
    from test_parking_mission_navigation import guarded_return
    guarded_return(m)                                        # stopped again, outside G1's region
    answer(m._goal_checker_set_client)                       # the shared checker
    assert navigation_result(m.navigation_client) == pytest.approx((1.0, 2.0, 0.25))
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'parking_failed'


def test_a_failed_final_leg_with_no_safe_departure_stays_stopped(manager_factory, monkeypatch):
    m, handle, _, _ = final_leg_failed(manager_factory, monkeypatch, attempts=3)
    calls = len(m.navigation_client.calls)
    stopped_near_g1(m, handle, [ahead(m)])
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'departure_not_safe'
    assert len(m.navigation_client.calls) == calls           # nothing was driven
