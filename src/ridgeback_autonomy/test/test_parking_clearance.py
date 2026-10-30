"""Live LiDAR clearance during the parking legs: geometry and mission integration."""

import math

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from ridgeback_autonomy.mission.parking_clearance import (
    clearance_reason, remaining_route, scan_points, stopping_poses,
)
from ridgeback_autonomy.mission.parking_feasibility import driven_poses
from ridgeback_autonomy.mission.parking_geometry import (
    normalize_angle, preparking_pose, rotated_footprint_vertices,
)
from ridgeback_autonomy.mission.parking_planner import plan_parking
from ridgeback_autonomy.mission.parking_safety import ProtectionLimits
from test_mission_manager import manager_factory  # noqa: F401 (fixture)
from test_parking_feasibility import path
from test_parking_mission_navigation import (
    answer, feasible, hold_still, navigation_result, release)
from test_parking_planner import APPROACH_FROM_WEST, CONFIG, grid, target

G1 = (3.0, 3.0)


def parking():
    """A planned parking pose on a free map, its pre-parking pose and the driven route."""
    plan = plan_parking(grid(size_m=8., origin=(-1., -1.)), target(), CONFIG, APPROACH_FROM_WEST)
    sel = plan.selected
    pre = preparking_pose(sel.base_x, sel.base_y, sel.yaw, .5)
    start = preparking_pose(*pre, 1.)
    route = driven_poses(path([start, pre, (sel.base_x, sel.base_y, sel.yaw)]), sel, start)
    return plan, sel, pre, route


def base_points(robot, xy):
    """Map-frame points as base-frame arrays for ``robot``."""
    c, s = math.cos(robot[2]), math.sin(robot[2])
    pts = [(c * (x - robot[0]) + s * (y - robot[1]), -s * (x - robot[0]) + c * (y - robot[1]))
           for x, y in xy]
    return np.array([p[0] for p in pts]), np.array([p[1] for p in pts])


def g1_legs(radius=0.12, n=24):
    return [(G1[0] + radius * math.cos(2 * math.pi * k / n), G1[1] + radius * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def check(points, robot, velocity=(0., 0., 0.), route=(), plan=None):
    keepout = plan.keepout_radius_m if plan is not None else 0.29
    return clearance_reason(points, CONFIG.footprint, velocity, 0.05, robot, list(route), G1,
                            keepout - 0.03)


def test_final_approach_backs_straight_in_with_the_rear_toward_g1():
    plan, sel, pre, route = parking()
    final_leg = remaining_route(route, pre)
    assert final_leg[-1] == pytest.approx((sel.base_x, sel.base_y, sel.yaw))
    for x, y, yaw in final_leg:
        assert normalize_angle(yaw - sel.yaw) == pytest.approx(0.0, abs=1e-9)   # no rotation
        # On the pre-parking -> final axis, i.e. pure translation along base -x.
        off_axis = -math.sin(sel.yaw) * (x - pre[0]) + math.cos(sel.yaw) * (y - pre[1])
        assert off_axis == pytest.approx(0.0, abs=1e-9)
    travel = math.atan2(sel.base_y - pre[1], sel.base_x - pre[0])
    assert normalize_angle(travel - sel.yaw - math.pi) == pytest.approx(0.0, abs=1e-9)
    to_g1 = math.atan2(G1[1] - sel.base_y, G1[0] - sel.base_x)
    assert normalize_angle(to_g1 - sel.yaw - math.pi) == pytest.approx(0.0, abs=1e-9)


def test_g1_visible_at_the_planned_distance_does_not_stop_the_final_approach():
    plan, sel, pre, route = parking()
    for robot, v in ((pre, -0.15), ((sel.base_x, sel.base_y, sel.yaw), -0.03)):
        result = check(base_points(robot, g1_legs()), robot, (v, 0., 0.), route, plan)
        assert result.reason is None, robot
    assert result.g1_gap_m >= plan.keepout_radius_m - 1e-9
    assert result.nearest_m > 0.05   # G1's returns are seen, and held to the same margin


def test_entering_g1_protected_region_is_vetoed():
    plan, sel, pre, route = parking()
    robot = (sel.base_x, sel.base_y, sel.yaw)
    gap = check(base_points(robot, []), robot, plan=plan).g1_gap_m
    limit = plan.keepout_radius_m - 0.03   # keep-out less the final-pose tolerance
    c, s = math.cos(sel.yaw), math.sin(sel.yaw)
    for inward, expected in ((gap - limit - 0.005, None), (gap - limit + 0.005, 'g1_protected_region')):
        pose = (sel.base_x - inward * c, sel.base_y - inward * s, sel.yaw)
        assert check(base_points(pose, []), pose, plan=plan).reason == expected


def test_g1_returns_inside_the_stopping_path_are_vetoed_like_any_obstacle():
    plan, sel, pre, route = parking()
    robot = (sel.base_x, sel.base_y, sel.yaw)
    c, s = math.cos(sel.yaw), math.sin(sel.yaw)
    # A real leg closer than the estimate suggests: 3 cm behind the footprint.
    leg = (robot[0] - (0.497 + 0.03) * c, robot[1] - (0.497 + 0.03) * s)
    assert check(base_points(robot, [leg]), robot, (-0.05, 0., 0.), (), plan).reason == \
        'lidar_obstacle_in_stopping_path'


def test_obstacle_appearing_on_the_remaining_route_invalidates_the_maneuver():
    plan, sel, pre, route = parking()
    # Placed where the final footprint will be, beyond the robot's stopping distance.
    c, s = math.cos(sel.yaw), math.sin(sel.yaw)
    box = [(sel.base_x - 0.40 * c + dy * -s, sel.base_y - 0.40 * s + dy * c)
           for dy in np.linspace(-0.1, 0.1, 9)]
    result = check(base_points(pre, box), pre, (0., 0., 0.), route, plan)
    assert result.reason == 'lidar_obstacle_on_route'
    # The same box beside the corridor is not on the route: no unnecessary stop.
    beside = [(x - 0.9 * s, y + 0.9 * c) for x, y in box]
    assert check(base_points(pre, beside), pre, (0., 0., 0.), route, plan).reason is None


def test_stopping_path_follows_the_holonomic_velocity_not_the_heading():
    robot = (0.0, 0.0, 0.0)
    left = [(0.0, 0.413607 + 0.08)]                 # 8 cm beside the left side
    assert check(base_points(robot, left), robot, (0., 0.3, 0.)).reason == \
        'lidar_obstacle_in_stopping_path'           # strafing toward it
    assert check(base_points(robot, left), robot, (0., -0.3, 0.)).reason is None
    assert check(base_points(robot, left), robot, (0.3, 0., 0.)).reason is None
    poses = stopping_poses((0., 0.3, 0.), 0.05, ProtectionLimits())
    assert poses[-1][0] == pytest.approx(0.0) and poses[-1][1] > 0.1


def test_rear_lidar_returns_are_placed_behind_the_robot():
    scan = LaserScan(angle_min=-0.1, angle_increment=0.1, range_min=0.05, range_max=10.0,
                     ranges=[float('inf'), 0.5, float('nan')])
    x, y = scan_points(scan, (-0.3922, 0.0, math.pi))
    assert (x[0], y[0]) == pytest.approx((-0.8922, 0.0), abs=1e-9)
    assert x.size == 1   # inf and nan are no returns


# Mission integration: the active parking legs.

def scan_msg(m, points=(), stamp_ns=None):
    """A 360-degree LaserScan in the base frame with returns at base-frame ``points``."""
    n = 720
    ranges = [float('inf')] * n
    for x, y in points:
        k = int(round((math.atan2(y, x) + math.pi) / (2 * math.pi / n))) % n
        ranges[k] = math.hypot(x, y)
    msg = LaserScan(angle_min=-math.pi, angle_increment=2 * math.pi / n, range_min=0.05,
                    range_max=20.0, ranges=ranges)
    msg.header.frame_id = 'laser'
    stamp_ns = m.test_ros_ns if stamp_ns is None else stamp_ns
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    return msg


def feed(m, points=(), stamp_ns=None):
    m._laser_in_base['laser'] = (0.0, 0.0, 0.0)
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(m.test_ros_ns, 1_000_000_000)
    m._on_odometry(odom)
    # Both LiDARs stream continuously; the newest scan triggers the check.
    *others, last = m.parking_lidar_topics
    for topic in others:
        m._scans[topic] = (scan_msg(m, points, stamp_ns), m._monotonic())
    m._on_scan(last, scan_msg(m, points, stamp_ns))


def driving(manager_factory, monkeypatch):
    m, published = feasible(manager_factory, monkeypatch)
    answer(m._goal_checker_set_client)
    _, accepted, handle = m.navigation_client.calls[-1]
    accepted.set_result(handle)
    feed(m)
    m.tick()
    assert m.state == MissionState.NAVIGATING_TO_PREPARKING   # clear scans: keep driving
    return m, handle, published


def on_route(m):
    """A base-frame point on the validated route, beyond the stopping distance."""
    route = driven_poses(m.parking_plan.selected_path, m.parking_plan.selected, (1.0, 2.0, 0.25))
    x, y, _ = route[len(route) // 2]
    return base_points((1.0, 2.0, 0.25), [(x, y)])


def test_obstacle_on_route_cancels_the_leg_and_returns_only_after_a_confirmed_stop(
        manager_factory, monkeypatch):
    m, handle, published = driving(manager_factory, monkeypatch)
    px, py = on_route(m)
    feed(m, [(px[0], py[0])])
    assert handle.cancels == 1
    assert m.state == MissionState.RETURNING_TO_START
    assert 'lidar_obstacle_on_route' in m._parking_failure
    calls = len(m.navigation_client.calls)
    hold_still(m)
    m.tick()
    # A stop is no proof Nav2 released the goal: no return, not even its goal-checker change.
    assert len(m.navigation_client.calls) == calls and len(m._goal_checker_set_client.calls) == 1
    release(handle)
    m.tick()
    assert len(m.navigation_client.calls) == calls    # not resumed, no return before the stop
    hold_still(m)
    m.tick()
    answer(m._goal_checker_set_client)
    assert navigation_result(m.navigation_client) == pytest.approx((1.0, 2.0, 0.25))
    assert m.state == MissionState.MISSION_FAILED     # controlled failure, reported
    assert m.last_issue.code == 'parking_failed'


@pytest.mark.parametrize('problem,reason', [('missing', 'lidar_unavailable'),
                                            ('stale', 'lidar_stale')])
def test_missing_or_stale_lidar_cancels_the_leg(manager_factory, monkeypatch, problem, reason):
    m, handle, _ = driving(manager_factory, monkeypatch)
    if problem == 'missing':
        m._scans.clear()
    else:
        m.test_ros_ns += 2_000_000_000            # no scan for 2 s of ROS time
    m.tick()
    assert handle.cancels == 1 and reason in m._parking_failure


def test_unconfirmed_stop_after_a_veto_fails_without_driving(manager_factory, monkeypatch):
    m, handle, _ = driving(manager_factory, monkeypatch)
    m._scans.clear()
    m.tick()
    release(handle)
    calls = len(m.navigation_client.calls)
    m._monotonic.value += m.motion_settlement_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'stop_not_confirmed'
    assert len(m.navigation_client.calls) == calls


def test_clearance_is_published_with_the_mission_state(manager_factory, monkeypatch):
    import json
    m, handle, published = driving(manager_factory, monkeypatch)
    px, py = on_route(m)
    feed(m, [(px[0], py[0])])
    payload = json.loads(published['state'][-1].data)
    assert payload['parking_clearance']['reason'] == 'lidar_obstacle_on_route'
    leg = payload['parking_clearance']['legs']['navigating_to_preparking']
    assert leg['checks'] >= 2 and leg['min_nearest_m'] is not None   # it ran on every scan


def dense_selected_route(m, step=0.05):
    """The selected path with NavFn's dense placeholder poses, as Nav2 returns it."""
    from geometry_msgs.msg import PoseStamped
    poses = m.parking_plan.selected_path.poses
    (x0, y0), goal = (poses[0].pose.position.x, poses[0].pose.position.y), poses[-1]
    n = int(math.hypot(goal.pose.position.x - x0, goal.pose.position.y - y0) / step)
    dense = []
    for k in range(n):
        p = PoseStamped()
        p.header = goal.header
        p.pose.position.x = x0 + (goal.pose.position.x - x0) * k / n
        p.pose.position.y = y0 + (goal.pose.position.y - y0) * k / n
        p.pose.orientation.w = 1.0
        dense.append(p)
    m.parking_plan.selected_path.poses = dense + list(poses[1:])
    return driven_poses(m.parking_plan.selected_path, m.parking_plan.selected, (1.0, 2.0, 0.25))


# Recorded 2026-10-30: 2 m into the leg the monitor swept the robot back to the route's
# start, across a real person the route had cleared, because the route it was given
# began with the robot's own pose and so was "nearest" there.
def test_only_the_route_ahead_of_the_robot_is_checked(manager_factory, monkeypatch):
    m, handle, _ = driving(manager_factory, monkeypatch)
    route = dense_selected_route(m)
    ahead = route[len(route) // 2]
    m.tf_buffer.x, m.tf_buffer.y = ahead[:2]               # the robot followed the route
    robot = (m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw)
    from ridgeback_autonomy.mission.parking_planner import points_to_polygon_distance
    here = rotated_footprint_vertices(m.planner_config.footprint, *robot)
    left_behind, final = route[0][:2], route[-1][:2]    # where it started; where it will park
    for point in (left_behind, final):                  # neither is in the stopping path
        assert points_to_polygon_distance([point[0]], [point[1]], here)[0] > 0.2
    feed(m, [tuple(zip(*base_points(robot, [left_behind])))[0]])
    assert handle.cancels == 0 and m.state == MissionState.NAVIGATING_TO_PREPARKING
    feed(m, [tuple(zip(*base_points(robot, [final])))[0]])
    assert handle.cancels == 1 and 'lidar_obstacle_on_route' in m._parking_failure
