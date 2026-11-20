"""The route revalidation cache: reused only while every input the sweep reads is unchanged.

SLAM rewrites the whole map every 0.5 s while the robot stands still; a whole-map key swept
a 5 m route on every tick (0.29 s), which starved the odometry callbacks (controlled
Gazebo run, 2026-11-20). Only the cells in ``route_window`` may now sweep again, and the
cached and uncached decisions must agree."""

from dataclasses import replace
import math
import random

import numpy as np
import pytest

from ridgeback_autonomy.mission.parking_feasibility import (
    ROUTE_REUSE_POSE_M, RouteCheckCache, path_rejection, route_window,
)
from ridgeback_autonomy.mission.parking_planner import PlannerConfig
from test_parking_feasibility import path, path_fixture
from test_parking_planner import CONFIG, target


class Counted:
    def __init__(self):
        self.calls = 0

    def __call__(self, *a, **kw):
        self.calls += 1
        return path_rejection(*a, **kw)


def checked(cache, sweep, g, c, start, p, *, t=None, config=CONFIG, keepout=.29, occupied=()):
    result = cache.check(p, c, start, g, t or target(), config, keepout, occupied=occupied, sweep=sweep)
    assert result == path_rejection(p, c, start, g, t or target(), config, keepout, occupied=occupied)
    return result


def window_mask(windows, shape):
    mask = np.zeros(shape, bool)
    for r0, r1, c0, c1 in windows:
        mask[r0:r1 + 1, c0:c1 + 1] = True
    return mask


def setup():
    g, c, start, p = path_fixture()
    return RouteCheckCache(), Counted(), g, c, start, p


def test_an_unchanged_route_is_swept_once():
    cache, sweep, g, c, start, p = setup()
    for _ in range(5):
        assert checked(cache, sweep, g, c, start, p) is None
    assert sweep.calls == 1


def test_changes_outside_the_route_cells_reuse_and_changes_inside_sweep_again():
    cache, sweep, g, c, start, p = setup()
    checked(cache, sweep, g, c, start, p)
    inside = window_mask(route_window(p, c, start, g, CONFIG), g.data.shape)
    assert not inside[0, 0]
    g.data[0, 0] = 100                                   # SLAM noise far away
    g.data[~inside & (np.arange(g.height)[:, None] > 100)] = -1
    assert checked(cache, sweep, g, c, start, p) is None and sweep.calls == 1
    g.data[84, 40] = 100                                 # a new obstacle on the route
    assert checked(cache, sweep, g, c, start, p) == 'path_footprint_occupied' and sweep.calls == 2
    g.data[84, 40] = -1                                  # now unknown there
    assert checked(cache, sweep, g, c, start, p) == 'path_footprint_unknown' and sweep.calls == 3
    g.data[84, 40] = 0
    assert checked(cache, sweep, g, c, start, p) is None and sweep.calls == 4


@pytest.mark.parametrize('change', ['origin', 'yaw', 'resolution', 'size', 'frame'])
def test_any_grid_geometry_change_sweeps_again(change):
    cache, sweep, g, c, start, p = setup()
    checked(cache, sweep, g, c, start, p)
    moved = {'origin': lambda: replace(g, origin_x=g.origin_x + 0.05),
             'yaw': lambda: replace(g, origin_yaw=0.001),
             'resolution': lambda: replace(g, resolution=0.051),
             'size': lambda: replace(g, width=g.width - 1, data=g.data[:, :-1]),
             'frame': lambda: replace(g, frame_id='odom')}[change]()
    checked(cache, sweep, moved, c, start, p)
    assert sweep.calls == 2


def test_route_target_keepout_footprint_and_occupied_changes_sweep_again():
    cache, sweep, g, c, start, p = setup()
    checked(cache, sweep, g, c, start, p)
    p.poses[-1].pose.position.y += 0.01                   # the same Path object, changed in place
    checked(cache, sweep, g, c, start, p)
    checked(cache, sweep, g, c, start, path([start, (3., 4.1, 0.)]))   # another route
    checked(cache, sweep, g, c, start, p, t=target(3.0, 3.1))
    checked(cache, sweep, g, c, start, p, t=target(3.0, 3.1), keepout=.30)
    bigger = PlannerConfig(footprint=tuple((x * 1.01, y * 1.01) for x, y in CONFIG.footprint))
    checked(cache, sweep, g, c, start, p, t=target(3.0, 3.1), keepout=.30, config=bigger)
    checked(cache, sweep, g, c, start, p, t=target(3.0, 3.1), keepout=.30, config=bigger,
            occupied=((1.0, 4.2, 0.0),))
    assert sweep.calls == 7


def test_robot_pose_reuse_is_bounded_from_the_swept_pose_and_never_chained():
    cache, sweep, g, c, start, p = setup()
    checked(cache, sweep, g, c, start, p)
    near = (start[0] + 0.8 * ROUTE_REUSE_POSE_M, start[1], start[2] + 0.0004)
    checked(cache, sweep, g, c, near, p)
    assert sweep.calls == 1
    farther = (start[0] + 1.6 * ROUTE_REUSE_POSE_M, start[1], start[2])   # 0.8 mm from `near`
    checked(cache, sweep, g, c, farther, p)
    assert sweep.calls == 2
    checked(cache, sweep, g, c, (farther[0], farther[1], farther[2] + 0.001), p)
    assert sweep.calls == 3


def test_an_unbounded_route_is_never_reused():
    cache, sweep, g, c, start, p = setup()
    p.poses[1].pose.position.x = float('nan')
    for _ in range(2):
        checked(cache, sweep, g, c, start, p)
    assert sweep.calls == 2


@pytest.mark.parametrize('yaw', [0.0, 0.3])
def test_cells_outside_the_window_never_change_a_decision(yaw):
    """The window is a bound on what the sweep reads: scrambling everything outside it
    leaves every decision unchanged, on rotated grids too."""
    rng = random.Random(3)
    for trial in range(60):
        g, c, start, p = path_fixture()
        if yaw:
            g = replace(g, origin_x=-1.5, origin_y=0.5, origin_yaw=yaw, data=g.data.copy())
        for _ in range(rng.randint(0, 6)):
            g.data[rng.randrange(g.height), rng.randrange(g.width)] = rng.choice([100, -1, 50])
        window = route_window(p, c, start, g, CONFIG)
        assert window is not None
        before = path_rejection(p, c, start, g, target(), CONFIG, .29)
        scrambled = g.data.copy()
        outside = ~window_mask(window, scrambled.shape)
        scrambled[outside] = np.random.default_rng(trial).choice([-1, 0, 100], outside.sum())
        assert path_rejection(p, c, start, replace(g, data=scrambled), target(), CONFIG, .29) == before


def test_random_edit_sequences_agree_with_uncached_sweeps():
    rng = random.Random(11)
    cache, sweep, g, c, start, p = setup()
    for _ in range(400):
        op = rng.random()
        if op < 0.6:
            g.data[rng.randrange(g.height), rng.randrange(g.width)] = rng.choice([0, 0, 100, -1])
        elif op < 0.8:
            start = (start[0] + rng.uniform(-6e-4, 6e-4), start[1] + rng.uniform(-6e-4, 6e-4),
                     start[2] + rng.uniform(-6e-4, 6e-4))
        checked(cache, sweep, g, c, start, p)          # asserts agreement every time
    assert sweep.calls < 400


@pytest.mark.parametrize('yaw', [0.0, 0.7])
def test_a_long_curved_route_never_reads_outside_its_windows(yaw):
    """Many overlapping windows: identity (placeholder) headings and real turns along an
    arc; scrambling every cell outside them never changes a decision."""
    from geometry_msgs.msg import Quaternion
    from ridgeback_autonomy.mission.parking_planner import CandidateDecision
    from test_parking_planner import grid
    rng = random.Random(5)
    for trial in range(25):
        g = grid(size_m=8.)
        if yaw:
            g = replace(g, origin_x=-1.0, origin_y=-1.0, origin_yaw=yaw)
        pts = [(1.5 + 4.5 * k / 40, 4.0 + 1.2 * math.sin(k / 40 * math.pi)) for k in range(41)]
        p = path([(x, y, 0.0) for x, y in pts])
        for i, pose in enumerate(p.poses[1:-1], 1):
            if i % 9:                                       # NavFn placeholder orientation
                pose.pose.orientation = Quaternion()
            else:
                a = rng.uniform(-1.0, 1.0)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        start = (pts[0][0] + 0.03, pts[0][1], 0.2)
        c = CandidateDecision(pts[-1][0], pts[-1][1], pts[-1][0], pts[-1][1], 0., 'valid', 'footprint_free', .6)
        for _ in range(rng.randint(0, 10)):
            g.data[rng.randrange(g.height), rng.randrange(g.width)] = rng.choice([100, -1])
        windows = route_window(p, c, start, g, CONFIG, occupied=((1.0, 1.0, 0.0),))
        assert windows is not None and len(windows) > 3
        before = path_rejection(p, c, start, g, target(7., 7.), CONFIG, .29, occupied=((1.0, 1.0, 0.0),))
        outside = ~window_mask(windows, g.data.shape)
        scrambled = g.data.copy()
        scrambled[outside] = np.random.default_rng(trial).choice([-1, 0, 100], outside.sum())
        assert path_rejection(p, c, start, replace(g, data=scrambled), target(7., 7.), CONFIG, .29,
                              occupied=((1.0, 1.0, 0.0),)) == before
