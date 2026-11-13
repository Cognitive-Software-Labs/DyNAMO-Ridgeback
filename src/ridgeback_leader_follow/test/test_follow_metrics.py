import csv
import json
import math

import pytest

from ridgeback_leader_follow.follow_metrics import aggregate, label, odom_to_world, run_metrics


def write_run(tmp_path, robot_path, leader_path, bystander, lock_points, odom_offset=(0., 0., 0.),
              monitor=(), status='following:leader_locked', complete=True):
    """Synthetic run: 0.1 s truth samples from sim time 2.0, lock messages every 0.1 s."""
    run = tmp_path / 'run'
    run.mkdir()
    steps = len(robot_path)
    (run / 'manifest.json').write_text(json.dumps({'backend': 'gz', 'status': 'complete' if complete else 'failed',
                                                   'crowd_size': 4, 'scenario': {'name': 'crossing'}}))
    with open(run / 'robot_truth.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['stamp', 'scenario_time', 'x', 'y', 'yaw', 'cmd_vx', 'cmd_vy', 'cmd_wz'])
        for k, (x, y) in enumerate(robot_path):
            w.writerow([2 + k * .1, k * .1, x, y, 0., 0., 0., 0.])
    with open(run / 'ground_truth.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sim_time', 'scenario_time', 'command_time', 'person', 'role', 'frame', 'x', 'y', 'z', 'yaw',
                    'command_x', 'command_y', 'command_yaw'])
        for k in range(steps):
            for name, role, path in (('lf_leader', 'leader', leader_path), ('lf_passer', 'bystander', bystander)):
                w.writerow([2 + k * .1, k * .1, k * .1, name, role, 'world', *path[k], 0., 0., *path[k], 0.])
    ox, oy, oyaw = odom_offset
    with open(tmp_path / 'run.events.jsonl', 'w') as f:
        f.write(json.dumps({'t': 1.9, 'kind': 'status', 'status': status}) + '\n')
        for k, point in enumerate(lock_points):
            t = 2 + k * .1
            rx, ry = robot_path[k]
            # Odometry = world shifted by the offset (drift), applied to robot and target alike.
            f.write(json.dumps({'t': t, 'kind': 'odom', 'stamp': t, 'x': rx + ox, 'y': ry + oy, 'yaw': oyaw}) + '\n')
            entry = {'t': t, 'kind': 'leader', 'stamp': t, 'state': 1 if point else 2, 'reason': '', 'track_id': 1,
                     'has_leader': point is not None, 'x': 0., 'y': 0., 'frame': 'odom'}
            if point:
                entry['x'], entry['y'] = point[0] + ox, point[1] + oy
            f.write(json.dumps(entry) + '\n')
        for t, action in monitor:
            f.write(json.dumps({'t': t, 'kind': 'monitor', 'action': action, 'polygon': 'StopZone'}) + '\n')
    return run


def test_label_needs_one_clear_nearest_person():
    people = {'a': ('leader', 0., 0.), 'b': ('bystander', 3., 0.)}
    assert label(people, .2, 0., .75) == 'a'
    assert label(people, 1.5, 0., .75) is None
    assert label({'a': ('leader', 0., 0.), 'b': ('bystander', .05, 0.)}, 0., 0., .75) is None


def test_odom_to_world_keeps_the_target_relative_to_the_robot():
    x, y = odom_to_world((2., 1.), (1., 1., 0.), (5., 5., math.pi / 2))
    assert (x, y) == pytest.approx((5., 6.))


def test_following_the_leader_counts_as_success(tmp_path):
    n = 40
    robot = [(k * .05, 0.) for k in range(n)]
    leader = [(1.2 + k * .05, 0.) for k in range(n)]
    passer = [(3., 5.)] * n
    run = write_run(tmp_path, robot, leader, passer, leader, odom_offset=(.4, -.3, 0.), monitor=[(2.5, 1), (2.6, 0)])
    m = run_metrics(run)
    assert m['success'] and m['wrong_follow_s'] == 0 and m['id_switches'] == 0
    assert m['correct_lock_fraction'] == pytest.approx(1., abs=.05)
    assert m['distance_error_median_m'] == pytest.approx(0., abs=1e-6)
    assert m['bystander_clearance_min_m'] == pytest.approx(math.hypot(3. - 1.95, 5.), abs=.01)
    assert m['safety_stops'] == 1


def test_lock_moving_to_a_bystander_is_wrong_follow_and_a_switch(tmp_path):
    n = 40
    robot = [(0., 0.)] * n
    leader = [(1.2, 0.)] * n
    passer = [(2.5, .5)] * n
    locks = [leader[k] if k < 10 else (None if k < 15 else passer[k]) for k in range(n)]
    m = run_metrics(write_run(tmp_path, robot, leader, passer, locks))
    assert m['wrong_follow_s'] == pytest.approx(2.5, abs=.11)
    assert m['id_switches'] == 1 and not m['success']
    assert m['losses'] == 1 and m['unrecovered_loss_s'] > 2


def test_recovery_time_after_a_gap(tmp_path):
    n = 40
    robot = [(0., 0.)] * n
    leader = [(1.2, 0.)] * n
    locks = [None if 10 <= k < 18 else leader[k] for k in range(n)]
    m = run_metrics(write_run(tmp_path, robot, leader, [(9., 9.)] * n, locks))
    assert m['losses'] == 1 and m['recovery_median_s'] == pytest.approx(.8, abs=1e-6)
    assert m['success']


def test_aggregate_reports_median_range_and_success_rate():
    rows = [dict(backend='gz', scenario='crowd', crowd_size=4, success=s, correct_lock_fraction=c, wrong_follow_s=0.,
                 id_switches=0, recovery_median_s=None, distance_error_median_m=.5, bystander_clearance_min_m=1.,
                 safety_stops=0) for s, c in ((True, .9), (False, .5), (True, .8))]
    table = aggregate(rows)['gz/crowd-4']
    assert table['runs'] == 3 and table['success_rate'] == pytest.approx(.67)
    assert table['correct_lock_fraction'] == {'median': .8, 'min': .5, 'max': .9}
    assert table['recovery_median_s'] is None
