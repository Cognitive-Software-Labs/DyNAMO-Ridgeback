#!/usr/bin/env python3
"""F4 following metrics from simulator truth and the follow recorder.

Per run (a crowd_runner ``--drive`` output directory plus the
``follow_recorder`` events beside it, ``<run>.events.jsonl``):

* correct-lock, wrong-lock, unmatched-lock and unlocked time;
* wrong-follow time: locked on a bystander while the controller was not stopped;
* ID switches: changes of the simulated person the lock points at;
* recovery time after each loss of the leader, and whether the last loss recovered;
* distance-keeping error |robot-leader centre distance - follow distance| once
  the leader is first locked, the closest leader approach and final distance;
* minimum robot-centre clearance to any bystander;
* safety stops: collision-monitor STOP actions;
* success: the scene completed, no wrong-follow time, locked on the leader at
  the end and within ``success_distance`` of them.

Lock targets are published in the odometry frame. Each is mapped into the
world through the recorded odometry pose and the simulator's robot pose at
the same time, so odometry drift does not mislabel targets. A target labels
the nearest simulated person within ``label_radius``; a second person within
0.1 m of that distance makes it ``unmatched``.

``follow_metrics RUN [RUN ...] [--json FILE]`` prints one row per run and,
for repeated configurations, the median and range of every metric.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

LOCKED = 1
STOP_ACTION = 1


def _nearest(times: Sequence[float], t: float) -> int:
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return 0
    if i >= len(times):
        return len(times) - 1
    return i if times[i] - t < t - times[i - 1] else i - 1


class Truth:
    """Simulator truth: every person and the robot, indexed by simulation time."""

    def __init__(self, run: Path) -> None:
        people: Dict[float, Dict[str, Tuple[str, float, float]]] = defaultdict(dict)
        scenario_of: Dict[float, float] = {}
        for row in csv.DictReader(open(run / 'ground_truth.csv')):
            t = float(row['sim_time'])
            people[t][row['person']] = (row['role'], float(row['x']), float(row['y']))
            scenario_of[t] = float(row['scenario_time'])
        self.times = sorted(people)
        self.people = [people[t] for t in self.times]
        self.scenario = [scenario_of[t] for t in self.times]
        robot = [(float(r['stamp']), float(r['scenario_time']), float(r['x']), float(r['y']), float(r['yaw']))
                 for r in csv.DictReader(open(run / 'robot_truth.csv'))]
        self.robot_times = [r[0] for r in robot]
        self.robot = robot
        self.robot_by_scenario = {round(r[1], 3): r for r in robot}

    def people_at(self, t: float) -> Dict[str, Tuple[str, float, float]]:
        return self.people[_nearest(self.times, t)]

    def robot_at(self, t: float) -> Tuple[float, float, float]:
        r = self.robot[_nearest(self.robot_times, t)]
        return r[2], r[3], r[4]


def label(people: Dict[str, Tuple[str, float, float]], x: float, y: float, radius: float) -> Optional[str]:
    """Name of the simulated person at (x, y), or None when nobody or ambiguous."""
    ranked = sorted((math.hypot(px - x, py - y), name) for name, (_, px, py) in people.items())
    if not ranked or ranked[0][0] > radius or (len(ranked) > 1 and ranked[1][0] - ranked[0][0] < .1):
        return None
    return ranked[0][1]


def odom_to_world(point, odom_pose, world_pose):
    """Carry a point fixed relative to the robot from the odometry frame to the world."""
    ox, oy, oyaw = odom_pose
    wx, wy, wyaw = world_pose
    dx, dy = point[0] - ox, point[1] - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    bx, by = c * dx - s * dy, s * dx + c * dy
    c, s = math.cos(wyaw), math.sin(wyaw)
    return wx + c * bx - s * by, wy + s * bx + c * by


def run_metrics(run: Path, events: Optional[Path] = None, follow_distance: float = 1.2,
                label_radius: float = .75, success_distance: float = 3.) -> dict:
    run = Path(run)
    events = Path(events) if events else run.parent / (run.name + '.events.jsonl')
    manifest = json.loads((run / 'manifest.json').read_text())
    truth = Truth(run)
    records = [json.loads(line) for line in open(events)]
    start, end = truth.times[0], truth.times[-1]
    odom = sorted((e['stamp'], e['x'], e['y'], e['yaw']) for e in records if e.get('kind') == 'odom')
    odom_times = [o[0] for o in odom]
    statuses = sorted((e['t'], e['status'].split(':')[0]) for e in records if e.get('kind') == 'status')
    status_times = [s[0] for s in statuses]
    leader_name = next(name for name, (role, _, _) in truth.people[0].items() if role == 'leader')

    def mode_at(t):
        i = bisect.bisect_right(status_times, t) - 1
        return statuses[i][1] if i >= 0 else 'stopped'

    def target_name(e):
        point = (e['x'], e['y'])
        if odom:
            o = odom[_nearest(odom_times, e['stamp'])]
            point = odom_to_world(point, o[1:], truth.robot_at(e['stamp']))
        return label(truth.people_at(e['stamp']), *point, label_radius)

    leader_states = sorted((e for e in records if e.get('kind') == 'leader' and start <= e['stamp'] <= end),
                           key=lambda e: e['stamp'])
    time = dict(correct=0., wrong=0., unmatched=0., unlocked=0., wrong_follow=0.)
    on_leader: List[Tuple[float, bool]] = []
    identities: List[str] = []
    for a, b in zip(leader_states, leader_states[1:] + [None]):
        dt = min((b['stamp'] if b else end) - a['stamp'], .5)
        if a['state'] == LOCKED and a['has_leader']:
            name = target_name(a)
            if name is None:
                time['unmatched'] += dt
            elif name == leader_name:
                time['correct'] += dt
            else:
                time['wrong'] += dt
                if mode_at(a['t']) != 'stopped':
                    time['wrong_follow'] += dt
            if name is not None and (not identities or identities[-1] != name):
                identities.append(name)
            on_leader.append((a['stamp'], name == leader_name))
        else:
            time['unlocked'] += dt
            on_leader.append((a['stamp'], False))

    recoveries, lost_at, first_lock = [], None, None
    for t, ok in on_leader:
        if ok and first_lock is None:
            first_lock = t
        if first_lock is None:
            continue
        if not ok and lost_at is None:
            lost_at = t
        elif ok and lost_at is not None:
            recoveries.append(t - lost_at)
            lost_at = None
    end_window = [ok for t, ok in on_leader if t >= end - 1.]
    locked_at_end = bool(end_window) and sum(end_window) > len(end_window) / 2

    distances, clearance, keeping = [], math.inf, []
    for i, t in enumerate(truth.times):
        robot = truth.robot_by_scenario.get(round(truth.scenario[i], 3))
        if robot is None:
            continue
        for name, (_, px, py) in truth.people[i].items():
            d = math.hypot(px - robot[2], py - robot[3])
            if name == leader_name:
                distances.append(d)
                if first_lock is not None and t >= first_lock:
                    keeping.append(abs(d - follow_distance))
            else:
                clearance = min(clearance, d)
    keeping.sort()
    stops = sum(1 for e in records if e.get('kind') == 'monitor' and e['action'] == STOP_ACTION)
    complete = manifest.get('status') == 'complete'
    final = distances[-1] if distances else math.nan
    result = {
        'run': run.name, 'backend': manifest.get('backend'), 'scenario': manifest['scenario']['name'],
        'crowd_size': manifest.get('crowd_size'), 'complete': complete, 'duration_s': round(end - start, 2),
        **{f'{k}_lock_s' if k not in ('unlocked', 'wrong_follow') else f'{k}_s': round(v, 2) for k, v in time.items()},
        'correct_lock_fraction': round(time['correct'] / max(end - start, 1e-9), 3),
        'id_switches': max(0, len(identities) - 1),
        'losses': len(recoveries) + (lost_at is not None),
        'recovery_median_s': round(statistics.median(recoveries), 2) if recoveries else None,
        'recovery_max_s': round(max(recoveries), 2) if recoveries else None,
        'unrecovered_loss_s': round(end - lost_at, 2) if lost_at is not None else 0.,
        'distance_error_median_m': round(statistics.median(keeping), 2) if keeping else None,
        'distance_error_p90_m': round(keeping[int(.9 * (len(keeping) - 1))], 2) if keeping else None,
        'leader_distance_min_m': round(min(distances), 2) if distances else None,
        'final_distance_m': round(final, 2),
        'bystander_clearance_min_m': round(clearance, 2) if math.isfinite(clearance) else None,
        'safety_stops': stops,
    }
    result['success'] = bool(complete and time['wrong_follow'] == 0 and locked_at_end and final <= success_distance)
    return result


SUMMARY = ('correct_lock_fraction', 'wrong_follow_s', 'id_switches', 'recovery_median_s', 'distance_error_median_m',
           'bystander_clearance_min_m', 'safety_stops')


def configuration(result: dict) -> str:
    size = f"-{result['crowd_size']}" if result['scenario'] == 'crowd' else ''
    return f"{result['backend']}/{result['scenario']}{size}"


def aggregate(results: List[dict]) -> Dict[str, dict]:
    """Median and range of each metric, and the success rate, per configuration."""
    groups: Dict[str, List[dict]] = defaultdict(list)
    for r in results:
        groups[configuration(r)].append(r)
    table = {}
    for key, rows in sorted(groups.items()):
        entry = {'runs': len(rows), 'success_rate': round(sum(r['success'] for r in rows) / len(rows), 2)}
        for metric in SUMMARY:
            values = [r[metric] for r in rows if r[metric] is not None]
            entry[metric] = ({'median': round(statistics.median(values), 3), 'min': min(values), 'max': max(values)}
                             if values else None)
        table[key] = entry
    return table


def _cell(value) -> str:
    if value is None:
        return '-'
    if isinstance(value, dict):
        return f"{value['median']:g} [{value['min']:g}–{value['max']:g}]"
    return f'{value:g}' if isinstance(value, float) else str(value)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--json', type=Path, help='write per-run metrics and the aggregate here')
    parser.add_argument('--follow-distance', type=float, default=1.2)
    args = parser.parse_args(argv)
    results, failures = [], []
    for run in args.runs:
        try:
            results.append(run_metrics(run, follow_distance=args.follow_distance))
        except (OSError, KeyError, ValueError, StopIteration) as error:
            failures.append((str(run), repr(error)))
    columns = ('run', 'success', 'correct_lock_fraction', 'wrong_follow_s', 'id_switches', 'losses',
               'recovery_median_s', 'unrecovered_loss_s', 'distance_error_median_m', 'leader_distance_min_m',
               'final_distance_m', 'bystander_clearance_min_m', 'safety_stops')
    print('| ' + ' | '.join(columns) + ' |')
    print('|' + '---|' * len(columns))
    for r in results:
        print('| ' + ' | '.join(_cell(r[c]) for c in columns) + ' |')
    table = aggregate(results)
    if any(entry['runs'] > 1 for entry in table.values()):
        head = ('configuration', 'runs', 'success_rate') + SUMMARY
        print('\n| ' + ' | '.join(head) + ' |')
        print('|' + '---|' * len(head))
        for key, entry in table.items():
            print('| ' + ' | '.join([key] + [_cell(entry[c]) for c in head[1:]]) + ' |')
    for run, error in failures:
        print(f'FAILED {run}: {error}')
    if args.json:
        args.json.write_text(json.dumps({'runs': results, 'aggregate': table, 'failures': failures}, indent=1))
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
