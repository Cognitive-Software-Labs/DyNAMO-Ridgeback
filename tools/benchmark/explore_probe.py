#!/usr/bin/env python3
"""Instrument frontier-explorer runs.

Tracks:
- navigate_to_pose action status transitions per goal ID (accept/succeed/abort/cancel)
- abort classification: PREEMPTED (another goal accepted within +/-1.5s) vs GENUINE
- explore/frontiers markers (cluster count)
- explore/status (std_msgs/String): "exploration_complete" = quit
- hud/coverage complete%

Writes events CSV + summary JSON. Exits on exploration_complete + 10s, or --max-wall.

--repeat N (P7): run N exploration cycles back-to-back WITHOUT relaunching the
sim. Between runs it resets in-session state — runner `sim/reset` (robot back
to spawn + odom re-zero), slam_toolbox `reset` (clears the pose graph/map), and
both nav2 costmap clears — then waits --settle for fresh frontiers so the
explorer's timer re-arms and republishes exploration_started. Emits one
<tag>_run<i>_summary.json per run plus a combined <tag>_summary.json (which is
the single run's summary verbatim when N==1, unchanged for ab_compare.py).
"""
import argparse
import json
import re
import sys
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatusArray
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import String

NS = '/r100_0001'
# HUD panels pad columns with non-breaking spaces; \s matches U+00A0 in Unicode
# mode, so these patterns work against the rendered OverlayText verbatim.
PCT = re.compile(r'complete\s+([0-9.]+)%')
ACC = re.compile(r'accuracy\s+([0-9.]+)%')
LOC = re.compile(r'trans\s+err\s+([0-9.]+)')
STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
PREEMPT_WINDOW = 1.5  # s: new goal accepted within this of an abort => preemption


class Probe(Node):
    def __init__(self, tag, namespace=NS):
        super().__init__('explore_probe')
        self.tag = tag
        self.ns = namespace
        self.t0 = time.time()
        self._init_state()
        self.events = open(f'{tag}_events.csv', 'w')
        self.events.write('t,kind,detail\n')

        self.create_subscription(
            GoalStatusArray, namespace + '/navigate_to_pose/_action/status',
            self.on_status, 10)
        self.create_subscription(
            MarkerArray, namespace + '/explore/frontiers', self.on_frontiers, 10)
        tl = QoSProfile(depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, namespace + '/explore/status', self.on_explore_status, tl)
        self.create_subscription(
            OverlayText, namespace + '/hud/coverage', self.on_cov, 10)
        # Achieved RTF (sim-time span / wall span); /clock is global, not
        # namespaced. Captured live so it survives an unclean runner teardown.
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        # GT-drift, isaac-only: gz publishes no hud/localization so this never
        # fires and the localization fields stay null (see localization_overlay).
        self.create_subscription(
            OverlayText, namespace + '/hud/localization', self.on_loc, 10)

        # --- reset service clients (P7 --repeat), created lazily-typed ------
        from std_srvs.srv import Trigger
        from nav2_msgs.srv import ClearEntireCostmap
        self._reset_clients = {
            'runner': self.create_client(Trigger, namespace + '/sim/reset'),
            'global_costmap': self.create_client(
                ClearEntireCostmap,
                namespace + '/global_costmap/clear_entirely_global_costmap'),
            'local_costmap': self.create_client(
                ClearEntireCostmap,
                namespace + '/local_costmap/clear_entirely_local_costmap'),
        }
        self._Trigger = Trigger
        self._ClearEntireCostmap = ClearEntireCostmap
        try:                                   # slam reset is optional/tolerant
            from slam_toolbox.srv import Reset
            self._Reset = Reset
            self._reset_clients['slam'] = self.create_client(
                Reset, namespace + '/slam_toolbox/reset')
        except Exception:
            self._Reset = None

    # ---- per-run state ------------------------------------------------------

    def _init_state(self):
        self.goals = OrderedDict()   # id -> {'accepted': t, 'terminal': (t, name)}
        self.aborts = []             # {'t','goal','kind'}
        self.frontier_log = []       # (t, avail, blacklisted)
        self.coverage = 0.0
        self.cov_log = []
        self.accuracy = None         # coverage accuracy% (hud/coverage)
        self.complete_at = None
        self.sim_first = None        # (wall_s, sim_s) at first /clock
        self.sim_last = None         # (wall_s, sim_s) at latest /clock
        self.loc_errs = []           # (wall_s, trans_err_m) from hud/localization

    def reset_state(self):
        """Clear per-run accumulators and re-base the clock for the next run."""
        self._init_state()
        self.t0 = time.time()

    def now(self):
        return time.time() - self.t0

    def ev(self, kind, detail):
        self.events.write(f'{self.now():.2f},{kind},{detail}\n')
        self.events.flush()

    def on_status(self, msg):
        t = self.now()
        for gs in msg.status_list:
            gid = bytes(gs.goal_info.goal_id.uuid).hex()[:8]
            st = gs.status
            g = self.goals.setdefault(gid, {'accepted': None, 'terminal': None})
            if st in (1, 2) and g['accepted'] is None:
                g['accepted'] = t
                self.ev('goal_accepted', gid)
            elif st in (4, 5, 6) and g['terminal'] is None:
                name = STATUS_NAMES[st]
                g['terminal'] = (t, name)
                self.ev('goal_' + name.lower(), gid)
                if st == 6:
                    self.aborts.append({'t': t, 'goal': gid, 'kind': None})

    def classify_aborts(self):
        """Preemption: another goal accepted within PREEMPT_WINDOW of abort."""
        accept_times = [g['accepted'] for g in self.goals.values()
                        if g['accepted'] is not None]
        for a in self.aborts:
            if a['kind'] is None or True:  # reclassify each call (late accepts)
                near = any(abs(t - a['t']) <= PREEMPT_WINDOW
                           for t in accept_times)
                a['kind'] = 'preempted' if near else 'genuine'

    def on_frontiers(self, msg):
        # frontier_explorer publishes one SPHERE marker per cluster
        avail = sum(1 for m in msg.markers if m.action == 0 and m.type == 2)
        black = 0
        self.frontier_log.append((self.now(), avail, black))
        self.ev('frontiers', f'avail={avail} black={black}')

    def on_explore_status(self, msg):
        self.ev('explore_status', msg.data)
        if msg.data == 'exploration_complete':
            self.complete_at = self.now()

    def on_cov(self, msg):
        m = PCT.search(msg.text)
        if m:
            v = float(m.group(1))
            self.coverage = max(self.coverage, v)
            self.cov_log.append((self.now(), v))
        a = ACC.search(msg.text)
        if a:
            self.accuracy = float(a.group(1))

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_first is None:
            self.sim_first = (self.now(), t)
        self.sim_last = (self.now(), t)

    def on_loc(self, msg):
        m = LOC.search(msg.text)
        if m:
            self.loc_errs.append((self.now(), float(m.group(1))))

    def summary(self):
        self.classify_aborts()
        term = [g['terminal'][1] for g in self.goals.values() if g['terminal']]
        pre = sum(1 for a in self.aborts if a['kind'] == 'preempted')
        gen = len(self.aborts) - pre
        last_f = self.frontier_log[-1] if self.frontier_log else (0, -1, -1)
        rtf = None
        if self.sim_first and self.sim_last:
            dwall = self.sim_last[0] - self.sim_first[0]
            dsim = self.sim_last[1] - self.sim_first[1]
            if dwall > 1.0:
                rtf = round(dsim / dwall, 3)
        loc = [e for _, e in self.loc_errs]
        return {
            'quit_at_s': self.complete_at,
            'coverage_peak_pct': self.coverage,
            'coverage_accuracy_pct': self.accuracy,
            'achieved_rtf': rtf,
            'localization_err_mean_m':
                round(sum(loc) / len(loc), 3) if loc else None,
            'localization_err_max_m': round(max(loc), 3) if loc else None,
            'localization_err_last_m': round(loc[-1], 3) if loc else None,
            'goals_total': len(self.goals),
            'succeeded': term.count('SUCCEEDED'),
            'aborted': term.count('ABORTED'),
            'canceled': term.count('CANCELED'),
            'aborts_preempted': pre,
            'aborts_genuine': gen,
            'frontiers_at_end': {'avail': last_f[1], 'blacklisted': last_f[2]},
            'frontier_peak_black': max((f[2] for f in self.frontier_log),
                                       default=0),
        }

    # ---- in-session reset (P7 --repeat) ------------------------------------

    def _call(self, name, req, timeout=15.0):
        cli = self._reset_clients.get(name)
        if cli is None:
            return f'{name}:absent'
        if not cli.wait_for_service(timeout_sec=5.0):
            return f'{name}:no-service'
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return f'{name}:ok' if fut.done() else f'{name}:timeout'

    def reset_run(self):
        """Return the robot to spawn + clear SLAM/costmaps for a fresh run."""
        results = []
        results.append(self._call('runner', self._Trigger.Request()))
        if self._Reset is not None:
            rq = self._Reset.Request()
            rq.pause_new_measurements = False
            results.append(self._call('slam', rq))
        for cm in ('global_costmap', 'local_costmap'):
            results.append(self._call(cm, self._ClearEntireCostmap.Request()))
        self.ev('reset', ' '.join(results))
        return results


def _run_once(p, run_idx, max_wall, min_run):
    """Observe one exploration cycle; returns when it completes (+10 s settle,
    honored only after min_run seconds) or max_wall elapses."""
    p.ev('run_start', str(run_idx))
    end = time.time() + max_wall
    last_print = 0.0
    while time.time() < end:
        rclpy.spin_once(p, timeout_sec=0.5)
        if (p.complete_at is not None and p.complete_at > min_run
                and p.now() > p.complete_at + 10.0):
            break
        if p.now() - last_print > 15.0:
            last_print = p.now()
            s = p.summary()
            print(f'[run {run_idx}] [{p.now():7.1f}s] cov={p.coverage:.1f}% '
                  f'goals={s["goals_total"]} ok={s["succeeded"]} '
                  f'abort={s["aborted"]} '
                  f'(pre={s["aborts_preempted"]}/gen={s["aborts_genuine"]}) '
                  f'frontiers avail={s["frontiers_at_end"]["avail"]}', flush=True)
    return p.summary()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('tag', nargs='?', default='probe')
    ap.add_argument('max_wall', nargs='?', type=float, default=1200.0,
                    help='per-run wall-clock cap (s)')
    ap.add_argument('--repeat', type=int, default=1,
                    help='number of exploration cycles (reset between)')
    ap.add_argument('--settle', type=float, default=15.0,
                    help='seconds to wait after a reset for fresh frontiers')
    ap.add_argument('--min-run', type=float, default=20.0,
                    help='ignore exploration_complete before this many s '
                         '(guards against a stale latched complete post-reset)')
    ap.add_argument('--namespace', default=NS)
    ap.add_argument('--no-reset', action='store_true',
                    help='do not call reset services between runs')
    args = ap.parse_args()

    rclpy.init()
    p = Probe(args.tag, namespace=args.namespace)
    summaries = []
    for run_idx in range(1, args.repeat + 1):
        s = _run_once(p, run_idx, args.max_wall, args.min_run)
        summaries.append(s)
        with open(f'{args.tag}_run{run_idx}_summary.json', 'w') as f:
            json.dump(s, f, indent=2)
        print(f'=== run {run_idx}/{args.repeat} done: '
              f'cov={s["coverage_peak_pct"]:.1f}% quit_at={s["quit_at_s"]} '
              f'rtf={s["achieved_rtf"]} ===', flush=True)
        if run_idx < args.repeat:
            if not args.no_reset:
                print(f'resetting for run {run_idx + 1}...', flush=True)
                print('  ' + ' '.join(p.reset_run()), flush=True)
            settle_end = time.time() + args.settle
            while time.time() < settle_end:
                rclpy.spin_once(p, timeout_sec=0.2)
            p.reset_state()

    if args.repeat == 1:
        combined = summaries[0]            # unchanged schema for ab_compare.py
    else:
        peaks = [s['coverage_peak_pct'] for s in summaries]
        completes = [s for s in summaries if s['quit_at_s'] is not None]
        combined = {
            'repeat': args.repeat,
            'runs': summaries,
            'coverage_peak_pct_mean': round(sum(peaks) / len(peaks), 1),
            'coverage_peak_pct_min': min(peaks),
            'coverage_peak_pct_max': max(peaks),
            'completed_runs': len(completes),
        }
    with open(f'{args.tag}_summary.json', 'w') as f:
        json.dump(combined, f, indent=2)
    print(json.dumps(combined, indent=2))
    p.events.close()


if __name__ == '__main__':
    main()
