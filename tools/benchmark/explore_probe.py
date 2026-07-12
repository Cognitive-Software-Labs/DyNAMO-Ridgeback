#!/usr/bin/env python3
"""Instrument frontier-explorer runs.

Tracks:
- navigate_to_pose action status transitions per goal ID (accept/succeed/abort/cancel)
- abort classification: PREEMPTED (another goal accepted within +/-1.5s) vs GENUINE
- explore/frontiers markers (cluster count)
- explore/status (std_msgs/String): "exploration_complete" = quit
- hud/coverage complete%

Writes events CSV + summary JSON. Exits on exploration_complete + 10s, or --max-wall.
"""
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
    def __init__(self, tag):
        super().__init__('explore_probe')
        self.tag = tag
        self.t0 = time.time()
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
        self.events = open(f'{tag}_events.csv', 'w')
        self.events.write('t,kind,detail\n')

        self.create_subscription(
            GoalStatusArray, NS + '/navigate_to_pose/_action/status',
            self.on_status, 10)
        self.create_subscription(
            MarkerArray, NS + '/explore/frontiers', self.on_frontiers, 10)
        tl = QoSProfile(depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, NS + '/explore/status', self.on_explore_status, tl)
        self.create_subscription(
            OverlayText, NS + '/hud/coverage', self.on_cov, 10)
        # Achieved RTF (sim-time span / wall span); /clock is global, not
        # namespaced. Captured live so it survives an unclean runner teardown.
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        # GT-drift, isaac-only: gz publishes no hud/localization so this never
        # fires and the localization fields stay null (see localization_overlay).
        self.create_subscription(
            OverlayText, NS + '/hud/localization', self.on_loc, 10)

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


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'probe'
    max_wall = float(sys.argv[2]) if len(sys.argv) > 2 else 1200.0
    rclpy.init()
    p = Probe(tag)
    end = time.time() + max_wall
    last_print = 0.0
    while time.time() < end:
        rclpy.spin_once(p, timeout_sec=0.5)
        if p.complete_at is not None and p.now() > p.complete_at + 10.0:
            break
        if p.now() - last_print > 15.0:
            last_print = p.now()
            s = p.summary()
            print(f'[{p.now():7.1f}s] cov={p.coverage:.1f}% goals={s["goals_total"]} '
                  f'ok={s["succeeded"]} abort={s["aborted"]} '
                  f'(pre={s["aborts_preempted"]}/gen={s["aborts_genuine"]}) '
                  f'frontiers avail={s["frontiers_at_end"]["avail"]} '
                  f'black={s["frontiers_at_end"]["blacklisted"]}', flush=True)
    s = p.summary()
    with open(f'{tag}_summary.json', 'w') as f:
        json.dump(s, f, indent=2)
    print(json.dumps(s, indent=2))
    p.events.close()


if __name__ == '__main__':
    main()
