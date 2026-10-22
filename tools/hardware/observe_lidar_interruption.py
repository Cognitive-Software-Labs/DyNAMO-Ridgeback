#!/usr/bin/env python3
"""Observe one Hokuyo TCP interruption and judge in-driver recovery (S3).

Usage, on Intel with /etc/clearpath/setup.bash sourced, base stationary and
e-stopped:
  python3 tools/hardware/observe_lidar_interruption.py --lidar front --run-dir DIR

The tool only subscribes and reads process, socket and journal state. After a
quiet baseline it prints the one command the operator runs as root to reset
that LiDAR's TCP session, then records until the end and writes report.json.
Limits are fixed here, before measurement:

- the interrupted scan resumes within RESUME_LIMIT_S, from the same driver PID;
- the other LiDAR never gaps more than INDIVIDUAL_GAP_S and the merged scan
  recovers too;
- over the final SETTLE_S, every scan sustains its rate with no gap above its
  limit (individual 250 ms, merged 500 ms) and increasing stamps;
- clearpath-sensors and the scan merger keep their PIDs and restart counts.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import time

LIDARS = {'front': ('sensors/lidar2d_0/scan', '192.168.131.21', 'lidar2d_0'),
          'rear': ('sensors/lidar2d_1/scan', '192.168.131.22', 'lidar2d_1')}
MERGED = 'sensors/scan'
PORT = 10940
RESUME_LIMIT_S = 10.0
SETTLE_S = 30.0
INDIVIDUAL_GAP_S = 0.25
MERGED_GAP_S = 0.5
MIN_INDIVIDUAL_HZ = 38.0
MIN_MERGED_RATIO = 0.9


def gaps(times):
    return [b - a for a, b in zip(times, times[1:])]


def window(rows, start, end):
    return [row for row in rows if start <= row[0] <= end]


def rate(rows):
    if len(rows) < 2 or rows[-1][0] == rows[0][0]:
        return 0.0
    return (len(rows) - 1) / (rows[-1][0] - rows[0][0])


def recovery(samples, interrupted, baseline_end):
    """(outage length, time scans resumed) of the interrupted LiDAR, or (None, None)."""

    after = [t for t, _ in samples[interrupted] if t > baseline_end]
    if not after:
        return None, None
    times = [baseline_end] + after
    index = max(range(1, len(times)), key=lambda i: times[i] - times[i - 1])
    return times[index] - times[index - 1], times[index]


def settled(samples, interrupted, baseline_end, now):
    """True once SETTLE_S of scans exist after the interruption ended."""

    outage, resumed = recovery(samples, interrupted, baseline_end)
    return outage is not None and outage > INDIVIDUAL_GAP_S and now - resumed >= SETTLE_S + 0.5


def evaluate(samples, interrupted, other, baseline_end, end, pids_before, pids_after):
    """Rows of (criterion, passed, detail). ``samples`` maps topic -> [(receive_s, stamp_s)].

    The settle window is the SETTLE_S after the interrupted scan resumed (plus
    0.5 s), not the end of the recording, so the operator's timing cannot put
    the outage inside it; a recording too short to hold it fails.
    """

    rows = []
    outage, resumed = recovery(samples, interrupted, baseline_end)
    rows.append(('interruption observed', outage is not None and outage > INDIVIDUAL_GAP_S,
                 f'largest {interrupted} gap after baseline {outage:.3f} s' if outage else 'no scans'))
    rows.append((f'{interrupted} resumes', outage is not None and outage <= RESUME_LIMIT_S,
                 f'limit {RESUME_LIMIT_S} s'))
    other_gap = max(gaps([t for t, _ in samples[other]]), default=None)
    rows.append((f'{other} unaffected', other_gap is not None and other_gap <= INDIVIDUAL_GAP_S,
                 f'max gap {other_gap:.3f} s' if other_gap is not None else 'no scans'))
    merged_gap = max(gaps([t for t, _ in window(samples[MERGED], baseline_end, end)]), default=None)
    rows.append(('merged gap during interruption (recorded)', True,
                 f'{merged_gap:.3f} s' if merged_gap is not None else 'no scans'))
    merged_base = rate(window(samples[MERGED], 0, baseline_end))
    start = (resumed if resumed is not None else end) + 0.5
    rows.append(('settle window recorded', start + SETTLE_S <= end,
                 f'{max(0.0, end - start):.1f} s after recovery (need {SETTLE_S:.0f} s)'))
    for topic, rows_ in samples.items():
        settle = window(rows_, start, start + SETTLE_S)
        limit = MERGED_GAP_S if topic == MERGED else INDIVIDUAL_GAP_S
        minimum = merged_base * MIN_MERGED_RATIO if topic == MERGED else MIN_INDIVIDUAL_HZ
        largest = max(gaps([t for t, _ in settle]), default=None)
        stamps_ok = all(b[1] > a[1] for a, b in zip(settle, settle[1:]))
        rows.append((f'{topic} settled', rate(settle) >= minimum and largest is not None
                     and largest <= limit and stamps_ok,
                     f'{rate(settle):.2f} Hz (min {minimum:.2f}), max gap '
                     + (f'{largest * 1000:.1f} ms' if largest is not None else '-')
                     + ('' if stamps_ok else ', stamps not increasing')))
    for name in sorted(set(pids_before) | set(pids_after)):
        rows.append((f'{name} not restarted', pids_before.get(name) == pids_after.get(name),
                     f'{pids_before.get(name)} -> {pids_after.get(name)}'))
    return rows


def process_state(lidar_node):
    def pid(pattern):
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()
        return out[0] if len(out) == 1 else ','.join(out) or None

    show = subprocess.run(['systemctl', 'show', 'clearpath-sensors', 'clearpath-scan-merger',
                           '-p', 'Id', '-p', 'MainPID', '-p', 'NRestarts'],
                          capture_output=True, text=True).stdout
    state = {f'{lidar} driver': pid(f'urg_node_driver.*{LIDARS[lidar][2]}') for lidar in LIDARS}
    state['scan merger'] = pid('ros2_laser_scan_merger --ros-args')
    for block in show.strip().split('\n\n'):
        fields = dict(line.split('=', 1) for line in block.splitlines() if '=' in line)
        if 'Id' in fields:
            state[fields['Id']] = f"pid {fields.get('MainPID')} restarts {fields.get('NRestarts')}"
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--lidar', choices=sorted(LIDARS), required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--namespace', default='r100_0160')
    parser.add_argument('--baseline', type=float, default=20.0)
    parser.add_argument('--after', type=float, default=180.0,
                        help='longest wait after the prompt; recording stops once recovery has settled')
    args = parser.parse_args()
    if args.after < SETTLE_S + RESUME_LIMIT_S:
        parser.error(f'--after must leave at least {SETTLE_S + RESUME_LIMIT_S} s')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    args.run_dir.mkdir(parents=True, exist_ok=False)
    topic, address, _ = LIDARS[args.lidar]
    other = LIDARS['rear' if args.lidar == 'front' else 'front'][0]
    topics = (LIDARS['front'][0], LIDARS['rear'][0], MERGED)
    samples = {name: [] for name in topics}
    t0 = time.monotonic()
    journal_since = time.strftime('%Y-%m-%d %H:%M:%S')

    rclpy.init()
    node = rclpy.create_node('lidar_interruption_observer', namespace=args.namespace)
    for name in topics:
        node.create_subscription(
            LaserScan, name,
            lambda msg, name=name: samples[name].append(
                (time.monotonic() - t0, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)),
            qos_profile_sensor_data)

    def spin_until(deadline, sockets_log, done=lambda now: False):
        next_socket = 0.0
        while time.monotonic() - t0 < deadline and not done(time.monotonic() - t0):
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic() - t0
            if now >= next_socket:
                out = subprocess.run(['ss', '-tn', 'dst', address], capture_output=True, text=True).stdout
                sockets_log.write(json.dumps({'t': round(now, 3), 'sockets': out.splitlines()[1:]}) + '\n')
                next_socket = now + 1.0

    pids_before = process_state(args.lidar)
    with open(args.run_dir / 'sockets.jsonl', 'w') as sockets_log:
        spin_until(args.baseline, sockets_log)
        baseline_end = time.monotonic() - t0
        print(f'Baseline done. Now run, once:\n  sudo ss -K dst {address} dport = {PORT}', flush=True)
        spin_until(args.baseline + args.after, sockets_log,
                   done=lambda now: settled(samples, topic, baseline_end, now) and now > baseline_end + 1)
    end = time.monotonic() - t0
    pids_after = process_state(args.lidar)
    node.destroy_node()
    rclpy.shutdown()

    journal = subprocess.run(['journalctl', '-u', 'clearpath-sensors', '--since', journal_since,
                              '--no-pager', '-o', 'short-iso'], capture_output=True, text=True).stdout
    (args.run_dir / 'clearpath-sensors.journal').write_text(journal)
    (args.run_dir / 'samples.json').write_text(json.dumps(samples))
    rows = evaluate(samples, topic, other, baseline_end, end, pids_before, pids_after)
    for name, passed, detail in rows:
        print(f"{'PASS' if passed else 'FAIL'}  {name:42} {detail}")
    passed = all(ok for _, ok, _ in rows)
    print('INTERRUPTION PASS' if passed else 'INTERRUPTION FAIL')
    reconnects = len(re.findall(r'reconnecting', journal))
    (args.run_dir / 'report.json').write_text(json.dumps({
        'lidar': args.lidar, 'address': address, 'baseline_end_s': baseline_end, 'end_s': end,
        'pids_before': pids_before, 'pids_after': pids_after, 'journal_reconnect_lines': reconnects,
        'rows': [{'criterion': n, 'passed': p, 'detail': d} for n, p, d in rows], 'passed': passed},
        indent=1))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
