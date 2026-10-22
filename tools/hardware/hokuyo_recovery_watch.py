#!/usr/bin/env python3
"""Bounded Hokuyo reconnect observation; recovery is explicitly stationary-only.

Default: observe for 60 seconds without changing any service. Recovery requires
--recover-stationary, --operator-confirmed, root and fresh asserted-stop/zero
odometry evidence. One restart maximum, followed by sustained scan verification.
This candidate is not installed or qualified against a real TCP interruption.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

TOPICS = ('sensors/lidar2d_0/scan', 'sensors/lidar2d_1/scan')
SIGNATURES = ('Error count exceeded limit, reconnecting',
              'Error connecting to Hokuyo: Could not open network Hokuyo')


def recovery_due(last_seen, now, journal, *, stale_s=12):
    if any(stamp is None for stamp in last_seen.values()):
        return None
    stale = [name for name, stamp in last_seen.items() if now - stamp >= stale_s]
    return stale if stale and all(s in journal for s in SIGNATURES) else None


def stationary_ok(stops, odom, now):
    stops = [(t, v) for t, v in stops if now - t <= 3]
    odom = [(t, v) for t, v in odom if now - t <= 3]
    return (len(stops) >= 2 and all(v for _, v in stops)
            and now - stops[-1][0] <= 2.5 and len(odom) >= 20
            and now - odom[-1][0] <= .25
            and all(all(math.isfinite(x) and abs(x) <= .001 for x in v)
                    for _, v in odom))


def scans_restored(samples, since, now):
    for topic in TOPICS:
        rows = [(t, stamp) for t, stamp in samples[topic] if t > since]
        if len(rows) < 2 or rows[-1][0] - rows[0][0] < 3:
            return False
        rate = (len(rows) - 1) / (rows[-1][0] - rows[0][0])
        gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
        if (rate < 38 or max(gaps) > .25 or now - rows[-1][0] > .25
                or any(b[1] <= a[1] for a, b in zip(rows, rows[1:]))):
            return False
    return True


def recent_journal():
    p = subprocess.run(('journalctl', '-u', 'clearpath-sensors.service',
                        '--since=-30s', '--no-pager', '--output=cat'),
                       capture_output=True, text=True, timeout=5)
    if p.returncode:
        raise RuntimeError('cannot read sensor journal: ' + p.stderr)
    return p.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/r100_0160')
    parser.add_argument('--dry-run', action='store_true', help='read-only (the default)')
    parser.add_argument('--recover-stationary', action='store_true')
    parser.add_argument('--operator-confirmed', action='store_true')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 35 <= args.duration <= 180:
        parser.error('duration must be 35..180 seconds')
    if args.recover_stationary and (args.dry_run or os.geteuid() != 0
                                   or not args.operator_confirmed):
        parser.error('recovery requires root, operator confirmation and no --dry-run')
    if args.report.exists():
        parser.error('use a fresh report file')
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from nav_msgs.msg import Odometry
    rclpy.init(args=[])
    node = rclpy.create_node('hokuyo_recovery_watch', namespace=args.namespace,
                             enable_rosout=False, start_parameter_services=False)
    last_seen = dict.fromkeys(TOPICS)
    samples = {topic: [] for topic in TOPICS}
    stops, odom = [], []
    def receive(topic, msg):
        now = time.monotonic()
        last_seen[topic] = now
        samples[topic].append((now, msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec))
    for topic in TOPICS:
        node.create_subscription(LaserScan, topic,
                                 lambda msg, name=topic: receive(name, msg), qos_profile_sensor_data)
    if args.recover_stationary:
        node.create_subscription(Bool, 'platform/emergency_stop',
                                 lambda m: stops.append((time.monotonic(), bool(m.data))), qos_profile_sensor_data)
        node.create_subscription(Odometry, 'platform/odom', lambda m: odom.append(
            (time.monotonic(), (m.twist.twist.linear.x, m.twist.twist.linear.y,
                               m.twist.twist.angular.z))), qos_profile_sensor_data)
    started = time.monotonic()
    last_check = started
    report = {'status': 'not-started', 'recovery_requested': args.recover_stationary,
              'restarts': 0, 'limits': 'Stationary scan delivery only; no moving recovery or physical braking qualification.'}
    try:
        print('Observing Hokuyo scans. Keep the physical e-stop pressed.', flush=True)
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if args.recover_stationary and now - started >= 3 and not stationary_ok(stops, odom, now):
                raise RuntimeError('fresh pressed-stop/stationary evidence lost; no further service actions')
            if now - started < 30 or now - last_check < 2:
                continue
            last_check = now
            if any(t is None for t in last_seen.values()) or not any(now - t >= 12 for t in last_seen.values()):
                continue
            journal = recent_journal()
            stale = recovery_due(last_seen, now, journal)
            if not stale:
                continue
            report.update(status='lockout-observed', stale_topics=stale, journal=journal)
            if not args.recover_stationary:
                break
            active = subprocess.run(('systemctl', 'is-active', 'clearpath-sensors.service'),
                                    capture_output=True, text=True, timeout=5)
            if active.returncode or active.stdout.strip() != 'active':
                raise RuntimeError('sensor service intentionally stopped or inactive; refusing restart')
            if not stationary_ok(stops, odom, time.monotonic()):
                raise RuntimeError('stationary evidence unavailable before restart')
            subprocess.run(('systemctl', 'restart', 'clearpath-sensors.service'), check=True, timeout=30)
            report['restarts'] = 1
            after = time.monotonic()
            while time.monotonic() - after < 30:
                rclpy.spin_once(node, timeout_sec=.1)
                now = time.monotonic()
                if not stationary_ok(stops, odom, now):
                    raise RuntimeError('stationary evidence lost during recovery')
                if scans_restored(samples, after, now):
                    report['status'] = 'recovered-stationary'
                    break
            else:
                raise RuntimeError('both scans did not sustain 38 Hz for three seconds after restart')
            break
        if report['status'] == 'not-started':
            report['status'] = ('healthy-observation' if scans_restored(samples, started, time.monotonic())
                                else 'scan-observation-incomplete')
    except Exception as exc:
        report.update(status='check-failed', error=str(exc))
    finally:
        report.update(duration_s=time.monotonic() - started, samples=samples)
        node.destroy_node()
        rclpy.shutdown()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'journal')}, indent=2))
    return 0 if report['status'] in ('healthy-observation', 'recovered-stationary', 'lockout-observed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
