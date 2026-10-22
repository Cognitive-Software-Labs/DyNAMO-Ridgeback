#!/usr/bin/env python3
"""Read-only stationary acceptance of the installed per-wheel field-age Puma driver.

  python3 tools/hardware/check_puma_field_driver_live.py --run-dir artifacts/hardware/<fresh>

Run with the physical stop pressed after install_puma_field_driver.py --apply.
It never publishes. Over the observation window it records aggregate feedback
arrivals, each driver's diagnostic motion/auxiliary field ages, the driver's
DYNAMO_PUMA_QUEUE_TIME journal buckets, the stop and raw odometry.

Pass: the running driver comes from /opt/dynamo-puma-field; the stop stays
pressed and odometry stationary; aggregates arrive at >= 19 Hz with no gap above
100 ms; every sampled driver reports valid ages within the 150/500 ms bounds;
every queue bucket's maximum callback-to-dequeue age is at most QUEUE_LIMIT_MS.
The driver drains its queue once per 50 ms (20 Hz) run cycle, so up to 50 ms of
queue age is structural; the limit allows one cycle plus 25 ms, inside the
0-100 ms delays that passed the offline replay. Stationary only: no motion, load
or fault response is tested.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
MOTION_MS, AUXILIARY_MS = 150.0, 500.0
QUEUE_LIMIT_MS = 75.0  # one 50 ms drain cycle plus margin; 50 ms failed on structure alone
MIN_RATE_HZ, MAX_GAP_MS = 19.0, 100.0
QUEUE = re.compile(r'DYNAMO_PUMA_QUEUE_TIME frames=(\d+) mean_ms=([\d.]+) max_ms=([\d.]+) over_100ms=(\d+)')


def evaluate(arrivals_s, ages, queue, stops, speeds, driver, duration):
    """Summary and pass/fail. ages: [(name, motion_ms, auxiliary_ms)], queue: [(frames, mean, max, over)]."""
    gaps = [(b - a) * 1000 for a, b in zip(arrivals_s, arrivals_s[1:])]
    rate = len(arrivals_s) / duration if duration > 0 else 0.0
    report = {
        'driver': driver, 'field_driver': driver.startswith('/opt/dynamo-puma-field/'),
        'feedback_samples': len(arrivals_s), 'feedback_rate_hz': round(rate, 2),
        'max_feedback_gap_ms': round(max(gaps), 1) if gaps else None,
        'age_samples': len(ages), 'drivers_reporting': sorted({name for name, _, _ in ages}),
        'max_motion_age_ms': max((m for _, m, _ in ages), default=None),
        'max_auxiliary_age_ms': max((a for _, _, a in ages), default=None),
        'invalid_age_samples': sum(m < 0 or a < 0 for _, m, a in ages),
        'queue_buckets': len(queue), 'queue_max_ms': max((q[2] for q in queue), default=None),
        'queue_over_100ms': sum(q[3] for q in queue),
        'stop_pressed_throughout': bool(stops) and all(stops),
        'max_raw_odom': max(speeds, default=None),
    }
    report['passed'] = bool(
        report['field_driver'] and report['stop_pressed_throughout']
        and speeds and max(speeds) <= 0.001
        and rate >= MIN_RATE_HZ and gaps and max(gaps) <= MAX_GAP_MS
        and len(report['drivers_reporting']) == 4 and ages and not report['invalid_age_samples']
        and report['max_motion_age_ms'] <= MOTION_MS and report['max_auxiliary_age_ms'] <= AUXILIARY_MS
        and queue and report['queue_max_ms'] <= QUEUE_LIMIT_MS and not report['queue_over_100ms'])
    return report


def running_driver():
    for process in Path('/proc').iterdir():
        if process.name.isdecimal():
            try:
                argv0 = (process / 'cmdline').read_bytes().split(b'\0')[0].decode()
            except (OSError, UnicodeError):
                continue
            if argv0.endswith('/multi_puma_node'):
                return argv0
    return ''


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 20 <= args.duration <= 300:
        parser.error('duration must be 20..300 seconds (queue buckets are five seconds)')
    root = args.run_dir.resolve()
    if not root.is_relative_to(ROOT / 'artifacts/hardware') or root.exists():
        parser.error('use a fresh directory below artifacts/hardware')
    from install_commissioning_caps import identity
    ns = identity()
    os.environ.update(ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml', ROS_LOG_DIR='/tmp')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from clearpath_motor_msgs.msg import PumaMultiFeedback

    root.mkdir(parents=True)
    rclpy.init(args=[])
    node = rclpy.create_node('puma_field_driver_observer', namespace=ns, use_global_arguments=False,
                             enable_rosout=False, start_parameter_services=False)
    arrivals, ages, stops, speeds = [], [], [], []

    def on_diagnostics(message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if 'Oldest motion field age (ms)' in values:
                ages.append((status.name, float(values['Oldest motion field age (ms)']),
                             float(values['Oldest auxiliary field age (ms)'])))

    node.create_subscription(PumaMultiFeedback, 'platform/motors/feedback',
                             lambda m: arrivals.append(time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, 'diagnostics', on_diagnostics, 10)
    node.create_subscription(Bool, 'platform/emergency_stop', lambda m: stops.append(bool(m.data)),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, 'platform/odom', lambda m: speeds.append(max(
        abs(m.twist.twist.linear.x), abs(m.twist.twist.linear.y), abs(m.twist.twist.angular.z))),
        qos_profile_sensor_data)
    started_wall, started = time.time(), time.monotonic()
    while time.monotonic() - started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    duration = time.monotonic() - started
    node.destroy_node()
    rclpy.shutdown()
    journal = subprocess.run(['journalctl', '-u', 'clearpath-platform.service', '-o', 'cat', '--no-pager',
                              '--since', f'@{started_wall:.0f}', '--until', f'@{time.time():.0f}'],
                             capture_output=True, text=True).stdout
    queue = [(int(f), float(mean), float(peak), int(over)) for f, mean, peak, over in QUEUE.findall(journal)]
    (root / 'queue-journal.txt').write_text('\n'.join(l for l in journal.splitlines() if 'DYNAMO_PUMA' in l) + '\n')
    (root / 'samples.json').write_text(json.dumps({'arrivals_s': [a - started for a in arrivals],
                                                   'ages': ages, 'queue': queue}) + '\n')
    report = evaluate(arrivals, ages, queue, stops, speeds, running_driver(), duration)
    report['duration_s'] = round(duration, 2)
    (root / 'result.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps(report, indent=1))
    print('FIELD DRIVER STATIONARY ACCEPTANCE ' + ('PASS' if report['passed'] else 'NOT PASSED'))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
