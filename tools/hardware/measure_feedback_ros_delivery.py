#!/usr/bin/env python3
"""Read-only stationary measurement of Puma aggregate ROS delivery age."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import time

ROOT = Path(__file__).resolve().parents[2]


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def summarize(rows):
    ages = [row['delivery_age_ms'] for row in rows if row['topic'] == 'feedback']
    stamps = [row['stamp_ns'] for row in rows if row['topic'] == 'feedback']
    arrivals = [row['monotonic_ns'] for row in rows if row['topic'] == 'feedback']
    gaps = [(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
    return {
        'feedback_samples': len(ages),
        'delivery_age_ms': ({'median': statistics.median(ages), 'p95': percentile(ages, .95),
                             'max': max(ages), 'min': min(ages)} if ages else None),
        'max_arrival_gap_ms': max(gaps, default=None),
        'nonincreasing_stamps': sum(b <= a for a, b in zip(stamps, stamps[1:])),
        'stops_pressed': sum(row['pressed'] for row in rows if row['topic'] == 'stop'),
        'stops_released': sum(not row['pressed'] for row in rows if row['topic'] == 'stop'),
        'max_raw_odom_m_s': max((max(abs(v) for v in row['velocity']) for row in rows
                                 if row['topic'] == 'odom'), default=None),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 10 <= args.duration <= 60:
        parser.error('duration must be 10..60 seconds')
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
    from clearpath_motor_msgs.msg import PumaMultiFeedback

    root.mkdir(parents=True)
    rclpy.init(args=[])
    node = rclpy.create_node('puma_delivery_observer', namespace=ns,
                             use_global_arguments=False, enable_rosout=False,
                             start_parameter_services=False)
    rows = []
    def record_stop(msg):
        rows.append({'topic': 'stop', 'monotonic_ns': time.monotonic_ns(),
                     'pressed': bool(msg.data)})
    def record_odom(msg):
        twist = msg.twist.twist
        rows.append({'topic': 'odom', 'monotonic_ns': time.monotonic_ns(),
                     'velocity': [twist.linear.x, twist.linear.y, twist.angular.z]})
    def record_feedback(msg):
        arrival_wall = time.time_ns()
        arrival_mono = time.monotonic_ns()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        rows.append({'topic': 'feedback', 'monotonic_ns': arrival_mono,
                     'stamp_ns': stamp, 'delivery_age_ms': (arrival_wall - stamp) / 1e6,
                     'wheel_count': len(msg.drivers_feedback)})
    subscriptions = [
        node.create_subscription(Bool, 'platform/emergency_stop', record_stop, qos_profile_sensor_data),
        node.create_subscription(Odometry, 'platform/odom', record_odom, qos_profile_sensor_data),
        node.create_subscription(PumaMultiFeedback, 'platform/motors/feedback', record_feedback,
                                 qos_profile_sensor_data),
    ]
    status = 'incomplete'
    reason = None
    started = time.monotonic()
    try:
        print('Read-only ROS feedback timing; keep physical e-stop pressed.', flush=True)
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
            elapsed = time.monotonic() - started
            recent = [r for r in rows if r['topic'] == 'stop']
            if any(not r['pressed'] for r in recent):
                reason = 'physical stop released during capture'; break
            if elapsed > 3 and not recent:
                reason = 'no physical stop report'; break
            if recent and elapsed > 3 and (time.monotonic_ns() - recent[-1]['monotonic_ns']) / 1e9 > 2.5:
                reason = 'physical stop observation stale'; break
            if any(r['topic'] == 'odom' and any(not math.isfinite(v) or abs(v) > .001
                    for v in r['velocity']) for r in rows[-20:]):
                reason = 'nonstationary raw odometry'; break
        summary = summarize(rows)
        if reason is None and (summary['feedback_samples'] < args.duration * 10 or
                summary['stops_pressed'] < 2 or summary['max_raw_odom_m_s'] is None or
                summary['nonincreasing_stamps'] or
                any(r['topic'] == 'feedback' and r['wheel_count'] != 4 for r in rows)):
            reason = 'missing, malformed or nonmonotonic stationary evidence'
        if reason is None: status = 'stationary-ros-delivery-observed'
    finally:
        del subscriptions
        node.destroy_node(); rclpy.shutdown()
    report = {'status': status, 'reason': reason, 'namespace': ns,
              'duration_s': time.monotonic() - started, 'summary': summary,
              'limits': ('Header stamp is set at aggregate generation in the Puma driver; '
                         'arrival age measures this separate observer ROS delivery path. '
                         'It does not measure driver CAN callback queue delay, the controller-manager '
                         'callback path, a loaded system, physical wheel synchronization, or motion.')}
    (root / 'samples.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    (root / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if status == 'stationary-ros-delivery-observed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
