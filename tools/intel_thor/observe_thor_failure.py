#!/usr/bin/env python3
"""Per-second record of the Intel-Thor pipeline across a Thor fault (P1). Read-only.

Usage, on Intel with /etc/clearpath/setup.bash and a ridgeback_interfaces build sourced:
  python3 tools/intel_thor/observe_thor_failure.py --duration 300 --output DIR

Each second it writes one JSON line: Thor's latest localization/health state and
age, detection and mask-measurement message counts, local colour/depth image
counts, and front/rear/merged scan counts. A summary reports when results
stopped and resumed and the worst local camera and scan rates, so a Thor fault
can be shown not to disturb Intel's own sensors. It sends nothing.
"""
import argparse
import json
from pathlib import Path
import sys
import time

STREAMS = {
    'detections': ('detections/target/raw', 'ridgeback_interfaces.msg', 'TargetDetections'),
    'measurements': ('measurements/target/mask', 'ridgeback_interfaces.msg', 'TargetMeasurements'),
    'colour': ('sensors/camera_0/color/camera_info', 'sensor_msgs.msg', 'CameraInfo'),
    'depth': ('sensors/camera_0/depth/camera_info', 'sensor_msgs.msg', 'CameraInfo'),
    'front': ('sensors/lidar2d_0/scan', 'sensor_msgs.msg', 'LaserScan'),
    'rear': ('sensors/lidar2d_1/scan', 'sensor_msgs.msg', 'LaserScan'),
    'merged': ('sensors/scan', 'sensor_msgs.msg', 'LaserScan'),
}


def summarize(rows):
    """Outage windows of Thor results and worst per-second local rates."""

    def outages(key):
        spans, start = [], None
        for row in rows:
            if row[key] == 0 and start is None:
                start = row['t']
            elif row[key] > 0 and start is not None:
                spans.append([start, row['t']])
                start = None
        if start is not None:
            spans.append([start, None])
        return spans

    steady = rows[2:] if len(rows) > 2 else rows
    return {
        'measurement_outages_s': outages('measurements'),
        'detection_outages_s': outages('detections'),
        'health_states': sorted({row['health'] for row in rows if row['health']}),
        'min_per_second': {key: min((row[key] for row in steady), default=None)
                           for key in ('colour', 'depth', 'front', 'rear', 'merged')},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--namespace', default='r100_0160')
    args = parser.parse_args()

    import importlib
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ridgeback_interfaces.msg import LocalizationHealth

    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('thor_failure_observer', namespace=args.namespace)
    counts = {key: 0 for key in STREAMS}
    health = {'state': None, 'received': None}
    for key, (topic, module, name) in STREAMS.items():
        msg_type = getattr(importlib.import_module(module), name)
        qos = 10 if key in ('detections', 'measurements') else qos_profile_sensor_data
        node.create_subscription(msg_type, topic, lambda _m, key=key: counts.__setitem__(key, counts[key] + 1), qos)
    node.create_subscription(LocalizationHealth, 'localization/health',
                             lambda m: health.update(state=m.state, received=time.monotonic()), 10)
    rows = []
    t0 = time.monotonic()
    next_tick = t0 + 1.0
    with open(args.output / 'seconds.jsonl', 'w') as out:
        while time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() >= next_tick:
                now = time.monotonic()
                age = None if health['received'] is None else round(now - health['received'], 2)
                row = {'t': round(now - t0, 1), 'utc': time.strftime('%H:%M:%S', time.gmtime()),
                       'health': health['state'], 'health_age_s': age, **counts}
                out.write(json.dumps(row) + '\n')
                out.flush()
                print(f"{row['utc']} health={row['health']}({age}s) det={counts['detections']} "
                      f"meas={counts['measurements']} cam={counts['colour']}/{counts['depth']} "
                      f"scan={counts['front']}/{counts['rear']}/{counts['merged']}", flush=True)
                rows.append(row)
                for key in counts:
                    counts[key] = 0
                next_tick += 1.0
    node.destroy_node()
    rclpy.shutdown()
    summary = summarize(rows)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    sys.exit(main())
