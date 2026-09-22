#!/usr/bin/env python3
"""Capture, isolate, and summarize the Isaac 6.1 LiDAR qualification.

Subcommands:

``capture`` records at least 500 live raw/merged scan triples and writes a
machine-readable contract report plus host/run provenance.

``skew`` launches the installed merger on isolated topics, publishes only a
namespaced odom TF and scans 10 ms apart while rotating, and proves the TF
remap, motion-compensation branch, rear contribution, and padded range.

``windows`` records consecutive RTX cloud azimuths (optionally the historical
four half-cloud streams). It is a focused diagnostic when sector coverage fails.

``matrix`` runs fresh simulator boots in the required condition order.
``plot`` renders saved raw scan evidence without requiring ROS or Isaac.

``summarize`` aggregates the 12 ``slam_quality_probe.py`` result JSON files.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import statistics
import subprocess
import sys
import threading
import time

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src/ridgeback_common'))

from ridgeback_common.lidar_contract import (  # noqa: E402
    FRONT_LIDAR_XY_YAW,
    MERGED_ANGLE_MAX,
    MERGED_ANGLE_MIN,
    MERGED_RANGE_MAX,
    MERGED_SCAN_BINS,
    RAW_ANGLE_INCREMENT,
    RAW_ANGLE_MIN,
    RAW_RANGE_MAX,
    RAW_RANGE_MIN,
    RAW_SCAN_BINS,
    REAR_LIDAR_XY_YAW,
)


COUNTER_PATTERNS = {
    'front': r'front=(\d+)',
    'paired': r'paired=(\d+)',
    'equal_stamp': r'equal_stamp=(\d+)',
    'motion_compensated': r'motion_compensated=(\d+)',
    'front_only': r'front_only\(no rear in tolerance\)=(\d+)',
    'rear_dropped': r'rear_dropped\(no odom TF\)=(\d+)',
    'tf_misses': r'tf_misses=(\d+)',
}


def stamp_ns(message) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def _command(command: list[str]) -> dict[str, object]:
    try:
        result = subprocess.run(
            command, cwd=REPO, capture_output=True, text=True, timeout=15,
            check=False)
        return {
            'command': command,
            'returncode': result.returncode,
            'stdout': result.stdout.strip(),
            'stderr': result.stderr.strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'command': command, 'error': str(exc)}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _host_sample():
    return {
        'utc': datetime.now(timezone.utc).isoformat(),
        'load_average': os.getloadavg(), 'cpu_count': os.cpu_count(),
        'gpu': _command(['nvidia-smi',
            '--query-gpu=name,utilization.gpu,memory.used,temperature.gpu',
            '--format=csv,noheader']),
        'gpu_processes': _command(['nvidia-smi',
            '--query-compute-apps=pid,process_name,used_gpu_memory',
            '--format=csv,noheader']),
    }


def _artifact_dir(requested: Path | None, label: str) -> Path:
    if requested is not None:
        path = requested
    else:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        path = REPO / 'artifacts/isaac-lidar-qualification' / f'{stamp}_{label}'
    path.mkdir(parents=True, exist_ok=False)
    return path


def _write_report(out_dir: Path, name: str, report: dict[str, object],
                  launch_args: list[str]) -> Path:
    report_path = out_dir / f'{name}.json'
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    (out_dir / 'working-tree.diff').write_text(
        _command(['git', 'diff', 'HEAD'])['stdout'] + '\n')
    source_paths = [Path(__file__).resolve(),
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/ust10lx_2d.json',
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/sensors.py',
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/ros_io.py',
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py',
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/worlds.py',
        REPO / 'src/ridgeback_autonomy_gz/sim/worlds/initial_test_world.sdf',
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/usd/worlds/initial_test_world.usda',
        REPO / 'src/ridgeback_autonomy_isaac/sim/isaac/usd/robots/ridgeback_r100/ridgeback_r100.usda',
        REPO / 'src/ridgeback_autonomy/ridgeback_autonomy/common/scan_merger_node.py',
        REPO / 'src/ridgeback_common/ridgeback_common/lidar_contract.py']
    provenance = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'launch_arguments': launch_args,
        'git_head': _command(['git', 'rev-parse', 'HEAD']),
        'git_status': _command(['git', 'status', '--short']),
        'host_uptime': _command(['uptime']),
        'gpu': _command([
            'nvidia-smi',
            '--query-gpu=name,driver_version,memory.used,memory.total,utilization.gpu',
            '--format=csv,noheader',
        ]),
        'isaac_version': _command([
            str(REPO / 'isaac_venv/bin/python3'), '-c',
            'import importlib.metadata as m; print(m.version("isaacsim"))',
        ]),
        'source_checksums': {str(path.relative_to(REPO)): _sha256(path)
                             for path in source_paths},
        'files': {str(path.relative_to(out_dir)): _sha256(path)
                  for path in out_dir.rglob('*') if path.is_file()},
    }
    (out_dir / 'manifest.json').write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + '\n')
    return report_path


def _metadata(scan) -> dict[str, object]:
    finite = np.asarray(scan.ranges, dtype=float)
    finite = finite[np.isfinite(finite)]
    return {
        'frame_id': scan.header.frame_id,
        'bins': len(scan.ranges),
        'angle_min': scan.angle_min,
        'angle_max': scan.angle_max,
        'angle_increment': scan.angle_increment,
        'range_min': scan.range_min,
        'range_max': scan.range_max,
        'finite_min': float(finite.min()) if finite.size else None,
        'finite_max': float(finite.max()) if finite.size else None,
    }


def _raw_points(scan, pose) -> np.ndarray:
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = (
        np.isfinite(ranges)
        & (ranges >= scan.range_min)
        & (ranges <= scan.range_max)
    )
    indices = np.nonzero(valid)[0]
    if not indices.size:
        return np.empty((0, 2))
    angles = scan.angle_min + indices * scan.angle_increment
    x = ranges[valid] * np.cos(angles)
    y = ranges[valid] * np.sin(angles)
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((px + c * x - s * y, py + s * x + c * y))


def _project(points: np.ndarray, increment: float = RAW_ANGLE_INCREMENT) -> np.ndarray:
    ranges = np.full(MERGED_SCAN_BINS, np.inf, dtype=float)
    if not points.size:
        return ranges
    radii = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    valid = (radii >= RAW_RANGE_MIN) & (radii <= MERGED_RANGE_MAX)
    radii, angles = radii[valid], angles[valid]
    bins = np.mod(np.round(
        (angles - MERGED_ANGLE_MIN) / increment).astype(int),
        MERGED_SCAN_BINS)
    np.minimum.at(ranges, bins, radii)
    return ranges


def _sector_hits(scans) -> list[int]:
    hits = np.zeros(6, dtype=int)
    edges = np.linspace(-135.0, 135.0, 7)
    for scan in scans:
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = np.degrees(
            scan.angle_min + np.arange(len(ranges)) * scan.angle_increment)
        finite = np.isfinite(ranges)
        for sector in range(6):
            upper_inclusive = sector == 5
            mask = (angles >= edges[sector]) & (
                angles <= edges[sector + 1] if upper_inclusive
                else angles < edges[sector + 1])
            hits[sector] += int(np.count_nonzero(finite & mask))
    return hits.tolist()


def _rate_hz(scans) -> float | None:
    stamps = sorted({stamp_ns(scan) for scan in scans})
    if len(stamps) < 2:
        return None
    deltas = np.diff(stamps) * 1e-9
    return float(1.0 / np.median(deltas)) if np.all(deltas > 0) else None


def _cadence(scans):
    deltas = np.diff([stamp_ns(scan) for scan in scans])
    return {
        # Floating simulator time converted to integer ROS timestamps can
        # alternate by a few ns. 10 ns is 0.00004% of the 25 ms period;
        # it cannot hide a lost scan or meaningful rate jitter.
        'regular_40hz': bool(deltas.size and np.all(np.abs(deltas - 25_000_000) <= 10)),
        'rounding_tolerance_ns': 10,
        'count': len(scans), 'delta_ns': deltas.tolist(),
        'non_increasing': int(np.count_nonzero(deltas <= 0)),
        'missing_40hz_periods': int(sum(max(0, round(value / 25_000_000) - 1)
                                      for value in deltas)),
        'min_ns': int(deltas.min()) if deltas.size else None,
        'max_ns': int(deltas.max()) if deltas.size else None,
    }


def _geometry_ok(meta: dict[str, object], *, merged: bool) -> bool:
    if merged:
        return all((
            meta['frame_id'] == 'base_link',
            meta['bins'] == MERGED_SCAN_BINS,
            math.isclose(meta['angle_min'], MERGED_ANGLE_MIN, abs_tol=1e-6),
            math.isclose(meta['angle_max'], MERGED_ANGLE_MAX, abs_tol=1e-6),
            math.isclose(meta['angle_increment'], RAW_ANGLE_INCREMENT, abs_tol=1e-8),
            math.isclose(meta['range_min'], RAW_RANGE_MIN, abs_tol=1e-6),
            math.isclose(meta['range_max'], MERGED_RANGE_MAX, abs_tol=1e-6),
        ))
    return all((
        meta['frame_id'] in ('lidar2d_0_laser', 'lidar2d_1_laser'),
        meta['bins'] == RAW_SCAN_BINS,
        math.isclose(meta['angle_min'], RAW_ANGLE_MIN, abs_tol=1e-6),
        math.isclose(meta['angle_max'], -RAW_ANGLE_MIN, abs_tol=1e-6),
        math.isclose(meta['angle_increment'], RAW_ANGLE_INCREMENT, abs_tol=1e-8),
        math.isclose(meta['range_min'], RAW_RANGE_MIN, abs_tol=1e-6),
        math.isclose(meta['range_max'], RAW_RANGE_MAX, abs_tol=1e-6),
    ))


def _parse_counters(message: str | None) -> dict[str, int] | None:
    if not message:
        return None
    counters = {}
    for name, pattern in COUNTER_PATTERNS.items():
        match = re.search(pattern, message)
        if match is None:
            return None
        counters[name] = int(match.group(1))
    return counters


def _counter_delta(start: dict[str, int] | None,
                   end: dict[str, int] | None) -> dict[str, int] | None:
    if start is None or end is None:
        return None
    return {name: end[name] - start[name] for name in COUNTER_PATTERNS}


def _paired_triples(scans: dict[str, list], limit: int | None = None):
    """Pair front/rear scans to merged output without assuming callback order."""
    by_stamp = {
        kind: {stamp_ns(message): message for message in values}
        for kind, values in scans.items()
    }
    front_stamps = sorted(set(by_stamp['front']) & set(by_stamp['merged']))
    rear_stamps = sorted(by_stamp['rear'])
    triples = []
    deltas_ns = []
    for front_stamp in front_stamps:
        if not rear_stamps:
            break
        position = bisect_left(rear_stamps, front_stamp)
        candidates = rear_stamps[max(0, position - 1):position + 1]
        rear_stamp = min(candidates, key=lambda stamp: abs(stamp - front_stamp))
        if abs(rear_stamp - front_stamp) > 20_000_000:
            continue
        triples.append((
            by_stamp['front'][front_stamp],
            by_stamp['rear'][rear_stamp],
            by_stamp['merged'][front_stamp],
        ))
        deltas_ns.append(front_stamp - rear_stamp)
        if limit is not None and len(triples) >= limit:
            break
    return triples, deltas_ns


def capture(args) -> int:
    from geometry_msgs.msg import TwistStamped
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    out_dir = _artifact_dir(args.out, 'capture')
    ros_log_dir = out_dir / 'ros-logs'
    ros_log_dir.mkdir()
    os.environ.setdefault('ROS_LOG_DIR', str(ros_log_dir))
    rclpy.init(args=[])
    node = rclpy.create_node(
        'lidar_qualification_capture', namespace=args.namespace,
        parameter_overrides=[rclpy.parameter.Parameter(
            'use_sim_time', value=True)])
    scans = {'front': [], 'rear': [], 'merged': []}
    ground_truth_samples = []
    merger_logs = []
    state = {'collecting': False}

    def keep(kind):
        def callback(message):
            if state['collecting']:
                scans[kind].append(message)
        return callback

    node.create_subscription(
        LaserScan, 'sensors/lidar2d_0/scan', keep('front'),
        qos_profile_sensor_data)
    node.create_subscription(
        LaserScan, 'sensors/lidar2d_1/scan', keep('rear'),
        qos_profile_sensor_data)
    node.create_subscription(
        LaserScan, 'sensors/scan_slam_merged', keep('merged'),
        qos_profile_sensor_data)

    def on_ground_truth(message):
        if state['collecting']:
            ground_truth_samples.append({
                'stamp_ns': stamp_ns(message),
                'x': message.pose.position.x,
                'y': message.pose.position.y,
                'yaw': _yaw(message.pose.orientation),
            })

    node.create_subscription(
        PoseStamped, 'ground_truth/pose', on_ground_truth, 50)

    def on_log(message):
        if 'scan_merger:' in message.msg:
            merger_logs.append(message.msg)

    node.create_subscription(Log, '/rosout', on_log, 100)
    cmd_pub = node.create_publisher(TwistStamped, 'cmd_vel', 10)
    host_samples = []
    stop_sampling = threading.Event()

    def sample_host():
        while not stop_sampling.is_set():
            host_samples.append(_host_sample())
            stop_sampling.wait(5.0)

    sampler = threading.Thread(target=sample_host, daemon=True)
    sampler.start()

    # Establish a post-startup counter baseline before collecting evidence.
    baseline_deadline = time.monotonic() + min(args.timeout, 15.0)
    while time.monotonic() < baseline_deadline and not merger_logs:
        rclpy.spin_once(node, timeout_sec=0.1)
    baseline_log = merger_logs[-1] if merger_logs else None
    baseline_counters = _parse_counters(baseline_log)
    state['collecting'] = True

    started = time.monotonic()
    last_pair_check_count = 0
    triples = []
    deltas_ns = []
    while time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        command = TwistStamped()
        command.header.stamp = node.get_clock().now().to_msg()
        command.twist.angular.z = args.angular_velocity
        cmd_pub.publish(command)
        common_count = min(len(values) for values in scans.values())
        if (common_count >= args.pairs
                and common_count >= last_pair_check_count + 20):
            last_pair_check_count = common_count
            triples, deltas_ns = _paired_triples(scans, args.pairs)
            if len(triples) >= args.pairs:
                break

    state['collecting'] = False
    stop = TwistStamped()
    for _ in range(3):
        cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    # Capture a counter snapshot after the measured interval. The simulator
    # remains live, but scan callbacks are frozen so the evidence set stays
    # exactly the requested matched triples.
    baseline_log_count = len(merger_logs)
    summary_deadline = time.monotonic() + 6.0
    while (time.monotonic() < summary_deadline
           and len(merger_logs) == baseline_log_count):
        rclpy.spin_once(node, timeout_sec=0.1)
    final_log = merger_logs[-1] if merger_logs else None
    counters = _parse_counters(final_log)
    counter_delta = _counter_delta(baseline_counters, counters)
    stop_sampling.set()
    sampler.join(timeout=16)
    (out_dir / 'host-samples.json').write_text(json.dumps(host_samples, indent=2))
    if args.gt_grid:
        shutil.copy2(args.gt_grid, out_dir / 'gt.npz')
    if args.launch_log:
        shutil.copy2(args.launch_log, out_dir / 'launch.log')

    # Preserve every received scan, including unmatched messages. Metadata is
    # per message; dense range arrays stay in a compact, lossless NPZ artifact.
    raw_arrays = {}
    raw_metadata = {}
    for kind, messages in scans.items():
        raw_metadata[kind] = [_metadata(message) for message in messages]
        raw_arrays[f'{kind}_stamps_ns'] = np.asarray(
            [stamp_ns(message) for message in messages], dtype=np.int64)
        for index, message in enumerate(messages):
            raw_arrays[f'{kind}_ranges_{index}'] = np.asarray(
                message.ranges, dtype=np.float32)
    np.savez_compressed(out_dir / 'raw-scans.npz', **raw_arrays)
    (out_dir / 'raw-metadata.json').write_text(json.dumps(raw_metadata, indent=2))
    (out_dir / 'ground-truth.json').write_text(json.dumps(ground_truth_samples, indent=2))
    (out_dir / 'merger.log').write_text('\n'.join(merger_logs) + '\n')
    (out_dir / 'matched-stamps.json').write_text(json.dumps([
        [stamp_ns(front), stamp_ns(rear), stamp_ns(merged)]
        for front, rear, merged in triples], indent=2))
    matched = {kind: [triple[index] for triple in triples]
               for index, kind in enumerate(('front', 'rear', 'merged'))}

    unique_front = unique_rear = collisions = mismatches = 0
    reconstructed_pairs = 0
    for front, rear, merged in triples:
        if stamp_ns(front) != stamp_ns(rear):
            continue
        reconstructed_pairs += 1
        expected_front = _project(
            _raw_points(front, FRONT_LIDAR_XY_YAW), front.angle_increment)
        expected_rear = _project(
            _raw_points(rear, REAR_LIDAR_XY_YAW), front.angle_increment)
        actual = np.asarray(merged.ranges, dtype=float)
        front_only = np.isfinite(expected_front) & ~np.isfinite(expected_rear)
        rear_only = np.isfinite(expected_rear) & ~np.isfinite(expected_front)
        both = np.isfinite(expected_front) & np.isfinite(expected_rear)
        unique_front += int(np.count_nonzero(
            front_only & np.isclose(actual, expected_front, atol=2e-4)))
        unique_rear += int(np.count_nonzero(
            rear_only & np.isclose(actual, expected_rear, atol=2e-4)))
        expected_near = np.minimum(expected_front, expected_rear)
        collisions += int(np.count_nonzero(
            both & np.isclose(actual, expected_near, atol=2e-4)))
        expected = np.minimum(expected_front, expected_rear)
        expected_finite = np.isfinite(expected)
        mismatches += int(np.count_nonzero(
            (expected_finite != np.isfinite(actual))
            | (expected_finite & ~np.isclose(actual, expected, atol=2e-4))))

    first = {kind: _metadata(values[0]) if values else None
             for kind, values in scans.items()}
    rates = {kind: _rate_hz(values) for kind, values in matched.items()}
    sectors = {
        'front': _sector_hits(matched['front']),
        'rear': _sector_hits(matched['rear']),
    }
    range_violations = {}
    for kind, values in matched.items():
        violations = 0
        for message in values:
            ranges = np.asarray(message.ranges, dtype=float)
            finite = ranges[np.isfinite(ranges)]
            violations += int(np.count_nonzero(
                (finite < message.range_min) | (finite > message.range_max)))
        range_violations[kind] = violations

    all_equal_stamps = bool(deltas_ns) and all(delta == 0 for delta in deltas_ns)
    ground_truth_yaws = [sample['yaw'] for sample in ground_truth_samples]
    yaw_span = (float(np.ptp(np.unwrap(ground_truth_yaws)))
                if len(ground_truth_yaws) >= 2 else 0.0)
    gates = {
        'enough_pairs': len(triples) >= max(500, args.pairs),
        **{f'{kind}_geometry': bool(messages) and all(
            _geometry_ok(_metadata(message), merged=kind == 'merged')
            and message.header.frame_id == {
                'front': 'lidar2d_0_laser', 'rear': 'lidar2d_1_laser',
                'merged': 'base_link'}[kind] for message in messages)
           for kind, messages in scans.items()},
        'rates_40hz': all(
            rate is not None and abs(rate - 40.0) <= 0.2
            for rate in rates.values()),
        'regular_40hz_cadence': all(
            _cadence(values)['regular_40hz']
            for values in matched.values()),
        'all_raw_sectors_observed': all(
            value > 0 for values in sectors.values() for value in values),
        'controlled_rotation_observed': yaw_span >= 2.0,
        'both_sensors_contribute': unique_front > 0 and unique_rear > 0,
        'independent_merge_matches': mismatches == 0 and reconstructed_pairs == len(triples),
        'ranges_within_declared_limits': all(
            value == 0 for value in range_violations.values()),
        'merger_health_counters': bool(
            counters
            and counters['front_only'] == 0
            and counters['rear_dropped'] == 0
            and counters['tf_misses'] == 0),
        'timestamp_branch_accounting': bool(
            counters and counter_delta and counter_delta['paired'] >= len(triples)
            and (not all_equal_stamps or (
                counters['paired'] == counters['equal_stamp']
                and counters['motion_compensated'] == 0))),
    }
    report = {
        'kind': 'live_capture',
        'requested_pairs': args.pairs,
        'received': {kind: len(values) for kind, values in scans.items()},
        'paired_triples': len(triples),
        'metadata': first,
        'rates_hz_sim_time': rates,
        'cadence': {kind: _cadence(values) for kind, values in matched.items()},
        'sector_finite_hits': sectors,
        'sector_finite_hits_first_50': {
            kind: _sector_hits(scans[kind][:50]) for kind in ('front', 'rear')
        },
        'sector_finite_hits_last_50': {
            kind: _sector_hits(scans[kind][-50:]) for kind in ('front', 'rear')
        },
        'ground_truth_start': ground_truth_samples[0]
        if ground_truth_samples else None,
        'ground_truth_end': ground_truth_samples[-1]
        if ground_truth_samples else None,
        'stamp_delta_ns': {
            'values': deltas_ns,
            'count': len(deltas_ns),
            'zero_count': sum(delta == 0 for delta in deltas_ns),
            'min': min(deltas_ns) if deltas_ns else None,
            'max': max(deltas_ns) if deltas_ns else None,
            'mean': statistics.fmean(deltas_ns) if deltas_ns else None,
        },
        'contributions': {
            'reconstructed_pairs': reconstructed_pairs,
            'unique_front_matches': unique_front,
            'unique_rear_matches': unique_rear,
            'collision_nearest_matches': collisions,
            'expected_finite_mismatches': mismatches,
        },
        'range_violations': range_violations,
        'controlled_angular_velocity_rad_s': args.angular_velocity,
        'ground_truth_yaw_span_rad': yaw_span,
        'merger_baseline_log': baseline_log,
        'merger_log': final_log,
        'merger_counter_baseline': baseline_counters,
        'merger_counters': counters,
        'merger_counter_delta': counter_delta,
        'gates': gates,
        'passed': all(gates.values()),
    }
    report_path = _write_report(out_dir, 'capture', report, args.launch_arg)
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f'capture report: {report_path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


def _make_scan(stamp: float, distance: float):
    from sensor_msgs.msg import LaserScan
    message = LaserScan()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    message.angle_min = 0.0
    message.angle_max = 0.0
    message.angle_increment = RAW_ANGLE_INCREMENT
    message.scan_time = 0.025
    message.range_min = RAW_RANGE_MIN
    message.range_max = RAW_RANGE_MAX
    message.ranges = [distance]
    return message


def _make_tf(stamp: float, yaw: float):
    from geometry_msgs.msg import TransformStamped
    transform = TransformStamped()
    transform.header.stamp.sec = int(stamp)
    transform.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def skew(args) -> int:
    import rclpy
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
    )
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    out_dir = _artifact_dir(args.out, 'skew')
    ros_log_dir = out_dir / 'ros-logs'
    ros_log_dir.mkdir()
    os.environ.setdefault('ROS_LOG_DIR', str(ros_log_dir))
    namespace = 'lidar_qualification'
    command = [
        'ros2', 'run', 'ridgeback_autonomy', 'scan_merger_node', '--ros-args',
        '-r', f'__ns:=/{namespace}',
        '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
        '-p', 'front_topic:=probe/front', '-p', 'rear_topic:=probe/rear',
        '-p', 'output_topic:=probe/merged', '-p', 'log_period_sec:=1.0',
    ]
    process = subprocess.Popen(
        command, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('scan_merger_skew_probe')
    tf_qos = QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)
    tf_pub = node.create_publisher(TFMessage, f'/{namespace}/tf', tf_qos)
    front_pub = node.create_publisher(
        LaserScan, f'/{namespace}/probe/front', qos_profile_sensor_data)
    rear_pub = node.create_publisher(
        LaserScan, f'/{namespace}/probe/rear', qos_profile_sensor_data)
    outputs = []
    node.create_subscription(
        LaserScan, f'/{namespace}/probe/merged', outputs.append,
        qos_profile_sensor_data)

    started = time.monotonic()
    while time.monotonic() - started < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (front_pub.get_subscription_count() > 0
                and rear_pub.get_subscription_count() > 0):
            break

    global_tf_publishers = node.get_publishers_info_by_topic('/tf')
    t_rear, t_front = 10.0, 10.01
    transforms = TFMessage(transforms=[
        _make_tf(t_rear, 0.0), _make_tf(t_front, 0.01),
    ])
    for _ in range(20):
        tf_pub.publish(transforms)
        rclpy.spin_once(node, timeout_sec=0.02)
    rear_pub.publish(_make_scan(t_rear, 3.0))
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.02)
    front_pub.publish(_make_scan(t_front, 10.0))

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not outputs:
        rclpy.spin_once(node, timeout_sec=0.05)
    # Allow the merger's one-second summary timer to report branch counters.
    timer_deadline = time.monotonic() + 1.3
    while time.monotonic() < timer_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    os.killpg(process.pid, signal.SIGINT)
    try:
        merger_log, _ = process.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            merger_log, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            merger_log, _ = process.communicate(timeout=5)

    output = outputs[-1] if outputs else None
    finite_indices = []
    finite_values = []
    if output is not None:
        ranges = np.asarray(output.ranges, dtype=float)
        finite_indices = np.nonzero(np.isfinite(ranges))[0].tolist()
        finite_values = ranges[np.isfinite(ranges)].tolist()
    expected_index = int(round(
        ((math.pi - 0.01) - MERGED_ANGLE_MIN) / RAW_ANGLE_INCREMENT
    )) % MERGED_SCAN_BINS
    counters = None
    lines = [line for line in merger_log.splitlines() if 'scan_merger:' in line]
    if lines:
        counters = _parse_counters(lines[-1])
    gates = {
        'no_global_tf_publishers': len(global_tf_publishers) == 0,
        'output_received': output is not None,
        'rear_contribution_retained': (
            expected_index in finite_indices
            and math.isclose(float(output.ranges[expected_index]), 3.3922,
                             abs_tol=2e-4)),
        'maximum_sensor_ray_retained': bool(
            output and len(finite_indices) == 2
            and math.isclose(float(output.ranges[720]), MERGED_RANGE_MAX,
                             abs_tol=1e-6)
            and output.ranges[720] <= output.range_max),
        'range_max_padded': bool(
            output and math.isclose(output.range_max, MERGED_RANGE_MAX, abs_tol=1e-6)),
        'motion_compensation_counted': bool(
            counters
            and counters['motion_compensated'] >= 1
            and counters['equal_stamp'] == 0),
        'tf_remap_healthy': bool(
            counters
            and counters['tf_misses'] == 0
            and counters['rear_dropped'] == 0),
    }
    report = {
        'kind': 'namespaced_tf_skew',
        'delta_ms': 10.0,
        'rotation_rad': 0.01,
        'expected_finite_index': expected_index,
        'finite_indices': finite_indices,
        'finite_values': finite_values,
        'output_range_max': output.range_max if output else None,
        'merger_counters': counters,
        'merger_log': merger_log,
        'gates': gates,
        'passed': all(gates.values()),
    }
    report_path = _write_report(out_dir, 'skew', report, args.launch_arg)
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f'skew report: {report_path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


def windows(args) -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    out_dir = _artifact_dir(args.out, 'windows')
    ros_log_dir = out_dir / 'ros-logs'
    ros_log_dir.mkdir()
    os.environ.setdefault('ROS_LOG_DIR', str(ros_log_dir))
    rclpy.init()
    node = rclpy.create_node('lidar_half_window_probe', namespace=args.namespace)
    observations = defaultdict(list)

    def callback(name):
        def receive(message):
            if len(observations[name]) >= args.samples:
                return
            points = point_cloud2.read_points(
                message, field_names=('x', 'y'), skip_nans=True)
            x = np.asarray(points['x'], dtype=float)
            y = np.asarray(points['y'], dtype=float)
            valid = np.hypot(x, y) >= RAW_RANGE_MIN * 0.5
            angles = np.degrees(np.arctan2(y[valid], x[valid]))
            observations[name].append({
                'stamp_ns': stamp_ns(message),
                'points': int(angles.size),
                'message_points': message.width * message.height,
                'azimuths_deg': angles.tolist(),
                'azimuth_min_deg': float(angles.min()) if angles.size else None,
                'azimuth_max_deg': float(angles.max()) if angles.size else None,
                'sector_counts': [
                    int(np.count_nonzero((angles >= low) & (angles < high)))
                    for low, high in zip(
                        np.linspace(-180.0, 180.0, 9)[:-1],
                        np.linspace(-180.0, 180.0, 9)[1:])
                ],
            })
        return receive

    suffixes = (('', 'primary'), ('_l', 'offset')) if args.legacy_halves else (('', 'primary'),)
    expected_streams = 2 * len(suffixes)
    for index in (0, 1):
        for suffix, label in suffixes:
            name = f'lidar{index}_{label}'
            node.create_subscription(
                PointCloud2, f'sensors/lidar2d_{index}/points{suffix}',
                callback(name), qos_profile_sensor_data)

    deadline = time.monotonic() + args.timeout
    while (time.monotonic() < deadline and not (
            len(observations) == expected_streams
            and all(len(values) >= args.samples for values in observations.values()))):
        rclpy.spin_once(node, timeout_sec=0.1)
    report = {
        'kind': 'rtx_half_windows',
        'observations': observations,
        'requested_samples_per_stream': args.samples,
        'passed': len(observations) == expected_streams and all(
            len(values) >= args.samples for values in observations.values()),
    }
    report_path = _write_report(out_dir, 'windows', report, [])
    print(json.dumps({name: [{key: value for key, value in sample.items()
                             if key != 'azimuths_deg'} for sample in samples]
                      for name, samples in observations.items()}, indent=2))
    print(f'window report: {report_path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


METRIC_PATHS = {
    'pose_rmse_m': ('pose_rmse_m',),
    'pose_max_m': ('pose_max_m',),
    'loop_error_mean_m': ('loop_error_mean_m',),
    'loop_error_max_m': ('loop_error_max_m',),
    'map_iou': ('map_iou',),
    'wall_precision': ('wall_precision',),
    'wall_recall': ('wall_recall',),
    'yaw_ekf_rms_deg': ('yaw_ekf_phi_deg', 'rms'),
    'yaw_slam_rms_deg': ('yaw_slam_mo_deg', 'rms'),
    'tf_correction_events': ('tf_correction_events',),
    'tf_max_jump_m': ('tf_max_jump_m',),
    'drive_duration_s': ('drive_duration_s',),
}


def _nested(record, path):
    value = record
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _stop_process(process):
    if process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def matrix(args) -> int:
    """Fresh, serial simulator boots; stop on the first failed trial."""
    pilot_load = getattr(args, 'pilot_load', None)
    shared_pilot = pilot_load == 'shared'
    if not os.environ.get('ROS_DOMAIN_ID'):
        raise ValueError('Set a dedicated ROS_DOMAIN_ID before running the matrix')
    gpu_query = ['nvidia-smi', '--query-compute-apps=pid,process_name,used_gpu_memory',
                 '--format=csv,noheader']
    gpu_before = _command(gpu_query)
    if gpu_before.get('returncode') != 0 or (gpu_before.get('stdout') and not shared_pilot):
        raise RuntimeError(f'Qualification requires an idle GPU: {gpu_before}')
    if shared_pilot and not gpu_before.get('stdout'):
        raise RuntimeError('Shared-load pilot requires an observed competing GPU process')
    existing = _command(['ros2', 'node', 'list', '--no-daemon'])
    if existing.get('returncode') != 0 or existing.get('stdout'):
        raise RuntimeError(f'Qualification domain is not empty: {existing}')
    out = _artifact_dir(args.out, 'matrix')
    shutil.copy2(args.gt_grid, out / 'gt.npz')
    conditions = [(noise, seed, source) for noise in (0.0, 1.0)
                  for seed in (0, 1, 2) for source in ('front_only', 'merged')]
    if args.condition:
        noise, seed, source = args.condition.split('/')
        condition = (float(noise), int(seed), source)
        if condition not in conditions:
            raise ValueError('condition must be noise/seed/source from the matrix')
        conditions = [condition]
    elif args.start_at:
        noise, seed, source = args.start_at.split('/')
        conditions = conditions[conditions.index((float(noise), int(seed), source)):]
    for noise, seed, source in conditions:
        tag = f'noise{noise:g}_seed{seed}_{source}'
        trial = out / tag
        trial.mkdir()
        launch_args = [
            'backend:=isaac', 'world:=initial_test_world', 'sim_mode:=deterministic',
            f'rtf:={0.5 if pilot_load else 1.0}', 'camera:=false', 'target_localization_enabled:=false',
            'exploration_rviz:=false', 'coverage_overlay_enabled:=false',
            'autonomous_motion_enabled:=false', f'odom_noise:={noise}',
            f'noise_seed:={seed}', f'slam_source:={source}']
        env = dict(os.environ, ROS_LOG_DIR=str(trial.resolve() / 'ros-logs'))
        print(f'START {tag}', flush=True)
        launch = probe = None
        samples = []
        invalid_reason = None
        try:
            with (trial / 'launch.log').open('w') as launch_log, \
                    (trial / 'probe.log').open('w') as probe_log:
                launch = subprocess.Popen([
                    'ros2', 'launch', 'ridgeback_autonomy',
                    'ridgeback_exploration.launch.py', *launch_args],
                    cwd=REPO, env=env, stdout=launch_log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                probe = subprocess.Popen([
                    sys.executable, '-u', str(REPO / 'tools/isaac/slam_quality_probe.py'),
                    '--tag', tag, '--gt-grid', str((out / 'gt.npz').resolve()),
                    '--out', str(trial.resolve()), '--slam-log', str((trial / 'launch.log').resolve()),
                    '--slam-source', source, '--odom-noise', str(noise),
                    '--noise-seed', str(seed)],
                    cwd=REPO, env=env, stdout=probe_log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + args.trial_timeout
                while probe.poll() is None and launch.poll() is None:
                    samples.append(_host_sample())
                    gpu_processes = samples[-1]['gpu_processes']
                    if (gpu_processes.get('returncode') != 0
                            or (not shared_pilot and
                                len(gpu_processes.get('stdout', '').splitlines()) > 1)):
                        invalid_reason = 'GPU contention or unavailable GPU process evidence'
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f'{tag}: trial timeout')
                    time.sleep(5)
                code = probe.poll() if invalid_reason is None else 1
        finally:
            if probe is not None:
                _stop_process(probe)
            if launch is not None:
                _stop_process(launch)
            (trial / 'host-samples.json').write_text(json.dumps(samples, indent=2))
            _write_report(trial, 'attempt', {
                'condition': tag, 'gt_sha256': _sha256(out / 'gt.npz'),
                'purpose': 'timing_pilot' if pilot_load else 'qualification',
                'requested_load': pilot_load,
                'target_rtf': 0.5 if pilot_load else 1.0,
                'gpu_before': gpu_before,
                'invalid_reason': invalid_reason,
                'probe_returncode': probe.returncode if probe else None,
                'ros_domain_id': os.environ['ROS_DOMAIN_ID']}, launch_args)
        print(f'END {tag}: probe_returncode={code}', flush=True)
        if code != 0:
            return 1
    return 0


def summarize(args) -> int:
    files = sorted({path for pattern in args.metrics for path in REPO.glob(pattern)})
    records = []
    for path in files:
        record = json.loads(path.read_text())
        host_path = path.parent / 'host-samples.json'
        attempt_path = path.parent / 'attempt.json'
        if host_path.exists():
            samples = json.loads(host_path.read_text())
            if any(len(sample.get('gpu_processes', {}).get('stdout', '').splitlines()) > 1
                   for sample in samples):
                record['aborted'] = True
                record['invalid_reason'] = 'Observed GPU contention'
        if attempt_path.exists():
            attempt = json.loads(attempt_path.read_text())
            if attempt.get('invalid_reason') or attempt.get('purpose') == 'timing_pilot':
                record['aborted'] = True
        record['_path'] = str(path.relative_to(REPO))
        records.append(record)
    groups = defaultdict(list)
    for record in records:
        key = (record.get('odom_noise'), record.get('slam_source'))
        groups[key].append(record)

    summary = {}
    complete = len(records) == 12 and set(groups) == {
        (noise, source) for noise in (0.0, 1.0) for source in ('front_only', 'merged')}
    for noise in (0.0, 1.0):
        for source in ('front_only', 'merged'):
            key = (noise, source)
            group = groups.get(key, [])
            label = f'noise_{noise:g}/{source}'
            seeds = sorted(record.get('noise_seed', -1) for record in group)
            valid = (
                len(group) == 3
                and seeds == [0, 1, 2]
                and all(not record.get('aborted', True) for record in group)
            )
            metrics = {}
            for name, path in METRIC_PATHS.items():
                values = [_nested(record, path) for record in group
                          if not record.get('aborted', True)]
                values = [float(value) for value in values
                          if isinstance(value, (float, int)) and math.isfinite(value)]
                if len(values) != 3:
                    valid = False
                    metrics[name] = None
                else:
                    metrics[name] = {
                        'min': min(values),
                        'median': statistics.median(values),
                        'max': max(values),
                    }
            summary[label] = {
                'valid': valid, 'seeds': seeds,
                'files': [record['_path'] for record in group],
                'metrics': metrics,
            }
            complete &= valid

    paired = {}
    by_key = {
        (record.get('odom_noise'), record.get('noise_seed'), record.get('slam_source')):
        record for record in records
    }
    for noise in (0.0, 1.0):
        deltas = defaultdict(list)
        for seed in (0, 1, 2):
            front = by_key.get((noise, seed, 'front_only'))
            merged = by_key.get((noise, seed, 'merged'))
            if (not front or not merged or front.get('aborted', True)
                    or merged.get('aborted', True)):
                continue
            for name, path in METRIC_PATHS.items():
                a, b = _nested(front, path), _nested(merged, path)
                if (isinstance(a, (float, int)) and isinstance(b, (float, int))
                        and math.isfinite(a) and math.isfinite(b)):
                    deltas[name].append(float(b) - float(a))
        paired[f'noise_{noise:g}'] = {
            name: {
                'values_by_seed': values,
                'min': min(values),
                'median': statistics.median(values),
                'max': max(values),
            }
            for name, values in deltas.items() if len(values) == 3
        }
        complete &= all(len(values) == 3 for values in deltas.values())

    report = {
        'kind': 'slam_quality_matrix',
        'input_files': [str(path.relative_to(REPO)) for path in files],
        'conditions': summary,
        'paired_merged_minus_front': paired,
        'passed': complete,
    }
    out_dir = _artifact_dir(args.out, 'summary')
    report_path = _write_report(out_dir, 'slam_matrix_summary', report, [])
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f'matrix summary: {report_path}')
    return 0 if complete else 1


def summary_plot(args) -> int:
    """Plot all accepted seeds; never select a visually favorable trial."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    summary = json.loads(args.summary.read_text())
    if not summary.get('passed'):
        raise ValueError('Only a complete accepted matrix can be plotted')
    records = [json.loads((REPO / path).read_text()) for path in summary['input_files']]
    metrics = [('pose_rmse_m', 'Pose RMSE (m)'),
               ('loop_error_mean_m', 'Mean loop error (m)'),
               ('map_iou', 'Map IoU'), ('wall_precision', 'Wall precision'),
               ('yaw_slam_rms_deg', 'SLAM yaw RMS (degrees)'),
               ('tf_correction_events', 'TF correction events')]
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    for ax, (metric, label) in zip(axes.flat, metrics):
        for noise, color in ((0.0, '#1673b8'), (1.0, '#db6b18')):
            for seed in (0, 1, 2):
                values = [next(_nested(record, METRIC_PATHS[metric])
                               for record in records
                               if record['odom_noise'] == noise
                               and record['noise_seed'] == seed
                               and record['slam_source'] == source)
                          for source in ('front_only', 'merged')]
                offset = -.035 if noise == 0 else .035
                ax.plot([offset, 1 + offset], values, '-o', color=color,
                        alpha=.7, linewidth=1.3,
                        label=f'Noise {noise:g}' if seed == 0 else None)
        ax.set(title=label, xticks=[0, 1], xticklabels=['Front only', 'Merged'],
               xlim=(-.25, 1.25))
        ax.grid(axis='y', alpha=.2)
    axes[0, 0].legend()
    fig.suptitle('Isaac 6.1: 12 fresh-boot trials\n'
                 'Each line pairs the same noise seed (0, 1, or 2)', fontsize=14)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    plt.close(fig)
    return 0


def contract_plot(args) -> int:
    """Render current geometry, not a benchmark result."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, Rectangle

    fig, (ax, ray) = plt.subplots(1, 2, figsize=(12, 4.6),
                                 gridspec_kw={'width_ratios': [1, 1.25]},
                                 constrained_layout=True)
    colors = ('#1673b8', '#db6b18')
    ax.add_patch(Rectangle((-.48, -.35), .96, .7, facecolor='#e8edf2',
                           edgecolor='#667788'))
    for pose, color, label in zip((FRONT_LIDAR_XY_YAW, REAR_LIDAR_XY_YAW),
                                  colors, ('Front', 'Rear')):
        x, y, yaw = pose
        start = math.degrees(yaw) - 135
        ax.add_patch(Arc((x, y), 2.3, 2.3, theta1=start,
                         theta2=start + 270, color=color, linewidth=3))
        for angle in (start, start + 270):
            a = math.radians(angle)
            ax.plot([x, x + 1.15 * math.cos(a)],
                    [y, y + 1.15 * math.sin(a)], ':', color=color)
        ax.scatter([x], [y], c=color, s=55, zorder=5)
        ax.text(x, -.52 if x > 0 else .48, label, color=color,
                ha='center', weight='bold')
    ax.scatter([0], [0], c='#172b4d', marker='+', s=90, zorder=6)
    ax.annotate('+x / forward', xy=(1.65, 0), xytext=(.8, .15),
                arrowprops={'arrowstyle': '->'}, fontsize=9)
    ax.set(xlim=(-1.75, 1.9), ylim=(-1.5, 1.5), aspect='equal',
           title='Two 270° raw scan windows\n1081 bins each · 0.25° spacing')
    ax.text(0, -1.4, 'Top view; arc radius is illustrative', ha='center', fontsize=9)
    ax.axis('off')
    for x, color in ((0, '#172b4d'), (.3922, colors[0]), (10.3922, '#172b4d')):
        ray.plot([x, x], [-.1, .1], color=color, linewidth=2)
    ray.plot([0, 10.3922], [0, 0], color=colors[0], linewidth=2)
    ray.scatter([.3922, 10.3922], [0, 0], c=[colors[0], '#172b4d'], s=45)
    ray.annotate('base_link', xy=(0, 0), xytext=(-.3, -.7),
                 arrowprops={'arrowstyle': '-'}, ha='center')
    ray.annotate('Front sensor\nx = +0.3922 m', xy=(.3922, 0), xytext=(2.2, -.7),
                 arrowprops={'arrowstyle': '-'}, ha='center', color=colors[0])
    ray.text(10.3922, -.7, 'Return', ha='center')
    for start, y, label, color in (
            (.3922, .55, '10.0 m in sensor frame', colors[0]),
            (0, 1.25, '10.3922 m in base_link', '#172b4d')):
        ray.annotate('', xy=(10.3922, y), xytext=(start, y),
                     arrowprops={'arrowstyle': '<->', 'color': color})
        ray.text(5.2, y + .12, label, ha='center', color=color)
    ray.text(5, -1.45, 'Merged: 1440 bins · −180° to +179.75°\n'
             'SLAM range limit follows the selected source', ha='center')
    ray.set(xlim=(-1, 11.5), ylim=(-1.8, 2),
            title='Why the merged range limit is larger')
    ray.axis('off')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    plt.close(fig)
    return 0


def plot(args) -> int:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    folder = args.capture
    metadata = json.loads((folder / 'raw-metadata.json').read_text())
    with np.load(folder / 'raw-scans.npz') as arrays:
        fig = plt.figure(figsize=(13, 7), constrained_layout=True)
        for index, kind in enumerate(('front', 'rear', 'merged')):
            meta = metadata[kind][0]
            ranges = arrays[f'{kind}_ranges_0']
            angles = meta['angle_min'] + np.arange(len(ranges)) * meta['angle_increment']
            ax = fig.add_subplot(2, 3, index + 1, projection='polar')
            valid = np.isfinite(ranges)
            ax.scatter(angles[valid], ranges[valid], s=3)
            ax.set_ylim(0, 11)
            ax.set_title(f'{kind.capitalize()}: one current scan\n{meta["frame_id"]}')
            ax = fig.add_subplot(2, 3, index + 4)
            counts = np.sum([np.isfinite(arrays[f'{kind}_ranges_{i}'])
                             for i in range(len(metadata[kind]))], axis=0)
            ax.plot(np.degrees(angles), counts, linewidth=1)
            ax.set(xlabel='Bearing (degrees)', ylabel='Finite observations',
                   title=f'Coverage across {len(metadata[kind])} received scans',
                   ylim=(0, len(metadata[kind])),
                   xlim=(np.degrees(angles[0]), np.degrees(angles[-1])))
            ax.grid(alpha=0.25)
        fig.suptitle('Isaac 6.1 LiDAR — raw angular coverage and merged output')
        destination = args.out or folder / 'scan-coverage.png'
        fig.savefig(destination, dpi=160)
        plt.close(fig)
    print(destination)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)

    capture_parser = sub.add_parser('capture')
    capture_parser.add_argument('--namespace', default='r100_0001')
    capture_parser.add_argument('--pairs', type=int, default=500)
    capture_parser.add_argument('--timeout', type=float, default=60.0)
    capture_parser.add_argument('--angular-velocity', type=float, default=0.3)
    capture_parser.add_argument('--out', type=Path)
    capture_parser.add_argument('--launch-arg', action='append', default=[])
    capture_parser.add_argument('--gt-grid', type=Path)
    capture_parser.add_argument('--launch-log', type=Path)
    capture_parser.set_defaults(function=capture)

    skew_parser = sub.add_parser('skew')
    skew_parser.add_argument('--out', type=Path)
    skew_parser.add_argument('--launch-arg', action='append', default=[])
    skew_parser.set_defaults(function=skew)

    windows_parser = sub.add_parser('windows')
    windows_parser.add_argument('--namespace', default='r100_0001')
    windows_parser.add_argument('--timeout', type=float, default=15.0)
    windows_parser.add_argument('--samples', type=int, default=20)
    windows_parser.add_argument('--legacy-halves', action='store_true')
    windows_parser.add_argument('--out', type=Path)
    windows_parser.set_defaults(function=windows)

    matrix_parser = sub.add_parser('matrix')
    matrix_parser.add_argument('--gt-grid', type=Path, required=True)
    matrix_parser.add_argument('--out', type=Path)
    selection = matrix_parser.add_mutually_exclusive_group()
    selection.add_argument('--condition', help='optional single noise/seed/source trial')
    selection.add_argument('--start-at', help='resume the ordered matrix at noise/seed/source')
    matrix_parser.add_argument('--trial-timeout', type=float, default=900.0)
    matrix_parser.set_defaults(function=matrix)

    pilot_parser = sub.add_parser('timing-pilot')
    pilot_parser.add_argument('--gt-grid', type=Path, required=True)
    pilot_parser.add_argument('--out', type=Path, required=True)
    pilot_parser.add_argument('--load', dest='pilot_load', choices=('quiet', 'shared'), required=True)
    pilot_parser.add_argument('--trial-timeout', type=float, default=900.0)
    pilot_parser.set_defaults(function=matrix, condition='0/0/merged', start_at=None)

    plot_parser = sub.add_parser('plot')
    plot_parser.add_argument('capture', type=Path)
    plot_parser.add_argument('--out', type=Path)
    plot_parser.set_defaults(function=plot)

    contract_parser = sub.add_parser('contract-plot')
    contract_parser.add_argument('--out', type=Path, required=True)
    contract_parser.set_defaults(function=contract_plot)

    summary_plot_parser = sub.add_parser('summary-plot')
    summary_plot_parser.add_argument('summary', type=Path)
    summary_plot_parser.add_argument('--out', type=Path, required=True)
    summary_plot_parser.set_defaults(function=summary_plot)

    summary_parser = sub.add_parser('summarize')
    summary_parser.add_argument(
        'metrics', nargs='+',
        help='repo-relative glob(s) for slam_quality_probe metrics JSON files')
    summary_parser.add_argument('--out', type=Path)
    summary_parser.set_defaults(function=summarize)

    args = parser.parse_args()
    return args.function(args)


if __name__ == '__main__':
    raise SystemExit(main())
