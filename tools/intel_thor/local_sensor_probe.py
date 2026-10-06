#!/usr/bin/env python3
"""Read-only local DDS observer; never publishes or restarts robot services.

Source ROS Jazzy and select ROS_DOMAIN_ID=0 / the experiment DDS profile first.
Use a separate output directory per process. Examples::

  python3 tools/intel_thor/local_sensor_probe.py --mode scans \
    --output-dir artifacts/hardware/run/scans --duration 0 --stop-file /tmp/dds-stop
  python3 tools/intel_thor/local_sensor_probe.py --mode camera --topics color,depth \
    --name reader_1 --output-dir artifacts/hardware/run/reader_1 \
    --warmup 10 --duration 60 --stop-file /tmp/dds-stop

--duration excludes warmup; zero runs until SIGINT/SIGTERM. The scans process
creates the first-failure sentinel, but remains alive to capture recovery.
Camera processes poll it and terminate themselves. Existing sentinels are never
removed or overwritten. Baseline is a prior scan summary.json; interface
mean_tx_mbps values feed the traffic guard, and merged rate feeds acceptance.
All callbacks, including warmup, go to events.jsonl. Summary acceptance excludes
warmup, but includes missing-stream and end-of-window gaps. Separate service
journal/PID watchdogs and packet capture are required: this tool cannot detect
driver reconnect log errors or prove multicast destinations.

Image callbacks request serialized data and inspect only its CDR metadata;
pixel arrays are never deserialized or copied by this code. rclpy/RMW may still
copy the serialized sample before callback entry. Callback observer CPU time
and callback wall duration are measured, including JSON writing. These cannot
measure latency before callback entry. MessageInfo additionally records DDS
publication and RMW take wall timestamps; take is not packet arrival. Exact stamp sets grow with frame count
(about 54,000 entries per stream for a 30-minute, 30-Hz soak); arrival histories
are streamed to disk rather than retained in memory.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import time


INTERFACES = ('eno1', 'enp2s0', 'br0', 'wlp3s0', 'lo')


def image_callback(stream, handler):
    # Two required parameters are essential: rclpy first tries binding one
    # argument and otherwise silently omits MessageInfo for default parameters.
    def callback(message, message_info):
        handler(stream, message, message_info)
    return callback


def message_timing(message_info):
    """Cyclone 2.2.3 received_timestamp is RMW take time, not wire arrival."""
    return {key: message_info.get(key) for key in
            ('source_timestamp', 'received_timestamp',
             'publication_sequence_number', 'reception_sequence_number')}

TOPICS = {
    'front': 'sensors/lidar2d_0/scan',
    'rear': 'sensors/lidar2d_1/scan',
    'merged': 'sensors/scan',
    'color': 'camera/color/image_raw',
    'depth': 'camera/depth/image_rect_raw',
    'aligned': 'camera/aligned_depth_to_color/image_raw',
}


def image_metadata(serialized):
    """Parse sensor_msgs/Image XCDR1 metadata without visiting pixel data.

    CDR offsets align relative to the body, after its four-byte encapsulation.
    Explicitly reject other representations instead of assuming little endian.
    """
    data = memoryview(serialized)
    if len(data) < 4:
        raise ValueError('truncated CDR encapsulation')
    representation = int.from_bytes(data[:2], 'big')
    if representation not in (0, 1):
        raise ValueError(f'unsupported CDR representation {representation}')
    endian = '<' if representation == 1 else '>'
    offset = 4

    def scalar(fmt, alignment=4):
        nonlocal offset
        offset += (-(offset - 4)) % alignment
        size = struct.calcsize(fmt)
        if offset + size > len(data):
            raise ValueError('truncated Image metadata')
        value = struct.unpack_from(endian + fmt, data, offset)[0]
        offset += size
        return value

    def string():
        nonlocal offset
        size = scalar('I')
        if size < 1 or offset + size > len(data) or data[offset + size - 1] != 0:
            raise ValueError('invalid CDR string')
        value = bytes(data[offset:offset + size - 1]).decode('utf-8')
        offset += size
        return value

    sec, nanosec = scalar('i'), scalar('I')
    if nanosec >= 1_000_000_000:
        raise ValueError('invalid stamp nanoseconds')
    frame = string()
    height, width, encoding = scalar('I'), scalar('I'), string()
    is_bigendian, step, size = scalar('B', 1), scalar('I'), scalar('I')
    if offset + size != len(data):
        # Some serializers append up to three zero padding bytes.
        trailing = len(data) - offset - size
        if trailing < 0 or trailing > 3 or any(data[offset + size:]):
            raise ValueError('Image payload length does not match CDR sequence')
    return {
        'stamp_ns': sec * 1_000_000_000 + nanosec, 'frame_id': frame,
        'height': height, 'width': width, 'encoding': encoding,
        'is_bigendian': is_bigendian, 'step': step, 'data_bytes': size,
        'empty': not height or not width or not size or not step,
    }


class StreamMetrics:
    """Bounded arrival state plus unique stamps for one measurement window."""

    def __init__(self, start):
        self.start = start
        self.stamps = set()
        self.received = self.empty = self.duplicates = self.regressions = 0
        self.last_stamp = self.last_receive = None
        self.max_receive_gap = self.max_stamp_gap = 0.0

    def record(self, stamp_ns, receive, empty=False):
        if receive < self.start:
            return
        if empty:
            self.empty += 1
            return
        self.received += 1
        if stamp_ns in self.stamps:
            self.duplicates += 1
        self.stamps.add(stamp_ns)
        if self.last_stamp is not None:
            if stamp_ns < self.last_stamp:
                self.regressions += 1
            self.max_stamp_gap = max(
                self.max_stamp_gap, (stamp_ns - self.last_stamp) / 1e9)
        self.max_receive_gap = max(
            self.max_receive_gap,
            receive - (self.start if self.last_receive is None else self.last_receive))
        self.last_stamp, self.last_receive = stamp_ns, receive

    def summary(self, end, min_rate, gap_limit):
        duration = max(0.0, end - self.start)
        max_gap = max(self.max_receive_gap,
                      end - (self.start if self.last_receive is None else self.last_receive))
        rate = len(self.stamps) / duration if duration else 0.0
        passed = (duration > 0 and bool(self.stamps) and not self.empty
                  and not self.duplicates and not self.regressions
                  and rate >= min_rate and max_gap <= gap_limit
                  and self.max_stamp_gap <= gap_limit)
        return {
            'received': self.received, 'unique': len(self.stamps),
            'duration_s': duration, 'rate_hz': rate, 'empty': self.empty,
            'duplicates': self.duplicates, 'regressions': self.regressions,
            'max_receive_gap_s': max_gap, 'max_stamp_gap_s': self.max_stamp_gap,
            'min_rate_hz': min_rate, 'gap_limit_s': gap_limit, 'passed': passed,
        }


def exact_matches(left, right):
    count = len(left & right)
    return {'matched': count, 'color_ratio': count / len(left) if left else 0.0,
            'aligned_ratio': count / len(right) if right else 0.0,
            'passed': bool(left and right) and count / len(left) >= .99
            and count / len(right) >= .99}


def write_stop(path, reason):
    """Atomically preserve the first failure across independent watchdogs."""
    if path is None:
        return False
    try:
        with Path(path).open('x', encoding='utf-8') as stream:
            json.dump(reason, stream)
            stream.write('\n')
        return True
    except FileExistsError:
        return False


class TrafficGuard:
    def __init__(self, baseline):
        self.baseline = baseline
        self.consecutive = {}

    def sample(self, rates):
        failed = []
        for name in ('eno1', 'enp2s0'):
            if name not in rates or name not in self.baseline:
                continue
            excess = rates[name]['tx_mbps'] - self.baseline[name]
            self.consecutive[name] = self.consecutive.get(name, 0) + 1 if excess > 20 else 0
            if self.consecutive[name] >= 2:
                failed.append({'interface': name, 'incremental_tx_mbps': excess})
        return failed


def read_counters():
    values = {}
    for name in INTERFACES:
        directory = Path('/sys/class/net') / name / 'statistics'
        try:
            values[name] = {direction: int((directory / f'{direction}_bytes').read_text())
                            for direction in ('rx', 'tx')}
        except OSError:
            continue
    return values


def counter_rates(previous, current, elapsed):
    rates = {}
    if elapsed <= 0:
        return rates
    for name in previous.keys() & current.keys():
        delta = {key: current[name][key] - previous[name][key] for key in ('rx', 'tx')}
        if any(value < 0 for value in delta.values()):
            raise ValueError(f'network byte counter reset: {name}')
        rates[name] = {f'{key}_mbps': value * 8 / elapsed / 1e6
                       for key, value in delta.items()}
    return rates


def sensor_qos():
    """Compatible observer QoS; never requests reliable delivery or history."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def parser():
    result = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    result.add_argument('--mode', required=True, choices=('scans', 'camera'))
    result.add_argument('--output-dir', required=True, type=Path)
    result.add_argument('--namespace', default='r100_0160')
    result.add_argument('--name', default='local_sensor_probe')
    result.add_argument('--warmup', type=float, default=0)
    result.add_argument('--duration', type=float, default=60)
    result.add_argument('--discovery-grace', type=float, default=3)
    result.add_argument('--topics', default='color,depth')
    result.add_argument('--baseline', type=Path)
    result.add_argument('--stop-file', type=Path)
    for name, default in TOPICS.items():
        result.add_argument(f'--{name}-topic', default=default)
    result.add_argument('--color-info-topic', default='camera/color/camera_info')
    result.add_argument('--depth-info-topic', default='camera/depth/camera_info')
    result.add_argument('--aligned-info-topic', default='camera/aligned_depth_to_color/camera_info')
    return result


def run(args):
    # Import lazily so metric/CDR tests require no live ROS installation.
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CameraInfo, Image, LaserScan

    if args.warmup < 0 or args.duration < 0 or args.discovery_grace < 0:
        raise ValueError('durations must be nonnegative')
    selected = ('front', 'rear', 'merged') if args.mode == 'scans' else tuple(args.topics.split(','))
    if args.mode == 'camera' and (not selected or len(set(selected)) != len(selected)
                                  or any(name not in ('color', 'depth', 'aligned') for name in selected)):
        raise ValueError('camera topics must be distinct color/depth/aligned names')
    baseline = json.loads(args.baseline.read_text()) if args.baseline else {}
    baseline_tx = {name: entry['mean_tx_mbps']
                   for name, entry in baseline.get('interfaces', {}).items()}
    if args.baseline and args.mode == 'scans' and any(name not in baseline_tx for name in ('eno1', 'enp2s0')):
        raise ValueError('baseline must contain both physical port TX rates')
    traffic_guard = TrafficGuard(baseline_tx)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    events = (args.output_dir / 'events.jsonl').open('x', encoding='utf-8', buffering=1)
    start = time.monotonic()
    measurement_start = start + args.warmup
    deadline = measurement_start + args.duration if args.duration else math.inf
    metrics = {name: StreamMetrics(measurement_start) for name in selected}
    last_seen = {name: None for name in selected}
    failures = []
    metadata = {}
    callback_cost = {'count': 0, 'cpu_s': 0.0, 'wall_s': 0.0, 'max_wall_s': 0.0}
    interface_totals = {}
    stopped = False
    stop_reason = 'duration'

    def emit(kind, **fields):
        events.write(json.dumps({'event': kind, 'monotonic_s': time.monotonic(),
                                 'wall_ns': time.time_ns(), **fields}) + '\n')

    def fail(reason, **fields):
        if any(item['reason'] == reason and item.get('stream') == fields.get('stream')
               and item.get('interface') == fields.get('interface') for item in failures):
            return
        item = {'reason': reason, 'monotonic_s': time.monotonic(), **fields}
        failures.append(item)
        emit('failure', **item)
        if args.mode == 'scans':
            write_stop(args.stop_file, item)

    def on_image(name, serialized, message_info):
        receive, cpu = time.monotonic(), time.process_time()
        callback_wall_ns = time.time_ns()
        try:
            info = image_metadata(serialized)
            if receive <= deadline:
                metrics[name].record(info['stamp_ns'], receive, info['empty'])
            last_seen[name] = receive
            first = name not in metadata
            if first:
                metadata[name] = info
            emit('image', stream=name, receive_monotonic_s=receive,
                 callback_wall_ns=callback_wall_ns, rmw_info=message_timing(message_info),
                 stamp_ns=info['stamp_ns'], empty=info['empty'],
                 serialized_bytes=len(serialized),
                 **({'metadata': info} if first else {}))
        except (ValueError, UnicodeError, struct.error) as error:
            fail('image_parse_error', stream=name, detail=str(error))
        finally:
            elapsed = time.monotonic() - receive
            callback_cost['count'] += 1
            callback_cost['cpu_s'] += time.process_time() - cpu
            callback_cost['wall_s'] += elapsed
            callback_cost['max_wall_s'] = max(callback_cost['max_wall_s'], elapsed)

    def on_scan(name, msg):
        receive = time.monotonic()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if receive <= deadline:
            metrics[name].record(stamp_ns, receive, not msg.ranges)
        prior = last_seen[name]
        if prior is None and receive - start > args.discovery_grace:
            fail('scan_missing', stream=name, gap_s=receive - start)
        elif prior is not None and receive - prior > (.5 if name == 'merged' else .25):
            fail('scan_gap', stream=name, gap_s=receive - prior)
        # Empty scans cannot suppress the safety watchdog.
        if msg.ranges:
            last_seen[name] = receive
        else:
            fail('empty_scan', stream=name)
        emit('scan', stream=name, receive_monotonic_s=receive,
             stamp_ns=stamp_ns, rays=len(msg.ranges))

    def on_info(name, msg):
        stamp = msg.header.stamp
        emit('camera_info', stream=name, stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
             frame_id=msg.header.frame_id, height=msg.height, width=msg.width,
             k=list(msg.k), d=list(msg.d), p=list(msg.p), distortion_model=msg.distortion_model)

    def on_signal(signum, frame):
        nonlocal stopped, stop_reason
        stopped, stop_reason = True, signal.Signals(signum).name

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, on_signal)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(args.name, namespace=args.namespace)
    qos = sensor_qos()
    subscriptions = []
    if args.mode == 'camera' and args.stop_file and args.stop_file.exists():
        stopped, stop_reason = True, 'sentinel'
        emit('sentinel_stop', content=args.stop_file.read_text())
    for name in selected:
        if stopped:
            break
        topic = getattr(args, f'{name}_topic')
        if args.mode == 'scans':
            subscription = node.create_subscription(
                LaserScan, topic, lambda msg, name=name: on_scan(name, msg), qos)
        else:
            subscription = node.create_subscription(
                Image, topic, image_callback(name, on_image), qos, raw=True)
            subscriptions.append(node.create_subscription(
                CameraInfo, getattr(args, f'{name}_info_topic'),
                lambda msg, name=name: on_info(name, msg), qos))
        subscriptions.append(subscription)
    emit('startup', args={key: str(value) if isinstance(value, Path) else value
                          for key, value in vars(args).items()},
         measurement_start_monotonic_s=measurement_start,
         environment={key: os.environ.get(key) for key in
                      ('RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'ROS_DOMAIN_ID')},
         qos={'reliability': 'best_effort', 'durability': 'volatile', 'depth': 5},
         limitations=['service restart/errors require external watchdog',
                      'Cyclone received_timestamp is RMW take time, not packet arrival',
                      'publish-to-take includes DDS transport, queueing and reader scheduling',
                      'header stamps do not establish camera acquisition time'])
    previous, counter_at = read_counters(), time.monotonic()
    if args.mode == 'scans':
        for name in ('eno1', 'enp2s0'):
            if name not in previous:
                fail('missing_interface', interface=name)
    cpu_previous = None
    try:
        while rclpy.ok() and not stopped and time.monotonic() < deadline:
            if args.mode == 'camera' and args.stop_file and args.stop_file.exists():
                stop_reason = 'sentinel'
                emit('sentinel_stop', content=args.stop_file.read_text())
                break
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if args.mode == 'scans':
                for name, receive in last_seen.items():
                    limit = .5 if name == 'merged' else .25
                    if receive is None:
                        if now - start > args.discovery_grace:
                            fail('scan_missing', stream=name, gap_s=now - start)
                    elif now - receive > limit:
                        fail('scan_gap', stream=name, gap_s=now - receive)
                if now - counter_at >= 1:
                    counters = read_counters()
                    for name in ('eno1', 'enp2s0'):
                        if name not in counters:
                            fail('missing_interface', interface=name)
                    try:
                        rates = counter_rates(previous, counters, now - counter_at)
                    except ValueError as error:
                        fail('counter_reset', detail=str(error))
                        rates = {}
                    for name, sample in rates.items():
                        # Split a counter interval that straddles warmup proportionally.
                        weight = max(0.0, now - max(counter_at, measurement_start))
                        totals = interface_totals.setdefault(name, {'duration_s': 0., 'rx_mbit': 0., 'tx_mbit': 0.})
                        totals['duration_s'] += weight
                        for key in ('rx', 'tx'):
                            totals[f'{key}_mbit'] += sample[f'{key}_mbps'] * weight
                    for failure in traffic_guard.sample(rates):
                        fail('traffic_guard', **failure)
                    cpu_fields = [int(value) for value in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
                    cpu_total, cpu_idle = sum(cpu_fields[:8]), sum(cpu_fields[3:5])
                    cpu_busy = None
                    if cpu_previous and cpu_total > cpu_previous[0]:
                        cpu_busy = 1 - (cpu_idle - cpu_previous[1]) / (cpu_total - cpu_previous[0])
                    cpu_previous = cpu_total, cpu_idle
                    emit('host_sample', interfaces=rates, counters=counters,
                         load_average=os.getloadavg(), cpu_busy_fraction=cpu_busy)
                    previous, counter_at = counters, now
    except Exception as error:
        stop_reason = 'exception'
        fail('probe_exception', detail=f'{type(error).__name__}: {error}')
    finally:
        end = min(time.monotonic(), deadline)
        if args.mode == 'scans' and end > counter_at:
            # Include the final partial counter interval in capture averages.
            counters = read_counters()
            try:
                rates = counter_rates(previous, counters, end - counter_at)
                for name, sample in rates.items():
                    weight = max(0.0, end - max(counter_at, measurement_start))
                    totals = interface_totals.setdefault(name, {'duration_s': 0., 'rx_mbit': 0., 'tx_mbit': 0.})
                    totals['duration_s'] += weight
                    for key in ('rx', 'tx'):
                        totals[f'{key}_mbit'] += sample[f'{key}_mbps'] * weight
                emit('host_sample_final', interfaces=rates, counters=counters)
            except ValueError as error:
                fail('counter_reset', detail=str(error))
        streams = {}
        for name, metric in metrics.items():
            min_rate = 29.0 if args.mode == 'camera' else 38.0
            if name == 'merged':
                min_rate = .9 * baseline.get('streams', {}).get('merged', {}).get('rate_hz', 0)
            streams[name] = metric.summary(end, min_rate, .5 if name == 'merged' else
                                          .25 if args.mode == 'scans' else .15)
        interfaces = {}
        for name, values in interface_totals.items():
            duration = values['duration_s']
            interfaces[name] = {**values, 'mean_rx_mbps': values['rx_mbit'] / duration if duration else 0,
                                'mean_tx_mbps': values['tx_mbit'] / duration if duration else 0}
            if name in baseline_tx:
                excess = interfaces[name]['mean_tx_mbps'] - baseline_tx[name]
                interfaces[name]['incremental_tx_mbps'] = excess
                interfaces[name]['traffic_passed'] = excess <= 5 if name in ('eno1', 'enp2s0') else None
        match = exact_matches(metrics['color'].stamps, metrics['aligned'].stamps) if 'color' in metrics and 'aligned' in metrics else None
        stop_marker = (args.stop_file.read_text()
                       if args.stop_file and args.stop_file.exists() else None)
        passed = (not failures and stop_marker is None and stop_reason == 'duration'
                  and all(value['passed'] for value in streams.values())
                  and (match is None or match['passed'])
                  and all(value.get('traffic_passed') is not False for value in interfaces.values()))
        emit('shutdown', reason=stop_reason)
        callback_cost['mean_wall_ms'] = (1000 * callback_cost['wall_s'] / callback_cost['count']
                                         if callback_cost['count'] else 0.0)
        callback_cost['cpu_fraction_one_core'] = callback_cost['cpu_s'] / max(end - start, 1e-9)
        summary = {'mode': args.mode, 'name': args.name, 'start_monotonic_s': start,
                   'measurement_start_monotonic_s': measurement_start,
                   'end_monotonic_s': end, 'stop_reason': stop_reason, 'passed': passed,
                   'streams': streams, 'interfaces': interfaces, 'exact_matches': match,
                   'failures': failures, 'stop_marker': stop_marker,
                   'metadata': metadata, 'observer': callback_cost,
                   'limitations': ['external service journal/PID watchdog required',
                                   'network interval overlapping warmup is prorated',
                                   'RMW/rclpy serialized copying before callback unmeasured']}
        (args.output_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        events.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(run(parser().parse_args()))
