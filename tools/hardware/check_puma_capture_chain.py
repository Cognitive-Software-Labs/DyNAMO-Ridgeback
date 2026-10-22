#!/usr/bin/env python3
"""Replay passive wheel CAN into maintained C++ producer/consumer policies."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from replay_puma_freshness import DEVICES, receive_field_update

ROOT = Path(__file__).resolve().parents[2]
SPLIT = (150, 500)  # driver patch candidate: motion, auxiliary field-age bounds (ms)


def check(capture, output):
    output = output.resolve()
    if output == ROOT / 'artifacts/hardware' or not output.is_relative_to(ROOT / 'artifacts/hardware'):
        raise ValueError('use a fresh directory below artifacts/hardware')
    output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), Path(__file__).with_name('puma_capture_chain_check.cpp'), Path(__file__).with_name('puma_field_freshness.hpp'),
               Path(__file__).with_name('puma_feedback_safety.hpp')]
    for source in sources:
        (output / source.name).write_bytes(source.read_bytes())
    events = [event for line in capture.read_text().splitlines()
              if (event := receive_field_update(line)) is not None]
    events.sort(key=lambda item: item[0])
    if not events:
        raise ValueError('capture has no relevant received wheel fields')
    # candump has microsecond resolution; preserve arrival order within equal stamps.
    first = events[0][0]
    last_ns = 0
    lines = []
    for stamp, device, field, valid in events:
        ns = max(last_ns + 1, round((stamp - first) * 1e9) + 1_000_000_000)
        last_ns = ns
        lines.append(f'{ns} {DEVICES.index(device)} {field} {int(valid)}')
    feed = '\n'.join(lines) + '\n'
    (output / 'field-events.tsv').write_text(feed)
    binary = output / 'policy-chain'
    build = subprocess.run(['g++', '-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            '-I', str(output), str(output / 'puma_capture_chain_check.cpp'),
                            '-o', str(binary)], capture_output=True, text=True)
    (output / 'build.log').write_text(build.stdout + build.stderr)
    build.check_returncode()
    results = []
    for bound in (150, 175, 200, 225, 250):
        for phase in range(0, 50, 5):
            run = subprocess.run([str(binary), str(bound), str(phase), '0'], input=feed,
                                 capture_output=True, text=True, check=True)
            results.append(json.loads(run.stdout))
    queue_delay_sweep = []
    for delay in (25, 50, 75, 100, 125, 150, 175, 200):
        for phase in range(0, 50, 5):
            run = subprocess.run([str(binary), '200', str(phase), str(delay)], input=feed,
                                 capture_output=True, text=True, check=True)
            queue_delay_sweep.append(json.loads(run.stdout))
    # One wheel's missing speed must suppress whole aggregates and latch the
    # downstream consumer even while all other wheel fields continue arriving.
    threshold = int(lines[0].split()[0]) + 5_000_000_000
    missing_speed_feed = '\n'.join(line for line in lines
        if not (int(line.split()[0]) >= threshold and line.split()[1:3] == ['0', '3'])) + '\n'
    injected = subprocess.run([str(binary), '200', '0', '0'], input=missing_speed_feed,
                              capture_output=True, text=True, check=True)
    fault_injection = json.loads(injected.stdout)
    if not fault_injection['consumer_faulted_at_end'] or fault_injection['consumer_fault_events'] == 0:
        raise RuntimeError('missing-wheel speed did not latch consumer fault')
    # Split candidate: motion fields 100 ms, auxiliary fields 500 ms.
    def chain(feed_text, *argv):
        return json.loads(subprocess.run([str(binary), *map(str, argv)], input=feed_text,
                                         capture_output=True, text=True, check=True).stdout)

    def without(wheel, field, start_s, duration_s):
        start = int(lines[0].split()[0]) + int(start_s * 1e9)
        end = start + int(duration_s * 1e9)
        return '\n'.join(line for line in lines if not (
            start <= int(line.split()[0]) < end and line.split()[1:3] == [str(wheel), str(field)])) + '\n'

    split = {'motion_bound_ms': SPLIT[0], 'auxiliary_bound_ms': SPLIT[1],
             'results': [chain(feed, SPLIT[0], phase, 0, SPLIT[1]) for phase in range(0, 50, 5)],
             'queue_delay_sweep': [chain(feed, SPLIT[0], phase, delay, SPLIT[1])
                                   for delay in (25, 50, 75, 100) for phase in range(0, 50, 5)]}
    # Asymmetric injections against the split bounds: one wheel's setpoint
    # silent for 400 ms must not reach the consumer; silent for 700 ms, or one
    # wheel's speed silent for 300 ms, must latch the consumer fault.
    cases = {'rear_setpoint_silent_400ms': (3, 4, .4, False), 'rear_setpoint_silent_700ms': (3, 4, .7, True),
             'front_speed_silent_300ms': (0, 3, .3, True), 'front_position_silent_120ms': (0, 2, .12, None)}
    split['injections'] = {}
    for name, (wheel, field, duration, must_fault) in cases.items():
        rows = [chain(without(wheel, field, 5, duration), SPLIT[0], phase, 0, SPLIT[1]) for phase in range(0, 50, 5)]
        faulted = [row['consumer_faulted_at_end'] for row in rows]
        if must_fault is not None and faulted != [must_fault] * len(rows):
            raise RuntimeError(f'{name}: consumer fault {faulted}, expected {must_fault}')
        split['injections'][name] = {'expected_consumer_fault': must_fault, 'phases': rows}
    report = {'status': 'offline-policy-chain-replay',
              'capture_sha256': hashlib.sha256(capture.read_bytes()).hexdigest(),
              'source_sha256': {source.name: hashlib.sha256(source.read_bytes()).hexdigest() for source in sources},
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'events': len(events), 'invalid_events': sum(not event[3] for event in events),
              'results': results, 'queue_delay_sweep': queue_delay_sweep,
              'missing_wheel_speed_injection': fault_injection, 'split_candidate': split,
              'limits': 'Actual maintained field-age and consumer policy classes over host CAN receipt timestamps with an ideal 20 Hz tick. No ROS queue, scheduling load, physical motion or deployed-driver execution.'}
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = check(args.capture, args.output)
    for bound in (150, 175, 200, 225, 250):
        rows = [row for row in report['results'] if row['bound_ms'] == bound]
        print(json.dumps({'bound_ms': bound, 'min_publications': min(row['publications'] for row in rows),
                          'max_gap_ms': max(row['max_gap_ms'] for row in rows),
                          'consumer_faulted': any(row['consumer_faulted_at_end'] for row in rows)}))
    split = report['split_candidate']
    print(json.dumps({'split_bounds_ms': SPLIT, 'min_publications': min(r['publications'] for r in split['results']),
                      'max_gap_ms': max(r['max_gap_ms'] for r in split['results']),
                      'consumer_faulted': any(r['consumer_faulted_at_end'] for r in split['results']),
                      'delay_sweep_max_gap_ms': max(r['max_gap_ms'] for r in split['queue_delay_sweep']),
                      'injections': {k: [any(r['consumer_faulted_at_end'] for r in v['phases']),
                                         max(r['max_gap_ms'] for r in v['phases'])]
                                     for k, v in split['injections'].items()}}))
