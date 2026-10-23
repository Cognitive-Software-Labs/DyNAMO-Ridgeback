#!/usr/bin/env python3
"""Align a bounded pulse or source handover's CAN receipts with its ROS command window."""
import argparse
import hashlib
import json
from pathlib import Path
from statistics import median

from replay_puma_freshness import DEVICES, FIELD_BYTES, receive_field_update

ROOT = Path(__file__).resolve().parents[2]


def summarize(capture, samples, observer, source='pulse'):
    rows = [json.loads(line) for line in samples.read_text().splitlines()]
    if observer.get('final_physical_stop_pressed') is not True:
        raise ValueError('final physical stop required')
    if source == 'pulse':
        sent = [row for row in rows if row['kind'] == 'sent' and row['values']['speed'] > 0]
        if len(sent) < 5 or observer.get('status') != 'pulse-completed-final-odometry-stationary':
            raise ValueError('complete bounded pulse required')
        anchors = [row['values']['stamp_ns']/1e9-row['elapsed'] for row in sent]
        start, end = sent[0]['elapsed'], sent[-1]['elapsed']
        feedback = [row['elapsed'] for row in rows
                    if row['kind'] == 'platform/motors/feedback' and start <= row['elapsed'] <= end]
        odom = [row['values']['velocity'][0] for row in rows
                if row['kind'] == 'raw_odom' and start <= row['elapsed'] <= end]
        motor_commands = [row for row in rows if row['kind'] == 'platform/motors/cmd'
                          and start <= row['elapsed'] <= end]
        status = [row for row in rows if row['kind'] == 'motor_status'
                  and start <= row['elapsed'] <= end]
        if not .35 <= end-start <= 1.1:
            raise ValueError('unexpected bounded command window')
    elif source == 'handover':
        if observer.get('status') != 'live-handover-observed-awaiting-operator' \
                or observer.get('software_handover_observed') is not True:
            raise ValueError('complete bounded handover required')
        sent = [row for row in rows if row['kind'] == 'limited'
                and row['phase'] in ('low', 'high_takeover', 'high_expiry_return')
                and row['velocity'][0] > 0]
        if len(sent) < 5:
            raise ValueError('missing limited handover commands')
        anchors = [row['stamp_ns']/1e9-row['t'] for row in sent]
        start, end = sent[0]['t'], sent[-1]['t']
        feedback = []  # The handover recorder does not capture ROS feedback aggregates.
        odom = [row['velocity'][0] for row in rows
                if row['kind'] == 'odom' and start <= row['t'] <= end]
        motor_commands = []  # The handover recorder does not capture motor commands.
        status = [row for row in rows if row['kind'] == 'motor_status'
                  and start <= row['t'] <= end]
        if not 1.0 <= end-start <= 2.5:
            raise ValueError('unexpected bounded handover window')
    else:
        raise ValueError('unknown source')
    anchor = median(anchors)
    events = [event for line in capture.read_text().splitlines()
              if (event := receive_field_update(line)) is not None]
    window = [event for event in events if anchor+start <= event[0] <= anchor+end]
    per_field = {}
    for device in DEVICES:
        for field in range(len(FIELD_BYTES)):
            stamps = [event[0] for event in window if event[1:3] == (device, field)
                      and event[3]]
            edges = [anchor+start, *stamps, anchor+end]
            per_field[f'{device}:{field}'] = {
                'valid_frames': len(stamps),
                'max_silence_including_edges_ms':
                    round(max(b-a for a,b in zip(edges, edges[1:]))*1000, 3),
            }
    return {
        'status': 'bounded-command-window-analyzed',
        'source': source,
        'capture_sha256': hashlib.sha256(capture.read_bytes()).hexdigest(),
        'samples_sha256': hashlib.sha256(samples.read_bytes()).hexdigest(),
        'wall_anchor_method': 'median command ROS header stamp minus local monotonic elapsed',
        'wall_anchor_s': anchor,
        'command_elapsed_s': [start, end],
        'command_duration_s': end-start,
        'valid_can_fields': sum(event[3] for event in window),
        'invalid_can_fields': sum(not event[3] for event in window),
        'per_field': per_field,
        'all_twenty_fields_received': all(value['valid_frames'] > 0 for value in per_field.values()),
        'ros_feedback_count': len(feedback),
        'ros_feedback_max_gap_ms': max(((b-a)*1000 for a,b in zip(feedback, feedback[1:])),
                                       default=None),
        'raw_odom_max_forward_m_s': max(odom, default=None),
        'nonzero_motor_command_samples': (sum(any(abs(v)>1e-8 for v in row['values']['velocity'])
                                              for row in motor_commands) if source == 'pulse' else None),
        'motor_status_samples': len(status),
        'all_motor_status_clear_during_command': bool(status) and all(
            all(driver['fault'] == 0 and driver['mode'] == 2
                for driver in (row['values']['drivers'] if source == 'pulse'
                               else row['summary']['drivers'])) for row in status),
        'limits': 'Host CAN receive timestamps aligned to ROS command header stamps; one short command window, not physical wheel synchronization, loaded scheduling, internal queue timing or a production field-age bound. Handover capture lacks motor-command and ROS-feedback aggregate samples.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', choices=('pulse', 'handover'), default='pulse')
    args = parser.parse_args()
    root, output = args.run_dir.resolve(), args.output.resolve()
    evidence = ROOT/'artifacts/hardware'
    if not root.is_relative_to(evidence) or not output.is_relative_to(evidence) or output.exists():
        parser.error('use preserved capture and a fresh output below artifacts/hardware')
    samples = root/'observer'/('trace.jsonl' if args.source == 'handover' else 'samples.jsonl')
    result = summarize(root/'passive-can.log', samples,
                       json.loads((root/'observer/result.json').read_text()), args.source)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: result[key] for key in ('status', 'command_duration_s',
        'valid_can_fields', 'invalid_can_fields', 'all_twenty_fields_received',
        'ros_feedback_count', 'raw_odom_max_forward_m_s',
        'nonzero_motor_command_samples', 'all_motor_status_clear_during_command')}, indent=2))


if __name__ == '__main__':
    main()
