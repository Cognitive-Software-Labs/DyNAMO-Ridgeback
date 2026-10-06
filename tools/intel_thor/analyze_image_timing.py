#!/usr/bin/env python3
"""Split observed image gaps at the CycloneDDS publication and take boundaries.

All wall-clock timestamps belong to this Intel host. RMW source time is set
before dds_write_ts in rmw_cyclonedds 2.2.3; received time is set at RMW take.
Neither is a packet arrival timestamp. Keep raw events for further tracing.
"""
import argparse
import json
from pathlib import Path


def split_gap(previous, current):
    gap = current['receive_monotonic_s'] - previous['receive_monotonic_s']
    result = {'previous_stamp_ns': previous['stamp_ns'], 'stamp_ns': current['stamp_ns'],
              'receive_gap_ms': gap * 1000,
              'header_gap_ms': (current['stamp_ns'] - previous['stamp_ns']) / 1e6,
              'valid': False}
    times = []
    for event in (previous, current):
        info = event.get('rmw_info', {})
        values = (info.get('source_timestamp'), info.get('received_timestamp'),
                  event.get('callback_wall_ns'))
        if any(not isinstance(value, int) or value <= 0 for value in values):
            result['limitation'] = 'Missing or unsupported publication/take/callback timestamp'
            return result
        if not values[0] <= values[1] <= values[2]:
            result['limitation'] = 'Wall timestamps out of order; inspect clocks or metadata'
            return result
        times.append(values)
    pub0, take0, cb0 = times[0]
    pub1, take1, cb1 = times[1]
    clock_disagreement = abs((cb1 - cb0) / 1e9 - gap)
    result['wall_monotonic_disagreement_ms'] = clock_disagreement * 1000
    if clock_disagreement > .002 or pub1 < pub0 or take1 < take0:
        result['limitation'] = 'Wall/monotonic discontinuity or regressing middleware timestamp'
        return result
    # Difference of consecutive latencies explains a receive gap. Absolute
    # header-to-publication latency cannot establish sensor acquisition time.
    result.update(valid=True,
                  publication_gap_ms=(pub1 - pub0) / 1e6,
                  take_gap_ms=(take1 - take0) / 1e6,
                  publish_to_take_ms=(take1 - pub1) / 1e6,
                  take_to_callback_ms=(cb1 - take1) / 1e6,
                  before_publication_delay_change_ms=((pub1 - pub0) -
                      (current['stamp_ns'] - previous['stamp_ns'])) / 1e6,
                  publish_to_take_delay_change_ms=((take1 - pub1) - (take0 - pub0)) / 1e6,
                  take_to_callback_delay_change_ms=((cb1 - take1) - (cb0 - take0)) / 1e6)
    return result


def analyze(events, start, end):
    streams = {}
    for event in events:
        if event.get('event') == 'image' and start <= event['receive_monotonic_s'] <= end:
            streams.setdefault(event['stream'], []).append(event)
    result = {}
    for stream, images in streams.items():
        gaps = [split_gap(a, b) for a, b in zip(images, images[1:])]
        result[stream] = {'images': len(images), 'gaps': len(gaps),
                          'valid_timing_gaps': sum(gap['valid'] for gap in gaps),
                          'largest_receive_gaps': sorted(gaps, key=lambda x: x['receive_gap_ms'],
                                                         reverse=True)[:10]}
    return {'streams': result,
            'semantics': {'publication': 'Cyclone RMW timestamp before dds_write_ts',
                          'take': 'RMW takes the sample; not packet arrival',
                          'callback': 'Python image callback entry'},
            'limitations': ['Only received frames are analyzed; missing publication events are unobserved',
                            'Publish-to-take combines write/transport/queueing/executor scheduling',
                            'No USB acquisition trace; header time is driver-supplied',
                            'Wall clocks checked against monotonic per pair; no cross-host inference'],
            'sources': ['https://github.com/ros2/rmw_cyclonedds/blob/2.2.3/rmw_cyclonedds_cpp/src/rmw_node.cpp',
                        'https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/subscription.cpp']}


def analyze_reader(directory):
    directory = Path(directory)
    summary = json.loads((directory / 'summary.json').read_text())
    with (directory / 'events.jsonl').open() as handle:
        return analyze((json.loads(line) for line in handle),
                       summary['measurement_start_monotonic_s'], summary['end_monotonic_s'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reader', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze_reader(args.reader), indent=2))


if __name__ == '__main__':
    main()
