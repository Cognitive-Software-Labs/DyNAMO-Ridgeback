"""Timing boundary checks; no robot graph or service changes."""
from analyze_image_timing import analyze, split_gap
from local_sensor_probe import image_callback, message_timing
import inspect


def event(mono, stamp, pub, take, cb):
    base = 1_790_000_000_000_000_000
    return {'event': 'image', 'stream': 'depth', 'receive_monotonic_s': mono,
            'stamp_ns': base + stamp, 'callback_wall_ns': base + cb,
            'rmw_info': {'source_timestamp': base + pub, 'received_timestamp': base + take}}


def test_delay_split_distinguishes_publication_and_reader_delay():
    a = event(10, 0, 1_000_000, 2_000_000, 3_000_000)
    b = event(10.130, 33_000_000, 80_000_000, 120_000_000, 133_000_000)
    gap = split_gap(a, b)
    assert gap['valid']
    assert gap['header_gap_ms'] == 33
    assert gap['publication_gap_ms'] == 79
    assert gap['before_publication_delay_change_ms'] == 46
    assert gap['publish_to_take_delay_change_ms'] == 39
    assert gap['take_to_callback_delay_change_ms'] == 12
    assert gap['publish_to_take_ms'] == 40
    assert sum(gap[k] for k in ('header_gap_ms', 'before_publication_delay_change_ms',
               'publish_to_take_delay_change_ms', 'take_to_callback_delay_change_ms')) == 130


def test_clock_jump_and_missing_metadata_do_not_claim_a_stage():
    a = event(10, 0, 1, 2, 3)
    b = event(10.03, 30_000_000, 130_000_000, 130_000_001, 130_000_002)
    assert not split_gap(a, b)['valid']
    b['rmw_info']['source_timestamp'] = 0
    assert not split_gap(a, b)['valid']
    del b['rmw_info']
    assert not split_gap(a, b)['valid']


def test_out_of_order_timestamps_and_warmup_excluded():
    a = event(10, 0, 3, 2, 4)
    b = event(10.03, 30_000_000, 30_000_001, 30_000_002, 30_000_004)
    assert not split_gap(a, b)['valid']
    assert analyze([a, b], 10.01, 11)['streams']['depth']['gaps'] == 0


def test_callback_requires_info_and_preserves_stream():
    calls = []
    callback = image_callback('depth', lambda *args: calls.append(args))
    try:
        inspect.signature(callback).bind(object())
    except TypeError:
        pass
    else:
        raise AssertionError('rclpy would select MessageOnly')
    callback(b'image', {'source_timestamp': 123})
    assert calls == [('depth', b'image', {'source_timestamp': 123})]
    assert message_timing({'source_timestamp': 0})['received_timestamp'] is None
