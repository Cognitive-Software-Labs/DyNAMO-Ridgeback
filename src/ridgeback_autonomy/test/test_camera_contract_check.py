"""Criteria of the camera contract check, on synthetic observations."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
FRAME = 'camera_0_color_optical_frame'
PERIOD_NS = 33_333_333


def load_tool():
    loader = SourceFileLoader('camera_contract_check', str(ROOT / 'tools/camera_contract_check'))
    spec = importlib.util.spec_from_loader('camera_contract_check', loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


TOOL = load_tool()
EXPECTED = {'grid': (640, 480), 'optical_frame': FRAME,
            'types': {'color': 'sensor_msgs/msg/Image', 'info': 'sensor_msgs/msg/CameraInfo',
                      'depth': 'sensor_msgs/msg/Image'}}


def stream(encoding, count=1800):
    return {'stamps': [1_000_000_000_000 + i * PERIOD_NS for i in range(count)],
            'encodings': {encoding}, 'sizes': {(640, 480)}, 'frames': {FRAME}}


def good():
    return {
        'color': stream('rgb8'), 'depth': stream('16UC1'),
        'info': {'sizes': {(640, 480)}, 'frames': {FRAME}},
        'types': dict(EXPECTED['types']), 'duration_s': 60.0,
        'tf_ok': True, 'tf_detail': 'base_link -> optical', 'extra_topics': [],
        'scans': {name: {'receive_s': [i * 0.025 for i in range(2400)]}
                  for name in ('front', 'rear', 'merged')},
    }


def failed(observation):
    return {name for name, passed, _ in TOOL.evaluate(observation, EXPECTED) if not passed}


def test_known_good_observation_passes_every_criterion():
    assert failed(good()) == set()


def drop(key, start, count):
    def mutate(observation):
        del observation[key]['stamps'][start:start + count]
    return mutate


def shift_depth(fraction):
    def mutate(observation):
        stamps = observation['depth']['stamps']
        for index in range(int(len(stamps) * fraction)):
            stamps[index] += 1
    return mutate


@pytest.mark.parametrize('mutate, criterion', [
    (lambda o: o['color'].update(encodings={'yuv422'}), 'colour encoding'),
    (lambda o: o['depth'].update(encodings={'8UC1'}), 'depth encoding'),
    # A profile the driver silently fell back to.
    (lambda o: o['depth'].update(sizes={(1280, 720)}), 'grids'),
    (lambda o: o['info'].update(sizes={(848, 480)}), 'grids'),
    (lambda o: o['depth'].update(frames={'camera_0_depth_optical_frame'}), 'frame ids'),
    (lambda o: o.update(tf_ok=False), 'TF base to optical'),
    (drop('color', 0, 900), 'colour rate'),
    # Five consecutive frames missing: a 200 ms gap.
    (drop('color', 100, 5), 'colour stamps'),
    (lambda o: o['depth']['stamps'].insert(10, o['depth']['stamps'][9]), 'depth stamps'),
    (shift_depth(0.03), 'colour/depth pairing'),
    (lambda o: o.update(extra_topics=['/r100_0001/sensors/camera_0/infra1/image_rect_raw']),
     'contract streams only'),
    (lambda o: o['types'].update(depth='absent'), 'types'),
    (lambda o: o['scans']['front']['receive_s'].__setitem__(
        slice(500, 512), []), 'front scan'),
])
def test_each_fault_fails_its_criterion(mutate, criterion):
    observation = good()
    mutate(observation)

    assert criterion in failed(observation)


def test_merged_scan_tolerates_the_longer_merger_gap():
    observation = good()
    del observation['scans']['merged']['receive_s'][500:515]  # 400 ms

    assert failed(observation) == set()


def test_simulators_need_only_one_shared_frame():
    observation = good()
    for key in ('color', 'depth'):
        observation[key]['frames'] = {'r100_0001/robot/camera_0_color_optical_frame'}

    rows = TOOL.evaluate(observation, {**EXPECTED, 'optical_frame': None})

    assert all(passed for _, passed, _ in rows)


def test_header_is_read_from_the_serialized_image_without_pixels():
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image

    message = Image(height=480, width=640, encoding='16UC1', step=1280, data=bytes(640 * 480 * 2))
    message.header.stamp.sec = 1791276509
    message.header.stamp.nanosec = 289081623
    message.header.frame_id = FRAME

    header = TOOL.image_header(serialize_message(message))

    assert header == {'stamp_ns': 1791276509_289081623, 'frame': FRAME,
                      'width': 640, 'height': 480, 'encoding': '16UC1'}


def test_timing_arithmetic_and_offset_sign():
    sample = TOOL.timing_sample(1_000_000_000, {'source_timestamp': 1_010_000_000,
        'received_timestamp': 1_032_000_000, 'publisher_gid': b'abc',
        'publication_sequence_number': 1}, 1_035_000_000)
    stats = TOOL.timing_stats([sample], offset_ms=2, bound_ms=.2)
    assert {k: v['median_ms'] for k, v in stats['components'].items()} == {
        'driver': 10, 'network_dds': 20, 'executor': 3, 'total': 33}


@pytest.mark.parametrize('bound', [None, .201, -1, float('nan')])
def test_unresolved_clock_keeps_only_same_clock_components(bound):
    sample = TOOL.timing_sample(1, {'source_timestamp': 2, 'received_timestamp': 3}, 4)
    stats = TOOL.timing_stats([sample], offset_ms=2, bound_ms=bound)
    assert not stats['clock_resolved']
    assert stats['components']['network_dds']['count'] == 0
    assert stats['components']['total']['median_ms'] is None
    assert stats['components']['driver']['count'] == 1
    assert stats['components']['executor']['count'] == 1
    assert TOOL.timing_stats([sample], same_host=True)['clock_resolved']


def test_percentiles_and_unavailable_rmw_fields():
    assert TOOL.percentiles([0, 10, 20]) == {
        'count': 3, 'median_ms': 10, 'p95_ms': 19, 'p99_ms': 19.8, 'max_ms': 20}
    stats = TOOL.timing_stats([TOOL.timing_sample(100, {}, 200)], same_host=True)
    assert stats['components']['driver']['count'] == 0
    assert stats['components']['executor']['count'] == 0
    assert stats['dds_loss']['fraction'] is None


def test_sequence_loss_is_separate_per_publisher():
    samples = [TOOL.timing_sample(1, {'publisher_gid': gid,
                'publication_sequence_number': seq}, 2)
               for gid, seq in [(b'a', 4), (b'b', 10), (b'a', 7), (b'a', 7), (b'b', 11)]]
    stats = TOOL.timing_stats(samples)
    assert stats['dds_loss'] == {'missing': 2, 'observed_intervals': 2, 'fraction': .5}
