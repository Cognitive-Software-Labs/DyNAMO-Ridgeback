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
