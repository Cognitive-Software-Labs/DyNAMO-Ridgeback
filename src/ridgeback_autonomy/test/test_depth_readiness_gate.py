"""Cold depth inference must finish before the timed benchmark window."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from ridgeback_common.messages import build_detections_message
from ridgeback_common.models import DetectionBatch
from ridgeback_localization.mask_measurement_node import TargetMaskMeasurementNode
from ridgeback_localization.worker_state import WorkerState
from ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node import (
    TargetDistanceBenchmarkRunner,
)


@pytest.fixture
def ros_context():
    import rclpy
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


class Source:
    input_kind = 'color'

    def __init__(self):
        self.frame = None
        self.calls = []

    def produce(self, image):
        self.calls.append(image)
        return self.frame


def node_for(source, estimators='projective_ranging'):
    return TargetMaskMeasurementNode(depth_source=source, parameter_overrides=[
        Parameter('depth_source', value='monocular'),
        Parameter('enabled_estimators', value=estimators)])


def empty_batch():
    header = Header()
    header.stamp.sec, header.stamp.nanosec = 3, 42
    batch = DetectionBatch(image_width=4, image_height=3)
    detections = build_detections_message(batch, header)
    image = Image(header=header, width=4, height=3, encoding='rgb8')
    return detections, image


def test_warmup_uses_a_real_frame_without_requiring_detection_or_tf(ros_context):
    source = Source()
    node = node_for(source)
    detections, image = empty_batch()
    try:
        node.warm_depth_source(detections, None)
        assert source.calls == []
        assert not node.depth_readiness()['ready']
        node.warm_depth_source(detections, image)
        assert not node.depth_readiness()['ready']  # source failed / still loading
        source.frame = (np.zeros((3, 4), dtype=np.float32), image.header)
        node.warm_depth_source(detections, image)
        assert node.depth_readiness() == {
            'depth_source': 'monocular', 'ready': True, 'stamp_ns': 3_000_000_042}
        node.warm_depth_source(detections, image)
        assert len(source.calls) == 2  # no repeated warmup after success
        assert not detections.detected
    finally:
        node.destroy_node()


def test_wrong_grid_does_not_open_warmup_gate(ros_context):
    source = Source()
    node = node_for(source)
    detections, image = empty_batch()
    source.frame = (np.ones((2, 2), dtype=np.float32), image.header)
    try:
        node.warm_depth_source(detections, image)
        assert not node.depth_readiness()['ready']
    finally:
        node.destroy_node()


def test_polar_only_does_not_warm_a_monocular_model(ros_context):
    source = Source()
    node = node_for(source, 'polar_profiling')
    try:
        node.warm_depth_source(*empty_batch())
        assert source.calls == []
        assert node.depth_source is None
    finally:
        node.destroy_node()


def test_worker_latches_readiness_changes_without_changing_processing_state():
    messages = []
    node = SimpleNamespace(create_publisher=lambda *args: SimpleNamespace(publish=messages.append))
    ready = {'value': False}
    worker = WorkerState(node, 'mask', readiness=lambda: {'ready': ready['value']})
    worker.set('processing')
    worker.set('processing')
    assert len(messages) == 2
    ready['value'] = True
    worker.set('processing')
    assert json.loads(messages[-1].data)['readiness']['ready'] is True
    assert len(messages) == 3
    legacy = WorkerState(node, 'detector')
    assert 'readiness' not in json.loads(messages[-1].data)
    legacy.close()


def test_gate_ignores_empty_or_wrong_source_readiness_and_worker_failures():
    runner = SimpleNamespace(depth_source='monocular', depth_pipeline_ready=False)
    callback = TargetDistanceBenchmarkRunner.on_depth_worker_state
    for payload in ['broken', [], {'component': 'mask', 'state': 'processing'},
                    {'component': 'mask', 'state': 'processing',
                     'readiness': {'depth_source': 'monocular', 'ready': False}},
                    {'component': 'mask', 'state': 'processing',
                     'readiness': {'depth_source': 'stereoscopic', 'ready': True}},
                    {'component': 'mask', 'state': 'failure',
                     'readiness': {'depth_source': 'monocular', 'ready': True}}]:
        callback(runner, String(data=payload if isinstance(payload, str) else json.dumps(payload)))
        assert not runner.depth_pipeline_ready
    callback(runner, String(data=json.dumps({
        'component': 'mask', 'epoch': 'new-worker', 'state': 'processing',
        'readiness': {'depth_source': 'monocular', 'ready': True, 'stamp_ns': 42}})))
    assert runner.depth_pipeline_ready


@pytest.mark.parametrize('needs_warmup', [False, True])
def test_stream_readiness_does_not_skip_required_depth_warmup(needs_warmup):
    waited = []
    runner = SimpleNamespace(
        needs_pointcloud=False, needs_mask=True, mask_measurement_seen=True,
        mask_measurement_topic='measurements', color_stream_seen=True, color_topic='color',
        needs_depth_warmup=needs_warmup, depth_pipeline_ready=False,
        depth_pipeline_state={'epoch': 'ready-producer'}, depth_warmup_gate=None,
        resolved_topic=lambda name: name,
        get_logger=lambda: SimpleNamespace(info=lambda text: None))

    def wait(condition, description):
        waited.append(description)
        if 'warmup' in description:
            assert not condition()  # empty measurement messages were insufficient
            runner.depth_pipeline_ready = True
        assert condition()

    runner.wait_for_flag = wait
    TargetDistanceBenchmarkRunner.wait_for_required_streams(runner)
    assert any('warmup' in item for item in waited) == needs_warmup
    assert (runner.depth_warmup_gate is not None) == needs_warmup
