from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import Image


class _StubDetector:
    """Injected in place of OwlV2Detector: no torch, scripted detect().

    ``load()`` is a no-op so the node stands up without OWLv2; ``detect`` raises
    on demand to exercise the worker guard.
    """

    def __init__(self) -> None:
        self.load_calls = 0
        self.detect_calls = 0
        self.raise_next = False

    def load(self) -> None:
        self.load_calls += 1

    def detect(self, image, **kwargs) -> list:
        self.detect_calls += 1
        if self.raise_next:
            raise RuntimeError('simulated CUDA OOM')
        return []


def _color_message(width: int = 8, height: int = 6) -> Image:
    msg = Image()
    msg.header.frame_id = 'camera'
    msg.height = height
    msg.width = width
    msg.encoding = 'rgb8'
    msg.step = width * 3
    msg.data = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
    return msg


@pytest.fixture
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def _make_node(detector):
    from ridgeback_autonomy.perception.g1_detector_node import G1DetectorNode

    return G1DetectorNode(detector=detector)


def test_seam_injects_stub_and_calls_load_without_owlv2(ros_context) -> None:
    detector = _StubDetector()
    node = _make_node(detector)
    try:
        assert node.detector is detector
        assert detector.load_calls == 1
    finally:
        node.destroy_node()


def test_worker_survives_detection_exception(ros_context) -> None:
    detector = _StubDetector()
    node = _make_node(detector)
    published: list = []
    node.detections_pub.publish = lambda msg: published.append(msg)
    try:
        # A frame that blows up in inference must not propagate or kill the worker.
        detector.raise_next = True
        node.run_detection_step(_color_message())

        # The worker is still able to handle the next (good) frame end-to-end.
        detector.raise_next = False
        node.run_detection_step(_color_message())

        assert detector.detect_calls == 2
        assert len(published) == 1
    finally:
        node.destroy_node()


def test_process_color_image_passes_node_threshold(ros_context) -> None:
    detector = _StubDetector()
    seen: list = []
    original_detect = detector.detect

    def recording_detect(image, **kwargs):
        seen.append(kwargs)
        return original_detect(image, **kwargs)

    detector.detect = recording_detect
    node = _make_node(detector)
    node.detections_pub.publish = lambda msg: None
    try:
        node.run_detection_step(_color_message())

        assert seen[0]['threshold'] == node.detection_threshold
    finally:
        node.destroy_node()
