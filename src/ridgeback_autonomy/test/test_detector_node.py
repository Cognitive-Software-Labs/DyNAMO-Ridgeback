from __future__ import annotations

import time

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


def _make_node(detector, **parameters):
    from rclpy.parameter import Parameter
    from ridgeback_autonomy.perception.target_localization.detector_node import TargetDetectorNode

    return TargetDetectorNode(
        detector=detector,
        parameter_overrides=[
            Parameter(name, value=value) for name, value in parameters.items()
        ],
    )


def _wait_for(predicate, timeout_s: float = 2.0) -> bool:
    """Poll until the worker thread has made the expected progress."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


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


def test_period_is_measured_from_the_start_of_a_step(ros_context) -> None:
    """The cadence must not stretch by however long inference took.

    The clock used to advance after the step, making the achieved period
    ``configured + inference`` -- a 5 FPS setting delivered 4.13 Hz on the
    measured stack. Advancing it before the step makes the period a floor on
    step *starts*, so the observed rate matches the setpoint.
    """

    detector = _StubDetector()
    node = _make_node(detector)
    node.detections_pub.publish = lambda msg: None
    observed: list[float] = []

    def recording_detect(image, **kwargs):
        observed.append(node.last_detection_time)
        return []

    detector.detect = recording_detect
    try:
        started = time.monotonic()
        node.on_color_image(_color_message())

        assert _wait_for(lambda: bool(observed)), 'worker never processed the frame'
        # Advanced before the step ran, not left at its initial 0.0.
        assert observed[0] >= started
    finally:
        node.destroy_node()


def test_default_rate_and_period_agree(ros_context) -> None:
    from ridgeback_autonomy.perception.target_localization.detector_node import (
        DETECTOR_FPS_DEFAULT,
    )

    detector = _StubDetector()
    node = _make_node(detector)
    try:
        assert DETECTOR_FPS_DEFAULT == 10.0
        assert node.detector_fps == 10.0
        assert node.detector_period == pytest.approx(0.1)
    finally:
        node.destroy_node()


def test_diagnostics_are_off_by_default_and_cost_nothing(ros_context) -> None:
    detector = _StubDetector()
    node = _make_node(detector)
    node.detections_pub.publish = lambda msg: None
    try:
        assert node.diagnostics is None

        # Every instrumentation seam stays inert rather than branching at each
        # call site, so the default path runs the same code as before.
        with node.timed_stage('inference'):
            pass
        node.record_stage_elapsed('inference', 1_000_000)
        node.synchronize_cuda_for_timing()
        node.run_detection_step(_color_message())

        assert node.diagnostics is None
        assert detector.detect_calls == 1
    finally:
        node.destroy_node()


def test_enabled_diagnostics_record_stages_without_changing_publishing(
    ros_context, monkeypatch,
) -> None:
    detector = _StubDetector()
    node = _make_node(detector, detector_debug=True)
    published: list = []
    node.detections_pub.publish = lambda msg: published.append(msg)
    # Unit proof needs neither torch nor a real CUDA barrier; the live run
    # exercises the diagnostic-only synchronize around inference.
    monkeypatch.setattr(node, 'synchronize_cuda_for_timing', lambda: None)
    try:
        node.run_detection_step(_color_message())

        assert len(published) == 1
        diagnostics = node.diagnostics
        assert diagnostics.steps == 1
        assert diagnostics.step_failures == 0
        for stage in ('decode', 'inference', 'parse', 'publish'):
            assert diagnostics.stage_timing[stage].count == 1
        summary = diagnostics.summary(configured_fps=node.detector_fps)
        assert 'fps configured=10.0' in summary
        assert 'stage inference_ms=count=1' in summary
    finally:
        node.destroy_node()


def test_enabled_diagnostics_count_a_failed_step(ros_context, monkeypatch) -> None:
    detector = _StubDetector()
    node = _make_node(detector, detector_debug=True)
    node.detections_pub.publish = lambda msg: None
    monkeypatch.setattr(node, 'synchronize_cuda_for_timing', lambda: None)
    try:
        detector.raise_next = True
        node.run_detection_step(_color_message())

        assert node.diagnostics.steps == 1
        assert node.diagnostics.step_failures == 1
        # The step is still timed, and nothing was published for it.
        assert node.diagnostics.stage_timing['inference'].count == 1
        assert node.diagnostics.stage_timing['publish'].count == 0
    finally:
        node.destroy_node()


def test_diagnostics_report_superseded_frames_and_achieved_rate() -> None:
    from ridgeback_autonomy.perception.target_localization.detector_node import (
        DetectorDiagnostics,
    )

    diagnostics = DetectorDiagnostics()

    diagnostics.record_frame_arrival(superseded=False)
    diagnostics.record_frame_arrival(superseded=True)
    diagnostics.record_frame_arrival(superseded=True)
    # A steady 100 ms cadence with one long gap in it, the shape a benchmark
    # scene change produces. The first five intervals are cold, so the warm
    # window holds four steady ones plus the gap.
    stamps_ms = [0, 100, 200, 300, 400, 500, 600, 700, 800, 1400, 1500]
    for offset_ms in stamps_ms:
        diagnostics.record_publication(now_ns=1_000_000_000 + offset_ms * 1_000_000)

    summary = diagnostics.summary(configured_fps=10.0)
    assert 'frames rx=3 superseded=2' in summary
    # The gap must not be reported as a rate the detector never ran at: the
    # warm mean here is 200 ms (5 Hz), the warm median is the true 100 ms.
    assert 'fps configured=10.0 achieved=10.00' in summary
    assert 'publish_interval_ms=count=10 first=100.000' in summary
    assert 'warm_n=5 warm_mean=200.000 warm_p50=100.000' in summary

    with pytest.raises(ValueError, match='Unknown timing stage'):
        diagnostics.record_stage('not_a_stage', 1)


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
