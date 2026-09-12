#!/usr/bin/env python3

from __future__ import annotations

from contextlib import contextmanager
import importlib
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.messages import build_detections_message
from ridgeback_autonomy.msg import TargetDetections
from ridgeback_autonomy.perception.target_localization.contracts import RAW_DETECTIONS_TOPIC
from ridgeback_autonomy.perception.target_localization.core.detection import (
    DETECTION_MODEL_DEFAULT,
    DETECTION_THRESHOLD,
    OwlV2Detector,
    parse_owl_detections,
)
from ridgeback_autonomy.perception.target_localization.core.image_utils import (
    bgr_frame_to_pil,
    convert_color_image_message,
)
from ridgeback_autonomy.perception.target_localization.core.timing import TimingStats


DETECTOR_FPS_DEFAULT = 10.0

DETECTOR_TIMING_STAGE_NAMES = (
    'gate_wait',
    'frame_slot_age',
    'decode',
    'inference',
    'parse',
    'publish',
    'detector_step',
    'cuda_sync',
)


class DetectorDiagnostics:
    """Bounded cold/warm accounting for the detector's own cadence and stages.

    The detector sets the cadence every downstream consumer inherits, but until
    now nothing measured it from inside: its cost could only be inferred from
    the gap between detections seen elsewhere, which conflates inference with
    the deliberate throttle wait. Splitting ``gate_wait`` from ``detector_step``
    separates the two, so a rate that misses its setpoint can be attributed to
    the model rather than the schedule (or the other way round).

    ``superseded`` counts frames that arrived into the latest-wins slot and were
    overwritten before the worker consumed them. That is by design whenever the
    camera outruns the detector; it is reported so the design is visible rather
    than assumed.
    """

    def __init__(self) -> None:
        self.frames_rx = 0
        self.frames_superseded = 0
        self.steps = 0
        self.step_failures = 0
        self.publish_interval = TimingStats()
        self.last_publish_monotonic_ns: int | None = None
        self.stage_timing = {
            name: TimingStats() for name in DETECTOR_TIMING_STAGE_NAMES
        }

    def record_frame_arrival(self, *, superseded: bool) -> None:
        self.frames_rx += 1
        if superseded:
            self.frames_superseded += 1

    def record_stage(self, name: str, elapsed_ns: int) -> None:
        if name not in self.stage_timing:
            raise ValueError(f'Unknown timing stage "{name}".')
        self.stage_timing[name].record(elapsed_ns)

    def record_step(self, *, failed: bool) -> None:
        self.steps += 1
        if failed:
            self.step_failures += 1

    def record_publication(self, *, now_ns: int | None = None) -> None:
        """Account for one published batch and the gap since the previous one."""

        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        if self.last_publish_monotonic_ns is not None:
            self.publish_interval.record(now_ns - self.last_publish_monotonic_ns)
        self.last_publish_monotonic_ns = now_ns

    def summary(self, *, configured_fps: float) -> str:
        # From the warm median, not the warm mean. A benchmark scene change or
        # a briefly paused camera contributes an interval several times the
        # cadence, and a mean reports those as a rate the detector never ran at.
        achieved = 'n/a'
        median_ns = self.publish_interval.warm_percentile_ns(0.50)
        if median_ns:
            achieved = f'{1e9 / median_ns:.2f}'
        lines = [
            f'detector: frames rx={self.frames_rx} '
            f'superseded={self.frames_superseded} | steps={self.steps} '
            f'failed={self.step_failures} | fps configured={configured_fps:.1f} '
            f'achieved={achieved}',
            f'  publish_interval_ms={self.publish_interval.format_ms()}',
        ]
        lines.extend(
            f'  stage {name}_ms={stats.format_ms()}'
            for name, stats in self.stage_timing.items()
        )
        return '\n'.join(lines)


class TargetDetectorNode(Node):
    def __init__(self, detector=None, **node_kwargs) -> None:
        # ``node_kwargs`` reaches rclpy's Node: tests pass
        # ``parameter_overrides`` to stand the node up on a chosen
        # configuration without a launch file or CLI arguments.
        super().__init__('target_detector_node', **node_kwargs)

        self.declare_parameter('detection_model', DETECTION_MODEL_DEFAULT)
        self.declare_parameter('detection_threshold', DETECTION_THRESHOLD)
        self.declare_parameter('detector_fps', DETECTOR_FPS_DEFAULT)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        # Periodic accounting of the detector's own cadence and per-stage cost.
        # Off by default: it is an investigation aid, not pipeline work, and it
        # adds CUDA barriers that the production path has no reason to pay for.
        self.declare_parameter('detector_debug', False)
        self.declare_parameter('detector_debug_period_s', 5.0)

        self.detection_model = self.get_parameter('detection_model').value
        self.detection_threshold = float(self.get_parameter('detection_threshold').value)
        self.detector_fps = max(0.1, float(self.get_parameter('detector_fps').value))
        self.detector_period = 1.0 / self.detector_fps
        self.color_topic = self.get_parameter('color_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.diagnostics: DetectorDiagnostics | None = (
            DetectorDiagnostics()
            if bool(self.get_parameter('detector_debug').value) else None)

        # Seam: tests inject a stub detector (whose load() is a no-op) so the
        # node stands up without pulling in OWLv2/torch.
        self.detector = detector or OwlV2Detector(self.detection_model, self.get_logger())
        self.detector.load()
        self.last_detection_time = 0.0
        self.last_error_log_monotonic = 0.0
        self.latest_color_msg: Image | None = None
        self.latest_color_arrival_ns: int | None = None
        self.processing_lock = threading.Lock()
        self.process_event = threading.Event()
        self.stop_event = threading.Event()

        self.detections_pub = self.create_publisher(
            TargetDetections,
            self.detections_topic,
            10,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.on_color_image,
            qos_profile_sensor_data,
        )
        # The timer runs on the executor, so a report still arrives while the
        # worker thread is inside a model call.
        if self.diagnostics is not None:
            self.create_timer(
                float(self.get_parameter('detector_debug_period_s').value),
                self.log_detector_diagnostics)

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(f'Subscribed to color stream: {self.color_topic}')
        self.get_logger().info(f'Publishing raw detections on: {self.detections_topic}')
        self.get_logger().info(f'Running detector at up to {self.detector_fps:.1f} FPS')

    def log_detector_diagnostics(self) -> None:
        with self.processing_lock:
            summary = self.diagnostics.summary(configured_fps=self.detector_fps)
        self.get_logger().info(summary)

    @contextmanager
    def timed_stage(self, name: str):
        """Time one diagnostic stage. A no-op while the diagnostic is off."""

        if self.diagnostics is None:
            yield
            return
        started_ns = time.monotonic_ns()
        try:
            yield
        finally:
            self.record_stage_elapsed(name, time.monotonic_ns() - started_ns)

    def record_stage_elapsed(self, name: str, elapsed_ns: int) -> None:
        """Record one already-measured stage duration."""

        if self.diagnostics is None:
            return
        with self.processing_lock:
            self.diagnostics.record_stage(name, elapsed_ns)

    def synchronize_cuda_for_timing(self) -> None:
        """Make the optional GPU stage timing honest, and account for its cost.

        Inference returns as soon as the kernels are queued, so without a
        barrier the ``inference`` stage would measure launch overhead and the
        real cost would land on whichever stage first reads the outputs.
        """

        if self.diagnostics is None:
            return
        with self.timed_stage('cuda_sync'):
            try:
                torch = importlib.import_module('torch')
                if torch.cuda.is_available():
                    torch.cuda.synchronize()
            except Exception:  # noqa: BLE001 - diagnostics must never break inference
                # A CPU-only run or unavailable CUDA runtime needs no barrier.
                pass

    def on_color_image(self, color_msg: Image) -> None:
        arrival_ns = time.monotonic_ns()
        with self.processing_lock:
            if self.diagnostics is not None:
                self.diagnostics.record_frame_arrival(
                    superseded=self.latest_color_msg is not None)
            self.latest_color_msg = color_msg
            self.latest_color_arrival_ns = arrival_ns
        self.process_event.set()

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            with self.timed_stage('gate_wait'):
                next_allowed_time = self.last_detection_time + self.detector_period
                wait_time = next_allowed_time - time.monotonic()
                if wait_time > 0.0 and self.stop_event.wait(wait_time):
                    return

            with self.processing_lock:
                color_msg = self.latest_color_msg
                arrival_ns = self.latest_color_arrival_ns
                self.latest_color_msg = None

            if color_msg is None:
                continue

            # The period is a floor on step *starts*, not a gap between a step
            # ending and the next beginning: advancing the clock here holds the
            # configured rate instead of stretching every cycle by however long
            # inference took. Advancing it before the step rather than after
            # also keeps the older guarantee that a frame which always raises
            # still waits a full period and cannot hot-loop.
            self.last_detection_time = time.monotonic()
            if arrival_ns is not None:
                self.record_stage_elapsed(
                    'frame_slot_age', time.monotonic_ns() - arrival_ns)
            with self.timed_stage('detector_step'):
                self.run_detection_step(color_msg)

    def run_detection_step(self, color_msg: Image) -> None:
        """Process one frame, surviving any inference/publish failure.

        Only image-decode errors were caught before, so one transient CUDA OOM
        (or any publish error) killed the worker thread while the node still
        looked alive -- every downstream consumer then silently starved.
        """
        failed = False
        try:
            self.process_color_image(color_msg)
        except Exception as exc:  # noqa: BLE001 - worker must survive any frame
            failed = True
            self.log_detection_error(exc)
        if self.diagnostics is not None:
            with self.processing_lock:
                self.diagnostics.record_step(failed=failed)

    def log_detection_error(self, exc: Exception) -> None:
        """Warn (throttled) that a detection frame failed."""

        now = time.monotonic()
        if now - self.last_error_log_monotonic < 5.0:
            return
        self.last_error_log_monotonic = now
        self.get_logger().warn(f'Detection frame failed, skipping: {exc}')

    def process_color_image(self, color_msg: Image) -> None:
        with self.timed_stage('decode'):
            try:
                frame = convert_color_image_message(color_msg)
            except ValueError as exc:
                self.get_logger().warn(
                    f'Cannot decode color image ({color_msg.encoding}): {exc}')
                return
            image = bgr_frame_to_pil(frame)

        self.synchronize_cuda_for_timing()
        with self.timed_stage('inference'):
            outputs = self.detector.detect(image, threshold=self.detection_threshold)
            self.synchronize_cuda_for_timing()

        with self.timed_stage('parse'):
            batch = parse_owl_detections(
                outputs,
                frame.shape[1],
                frame.shape[0],
                self.detection_threshold,
            )
        with self.timed_stage('publish'):
            self.detections_pub.publish(build_detections_message(batch, color_msg.header))
        if self.diagnostics is not None:
            with self.processing_lock:
                self.diagnostics.record_publication()

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.process_event.set()
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = TargetDetectorNode()
    except RuntimeError as exc:
        # Perception enabled without the venv (no transformers/torch). Fail
        # cleanly with a clear message instead of dumping a traceback.
        get_logger('target_detector').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
