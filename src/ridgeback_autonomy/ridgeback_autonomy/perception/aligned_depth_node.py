#!/usr/bin/env python3
"""Aligned depth frame producer (object_localization_documentation/aligned_depth.md).

Publishes the "aligned depth frame" contract: a float32 depth image in meters
on the color camera's pixel grid, stamped with the frame it is aligned to,
with 0/NaN/inf meaning "no depth here". Two interchangeable producers satisfy
the contract behind a single config switch (`depth_source`):

- ``stereoscopic``: passes the camera depth stream through (sim: the
  co-registered gz render; real: the driver's ``aligned_depth_to_color``
  topic — point ``depth_topic`` at it), converting to float meters.
- ``monocular``: predicts metric depth from the RGB stream with
  Depth-Anything V2, aligned by construction.

Consumers subscribe to the output topics and never branch on the source.
This module is deliberately independent of the distance-estimator stack
(geometry.py / g1_camera_measurement_node); it shares no code with it.
"""

from __future__ import annotations

import importlib
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ridgeback_autonomy.perception.core.image_utils import (
    convert_depth_to_meters_message,
    decode_image_message,
)


ALIGNED_DEPTH_TOPIC = 'perception/aligned_depth/image'
ALIGNED_CAMERA_INFO_TOPIC = 'perception/aligned_depth/camera_info'
DEPTH_SOURCE_STEREOSCOPIC = 'stereoscopic'
DEPTH_SOURCE_MONOCULAR = 'monocular'
DEPTH_ANYTHING_MODEL_ID_DEFAULT = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'
# After a load/inference failure the monocular source waits this long before
# re-attempting, instead of disabling itself for the rest of the run. A single
# transient hiccup (e.g. a CUDA OOM) then self-heals rather than silently
# starving every ``monocular`` benchmark row.
MONOCULAR_RETRY_COOLDOWN_S_DEFAULT = 30.0


def decode_depth_to_meters(msg: Image) -> np.ndarray:
    """Decode a depth Image message into float32 meters, invalid pixels kept
    as 0/NaN/inf per the aligned-depth-frame contract.

    Delegates to the shared decoder, which honours ``msg.step`` (row padding).
    A real camera driver may emit row-aligned buffers (``step > width *
    itemsize``); the previous hand-rolled ``reshape(height, width)`` rejected
    every such frame. This stays the contract's single entry point,
    so both importers (this node and ``g1_mask_measurement_node``) get the fix.
    """

    if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
        raise ValueError(f'unsupported depth encoding "{msg.encoding}"')
    return convert_depth_to_meters_message(msg)


def decode_color_to_rgb(msg: Image) -> np.ndarray:
    """Decode a color Image message into an RGB uint8 array.

    Delegates the row decode to the shared step-aware decoder (honours
    ``msg.step``) -- the same fix as the depth path: 
    a real driver may pad rows (``step > width * channels``),
    which the old hand-rolled ``reshape`` rejected, throwing every frame. The
    explicit whitelist stays here to preserve the exact error message and the
    mono rejection. ``produce`` re-contiguous-izes, so returning views is fine.
    """

    if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        raise ValueError(f'unsupported color encoding "{msg.encoding}"')
    image = decode_image_message(msg)
    if msg.encoding == 'rgb8':
        return image
    if msg.encoding == 'bgr8':
        return image[:, :, ::-1]
    if msg.encoding == 'rgba8':
        return image[:, :, :3]
    return image[:, :, :3][:, :, ::-1]  # bgra8


def encode_depth_message(depth_m: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = depth_m.shape
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = msg.width * 4
    msg.data = np.ascontiguousarray(depth_m, dtype=np.float32).tobytes()
    return msg


def camera_info_matches_depth(depth_shape: tuple[int, int], camera_info) -> bool:
    """True when the produced depth grid matches the color ``camera_info`` grid.

    The aligned-depth contract puts depth on the color grid; if a misconfigured
    real camera ships a mismatched pair, republishing the intrinsics anyway
    would mislabel the frame and break deprojection confusingly downstream. 
    Checked at the source so the failure is loud and local.
    """

    return depth_shape == (int(camera_info.height), int(camera_info.width))


def throttle_remainder(last_monotonic: float, now: float, max_fps: float) -> float:
    """Seconds to wait before the next frame to honour ``max_fps``.

    ``max_fps <= 0`` means no cap (returns 0.0). Otherwise the minimum period
    is ``1 / max_fps`` since the previous step; a non-positive remainder (the
    period already elapsed) also returns 0.0.
    """

    if max_fps <= 0.0:
        return 0.0
    return max(0.0, (last_monotonic + 1.0 / max_fps) - now)


class StereoDepthSource:
    """camera depth stream -> aligned depth frame.

    In sim the gz render is co-registered with color by construction; on real
    hardware the input topic must already be the driver-aligned
    ``aligned_depth_to_color`` stream (alignment happens in the driver, not
    here). Either way the producer only converts units and re-stamps nothing:
    the incoming header already carries the aligned grid's stamp and frame.
    """

    input_kind = 'depth'

    def __init__(self, logger) -> None:
        self.logger = logger

    def produce(self, msg: Image) -> tuple[np.ndarray, object] | None:
        try:
            depth_m = decode_depth_to_meters(msg)
        except ValueError as exc:
            self.logger.warn(f'Stereo source skipped a frame: {exc}')
            return None
        return depth_m, msg.header


class MonocularDepthSource:
    """RGB stream -> Depth-Anything V2 metric depth -> aligned depth frame.

    Aligned by construction: the network input is the color image, so the
    output grid is the color grid. The frame is stamped with the color
    message header it was predicted from.
    """

    input_kind = 'color'

    def __init__(
        self,
        model_id: str,
        device: str,
        logger,
        *,
        now_fn=time.monotonic,
        cooldown_s: float = MONOCULAR_RETRY_COOLDOWN_S_DEFAULT,
    ) -> None:
        self.model_id = model_id
        self.device = device or self.resolve_device()
        self.logger = logger
        self._pipeline = None
        # Retry-with-cooldown instead of a permanent fail latch:
        # ``_retry_after`` is the earliest time (``now_fn`` seconds) a new load
        # is allowed. 0 lets the first load run immediately.
        self._now_fn = now_fn
        self._cooldown_s = float(cooldown_s)
        self._retry_after = 0.0

    @staticmethod
    def resolve_device() -> str:
        try:
            torch = importlib.import_module('torch')
            if torch.cuda.is_available():
                return 'cuda'
        except Exception:
            pass
        return 'cpu'

    def _build_pipeline(self):
        """Construct the Depth-Anything pipeline (seam for tests)."""

        transformers = importlib.import_module('transformers')
        return transformers.pipeline(
            task='depth-estimation',
            model=self.model_id,
            device=self.device,
        )

    def _schedule_retry(self, reason: str) -> None:
        """Drop the pipeline and arm the cooldown, warning on each occurrence."""

        self._pipeline = None
        self._retry_after = self._now_fn() + self._cooldown_s
        self.logger.warn(
            f'Depth-Anything unavailable ({reason}); retrying after '
            f'{self._cooldown_s:.0f} s cooldown.')

    def load(self) -> bool:
        if self._pipeline is not None:
            return True
        if self._now_fn() < self._retry_after:
            return False
        self.logger.info(f'Loading Depth-Anything model {self.model_id} on {self.device}')
        try:
            self._pipeline = self._build_pipeline()
        except Exception as exc:
            self._schedule_retry(f'load failed: {exc}')
            return False
        self.logger.info('Depth-Anything model loaded.')
        return True

    def produce(self, msg: Image) -> tuple[np.ndarray, object] | None:
        if not self.load():
            return None
        try:
            rgb = decode_color_to_rgb(msg)
        except ValueError as exc:
            self.logger.warn(f'Monocular source skipped a frame: {exc}')
            return None

        pil = importlib.import_module('PIL.Image')
        try:
            outputs = self._pipeline(pil.fromarray(np.ascontiguousarray(rgb)))
        except Exception as exc:
            self._schedule_retry(f'inference failed: {exc}')
            return None

        predicted = outputs.get('predicted_depth')
        if predicted is None:
            self.logger.warn('Depth-Anything returned no predicted_depth output.')
            return None
        if hasattr(predicted, 'detach'):
            depth_m = predicted.detach().cpu().numpy()
        else:
            depth_m = np.asarray(predicted)
        depth_m = np.squeeze(depth_m).astype(np.float32)
        if depth_m.ndim != 2:
            self.logger.warn(f'Depth-Anything returned unexpected shape {depth_m.shape}.')
            return None

        if depth_m.shape != (msg.height, msg.width):
            cv2 = importlib.import_module('cv2')
            depth_m = cv2.resize(
                depth_m,
                (msg.width, msg.height),
                interpolation=cv2.INTER_LINEAR,
            )
        return depth_m, msg.header


class AlignedDepthNode(Node):
    def __init__(self, source=None) -> None:
        super().__init__('aligned_depth_node')

        self.declare_parameter('depth_source', DEPTH_SOURCE_STEREOSCOPIC)
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('camera_info_topic', 'sensors/camera_0/color/camera_info')
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_TOPIC)
        self.declare_parameter('aligned_camera_info_topic', ALIGNED_CAMERA_INFO_TOPIC)
        self.declare_parameter('depth_anything_model_id', DEPTH_ANYTHING_MODEL_ID_DEFAULT)
        self.declare_parameter('depth_anything_device', '')
        # Cap producer cadence (default 0 = unlimited = current behavior).
        # Set on a real robot sharing one GPU with the detector + segmenter so
        # monocular inference can't starve them; benchmark leaves it 0.
        self.declare_parameter('max_fps', 0.0)

        depth_source = str(self.get_parameter('depth_source').value).strip().lower()
        # Seam: tests inject a stub source (with an ``input_kind`` and a
        # scripted ``produce``) so the node stands up without a real model,
        # mirroring G1DetectorNode(detector=...).
        self.source = source or self._build_source(depth_source)
        self.max_fps = float(self.get_parameter('max_fps').value)

        self.latest_input_msg: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.input_lock = threading.Lock()
        self.input_event = threading.Event()
        self.stop_event = threading.Event()
        self.last_step_monotonic = 0.0
        self.last_error_log_monotonic = 0.0
        self.last_grid_warn_monotonic = 0.0

        input_topic = str(
            self.get_parameter('depth_topic').value
            if self.source.input_kind == 'depth'
            else self.get_parameter('color_topic').value
        )
        self.create_subscription(
            Image, input_topic, self.input_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback, qos_profile_sensor_data)

        self.depth_pub = self.create_publisher(
            Image, str(self.get_parameter('aligned_depth_topic').value),
            qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('aligned_camera_info_topic').value),
            qos_profile_sensor_data)

        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(
            f'Aligned depth producer up: source={depth_source} input="{input_topic}"')

    def _build_source(self, depth_source: str):
        """Dispatch the configured producer (production path; the test seam
        bypasses this by injecting a source)."""

        if depth_source == DEPTH_SOURCE_STEREOSCOPIC:
            return StereoDepthSource(self.get_logger())
        if depth_source == DEPTH_SOURCE_MONOCULAR:
            return MonocularDepthSource(
                str(self.get_parameter('depth_anything_model_id').value),
                str(self.get_parameter('depth_anything_device').value),
                self.get_logger(),
                # Node clock so the cooldown respects use_sim_time.
                now_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
            )
        raise ValueError(
            f'Unknown depth_source "{depth_source}"; expected '
            f'"{DEPTH_SOURCE_STEREOSCOPIC}" or "{DEPTH_SOURCE_MONOCULAR}".'
        )

    def input_callback(self, msg: Image) -> None:
        with self.input_lock:
            self.latest_input_msg = msg
        self.input_event.set()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.input_event.wait(timeout=0.1):
                continue
            self.input_event.clear()

            with self.input_lock:
                msg = self.latest_input_msg
                self.latest_input_msg = None
            if msg is None:
                continue

            # optional cadence cap before the (expensive) produce step.
            if self.max_fps > 0.0:
                remainder = throttle_remainder(
                    self.last_step_monotonic, time.monotonic(), self.max_fps)
                if remainder > 0.0 and self.stop_event.wait(remainder):
                    break

            self.run_producer_step(msg)
            # Advance outside the guard so a frame that always fails cannot
            # hot-loop the worker.
            self.last_step_monotonic = time.monotonic()

    def run_producer_step(self, msg: Image) -> None:
        """Produce, publish, and republish intrinsics for one input frame.

        Guarded against any producer/encode/publish error: this is the
        third daemon worker (with the detector and mask nodes) and the only one
        that lacked the guard, so an unexpected failure silently killed it while
        the node still looked alive. ``KeyboardInterrupt``/``SystemExit`` are
        not ``Exception`` subclasses, so shutdown is unaffected.
        """

        try:
            frame = self.source.produce(msg)
            if frame is None:
                return
            depth_m, header = frame
            self.depth_pub.publish(encode_depth_message(depth_m, header))
            self.publish_aligned_camera_info(depth_m.shape, header)
        except Exception as exc:  # noqa: BLE001 - worker must survive any frame
            self.log_producer_error(exc)

    def publish_aligned_camera_info(self, depth_shape, header) -> None:
        """Republish the color intrinsics stamped with the frame, so consumers
        deproject with the grid's true intrinsics instead of static FoV
        constants -- but only when the depth grid matches."""

        camera_info = self.latest_camera_info
        if camera_info is None:
            return
        if not camera_info_matches_depth(depth_shape, camera_info):
            self.log_grid_mismatch(depth_shape, camera_info)
            return
        info = CameraInfo()
        info.header.stamp = header.stamp
        info.header.frame_id = camera_info.header.frame_id
        info.height = camera_info.height
        info.width = camera_info.width
        info.distortion_model = camera_info.distortion_model
        info.d = camera_info.d
        info.k = camera_info.k
        info.r = camera_info.r
        info.p = camera_info.p
        self.camera_info_pub.publish(info)

    def log_producer_error(self, exc: Exception) -> None:
        """Warn (throttled) that a producer frame failed."""

        now = time.monotonic()
        if now - self.last_error_log_monotonic < 5.0:
            return
        self.last_error_log_monotonic = now
        self.get_logger().warn(f'Depth producer frame failed, skipping: {exc}')

    def log_grid_mismatch(self, depth_shape, camera_info) -> None:
        """Warn (throttled) that depth and camera_info grids disagree."""

        now = time.monotonic()
        if now - self.last_grid_warn_monotonic < 5.0:
            return
        self.last_grid_warn_monotonic = now
        self.get_logger().warn(
            f'Depth grid {depth_shape[0]}x{depth_shape[1]} does not match '
            f'camera_info grid {camera_info.height}x{camera_info.width}; '
            'skipping intrinsics republish (consumers degrade to no-intrinsics).')

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.input_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AlignedDepthNode()
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
