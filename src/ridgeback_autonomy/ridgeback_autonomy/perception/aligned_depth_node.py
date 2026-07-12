#!/usr/bin/env python3
"""Aligned depth frame producer (object_localization_documentation/depth_based_path.md).

Publishes the "aligned depth frame" contract: a float32 depth image in meters
on the color camera's pixel grid, stamped with the frame it is aligned to,
with 0/NaN/inf meaning "no depth here". Two interchangeable producers satisfy
the contract behind a single config switch (`depth_source`):

- ``stereo``: passes the camera depth stream through (sim: the co-registered
  gz render; real: the driver's ``aligned_depth_to_color`` topic — point
  ``depth_topic`` at it), converting to float meters.
- ``depth_anything``: predicts metric depth from the RGB stream with
  Depth-Anything V2, aligned by construction.

Consumers subscribe to the output topics and never branch on the source.
This module is deliberately independent of the distance-estimator stack
(geometry.py / g1_camera_measurement_node); it shares no code with it.
"""

from __future__ import annotations

import importlib
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


ALIGNED_DEPTH_TOPIC = 'perception/aligned_depth/image'
ALIGNED_CAMERA_INFO_TOPIC = 'perception/aligned_depth/camera_info'
DEPTH_SOURCE_STEREO = 'stereo'
DEPTH_SOURCE_DEPTH_ANYTHING = 'depth_anything'
DEPTH_ANYTHING_MODEL_ID_DEFAULT = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'


def decode_depth_to_meters(msg: Image) -> np.ndarray:
    """Decode a depth Image message into float32 meters, invalid pixels kept
    as 0/NaN/inf per the aligned-depth-frame contract."""
    if msg.encoding in ('16UC1', 'mono16'):
        raw = np.frombuffer(msg.data, dtype=np.uint16)
        return raw.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        raw = np.frombuffer(msg.data, dtype=np.float32)
        return raw.reshape(msg.height, msg.width).astype(np.float32, copy=True)
    raise ValueError(f'unsupported depth encoding "{msg.encoding}"')


def decode_color_to_rgb(msg: Image) -> np.ndarray:
    """Decode a color Image message into an RGB uint8 array."""
    if msg.encoding in ('rgb8', 'bgr8'):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return raw if msg.encoding == 'rgb8' else raw[:, :, ::-1]
    if msg.encoding in ('rgba8', 'bgra8'):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        rgb = raw[:, :, :3]
        return rgb if msg.encoding == 'rgba8' else rgb[:, :, ::-1]
    raise ValueError(f'unsupported color encoding "{msg.encoding}"')


def encode_depth_message(depth_m: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = depth_m.shape
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = msg.width * 4
    msg.data = np.ascontiguousarray(depth_m, dtype=np.float32).tobytes()
    return msg


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

    def __init__(self, model_id: str, device: str, logger) -> None:
        self.model_id = model_id
        self.device = device or self.resolve_device()
        self.logger = logger
        self._pipeline = None
        self._failed = False

    @staticmethod
    def resolve_device() -> str:
        try:
            torch = importlib.import_module('torch')
            if torch.cuda.is_available():
                return 'cuda'
        except Exception:
            pass
        return 'cpu'

    def load(self) -> bool:
        if self._pipeline is not None:
            return True
        if self._failed:
            return False
        self.logger.info(f'Loading Depth-Anything model {self.model_id} on {self.device}')
        try:
            transformers = importlib.import_module('transformers')
            self._pipeline = transformers.pipeline(
                task='depth-estimation',
                model=self.model_id,
                device=self.device,
            )
        except Exception as exc:
            self.logger.error(f'Cannot load Depth-Anything model ({self.model_id}): {exc}')
            self._failed = True
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
            self.logger.error(f'Depth-Anything inference failed: {exc}')
            self._failed = True
            self._pipeline = None
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
    def __init__(self) -> None:
        super().__init__('aligned_depth_node')

        self.declare_parameter('depth_source', DEPTH_SOURCE_STEREO)
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('camera_info_topic', 'sensors/camera_0/color/camera_info')
        self.declare_parameter('aligned_depth_topic', ALIGNED_DEPTH_TOPIC)
        self.declare_parameter('aligned_camera_info_topic', ALIGNED_CAMERA_INFO_TOPIC)
        self.declare_parameter('depth_anything_model_id', DEPTH_ANYTHING_MODEL_ID_DEFAULT)
        self.declare_parameter('depth_anything_device', '')

        depth_source = str(self.get_parameter('depth_source').value).strip().lower()
        if depth_source == DEPTH_SOURCE_STEREO:
            self.source = StereoDepthSource(self.get_logger())
        elif depth_source == DEPTH_SOURCE_DEPTH_ANYTHING:
            self.source = MonocularDepthSource(
                str(self.get_parameter('depth_anything_model_id').value),
                str(self.get_parameter('depth_anything_device').value),
                self.get_logger(),
            )
        else:
            raise ValueError(
                f'Unknown depth_source "{depth_source}"; expected '
                f'"{DEPTH_SOURCE_STEREO}" or "{DEPTH_SOURCE_DEPTH_ANYTHING}".'
            )

        self.latest_input_msg: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.input_lock = threading.Lock()
        self.input_event = threading.Event()
        self.stop_event = threading.Event()

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

            frame = self.source.produce(msg)
            if frame is None:
                continue
            depth_m, header = frame

            self.depth_pub.publish(encode_depth_message(depth_m, header))
            camera_info = self.latest_camera_info
            if camera_info is not None:
                # Republish the color camera's intrinsics stamped with the
                # frame, so consumers deproject with the grid's true
                # intrinsics instead of static FoV constants.
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
