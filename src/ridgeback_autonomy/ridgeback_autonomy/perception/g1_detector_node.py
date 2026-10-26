#!/usr/bin/env python3

from __future__ import annotations

import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.messages import build_detections_message
from ridgeback_autonomy.msg import G1Detections
from ridgeback_autonomy.perception.core.detection import (
    DETECTION_MODEL_DEFAULT,
    DETECTION_LABELS,
    DETECTION_THRESHOLD,
    NMS_IOU_THRESHOLD,
    OwlV2Detector,
    parse_owl_detections,
)
from ridgeback_autonomy.perception.core.image_utils import (
    bgr_frame_to_pil,
    convert_color_image_message,
)


RAW_DETECTIONS_TOPIC = 'detections/g1/raw'
DETECTOR_FPS_DEFAULT = 5.0
DEBUG_IMAGE_TOPIC = 'debug/g1/detector/top_candidate'


class G1DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_detector_node')

        self.declare_parameter('detection_model', DETECTION_MODEL_DEFAULT)
        self.declare_parameter('detection_threshold', DETECTION_THRESHOLD)
        self.declare_parameter('detector_fps', DETECTOR_FPS_DEFAULT)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('debug_image_topic', DEBUG_IMAGE_TOPIC)

        self.detection_model = self.get_parameter('detection_model').value
        self.detection_threshold = float(self.get_parameter('detection_threshold').value)
        self.detector_fps = max(0.1, float(self.get_parameter('detector_fps').value))
        self.detector_period = 1.0 / self.detector_fps
        self.color_topic = self.get_parameter('color_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.detector = OwlV2Detector(self.detection_model, self.get_logger())
        self.detector.load()
        self.last_detection_time = 0.0
        self.latest_color_msg: Image | None = None
        self.processing_lock = threading.Lock()
        self.process_event = threading.Event()
        self.stop_event = threading.Event()

        self.detections_pub = self.create_publisher(
            G1Detections,
            self.detections_topic,
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.on_color_image,
            qos_profile_sensor_data,
        )
        self.worker_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(f'Subscribed to color stream: {self.color_topic}')
        self.get_logger().info(f'Publishing raw detections on: {self.detections_topic}')
        self.get_logger().info(
            f'Publishing detector debug images on: {self.debug_image_topic}'
        )
        self.get_logger().info(
            'G1 detector config: '
            f'model={self.detection_model}, '
            f'prompts={DETECTION_LABELS}, '
            f'confidence_threshold={self.detection_threshold:.3f}, '
            f'nms_iou_threshold={NMS_IOU_THRESHOLD:.3f}, '
            'roi=full_frame, min_box_filter=none'
        )
        self.get_logger().info(f'Running detector at up to {self.detector_fps:.1f} FPS')

    def on_color_image(self, color_msg: Image) -> None:
        with self.processing_lock:
            self.latest_color_msg = color_msg
        self.process_event.set()

    def processing_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.process_event.wait(timeout=0.1):
                continue
            self.process_event.clear()

            next_allowed_time = self.last_detection_time + self.detector_period
            wait_time = next_allowed_time - time.monotonic()
            if wait_time > 0.0 and self.stop_event.wait(wait_time):
                break

            with self.processing_lock:
                color_msg = self.latest_color_msg
                self.latest_color_msg = None

            if color_msg is None:
                continue

            self.process_color_image(color_msg)
            self.last_detection_time = time.monotonic()

    def process_color_image(self, color_msg: Image) -> None:
        try:
            frame = convert_color_image_message(color_msg)
        except ValueError as exc:
            self.get_logger().warn(f'Cannot decode color image ({color_msg.encoding}): {exc}')
            return

        outputs = self.detector.detect(bgr_frame_to_pil(frame))
        batch = parse_owl_detections(
            outputs,
            frame.shape[1],
            frame.shape[0],
            self.detection_threshold,
        )
        top_candidate = self._top_raw_candidate(outputs, frame.shape[1], frame.shape[0])
        self._log_detection_diagnostics(color_msg, top_candidate, batch.count)
        self.debug_image_pub.publish(
            self._build_debug_image_message(frame, color_msg.header, top_candidate, batch.detected)
        )
        self.detections_pub.publish(build_detections_message(batch, color_msg.header))

    def _top_raw_candidate(
        self,
        outputs: list[dict],
        image_width: int,
        image_height: int,
    ) -> dict | None:
        candidates = []
        for raw_detection in outputs:
            try:
                score = float(raw_detection['score'])
                box = raw_detection['box']
                label = str(raw_detection.get('label', ''))
                xmin = float(box['xmin'])
                ymin = float(box['ymin'])
                xmax = float(box['xmax'])
                ymax = float(box['ymax'])
            except (KeyError, TypeError, ValueError):
                continue

            x1 = max(0, min(image_width - 1, int(xmin)))
            y1 = max(0, min(image_height - 1, int(ymin)))
            x2 = max(x1 + 1, min(image_width, int(xmax + 0.999)))
            y2 = max(y1 + 1, min(image_height, int(ymax + 0.999)))
            candidates.append(
                {
                    'score': score,
                    'label': label,
                    'bbox': (x1, y1, x2, y2),
                    'raw_bbox': (xmin, ymin, xmax, ymax),
                }
            )

        if not candidates:
            return None
        return max(candidates, key=lambda candidate: candidate['score'])

    def _log_detection_diagnostics(
        self,
        color_msg: Image,
        top_candidate: dict | None,
        postprocessed_count: int,
    ) -> None:
        stamp = color_msg.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if top_candidate is None:
            self.get_logger().info(
                'G1 detector diagnostic: '
                f'stamp={stamp_s:.3f}, '
                f'prompt={DETECTION_LABELS}, '
                'top_score=none, top_bbox=none, '
                'passed_threshold=false, '
                'detected=false, '
                f'postprocessed_boxes={postprocessed_count}'
            )
            return

        score = float(top_candidate['score'])
        bbox = top_candidate['bbox']
        passed_threshold = score >= self.detection_threshold
        detected = postprocessed_count > 0
        self.get_logger().info(
            'G1 detector diagnostic: '
            f'stamp={stamp_s:.3f}, '
            f'prompt={DETECTION_LABELS}, '
            f'top_score={score:.3f}, '
            f'top_bbox=({bbox[0]}, {bbox[1]}, {bbox[2]}, {bbox[3]}), '
            f'passed_threshold={str(passed_threshold).lower()}, '
            f'detected={str(detected).lower()}, '
            f'postprocessed_boxes={postprocessed_count}'
        )

    def _build_debug_image_message(
        self,
        frame_bgr,
        header,
        top_candidate: dict | None,
        detected: bool,
    ) -> Image:
        debug = frame_bgr.copy()
        if top_candidate is None:
            label = 'top: none | detected=false'
            color = (0, 0, 255)
        else:
            score = float(top_candidate['score'])
            bbox = top_candidate['bbox']
            passed_threshold = score >= self.detection_threshold
            color = (0, 255, 0) if passed_threshold else (0, 0, 255)
            cv2.rectangle(debug, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
            label = (
                f'top={score:.3f} '
                f'thr={self.detection_threshold:.3f} '
                f'detected={str(detected).lower()}'
            )

        cv2.putText(
            debug,
            label,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        msg = Image()
        msg.header = header
        msg.height = int(debug.shape[0])
        msg.width = int(debug.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(debug.shape[1] * debug.shape[2])
        msg.data = debug.tobytes()
        return msg

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.process_event.set()
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = G1DetectorNode()
    except RuntimeError as exc:
        # Perception enabled without the venv (no transformers/torch). Fail
        # cleanly with a clear message instead of dumping a traceback.
        get_logger('g1_detector').fatal(str(exc))
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
