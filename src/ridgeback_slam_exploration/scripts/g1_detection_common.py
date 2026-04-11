import json
import math
import os

import cv2
import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image


DETECTION_MODEL_DEFAULT = 'google/owlv2-base-patch16-ensemble'
DETECTION_LABELS = ['humanoid robot']
DETECTION_THRESHOLD = 0.55
NMS_IOU_THRESHOLD = 0.5

SYNC_QUEUE_SIZE_DEFAULT = 10
SYNC_SLOP_SEC_DEFAULT = 0.1
DEPTH_MAX_METERS_DEFAULT = 10.0

TORSO_FOCUS_X_MIN = 0.24
TORSO_FOCUS_X_MAX = 0.76
TORSO_FOCUS_Y_MIN = 0.18
TORSO_FOCUS_Y_MAX = 0.62


def load_config(config_path: str) -> dict:
    if not os.path.exists(config_path):
        return {
            'camera': {
                'depth_hfov_deg': 87.0,
                'depth_vfov_deg': 58.0,
                'pitch_deg': 0.0,
                'height_m': 0.50476,
            }
        }
    with open(config_path, 'r', encoding='utf-8') as stream:
        return json.load(stream)


def compute_iou(box_a, box_b):
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def nms(detections, iou_threshold):
    detections.sort(key=lambda detection: detection['score'], reverse=True)
    keep = []
    for detection in detections:
        if any(
            compute_iou(detection['bbox_pixel'], kept['bbox_pixel']) > iou_threshold
            for kept in keep
        ):
            continue
        keep.append(detection)
    return keep


def parse_owl_detections(outputs: list, img_width: int, img_height: int,
                         threshold: float) -> dict:
    result = {
        'detected': False,
        'count': 0,
        'robots': [],
        'img_width': img_width,
        'img_height': img_height,
    }

    for detection in outputs:
        score = detection['score']
        if score < threshold:
            continue

        box = detection['box']
        x1 = max(0, min(img_width - 1, int(box['xmin'])))
        y1 = max(0, min(img_height - 1, int(box['ymin'])))
        x2 = max(x1 + 1, min(img_width, int(math.ceil(box['xmax']))))
        y2 = max(y1 + 1, min(img_height, int(math.ceil(box['ymax']))))

        result['robots'].append({
            'bbox_pixel': [x1, y1, x2, y2],
            'center_px': [(x1 + x2) // 2, (y1 + y2) // 2],
            'score': score,
            'label': detection['label'],
        })

    result['robots'] = nms(result['robots'], NMS_IOU_THRESHOLD)
    result['count'] = len(result['robots'])
    result['detected'] = result['count'] > 0
    return result


def focus_bbox(bbox_pixel):
    x1, y1, x2, y2 = bbox_pixel
    width = max(x2 - x1, 1)
    height = max(y2 - y1, 1)

    focus_x1 = x1 + int(math.floor(width * TORSO_FOCUS_X_MIN))
    focus_x2 = x1 + int(math.ceil(width * TORSO_FOCUS_X_MAX))
    focus_y1 = y1 + int(math.floor(height * TORSO_FOCUS_Y_MIN))
    focus_y2 = y1 + int(math.ceil(height * TORSO_FOCUS_Y_MAX))

    focus_x1 = max(x1, min(x2 - 1, focus_x1))
    focus_y1 = max(y1, min(y2 - 1, focus_y1))
    focus_x2 = max(focus_x1 + 1, min(x2, focus_x2))
    focus_y2 = max(focus_y1 + 1, min(y2, focus_y2))
    return [focus_x1, focus_y1, focus_x2, focus_y2]


def bgr_frame_to_pil(image: np.ndarray) -> PILImage.Image:
    return PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))


def convert_color_image_message(msg: Image) -> np.ndarray:
    image = decode_image_message(msg)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'bgr8':
        return image
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if msg.encoding == 'mono16':
        return cv2.cvtColor(normalize_to_uint8(image), cv2.COLOR_GRAY2BGR)
    raise ValueError('unsupported color encoding')


def convert_depth_to_meters_message(msg: Image) -> np.ndarray:
    image = decode_image_message(msg)
    if msg.encoding in ('16UC1', 'mono16'):
        return image.astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        return image.astype(np.float32)
    raise ValueError('unsupported depth encoding')


def decode_image_message(msg: Image) -> np.ndarray:
    if msg.encoding in ('rgb8', 'bgr8'):
        return decode_buffer(msg, np.uint8, channels=3)
    if msg.encoding in ('rgba8', 'bgra8'):
        return decode_buffer(msg, np.uint8, channels=4)
    if msg.encoding == 'mono8':
        return decode_buffer(msg, np.uint8, channels=1)
    if msg.encoding in ('mono16', '16UC1'):
        return decode_buffer(msg, np.uint16, channels=1)
    if msg.encoding == '32FC1':
        return decode_buffer(msg, np.float32, channels=1)
    raise ValueError('unsupported encoding')


def decode_buffer(msg: Image, dtype: np.dtype, channels: int) -> np.ndarray:
    dtype = np.dtype(dtype)
    row_elements = msg.step // dtype.itemsize
    expected_columns = msg.width * channels

    if row_elements < expected_columns:
        raise ValueError('message step is smaller than expected image width')

    buffer = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_elements)
    cropped = buffer[:, :expected_columns]

    if channels == 1:
        return cropped.reshape(msg.height, msg.width)
    return cropped.reshape(msg.height, msg.width, channels)


def normalize_to_uint8(image: np.ndarray) -> np.ndarray:
    min_value = float(np.min(image))
    max_value = float(np.max(image))
    if math.isclose(min_value, max_value):
        return np.zeros(image.shape, dtype=np.uint8)
    scaled = (image.astype(np.float32) - min_value) / (max_value - min_value)
    return np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)
