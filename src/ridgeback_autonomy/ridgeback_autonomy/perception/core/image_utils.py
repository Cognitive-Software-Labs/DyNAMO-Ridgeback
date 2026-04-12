from __future__ import annotations

import math

import cv2
import numpy as np
from sensor_msgs.msg import Image


def bgr_frame_to_pil(image: np.ndarray):
    from PIL import Image as PILImage

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

