from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image

from ridgeback_autonomy.perception.core.image_utils import (
    convert_color_image_message,
    convert_depth_to_meters_message,
    normalize_to_uint8,
)


def make_image_msg(array: np.ndarray, encoding: str, step: int | None = None) -> Image:
    msg = Image()
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(step if step is not None else array.strides[0])
    msg.data = array.tobytes()
    return msg


def test_convert_color_image_message_rgb8_to_bgr() -> None:
    rgb = np.array([[[10, 20, 30]]], dtype=np.uint8)
    msg = make_image_msg(rgb, 'rgb8')

    converted = convert_color_image_message(msg)

    assert converted.shape == (1, 1, 3)
    assert converted.tolist() == [[[30, 20, 10]]]


def test_convert_depth_to_meters_from_16uc1() -> None:
    depth_mm = np.array([[1000, 2500]], dtype=np.uint16)
    msg = make_image_msg(depth_mm, '16UC1')

    converted = convert_depth_to_meters_message(msg)

    assert converted.dtype == np.float32
    assert converted.tolist() == [[1.0, 2.5]]


def test_convert_color_image_message_supports_padded_rows() -> None:
    padded = np.array([[1, 2, 3, 4, 5, 6, 99, 99]], dtype=np.uint8)
    msg = Image()
    msg.height = 1
    msg.width = 2
    msg.encoding = 'rgb8'
    msg.is_bigendian = False
    msg.step = 8
    msg.data = padded.tobytes()

    converted = convert_color_image_message(msg)

    assert converted.tolist() == [[[3, 2, 1], [6, 5, 4]]]


def test_normalize_to_uint8_flat_image_returns_zeroes() -> None:
    image = np.full((2, 2), 7.0, dtype=np.float32)

    normalized = normalize_to_uint8(image)

    assert normalized.dtype == np.uint8
    assert np.count_nonzero(normalized) == 0
