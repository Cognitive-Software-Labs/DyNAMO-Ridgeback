from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from ridgeback_autonomy.perception.aligned_depth_node import (
    decode_color_to_rgb,
    decode_depth_to_meters,
    encode_depth_message,
)


def make_image(encoding: str, array: np.ndarray) -> Image:
    msg = Image()
    msg.encoding = encoding
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.data = array.tobytes()
    return msg


def test_decode_depth_16uc1_millimeters_to_meters() -> None:
    raw = np.array([[1000, 2500], [0, 65535]], dtype=np.uint16)

    depth = decode_depth_to_meters(make_image('16UC1', raw))

    assert depth.dtype == np.float32
    assert np.allclose(depth, [[1.0, 2.5], [0.0, 65.535]])


def test_decode_depth_mono16_uses_the_millimeter_path() -> None:
    raw = np.array([[500]], dtype=np.uint16)

    assert np.allclose(decode_depth_to_meters(make_image('mono16', raw)), [[0.5]])


def test_decode_depth_32fc1_passthrough_keeps_invalids() -> None:
    raw = np.array([[1.5, 0.0], [np.nan, np.inf]], dtype=np.float32)

    depth = decode_depth_to_meters(make_image('32FC1', raw))

    assert depth.dtype == np.float32
    np.testing.assert_array_equal(depth, raw)


def test_decode_depth_rejects_unknown_encoding() -> None:
    raw = np.zeros((2, 2), dtype=np.uint16)

    with pytest.raises(ValueError, match='unsupported depth encoding'):
        decode_depth_to_meters(make_image('8UC1', raw))


def test_decode_color_rgb8_passthrough_and_bgr8_swaps_channels() -> None:
    rgb = np.arange(2 * 2 * 3, dtype=np.uint8).reshape(2, 2, 3)

    np.testing.assert_array_equal(decode_color_to_rgb(make_image('rgb8', rgb)), rgb)
    np.testing.assert_array_equal(
        decode_color_to_rgb(make_image('bgr8', rgb)), rgb[:, :, ::-1])


def test_decode_color_rgba8_and_bgra8_drop_alpha() -> None:
    rgba = np.arange(2 * 2 * 4, dtype=np.uint8).reshape(2, 2, 4)

    np.testing.assert_array_equal(
        decode_color_to_rgb(make_image('rgba8', rgba)), rgba[:, :, :3])
    np.testing.assert_array_equal(
        decode_color_to_rgb(make_image('bgra8', rgba)), rgba[:, :, 2::-1])


def test_decode_color_rejects_unknown_encoding() -> None:
    raw = np.zeros((2, 2, 3), dtype=np.uint8)

    with pytest.raises(ValueError, match='unsupported color encoding'):
        decode_color_to_rgb(make_image('yuv422', raw))


def test_encode_depth_message_round_trip() -> None:
    depth = np.array([[0.5, 2.0], [np.nan, 10.0]], dtype=np.float32)
    header = Header()
    header.frame_id = 'camera_0_color_optical'

    msg = encode_depth_message(depth, header)

    assert msg.encoding == '32FC1'
    assert (msg.height, msg.width) == depth.shape
    assert msg.step == depth.shape[1] * 4
    assert msg.header.frame_id == 'camera_0_color_optical'
    np.testing.assert_array_equal(decode_depth_to_meters(msg), depth)
