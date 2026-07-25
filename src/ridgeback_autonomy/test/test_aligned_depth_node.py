from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from ridgeback_autonomy.perception.aligned_depth_node import (
    MonocularDepthSource,
    camera_info_matches_depth,
    decode_color_to_rgb,
    decode_depth_to_meters,
    encode_depth_message,
    throttle_remainder,
)


class _NullLogger:
    def info(self, *args, **kwargs) -> None:
        pass

    def warn(self, *args, **kwargs) -> None:
        pass

    def error(self, *args, **kwargs) -> None:
        pass


def make_image(encoding: str, array: np.ndarray) -> Image:
    msg = Image()
    msg.encoding = encoding
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    # Packed rows: byte stride of the contiguous array (the decoder honours it).
    msg.step = int(np.ascontiguousarray(array).strides[0])
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


def test_decode_depth_16uc1_honours_row_padding() -> None:
    # A real driver may emit row-aligned buffers (step > width*itemsize); the
    # decoder must strip the padding, not choke on it.
    rows = [[1000, 2500], [500, 0]]
    itemsize = np.dtype(np.uint16).itemsize
    width, pad_elements = 2, 2
    buf = bytearray()
    for row in rows:
        buf += np.array(row + [9999] * pad_elements, dtype=np.uint16).tobytes()

    msg = Image()
    msg.encoding = '16UC1'
    msg.height, msg.width = 2, width
    msg.step = (width + pad_elements) * itemsize
    msg.data = bytes(buf)

    depth = decode_depth_to_meters(msg)

    assert depth.shape == (2, 2)
    assert np.allclose(depth, [[1.0, 2.5], [0.5, 0.0]])


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


def test_monocular_source_retries_after_cooldown(monkeypatch) -> None:
    # A transient load/inference failure must NOT permanently disable the
    # source. It arms a cooldown, skips while it is active, then re-attempts
    # once the cooldown elapses -- here the retry succeeds.
    clock = {'t': 100.0}
    source = MonocularDepthSource(
        'model', 'cpu', _NullLogger(),
        now_fn=lambda: clock['t'], cooldown_s=30.0)

    attempts = {'n': 0}

    def build():
        attempts['n'] += 1
        if attempts['n'] == 1:
            raise RuntimeError('transient CUDA OOM')
        return object()  # a "loaded pipeline"

    monkeypatch.setattr(source, '_build_pipeline', build)

    assert source.load() is False        # first attempt fails -> cooldown armed
    assert attempts['n'] == 1

    clock['t'] = 120.0                    # still inside the 30 s cooldown
    assert source.load() is False
    assert attempts['n'] == 1             # no re-attempt while cooling down

    clock['t'] = 131.0                    # cooldown elapsed
    assert source.load() is True          # re-attempts and succeeds
    assert attempts['n'] == 2

    assert source.load() is True          # stays loaded, no further attempts
    assert attempts['n'] == 2


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


def test_decode_color_honours_row_padding() -> None:
    # A real driver may pad rows (step > width*channels); the decoder must strip
    # the padding, not choke on it (the color twin of the depth-padding case).
    height, width, channels, pad_bytes = 2, 2, 3, 4
    pixels = np.arange(height * width * channels, dtype=np.uint8).reshape(
        height, width, channels)
    buf = bytearray()
    for row in pixels:
        buf += row.tobytes() + bytes(pad_bytes)

    def padded(encoding: str) -> Image:
        msg = Image()
        msg.encoding = encoding
        msg.height, msg.width = height, width
        msg.step = width * channels + pad_bytes
        msg.data = bytes(buf)
        return msg

    np.testing.assert_array_equal(decode_color_to_rgb(padded('rgb8')), pixels)
    np.testing.assert_array_equal(
        decode_color_to_rgb(padded('bgr8')), pixels[:, :, ::-1])


def test_camera_info_matches_depth_true_on_equal_grid() -> None:
    info = CameraInfo()
    info.height, info.width = 480, 640

    assert camera_info_matches_depth((480, 640), info) is True


def test_camera_info_matches_depth_false_on_mismatch() -> None:
    info = CameraInfo()
    info.height, info.width = 480, 640

    assert camera_info_matches_depth((481, 640), info) is False
    assert camera_info_matches_depth((480, 641), info) is False


def test_throttle_remainder_uncapped_is_zero() -> None:
    assert throttle_remainder(100.0, 100.05, 0.0) == 0.0


def test_throttle_remainder_waits_out_the_period() -> None:
    # 10 fps -> 0.1 s period; 0.05 s elapsed -> 0.05 s remaining.
    assert throttle_remainder(100.0, 100.05, 10.0) == pytest.approx(0.05)


def test_throttle_remainder_zero_once_period_elapsed() -> None:
    assert throttle_remainder(100.0, 100.2, 10.0) == 0.0


class _StubSource:
    """Injected in place of a real depth producer: scripted produce(), no model."""

    input_kind = 'depth'

    def __init__(self) -> None:
        self.produce_calls = 0
        self.raise_next = False
        self.frame = None

    def produce(self, msg):
        self.produce_calls += 1
        if self.raise_next:
            raise RuntimeError('simulated producer failure')
        return self.frame


@pytest.fixture
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def _make_node(source):
    from ridgeback_autonomy.perception.aligned_depth_node import AlignedDepthNode

    return AlignedDepthNode(source=source)


def _input_image() -> Image:
    msg = Image()
    msg.encoding = '32FC1'
    msg.height = msg.width = 4
    return msg


def test_producer_worker_survives_exception(ros_context) -> None:
    # An unexpected produce/publish error must not propagate out of the
    # guarded step or leave the worker unable to handle the next good frame.
    source = _StubSource()
    node = _make_node(source)
    published: list = []
    node.depth_pub.publish = lambda m: published.append(m)
    try:
        source.raise_next = True
        node.run_producer_step(_input_image())  # no exception escapes

        source.raise_next = False
        header = Header()
        header.frame_id = 'camera_0_color_optical'
        source.frame = (np.zeros((4, 4), dtype=np.float32), header)
        node.run_producer_step(_input_image())

        assert source.produce_calls == 2
        assert len(published) == 1
    finally:
        node.destroy_node()
