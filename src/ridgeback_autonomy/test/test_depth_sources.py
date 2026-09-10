from __future__ import annotations

import math

import numpy as np
import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from ridgeback_autonomy.perception.target_localization.core.depth_sources import (
    MONOCULAR_USABLE_RANGE_FRACTION,
    MonocularDepthSource,
    StereoDepthSource,
    build_depth_source,
    decode_color_to_rgb,
    decode_depth_to_meters,
    encode_depth_message,
)
from ridgeback_autonomy.perception.target_localization.core.image_utils import (
    decode_color_to_rgb as canonical_decode_color_to_rgb,
)


class _NullLogger:
    def info(self, *args, **kwargs) -> None:
        pass

    def warn(self, *args, **kwargs) -> None:
        pass

    def error(self, *args, **kwargs) -> None:
        pass


class _RecordingLogger(_NullLogger):
    def __init__(self) -> None:
        self.warnings = []

    def warn(self, message, *args, **kwargs) -> None:
        self.warnings.append(str(message))


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


def test_depth_sources_reexports_the_canonical_color_decoder() -> None:
    assert decode_color_to_rgb is canonical_decode_color_to_rgb


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
        # A metric checkpoint that states its range: this test is about the
        # cooldown, and a pipeline whose config cannot be read is refused on
        # its own grounds.
        return _FakePipeline(_FakeConfig('metric', 20))

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


class _FakeConfig:
    def __init__(self, depth_estimation_type=None, max_depth=None) -> None:
        if depth_estimation_type is not None:
            self.depth_estimation_type = depth_estimation_type
        if max_depth is not None:
            self.max_depth = max_depth


class _FakePipeline:
    def __init__(self, config) -> None:
        self.model = type('_FakeModel', (), {'config': config})()


class _ProducingPipeline(_FakePipeline):
    def __init__(self, predicted_depth: np.ndarray) -> None:
        super().__init__(_FakeConfig('metric', 20))
        self.predicted_depth = predicted_depth
        self.inputs = []

    def __call__(self, image):
        self.inputs.append(image)
        return {'predicted_depth': self.predicted_depth}


class _FakePilImage:
    @staticmethod
    def fromarray(image: np.ndarray) -> np.ndarray:
        return image


class _FakeCv2:
    INTER_LINEAR = object()

    def __init__(self, resized: np.ndarray) -> None:
        self.resized = resized
        self.calls = []

    def resize(self, depth: np.ndarray, size, *, interpolation) -> np.ndarray:
        self.calls.append((depth.copy(), size, interpolation))
        return self.resized.copy()


def _loaded_source(config, monkeypatch) -> MonocularDepthSource:
    source = MonocularDepthSource('model', 'cpu', _NullLogger())
    monkeypatch.setattr(source, '_build_pipeline', lambda: _FakePipeline(config))
    return source


def test_monocular_caches_pillow_and_opencv_across_frames(monkeypatch) -> None:
    predicted = np.array(
        [[1.0, 1.1, 1.2], [1.3, 1.4, 1.5]], dtype=np.float32)
    expected = np.array([[1.0, 1.1], [1.2, 1.3]], dtype=np.float32)
    pipeline = _ProducingPipeline(predicted)
    cv2 = _FakeCv2(expected)
    imports = []

    def import_module(name: str):
        imports.append(name)
        if name == 'PIL.Image':
            return _FakePilImage
        if name == 'cv2':
            return cv2
        raise AssertionError(f'unexpected import: {name}')

    monkeypatch.setattr(
        'ridgeback_autonomy.perception.target_localization.core.depth_sources.importlib.import_module',
        import_module)
    source = MonocularDepthSource('model', 'cpu', _NullLogger())
    monkeypatch.setattr(source, '_build_pipeline', lambda: pipeline)
    msg = make_image('rgb8', np.arange(12, dtype=np.uint8).reshape(2, 2, 3))

    first_depth, first_header = source.produce(msg)
    second_depth, second_header = source.produce(msg)

    assert imports == ['PIL.Image', 'cv2']
    assert len(pipeline.inputs) == 2
    assert len(cv2.calls) == 2
    assert cv2.calls[0][1:] == ((2, 2), cv2.INTER_LINEAR)
    np.testing.assert_array_equal(first_depth, expected)
    np.testing.assert_array_equal(second_depth, expected)
    assert first_header is msg.header
    assert second_header is msg.header


def test_monocular_does_not_require_opencv_without_resizing(monkeypatch) -> None:
    predicted = np.array([[0.5, 1.0], [1.5, 2.0]], dtype=np.float32)
    pipeline = _ProducingPipeline(predicted)
    imports = []

    def import_module(name: str):
        imports.append(name)
        if name == 'PIL.Image':
            return _FakePilImage
        if name == 'cv2':
            raise ModuleNotFoundError('OpenCV is not installed')
        raise AssertionError(f'unexpected import: {name}')

    monkeypatch.setattr(
        'ridgeback_autonomy.perception.target_localization.core.depth_sources.importlib.import_module',
        import_module)
    source = MonocularDepthSource('model', 'cpu', _NullLogger())
    monkeypatch.setattr(source, '_build_pipeline', lambda: pipeline)
    msg = make_image('rgb8', np.zeros((2, 2, 3), dtype=np.uint8))

    depth_m, header = source.produce(msg)

    assert imports == ['PIL.Image']
    np.testing.assert_array_equal(depth_m, predicted)
    assert header is msg.header


def test_monocular_produce_decodes_then_delegates_to_prepared_rgb(monkeypatch) -> None:
    source = MonocularDepthSource('model', 'cpu', _NullLogger())
    msg = make_image(
        'bgr8', np.arange(12, dtype=np.uint8).reshape(2, 2, 3))
    seen = {}

    def produce_from_rgb(rgb, header):
        seen['rgb'] = rgb
        seen['header'] = header
        return 'depth', header

    monkeypatch.setattr(source, 'produce_from_rgb', produce_from_rgb)

    assert source.produce(msg) == ('depth', msg.header)
    np.testing.assert_array_equal(
        seen['rgb'], np.arange(12, dtype=np.uint8).reshape(2, 2, 3)[:, :, ::-1])
    assert seen['rgb'].flags.c_contiguous
    assert seen['header'] is msg.header


def test_monocular_produce_from_rgb_does_not_decode_a_ros_message(monkeypatch) -> None:
    rgb = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    pipeline = _ProducingPipeline(np.ones((2, 2), dtype=np.float32))
    source = MonocularDepthSource('model', 'cpu', _NullLogger())
    monkeypatch.setattr(source, '_build_pipeline', lambda: pipeline)
    monkeypatch.setattr(
        'ridgeback_autonomy.perception.target_localization.core.depth_sources.decode_color_to_rgb',
        lambda _msg: pytest.fail('produce_from_rgb must not decode a ROS message'),
    )
    monkeypatch.setattr(
        'ridgeback_autonomy.perception.target_localization.core.depth_sources.importlib.import_module',
        lambda name: _FakePilImage if name == 'PIL.Image' else pytest.fail(name),
    )
    header = Header()

    depth_m, returned_header = source.produce_from_rgb(rgb, header)

    np.testing.assert_array_equal(depth_m, np.ones((2, 2), dtype=np.float32))
    assert returned_header is header
    assert pipeline.inputs == [rgb]
    assert pipeline.inputs[0] is rgb


def test_monocular_retries_a_failed_pillow_import_after_cooldown(monkeypatch) -> None:
    clock = {'t': 100.0}
    logger = _RecordingLogger()
    predicted = np.full((2, 2), 2.0, dtype=np.float32)
    pipeline = _ProducingPipeline(predicted)
    import_attempts = {'n': 0}

    def import_module(name: str):
        assert name == 'PIL.Image'
        import_attempts['n'] += 1
        if import_attempts['n'] == 1:
            raise ModuleNotFoundError('Pillow is temporarily unavailable')
        return _FakePilImage

    monkeypatch.setattr(
        'ridgeback_autonomy.perception.target_localization.core.depth_sources.importlib.import_module',
        import_module)
    source = MonocularDepthSource(
        'model', 'cpu', logger,
        now_fn=lambda: clock['t'], cooldown_s=30.0)
    monkeypatch.setattr(source, '_build_pipeline', lambda: pipeline)
    msg = make_image('rgb8', np.zeros((2, 2, 3), dtype=np.uint8))

    assert source.produce(msg) is None
    assert import_attempts['n'] == 1
    assert 'Pillow is temporarily unavailable' in logger.warnings[-1]

    clock['t'] = 120.0
    assert source.produce(msg) is None
    assert import_attempts['n'] == 1

    clock['t'] = 131.0
    depth_m, header = source.produce(msg)
    np.testing.assert_array_equal(depth_m, predicted)
    assert header is msg.header
    assert import_attempts['n'] == 2


def test_monocular_usable_max_derives_from_checkpoint_max_depth(monkeypatch) -> None:
    # The metric head is sigmoid * max_depth, so the ceiling is a property of
    # the checkpoint rather than a constant this repo picks.
    source = _loaded_source(_FakeConfig('metric', 20), monkeypatch)

    assert source.load() is True
    assert source.usable_max_m == pytest.approx(20 * MONOCULAR_USABLE_RANGE_FRACTION)


def test_monocular_usable_max_tracks_a_different_checkpoint(monkeypatch) -> None:
    # The Outdoor checkpoint ships max_depth=80; nothing should be hardcoded to
    # the Indoor figure.
    source = _loaded_source(_FakeConfig('metric', 80), monkeypatch)

    assert source.load() is True
    assert source.usable_max_m == pytest.approx(80 * MONOCULAR_USABLE_RANGE_FRACTION)


def test_monocular_refuses_a_relative_checkpoint(monkeypatch) -> None:
    # A relative checkpoint predicts unitless inverse depth. Nothing downstream
    # would error on it -- it would publish disparity as meters -- so the
    # source has to refuse rather than let it through.
    source = _loaded_source(_FakeConfig('relative', 1), monkeypatch)

    assert source.load() is False
    assert source._pipeline is None


def test_monocular_refuses_a_checkpoint_that_states_no_max_depth(monkeypatch) -> None:
    # MONOCULAR_USABLE_RANGE_FRACTION scales config.max_depth and means nothing
    # without it. This used to assume 20 m -- the Indoor checkpoint's figure,
    # hardcoded -- which silently capped the scene at 18 m and reported the
    # deleted far end as a mask shortfall.
    source = _loaded_source(_FakeConfig('metric'), monkeypatch)

    assert source.load() is False
    assert source._pipeline is None
    assert source.usable_max_m == math.inf


def test_monocular_declares_no_ceiling_before_it_loads() -> None:
    # produce() returns None while unloaded, so nothing is cleaned against this
    # value; naming a finite one here would be a range this module invented.
    source = MonocularDepthSource('some/checkpoint', 'cpu', _NullLogger())

    assert source.usable_max_m == math.inf


def test_stereo_source_declares_no_ceiling_but_accepts_one() -> None:
    # Unbounded by default: no defensible number exists for this source, and
    # out-of-range readings already arrive as 0/NaN/inf under the frame
    # contract. An operator who knows their sensor can still name one.
    assert StereoDepthSource(_NullLogger()).usable_max_m == math.inf
    assert StereoDepthSource(_NullLogger(), usable_max_m=6.0).usable_max_m == 6.0


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


def test_build_depth_source_dispatches_both_names() -> None:
    stereo = build_depth_source('stereoscopic', _NullLogger())
    monocular = build_depth_source(
        'Monocular', _NullLogger(), model_id='model', device='cpu')

    assert isinstance(stereo, StereoDepthSource)
    assert stereo.input_kind == 'depth'
    # Case and surrounding whitespace are normalized, so a launch argument
    # typed loosely still resolves.
    assert isinstance(monocular, MonocularDepthSource)
    assert monocular.input_kind == 'color'


def test_build_depth_source_rejects_unknown_name() -> None:
    with pytest.raises(ValueError, match='Unknown depth_source'):
        build_depth_source('lidar', _NullLogger())


def test_stereo_source_produces_meters_and_keeps_the_input_header() -> None:
    raw = np.array([[1000, 2500]], dtype=np.uint16)
    msg = make_image('16UC1', raw)
    msg.header.frame_id = 'camera_0_color_optical'

    depth_m, header = StereoDepthSource(_NullLogger()).produce(msg)

    assert np.allclose(depth_m, [[1.0, 2.5]])
    assert header.frame_id == 'camera_0_color_optical'


def test_stereo_source_returns_none_on_unsupported_encoding() -> None:
    # The mask node reads this ``None`` as "no usable depth at this stamp".
    msg = make_image('8UC1', np.zeros((2, 2), dtype=np.uint16))

    assert StereoDepthSource(_NullLogger()).produce(msg) is None
