from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import LookupException, TransformException

from ridgeback_autonomy.common.tf_utils import (
    TF_LOOKUP_TIMEOUT_SEC,
    candidate_base_frames,
    lookup_transform_components,
    rotation_matrix_from_quaternion,
)


NAMESPACED_FRAME = 'r100_0001/robot/base_link'
BARE_FRAME = 'base_link'
SOURCE_FRAME = 'camera_color_optical_frame'

# A detection stamp, not latest-TF: the callers all look up at the stamp the
# masks were made on.
DETECTION_STAMP = Time(seconds=1788174223, nanoseconds=255000000)


def make_transform(translation, quaternion=(0.0, 0.0, 0.0, 1.0)):
    x, y, z = translation
    qx, qy, qz, qw = quaternion
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    ))


class RecordingBuffer:
    """TF buffer stub recording target/source/stamp/timeout for every lookup.

    Frames absent from ``available`` are absent at any timeout, which is what an
    unpublished frame does in a real graph: waiting on it cannot help.
    """

    def __init__(self, available):
        self.available = dict(available)
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, time, timeout=Duration()):
        self.calls.append(SimpleNamespace(
            target_frame=target_frame,
            source_frame=source_frame,
            time=time,
            timeout=timeout,
        ))
        transform = self.available.get(target_frame)
        if transform is None:
            raise LookupException(
                f'"{target_frame}" passed to lookupTransform does not exist.')
        return transform

    @property
    def waiting_calls(self):
        return [call for call in self.calls if call.timeout.nanoseconds > 0]


class RecordingLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


def test_available_fallback_is_returned_without_any_wait():
    """A missing configured frame must not charge its timeout to every call.

    The mask node looks this up once per detected batch into a latest-wins
    pending slot. Half a second of waiting per batch holds the worker below the
    incoming batch rate, so batches are overwritten before they are measured and
    their observations score UNSET.
    """

    buffer = RecordingBuffer({BARE_FRAME: make_transform((1.5, -0.25, 0.75))})
    logger = RecordingLogger()

    rotation, translation, fallback = lookup_transform_components(
        buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP, logger, None)

    assert buffer.waiting_calls == []
    assert [call.target_frame for call in buffer.calls] == [
        NAMESPACED_FRAME, BARE_FRAME]
    assert fallback == BARE_FRAME
    assert translation == pytest.approx([1.5, -0.25, 0.75])
    assert rotation == pytest.approx(np.eye(3))


def test_repeated_fallback_calls_stay_nonblocking_and_warn_once():
    """Fallback state suppresses the warning; it must not re-arm the wait.

    Suppressing only the log while every later call still blocks is the defect
    this covers -- the cost is invisible in the log but paid on every batch.
    """

    buffer = RecordingBuffer({BARE_FRAME: make_transform((1.0, 0.0, 0.0))})
    logger = RecordingLogger()

    fallback = None
    for _ in range(5):
        _, _, fallback = lookup_transform_components(
            buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP, logger,
            fallback)

    assert buffer.waiting_calls == []
    assert len(buffer.calls) == 10
    assert len(logger.warnings) == 1
    assert NAMESPACED_FRAME in logger.warnings[0]
    assert BARE_FRAME in logger.warnings[0]


def test_configured_frame_wins_when_both_are_available():
    """Order still decides, so a namespaced deployment keeps its own frame.

    The fix removes waiting, not the configured-frame-first priority.
    """

    buffer = RecordingBuffer({
        NAMESPACED_FRAME: make_transform((2.0, 0.0, 0.0)),
        BARE_FRAME: make_transform((9.0, 9.0, 9.0)),
    })
    logger = RecordingLogger()

    _, translation, fallback = lookup_transform_components(
        buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP, logger, None)

    assert [call.target_frame for call in buffer.calls] == [NAMESPACED_FRAME]
    assert translation == pytest.approx([2.0, 0.0, 0.0])
    assert fallback is None
    assert logger.warnings == []


def test_configured_frame_is_preferred_again_after_an_earlier_fallback():
    """Nothing about the chosen fallback may be cached across calls.

    Frames appear late. A cached choice would keep reporting the bare frame's
    pose after the configured one starts publishing, silently measuring against
    the wrong body.
    """

    buffer = RecordingBuffer({BARE_FRAME: make_transform((1.0, 0.0, 0.0))})
    logger = RecordingLogger()

    _, _, fallback = lookup_transform_components(
        buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP, logger, None)
    assert fallback == BARE_FRAME

    buffer.available[NAMESPACED_FRAME] = make_transform((3.0, 0.5, 0.0))
    _, translation, fallback = lookup_transform_components(
        buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP, logger,
        fallback)

    assert translation == pytest.approx([3.0, 0.5, 0.0])
    assert buffer.calls[-1].target_frame == NAMESPACED_FRAME
    assert len(logger.warnings) == 1


def test_unqualified_frame_is_looked_up_once_per_pass():
    """De-duplicated candidates must not double the work they save.

    ``base_link`` yields one candidate, so an unqualified deployment pays one
    nonblocking lookup on success and one bounded wait on failure.
    """

    assert candidate_base_frames(BARE_FRAME) == [BARE_FRAME]

    buffer = RecordingBuffer({BARE_FRAME: make_transform((1.0, 0.0, 0.0))})
    lookup_transform_components(
        buffer, BARE_FRAME, SOURCE_FRAME, DETECTION_STAMP, RecordingLogger(),
        None)
    assert len(buffer.calls) == 1

    empty = RecordingBuffer({})
    with pytest.raises(TransformException):
        lookup_transform_components(
            empty, BARE_FRAME, SOURCE_FRAME, DETECTION_STAMP,
            RecordingLogger(), None)
    assert [call.timeout.nanoseconds for call in empty.calls] == [
        0, int(TF_LOOKUP_TIMEOUT_SEC * 1e9)]


def test_no_candidate_available_keeps_the_bounded_wait_and_raises():
    """When nothing is buffered yet the timeout is still the right answer.

    Removing the wait outright would trade dropped batches for transforms
    abandoned a few milliseconds early. Failure must stay an exception -- an
    identity or stale transform would place every target at the robot origin.
    """

    buffer = RecordingBuffer({})
    logger = RecordingLogger()

    with pytest.raises(TransformException):
        lookup_transform_components(
            buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP, logger,
            None)

    assert [call.target_frame for call in buffer.calls] == [
        NAMESPACED_FRAME, BARE_FRAME, NAMESPACED_FRAME, BARE_FRAME]
    assert [call.timeout.nanoseconds for call in buffer.calls] == [
        0, 0, int(TF_LOOKUP_TIMEOUT_SEC * 1e9), int(TF_LOOKUP_TIMEOUT_SEC * 1e9)]
    assert logger.warnings == []


def test_no_valid_candidates_raises_without_any_lookup():
    """An empty configured frame yields no candidates, and so no fabricated pose."""

    buffer = RecordingBuffer({BARE_FRAME: make_transform((1.0, 0.0, 0.0))})

    with pytest.raises(TransformException):
        lookup_transform_components(
            buffer, '', SOURCE_FRAME, DETECTION_STAMP, RecordingLogger(), None)

    assert buffer.calls == []


def test_every_lookup_carries_the_caller_source_and_stamp():
    """The caller's exact stamp must reach tf2, on the retry pass too.

    Substituting latest TF would silently pair a current robot pose with an
    older detection, which reads as estimator error rather than as a bad lookup.
    """

    buffer = RecordingBuffer({})

    with pytest.raises(TransformException):
        lookup_transform_components(
            buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP,
            RecordingLogger(), None)

    assert buffer.calls
    for call in buffer.calls:
        assert call.source_frame == SOURCE_FRAME
        assert call.time is DETECTION_STAMP


def test_returned_components_match_the_transform_message():
    """Return shape and numeric conversion are unchanged by the fix."""

    quaternion = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
    buffer = RecordingBuffer({
        BARE_FRAME: make_transform((0.31, -1.22, 0.08), quaternion),
    })

    rotation, translation, _ = lookup_transform_components(
        buffer, NAMESPACED_FRAME, SOURCE_FRAME, DETECTION_STAMP,
        RecordingLogger(), None)

    expected_rotation = rotation_matrix_from_quaternion(
        SimpleNamespace(x=quaternion[0], y=quaternion[1], z=quaternion[2],
                        w=quaternion[3]))

    assert rotation.dtype == np.float32
    assert translation.dtype == np.float32
    assert rotation == pytest.approx(expected_rotation)
    assert translation == pytest.approx([0.31, -1.22, 0.08], abs=1e-7)
