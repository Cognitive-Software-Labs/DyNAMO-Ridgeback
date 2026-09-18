"""Shared base-frame resolution and transform extraction for perception nodes."""

from __future__ import annotations

import numpy as np
from rclpy.duration import Duration
from tf2_ros import TransformException


TF_LOOKUP_TIMEOUT_SEC = 0.5

# A zero Duration makes tf2 skip its wait loop and answer from the buffer it
# already holds.
TF_NO_WAIT = Duration()


def candidate_base_frames(base_frame: str) -> list[str]:
    candidates: list[str] = []
    for candidate in (base_frame, base_frame.split('/')[-1]):
        if candidate and candidate not in candidates:
            candidates.append(candidate)
    return candidates


def rotation_matrix_from_quaternion(quaternion) -> np.ndarray:
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


def _first_available_transform(
    tf_buffer,
    candidates: list[str],
    source_frame: str,
    timestamp,
    timeout: Duration,
):
    last_error = None

    for target_frame in candidates:
        try:
            transform = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                timestamp,
                timeout=timeout,
            )
        except TransformException as exc:
            last_error = exc
            continue
        return transform, target_frame, None

    return None, None, last_error


def lookup_transform_components(
    tf_buffer,
    base_frame: str,
    source_frame: str,
    timestamp,
    logger,
    last_fallback_frame: str | None,
) -> tuple[np.ndarray, np.ndarray, str | None]:
    """``(rotation, translation, last_fallback_frame)`` at ``timestamp``.

    Candidates stay in configured-frame-first order, but they are tried twice:
    once without waiting, then with the bounded per-candidate wait. A configured
    frame no publisher provides never resolves, so a single waiting pass charged
    the full timeout to every call before reaching the bare fallback that was
    already buffered -- enough, at benchmark detection rates, to stall the
    caller's worker and drop batches. The waiting pass is still reached whenever
    nothing is buffered yet, which is the case the timeout exists for.
    """

    candidates = candidate_base_frames(base_frame)

    transform, target_frame, last_error = _first_available_transform(
        tf_buffer, candidates, source_frame, timestamp, TF_NO_WAIT)
    if transform is None:
        transform, target_frame, last_error = _first_available_transform(
            tf_buffer, candidates, source_frame, timestamp,
            Duration(seconds=TF_LOOKUP_TIMEOUT_SEC))

    if transform is None:
        if last_error is None:
            raise TransformException('No candidate base frames were available.')
        raise last_error

    if target_frame != base_frame and target_frame != last_fallback_frame:
        logger.warn(
            f'Configured base frame "{base_frame}" is unavailable; '
            f'using "{target_frame}" for transforms.'
        )
        last_fallback_frame = target_frame

    translation = np.array([
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ], dtype=np.float32)
    rotation = rotation_matrix_from_quaternion(transform.transform.rotation)
    return rotation, translation, last_fallback_frame

