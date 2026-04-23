from __future__ import annotations

import numpy as np
from rclpy.duration import Duration
from tf2_ros import TransformException


TF_LOOKUP_TIMEOUT_SEC = 0.5


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


def lookup_transform_components(
    tf_buffer,
    base_frame: str,
    source_frame: str,
    timestamp,
    logger,
    last_fallback_frame: str | None,
) -> tuple[np.ndarray, np.ndarray, str | None]:
    last_error = None

    for target_frame in candidate_base_frames(base_frame):
        try:
            transform = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                timestamp,
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_SEC),
            )
        except TransformException as exc:
            last_error = exc
            continue

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

    if last_error is None:
        raise TransformException('No candidate base frames were available.')
    raise last_error

