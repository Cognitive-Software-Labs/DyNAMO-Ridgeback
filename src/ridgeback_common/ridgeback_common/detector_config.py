"""ROS-free startup contract for the target detector.

Only the implementation selector is an enum. The checkpoint remains a separate
identifier passed unchanged to the selected implementation.
"""

from __future__ import annotations

import math


DETECTOR_DEFAULT = 'owlv2'
SUPPORTED_DETECTORS = (DETECTOR_DEFAULT,)
DETECTION_MODEL_DEFAULT = 'google/owlv2-base-patch16-ensemble'
TARGET_LABELS_DEFAULT = 'humanoid robot'
DETECTION_THRESHOLD_DEFAULT = 0.55
DETECTOR_FPS_DEFAULT = 10.0
DETECTOR_DEBUG_DEFAULT = False
DETECTOR_ENVIRONMENT_KEYS = frozenset({
    'detector', 'detection_model', 'target_labels',
    'detection_threshold', 'detector_fps', 'detector_debug',
})


def validate_detector(value: object, *, allowed=SUPPORTED_DETECTORS) -> str:
    if not isinstance(value, str) or value.strip() not in allowed:
        raise ValueError(
            f'detector must be one of: {", ".join(allowed)}; got {value!r}')
    return value.strip()


def validate_detection_model(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError('detection_model must be a non-empty checkpoint identifier')
    return value.strip()


def parse_target_labels(value: object) -> list[str]:
    if not isinstance(value, str):
        raise ValueError('target_labels must be a comma-separated string')
    labels = [part.strip() for part in value.split(',') if part.strip()]
    if not labels:
        raise ValueError('target_labels must contain at least one label')
    return labels


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def validate_detection_threshold(value: object) -> float:
    threshold = _finite_number(value, 'detection_threshold')
    if not 0 < threshold <= 1:
        raise ValueError('detection_threshold must be in (0, 1]')
    return threshold


def validate_detector_fps(value: object) -> float:
    fps = _finite_number(value, 'detector_fps')
    if fps <= 0:
        raise ValueError('detector_fps must be positive')
    return fps


def validate_detector_debug(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.strip().lower() in {'true', 'false'}:
        return value.strip().lower() == 'true'
    raise ValueError('detector_debug must be true or false')


def resolve_detector_settings(values: dict, *, fps_default=DETECTOR_FPS_DEFAULT) -> dict:
    """Validate and return canonical effective settings without model imports."""
    labels = parse_target_labels(values.get('target_labels', TARGET_LABELS_DEFAULT))
    return {
        'detector': validate_detector(values.get('detector', DETECTOR_DEFAULT)),
        'detection_model': validate_detection_model(
            values.get('detection_model', DETECTION_MODEL_DEFAULT)),
        'target_labels': ','.join(labels),
        'detection_threshold': validate_detection_threshold(
            values.get('detection_threshold', DETECTION_THRESHOLD_DEFAULT)),
        'detector_fps': validate_detector_fps(
            values.get('detector_fps', fps_default)),
        'detector_debug': validate_detector_debug(
            values.get('detector_debug', DETECTOR_DEBUG_DEFAULT)),
    }


def require_matching_detector_settings(expected: dict, observed: dict) -> dict:
    """Use live detector parameters as producer identity, refusing stale config."""
    intended = resolve_detector_settings(expected)
    actual = resolve_detector_settings(observed)
    mismatches = [name for name in sorted(DETECTOR_ENVIRONMENT_KEYS)
                  if intended[name] != actual[name]]
    if mismatches:
        raise ValueError(
            'Persistent detector configuration does not match the running '
            'target_detector: ' + ', '.join(mismatches))
    return actual
