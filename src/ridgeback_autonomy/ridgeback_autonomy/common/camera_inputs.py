"""Pure launch-time contract for camera inputs shared across backends.

The contract describes source topics only. It deliberately imports neither ROS
nor launch: camera drivers own synchronization, alignment, and sensor headers;
consumers only receive the already-compatible inputs named here.

Invariants: color and camera info use one pixel grid; aligned depth uses that
grid and clock; sensor timestamps are never rewritten; organized points, when
present, are indexable by color pixels; optical frames have a TF path to base;
and topics are relative to the robot namespace.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping


SIMULATION_BACKEND = 'simulation'
REALSENSE_BACKEND = 'realsense'


@dataclass(frozen=True)
class CameraInputs:
    """Backend-normalized camera input topics.

    ``organized_points_topic`` is optional because RealSense pointcloud output
    is driver-configurable. All non-empty topics are robot-namespace-relative.
    """

    color_image_topic: str
    color_camera_info_topic: str
    aligned_depth_topic: str
    organized_points_topic: str | None
    backend: str


# Gazebo renders these streams from one RGBD sensor, so depth is aligned to
# color by construction.
_BACKEND_DEFAULTS = {
    SIMULATION_BACKEND: CameraInputs(
        color_image_topic='sensors/camera_0/color/image',
        color_camera_info_topic='sensors/camera_0/color/camera_info',
        aligned_depth_topic='sensors/camera_0/depth/image',
        organized_points_topic='sensors/camera_0/points',
        backend=SIMULATION_BACKEND,
    ),
    REALSENSE_BACKEND: CameraInputs(
        color_image_topic='sensors/camera_0/color/image_raw',
        color_camera_info_topic='sensors/camera_0/color/camera_info',
        aligned_depth_topic='sensors/camera_0/aligned_depth_to_color/image_raw',
        # ``pointcloud.enable`` is driver-configurable. Callers selecting the
        # pointcloud estimator must provide and confirm this input on hardware.
        organized_points_topic=None,
        backend=REALSENSE_BACKEND,
    ),
}

_OVERRIDE_FIELDS = {
    'color_topic': 'color_image_topic',
    'camera_info_topic': 'color_camera_info_topic',
    'depth_topic': 'aligned_depth_topic',
    'pointcloud_topic': 'organized_points_topic',
    'color_image_topic': 'color_image_topic',
    'color_camera_info_topic': 'color_camera_info_topic',
    'aligned_depth_topic': 'aligned_depth_topic',
    'organized_points_topic': 'organized_points_topic',
}
_REQUIRED_TOPIC_FIELDS = (
    'color_image_topic', 'color_camera_info_topic', 'aligned_depth_topic')


def _validated_topic(field: str, value: str | None) -> str | None:
    if value is None:
        if field in _REQUIRED_TOPIC_FIELDS:
            raise ValueError(f'{field} is required')
        return None
    if not isinstance(value, str):
        raise TypeError(f'{field} must be a string or None')
    topic = value.strip()
    if not topic:
        if field in _REQUIRED_TOPIC_FIELDS:
            raise ValueError(f'{field} must not be empty')
        return None
    if topic.startswith('/'):
        raise ValueError(f'{field} must be relative to the robot namespace: {topic!r}')
    return topic


def _validate(inputs: CameraInputs) -> CameraInputs:
    return replace(
        inputs,
        color_image_topic=_validated_topic('color_image_topic', inputs.color_image_topic),
        color_camera_info_topic=_validated_topic(
            'color_camera_info_topic', inputs.color_camera_info_topic),
        aligned_depth_topic=_validated_topic('aligned_depth_topic', inputs.aligned_depth_topic),
        organized_points_topic=_validated_topic(
            'organized_points_topic', inputs.organized_points_topic),
    )


def resolve_camera_inputs(
    backend: str,
    overrides: Mapping[str, str | None] | None = None,
) -> CameraInputs:
    """Resolve a backend mapping, applying compatibility overrides last.

    Compatibility names are the existing launch arguments: ``color_topic``,
    ``camera_info_topic``, ``depth_topic``, and ``pointcloud_topic``. Required
    topics reject empty values; an empty optional pointcloud disables it.
    """

    try:
        resolved = _BACKEND_DEFAULTS[backend]
    except KeyError as exc:
        known = ', '.join(sorted(_BACKEND_DEFAULTS))
        raise ValueError(f'unknown camera backend {backend!r}; expected one of {known}') from exc
    if overrides is None:
        return _validate(resolved)

    updates: dict[str, str | None] = {}
    for name, value in overrides.items():
        try:
            updates[_OVERRIDE_FIELDS[name]] = value
        except KeyError as exc:
            raise ValueError(f'unknown camera input override {name!r}') from exc
    return _validate(replace(resolved, **updates))
