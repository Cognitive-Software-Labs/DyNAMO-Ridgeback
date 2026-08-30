"""Declarative benchmark scenario spec: robots + object occluders per scene.

The runner iterates a list of scenes (this module only loads and validates them);
each scene declares N robot instances and M static object occluders as world-frame
planar poses. Kept pure (no ROS) so it is unit-testable without a sim.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import yaml


# The target faces back toward the sensor robot at the world origin. Same value as the
# runner's historical ``G1_FACING_ROBOT_YAW_RAD`` (mirrored, not imported, so the
# spec stays ROS-free).
DEFAULT_ROBOT_YAW_RAD = math.pi
# Objects have no inherent facing requirement; default to axis-aligned.
DEFAULT_OBJECT_YAW_RAD = 0.0


@dataclass(frozen=True)
class RobotSpec:
    """One target instance, world-frame planar pose (z pinned to floor by the runner)."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ObjectSpec:
    """One static object occluder spawned from ``model://<model>``."""

    model: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Scene:
    """A named scene: the robots to score plus the objects that clutter/occlude them."""

    id: str
    robots: tuple[RobotSpec, ...]
    objects: tuple[ObjectSpec, ...]
    repeats_override: int | None


def _require_number(value, field_name: str, scene_id: str) -> float:
    # bool is an int subclass; reject it so `yaw: true` is an error, not 1.0.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            f'Scene "{scene_id}": field "{field_name}" must be a number, got {value!r}.'
        )
    return float(value)


def _parse_robot(raw, defaults: dict, scene_id: str) -> RobotSpec:
    if not isinstance(raw, dict):
        raise ValueError(f'Scene "{scene_id}": each robot must be a mapping, got {raw!r}.')
    yaw_raw = raw.get('yaw', defaults['robot_yaw_rad'])
    return RobotSpec(
        x=_require_number(raw.get('x'), 'robot.x', scene_id),
        y=_require_number(raw.get('y'), 'robot.y', scene_id),
        yaw=_require_number(yaw_raw, 'robot.yaw', scene_id),
    )


def _parse_object(raw, scene_id: str) -> ObjectSpec:
    if not isinstance(raw, dict):
        raise ValueError(f'Scene "{scene_id}": each object must be a mapping, got {raw!r}.')
    model = raw.get('model')
    if not isinstance(model, str) or not model.strip():
        raise ValueError(
            f'Scene "{scene_id}": object "model" must be a non-empty string, got {model!r}.'
        )
    return ObjectSpec(
        model=model.strip(),
        x=_require_number(raw.get('x'), 'object.x', scene_id),
        y=_require_number(raw.get('y'), 'object.y', scene_id),
        yaw=_require_number(raw.get('yaw', DEFAULT_OBJECT_YAW_RAD), 'object.yaw', scene_id),
    )


def parse_scene(raw, defaults: dict) -> Scene:
    """Validate one scene mapping into a :class:`Scene` (raises ``ValueError`` on bad input)."""

    if not isinstance(raw, dict):
        raise ValueError(f'Each scene must be a mapping, got {raw!r}.')
    scene_id = raw.get('id')
    if not isinstance(scene_id, str) or not scene_id.strip():
        raise ValueError(f'Every scene needs a non-empty string "id", got {scene_id!r}.')
    scene_id = scene_id.strip()

    robots_raw = raw.get('robots')
    if not isinstance(robots_raw, list) or not robots_raw:
        raise ValueError(f'Scene "{scene_id}": "robots" must be a non-empty list.')
    robots = tuple(_parse_robot(item, defaults, scene_id) for item in robots_raw)

    objects_raw = raw.get('objects') or []
    if not isinstance(objects_raw, list):
        raise ValueError(f'Scene "{scene_id}": "objects" must be a list when present.')
    objects = tuple(_parse_object(item, scene_id) for item in objects_raw)

    repeats_override = raw.get('repeats_override')
    if repeats_override is not None:
        if isinstance(repeats_override, bool) or not isinstance(repeats_override, int) or repeats_override < 1:
            raise ValueError(
                f'Scene "{scene_id}": "repeats_override" must be a positive integer or omitted, '
                f'got {repeats_override!r}.'
            )

    return Scene(
        id=scene_id,
        robots=robots,
        objects=objects,
        repeats_override=repeats_override,
    )


def parse_scenarios(document, *, source: str = '<memory>') -> tuple[Scene, ...]:
    """Parse a loaded YAML document (a mapping with ``scenes`` + optional ``defaults``)."""

    if not isinstance(document, dict):
        raise ValueError(f'Scenario file {source} must be a mapping at the top level.')

    defaults_raw = document.get('defaults') or {}
    if not isinstance(defaults_raw, dict):
        raise ValueError(f'Scenario file {source}: "defaults" must be a mapping when present.')
    defaults = {
        'robot_yaw_rad': _require_number(
            defaults_raw.get('robot_yaw_rad', DEFAULT_ROBOT_YAW_RAD),
            'defaults.robot_yaw_rad',
            '<defaults>',
        ),
    }

    scenes_raw = document.get('scenes')
    if not isinstance(scenes_raw, list) or not scenes_raw:
        raise ValueError(f'Scenario file {source} must define a non-empty "scenes" list.')

    scenes = tuple(parse_scene(item, defaults) for item in scenes_raw)

    seen: set[str] = set()
    for scene in scenes:
        if scene.id in seen:
            raise ValueError(f'Scenario file {source}: duplicate scene id "{scene.id}".')
        seen.add(scene.id)
    return scenes


def load_scenarios(path: str) -> tuple[Scene, ...]:
    """Load and validate a scenario YAML file into a tuple of :class:`Scene`."""

    with open(path, 'r', encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return parse_scenarios(document, source=path)
