"""Benchmark background: one fixed room layout, minus each scene's conflicts.

A background is a master layout of static props for the benchmark room. Props
are never moved and never randomized. For each scene, a deterministic rule
drops every prop that would sit on, beside, in front of, or close behind that
scene's targets and objects; every other prop stays. Near-background
interference is the job of the designed scenes, not of the room.

The rule runs offline: ``tools/benchmark/build_background.py`` evaluates it for
every packaged scenario file and writes the per-scene drop lists. The runner
only reads those lists, so the room a scene is captured in is fixed by data
under review rather than by code that could change between runs. The drop file
pins the layout and each scenario file by SHA-256, so an edit to either
without regeneration is refused instead of silently mismatched.

Kept pure (no ROS): the generator, the tests and the runner share it.

Layout file::

    schema: ridgeback_benchmark_background_layout/1
    world: <public world>
    room: {x_min: ..., x_max: ..., y_min: ..., y_max: ...}
    props:
      - {id: <prop id>, model: <logical prop model>, x: ..., y: ..., yaw: ...}

Drop file (generated)::

    schema: ridgeback_benchmark_background_drops/1
    background: <name>
    layout: {file: <layout file name>, sha256: ...}
    rules: {...}
    footprints: {<logical prop model>: [x0, x1, y0, y1], ...}
    scenarios:
      <scenario file name>:
        sha256: ...
        scenes: {<scene id>: [<dropped prop id>, ...], ...}
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
import hashlib
import math
import os
import re
from typing import Any

import yaml


LAYOUT_SCHEMA = 'ridgeback_benchmark_background_layout/1'
DROPS_SCHEMA = 'ridgeback_benchmark_background_drops/1'

# The packaged layouts and their generated drop files, both under the
# package's config directory.
LAYOUT_FILE_PATTERN = 'benchmark_background_{name}.yaml'
DROPS_DIRECTORY = 'benchmark_backgrounds'

# Planar position of camera_0's mount on the robot (clearpath/robot.yaml);
# the benchmark robot always stands at the world origin facing +x.
CAMERA_XY = (0.2692, 0.0)
# A standing G1's footprint radius, and the clearance every rule adds to it.
TARGET_RADIUS_M = 0.40
MARGIN_M = 0.30
# Half-width of the corridor kept clear around each camera-to-target and
# camera-to-object sight line.
SIGHTLINE_CORRIDOR_M = 0.45
# How far behind a target its keep-out cone reaches. 0.5 m beyond the
# farthest designed behind-target object: trap_layered_01's cabinet stands
# 1.78 m behind its target. Objects farther back on a target's bearing (the
# interfere_bearing family, 2.9 m) are protected by their own sight line.
KEEP_OUT_BEHIND_M = 2.3
# Spacing of the outline points tested against a keep-out cone.
OUTLINE_SAMPLE_STEP_M = 0.05
# Two props closer than this count as touching in the layout check.
MIN_PROP_GAP_M = 0.05
# Wall props sit flush against the wall; a yaw written to four decimals moves
# a corner by micrometres, which must not read as leaving the room.
ROOM_TOLERANCE_M = 1e-3

RULES = {
    'camera_xy': list(CAMERA_XY),
    'target_radius_m': TARGET_RADIUS_M,
    'margin_m': MARGIN_M,
    'sightline_corridor_m': SIGHTLINE_CORRIDOR_M,
    'keep_out_behind_m': KEEP_OUT_BEHIND_M,
}

# Why a prop is dropped from a scene.
TARGET_FOOTPRINT = 'target_footprint'
TARGET_SIGHTLINE = 'target_sightline'
BEHIND_TARGET = 'behind_target_cone'
OBJECT_FOOTPRINT = 'object_footprint'
OBJECT_SIGHTLINE = 'object_sightline'

_PROP_ID = re.compile(r'^[a-z][a-z0-9_]*$')
_LAYOUT_FIELDS = frozenset({'schema', 'world', 'room', 'props'})
_PROP_FIELDS = frozenset({'id', 'model', 'x', 'y', 'yaw'})
_ROOM_FIELDS = ('x_min', 'x_max', 'y_min', 'y_max')

Point = tuple[float, float]
Outline = tuple[Point, Point, Point, Point]


class BackgroundError(ValueError):
    """A background layout or drop file that is malformed, stale or incomplete."""


@dataclass(frozen=True)
class Footprint:
    """A model's planar bounds in its own frame: x0 <= x <= x1, y0 <= y <= y1."""

    x0: float
    x1: float
    y0: float
    y1: float

    def as_list(self) -> list[float]:
        return [self.x0, self.x1, self.y0, self.y1]


@dataclass(frozen=True)
class BackgroundProp:
    id: str
    model: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Room:
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class BackgroundLayout:
    path: str
    sha256: str
    world: str
    room: Room
    props: tuple[BackgroundProp, ...]

    @property
    def models(self) -> frozenset[str]:
        return frozenset(prop.model for prop in self.props)


@dataclass(frozen=True)
class BackgroundDrops:
    """The generated per-scene drop lists of one background."""

    path: str
    sha256: str
    background: str
    layout_file: str
    layout_sha256: str
    scenarios: Mapping[str, tuple[str, Mapping[str, tuple[str, ...]]]]


@dataclass(frozen=True)
class SceneBackground:
    """One run's background: the layout and the props each scene drops."""

    name: str
    layout: BackgroundLayout
    drops: BackgroundDrops
    scene_drops: Mapping[str, tuple[str, ...]]

    def kept(self, scene_id: str) -> tuple[BackgroundProp, ...]:
        dropped = set(self.scene_drops[scene_id])
        return tuple(prop for prop in self.layout.props if prop.id not in dropped)

    def provenance(self) -> dict[str, Any]:
        return {
            'name': self.name,
            'layout': {'path': self.layout.path, 'sha256': self.layout.sha256},
            'drops': {'path': self.drops.path, 'sha256': self.drops.sha256},
            'props': len(self.layout.props),
            'scene_drops': {
                scene_id: list(dropped) for scene_id, dropped in self.scene_drops.items()},
        }


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def background_files(config_dir: str, name: str) -> tuple[str, str]:
    """The layout and drop-file paths of a packaged background."""

    if not _PROP_ID.match(name):
        raise BackgroundError(f'Background name {name!r} must be a lower-case identifier.')
    return (
        os.path.join(config_dir, LAYOUT_FILE_PATTERN.format(name=name)),
        os.path.join(config_dir, DROPS_DIRECTORY, f'{name}.yaml'),
    )


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise BackgroundError(f'{where} must be a finite number, got {value!r}.')
    return float(value)


def _mapping(value: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise BackgroundError(f'{where} must be a mapping, got {value!r}.')
    return value


def parse_background_layout(
    document: Any,
    *,
    path: str = '<memory>',
    sha256: str = '',
) -> BackgroundLayout:
    document = _mapping(document, f'Background layout {path}')
    if document.get('schema') != LAYOUT_SCHEMA:
        raise BackgroundError(
            f'Background layout {path}: unsupported schema {document.get("schema")!r}; '
            f'expected "{LAYOUT_SCHEMA}".')
    unknown = sorted(set(document) - _LAYOUT_FIELDS)
    if unknown:
        raise BackgroundError(f'Background layout {path} has unknown fields: {", ".join(unknown)}.')
    world = document.get('world')
    if not isinstance(world, str) or not world.strip():
        raise BackgroundError(f'Background layout {path}: "world" must be a non-empty string.')
    raw_room = _mapping(document.get('room'), f'{path}: "room"')
    room = Room(*(_number(raw_room.get(field), f'{path}: room.{field}') for field in _ROOM_FIELDS))
    if room.x_min >= room.x_max or room.y_min >= room.y_max:
        raise BackgroundError(f'Background layout {path}: room bounds are empty.')

    raw_props = document.get('props')
    if not isinstance(raw_props, list) or not raw_props:
        raise BackgroundError(f'Background layout {path}: "props" must be a non-empty list.')
    props = []
    seen = set()
    for index, raw in enumerate(raw_props):
        where = f'{path}: props[{index}]'
        raw = _mapping(raw, where)
        unknown = sorted(set(raw) - _PROP_FIELDS)
        if unknown:
            raise BackgroundError(f'{where} has unknown fields: {", ".join(unknown)}.')
        prop_id = raw.get('id')
        if not isinstance(prop_id, str) or not _PROP_ID.match(prop_id):
            raise BackgroundError(f'{where}: "id" must be a lower-case identifier, got {prop_id!r}.')
        if prop_id in seen:
            raise BackgroundError(f'{where}: duplicate prop id "{prop_id}".')
        seen.add(prop_id)
        model = raw.get('model')
        if not isinstance(model, str) or not _PROP_ID.match(model):
            raise BackgroundError(f'{where}: "model" must be a lower-case identifier, got {model!r}.')
        props.append(BackgroundProp(
            id=prop_id,
            model=model,
            x=_number(raw.get('x'), f'{where}.x'),
            y=_number(raw.get('y'), f'{where}.y'),
            yaw=_number(raw.get('yaw', 0.0), f'{where}.yaw'),
        ))
    return BackgroundLayout(
        path=path, sha256=sha256, world=world.strip(), room=room, props=tuple(props))


def load_background_layout(path: str) -> BackgroundLayout:
    path = os.path.abspath(path)
    with open(path, 'rb') as handle:
        raw = handle.read()
    return parse_background_layout(yaml.safe_load(raw), path=path, sha256=sha256_bytes(raw))


def parse_background_drops(
    document: Any,
    *,
    path: str = '<memory>',
    sha256: str = '',
) -> BackgroundDrops:
    document = _mapping(document, f'Background drop file {path}')
    if document.get('schema') != DROPS_SCHEMA:
        raise BackgroundError(
            f'Background drop file {path}: unsupported schema {document.get("schema")!r}; '
            f'expected "{DROPS_SCHEMA}".')
    layout = _mapping(document.get('layout'), f'{path}: "layout"')
    scenarios = {}
    for name, raw in _mapping(document.get('scenarios'), f'{path}: "scenarios"').items():
        raw = _mapping(raw, f'{path}: scenarios.{name}')
        scenes = {
            str(scene_id): tuple(str(prop_id) for prop_id in (dropped or ()))
            for scene_id, dropped in _mapping(
                raw.get('scenes'), f'{path}: scenarios.{name}.scenes').items()
        }
        scenarios[str(name)] = (str(raw.get('sha256', '')), scenes)
    return BackgroundDrops(
        path=path,
        sha256=sha256,
        background=str(document.get('background', '')),
        layout_file=str(layout.get('file', '')),
        layout_sha256=str(layout.get('sha256', '')),
        scenarios=scenarios,
    )


def load_background_drops(path: str) -> BackgroundDrops:
    path = os.path.abspath(path)
    with open(path, 'rb') as handle:
        raw = handle.read()
    return parse_background_drops(yaml.safe_load(raw), path=path, sha256=sha256_bytes(raw))


def resolve_scene_background(
    name: str,
    config_dir: str,
    scenario_path: str,
    scene_ids: Sequence[str],
) -> SceneBackground:
    """Load a packaged background and the drop list of every scene to run.

    Refuses a layout or scenario file that changed since the drop file was
    generated, and any scene the drop file does not cover: either would put
    props where the drop rule never checked them.
    """

    layout_path, drops_path = background_files(config_dir, name)
    layout = load_background_layout(layout_path)
    drops = load_background_drops(drops_path)
    regenerate = 'regenerate it with tools/benchmark/build_background.py'
    if (drops.layout_file != os.path.basename(layout_path)
            or drops.layout_sha256 != layout.sha256):
        raise BackgroundError(
            f'Background drop file {drops_path} was generated from a different layout '
            f'than {layout_path}; {regenerate}.')
    scenario_file = os.path.basename(scenario_path)
    if scenario_file not in drops.scenarios:
        raise BackgroundError(
            f'Background "{name}" has no drop lists for scenario file {scenario_file}; '
            f'it covers {", ".join(sorted(drops.scenarios))}.')
    expected_sha, scenes = drops.scenarios[scenario_file]
    with open(scenario_path, 'rb') as handle:
        actual_sha = sha256_bytes(handle.read())
    if actual_sha != expected_sha:
        raise BackgroundError(
            f'Scenario file {scenario_path} differs from the one background "{name}" '
            f'was generated for; {regenerate}.')
    missing = [scene_id for scene_id in scene_ids if scene_id not in scenes]
    if missing:
        raise BackgroundError(
            f'Background "{name}" has no drop list for scene(s) {", ".join(missing)}; '
            f'{regenerate}.')
    prop_ids = {prop.id for prop in layout.props}
    unknown = sorted({
        prop_id for scene_id in scene_ids for prop_id in scenes[scene_id]
        if prop_id not in prop_ids})
    if unknown:
        raise BackgroundError(
            f'Background drop file {drops_path} names props the layout lacks: '
            f'{", ".join(unknown)}.')
    return SceneBackground(
        name=name,
        layout=layout,
        drops=drops,
        scene_drops={scene_id: scenes[scene_id] for scene_id in scene_ids},
    )


# --- Geometry ----------------------------------------------------------------

def placed_outline(footprint: Footprint, x: float, y: float, yaw: float) -> Outline:
    """World-frame corners of a footprint placed at (x, y, yaw)."""

    c, s = math.cos(yaw), math.sin(yaw)
    return tuple(
        (x + lx * c - ly * s, y + lx * s + ly * c)
        for lx, ly in (
            (footprint.x1, footprint.y1), (footprint.x0, footprint.y1),
            (footprint.x0, footprint.y0), (footprint.x1, footprint.y0))
    )


def _point_segment_distance(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(
        0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _segments_cross(a: Point, b: Point, c: Point, d: Point) -> bool:
    def orient(p: Point, q: Point, r: Point) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return (orient(a, b, c) * orient(a, b, d) < 0.0
            and orient(c, d, a) * orient(c, d, b) < 0.0)


def _inside(p: Point, outline: Outline) -> bool:
    sign = None
    for i in range(4):
        a, b = outline[i], outline[(i + 1) % 4]
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if sign is None:
            sign = cross > 0.0
        elif (cross > 0.0) != sign:
            return False
    return True


def _edges(outline: Outline) -> Iterable[tuple[Point, Point]]:
    return ((outline[i], outline[(i + 1) % 4]) for i in range(4))


def outline_segment_distance(outline: Outline, a: Point, b: Point) -> float:
    """Distance from a convex outline to segment ab; zero when they touch."""

    if _inside(a, outline) or _inside(b, outline):
        return 0.0
    best = math.inf
    for e0, e1 in _edges(outline):
        if _segments_cross(a, b, e0, e1):
            return 0.0
        best = min(best, _point_segment_distance(a, e0, e1),
                   _point_segment_distance(b, e0, e1), _point_segment_distance(e0, a, b))
    return best


def outline_distance(first: Outline, second: Outline) -> float:
    return min(outline_segment_distance(first, a, b) for a, b in _edges(second))


def outline_samples(outline: Outline, step: float = OUTLINE_SAMPLE_STEP_M) -> list[Point]:
    points = []
    for a, b in _edges(outline):
        count = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        points.extend(
            (a[0] + (b[0] - a[0]) * k / count, a[1] + (b[1] - a[1]) * k / count)
            for k in range(count))
    return points


def camera_range(p: Point) -> float:
    return math.hypot(p[0] - CAMERA_XY[0], p[1] - CAMERA_XY[1])


def camera_bearing(p: Point) -> float:
    return math.atan2(p[1] - CAMERA_XY[1], p[0] - CAMERA_XY[0])


def drop_reasons(
    outline: Outline,
    scene: Any,
    object_footprints: Mapping[str, Footprint],
) -> tuple[str, ...]:
    """Why a prop with this outline cannot stay in ``scene``; empty when it can.

    A prop is dropped when it comes within the target radius plus margin of a
    target centre, within the margin of an object's footprint, within the
    corridor of a camera-to-target or camera-to-object sight line, or inside a
    target's bearing cone (the target's width plus margin as seen from the
    camera) closer than the target's range plus the keep-out distance.
    """

    reasons = set()
    samples = None
    for robot in scene.robots:
        target = (robot.x, robot.y)
        if outline_segment_distance(outline, target, target) < TARGET_RADIUS_M + MARGIN_M:
            reasons.add(TARGET_FOOTPRINT)
        if outline_segment_distance(outline, CAMERA_XY, target) < SIGHTLINE_CORRIDOR_M:
            reasons.add(TARGET_SIGHTLINE)
        target_range = camera_range(target)
        half_width = math.atan((TARGET_RADIUS_M + MARGIN_M) / target_range)
        target_bearing = camera_bearing(target)
        if samples is None:
            samples = outline_samples(outline)
        for p in samples:
            offset = (camera_bearing(p) - target_bearing + math.pi) % (2.0 * math.pi) - math.pi
            if camera_range(p) < target_range + KEEP_OUT_BEHIND_M and abs(offset) < half_width:
                reasons.add(BEHIND_TARGET)
                break
    for obj in scene.objects:
        footprint = object_footprints[obj.model]
        if outline_distance(outline, placed_outline(footprint, obj.x, obj.y, obj.yaw)) < MARGIN_M:
            reasons.add(OBJECT_FOOTPRINT)
        if outline_segment_distance(outline, CAMERA_XY, (obj.x, obj.y)) < SIGHTLINE_CORRIDOR_M:
            reasons.add(OBJECT_SIGHTLINE)
    return tuple(sorted(reasons))


def prop_outlines(
    layout: BackgroundLayout,
    prop_footprints: Mapping[str, Footprint],
) -> dict[str, Outline]:
    return {
        prop.id: placed_outline(prop_footprints[prop.model], prop.x, prop.y, prop.yaw)
        for prop in layout.props
    }


def scene_drops(
    layout: BackgroundLayout,
    prop_footprints: Mapping[str, Footprint],
    scene: Any,
    object_footprints: Mapping[str, Footprint],
) -> dict[str, tuple[str, ...]]:
    """The props ``scene`` drops, in layout order, each with its reasons."""

    dropped = {}
    for prop_id, outline in prop_outlines(layout, prop_footprints).items():
        reasons = drop_reasons(outline, scene, object_footprints)
        if reasons:
            dropped[prop_id] = reasons
    return dropped


def layout_problems(
    layout: BackgroundLayout,
    prop_footprints: Mapping[str, Footprint],
) -> list[str]:
    """Props that leave the room or touch each other."""

    problems = []
    outlines = prop_outlines(layout, prop_footprints)
    room = layout.room
    tolerance = ROOM_TOLERANCE_M
    for prop_id, outline in outlines.items():
        for x, y in outline:
            if not (room.x_min - tolerance <= x <= room.x_max + tolerance
                    and room.y_min - tolerance <= y <= room.y_max + tolerance):
                problems.append(f'{prop_id} leaves the room at ({x:.3f}, {y:.3f})')
                break
    ids = list(outlines)
    for i, first in enumerate(ids):
        for second in ids[i + 1:]:
            gap = outline_distance(outlines[first], outlines[second])
            if gap < MIN_PROP_GAP_M:
                problems.append(f'{first} and {second} are {gap:.3f} m apart')
    return problems
