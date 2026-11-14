"""Benchmark background: one fixed room layout, a placement and drops per scene.

A background is a master layout of static props for the benchmark room. Props
are never moved and never randomized. Each scene is captured with the whole
scene -- the sensor robot, its targets and its objects -- shifted along world
x by that scene's placement ``dx``. Every pose relative to the robot stays
exactly as the scenario file declares it, so ranges, bearings and the scene's
occlusion certifications are untouched; only the room behind the targets
changes. The placement is chosen so the farthest target stands a chosen gap in
front of the prop seen behind it.

The designed scenes control two things the room must not disturb: what stands
between the camera and each target or object, and the objects a scene places
behind a target on purpose. For each scene, a deterministic rule drops every
prop that would sit on or beside a target, an object or the robot, in a
camera-to-target or camera-to-object sight line, or in front of a designed
behind-target object. Every other prop stays, so clutter close behind a target
is intended.

The rule and the placement run offline: ``tools/benchmark/build_background.py``
evaluates them for every packaged scenario file and writes each scene's
placement and drop list. The runner only reads that file, so the room a scene
is captured in is fixed by data under review rather than by code that could
change between runs. The drop file pins the layout and each scenario file by
SHA-256, so an edit to either without regeneration is refused instead of
silently mismatched.

Kept pure (no ROS): the generator, the tests and the runner share it.

Layout file::

    schema: ridgeback_benchmark_background_layout/2
    world: <public world>
    room: {x_min: ..., x_max: ..., y_min: ..., y_max: ...}
    placement: {backdrop_gap_m: ..., dx_step_m: ...}
    props:
      - {id: <prop id>, model: <logical prop model>, x: ..., y: ..., yaw: ...}

Drop file (generated)::

    schema: ridgeback_benchmark_background_drops/2
    background: <name>
    layout: {file: <layout file name>, sha256: ...}
    rules: {...}
    footprints: {<logical prop model>: [x0, x1, y0, y1], ...}
    scenarios:
      <scenario file name>:
        sha256: ...
        scenes:
          <scene id>: {dx_m: ..., anchor: {target_index: ..., backdrop: <prop id> | wall,
                       gap_m: ...}, drops: [<dropped prop id>, ...]}
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


LAYOUT_SCHEMA = 'ridgeback_benchmark_background_layout/2'
DROPS_SCHEMA = 'ridgeback_benchmark_background_drops/2'
# Version 1 placed every scene with the robot at the world origin.
_SCHEMA_V1_SUFFIX = '/1'

# The packaged layouts and their generated drop files, both under the
# package's config directory.
LAYOUT_FILE_PATTERN = 'benchmark_background_{name}.yaml'
DROPS_DIRECTORY = 'benchmark_backgrounds'

# Planar position of camera_0's mount in the robot frame (clearpath/robot.yaml).
# The robot always faces +x, so a scene placed at dx has its camera at
# (dx + x, y).
CAMERA_MOUNT_XY = (0.2692, 0.0)
# A standing G1's footprint radius: the half-width of its silhouette as seen
# from the camera, and with the margin its clearance from the room's walls.
TARGET_RADIUS_M = 0.40
# How close a prop may come to a G1's spawn point. The G1's farthest planar
# point is 0.393 m from it, so 0.50 m keeps every prop clear of the G1
# whatever its yaw; the tests pin that against the model's meshes.
TARGET_PROP_CLEARANCE_M = 0.50
# Clearance around objects and the robot, and of every scene body from the
# room's walls.
MARGIN_M = 0.30
# Half-width of the corridor kept clear around each camera-to-target and
# camera-to-object sight line.
SIGHTLINE_CORRIDOR_M = 0.45
# Two props closer than this count as touching in the layout check.
MIN_PROP_GAP_M = 0.05
# Wall props sit flush against the wall; a yaw written to four decimals moves
# a corner by micrometres, which must not read as leaving the room.
ROOM_TOLERANCE_M = 1e-3
# Gaps that differ by less than this compare equal when choosing a placement,
# so the choice does not hinge on floating-point noise.
GAP_RESOLUTION_M = 1e-6

RULES = {
    'camera_mount_xy': list(CAMERA_MOUNT_XY),
    'target_radius_m': TARGET_RADIUS_M,
    'target_prop_clearance_m': TARGET_PROP_CLEARANCE_M,
    'margin_m': MARGIN_M,
    'sightline_corridor_m': SIGHTLINE_CORRIDOR_M,
}

# Why a prop is dropped from a scene.
TARGET_FOOTPRINT = 'target_footprint'
TARGET_SIGHTLINE = 'target_sightline'
OBJECT_FOOTPRINT = 'object_footprint'
OBJECT_SIGHTLINE = 'object_sightline'
ROBOT_FOOTPRINT = 'robot_footprint'
BEHIND_TARGET_OBJECT = 'behind_target_object'

# The backdrop of a scene whose anchor sees no prop behind it.
WALL = 'wall'

_PROP_ID = re.compile(r'^[a-z][a-z0-9_]*$')
_LAYOUT_FIELDS = frozenset({'schema', 'world', 'room', 'placement', 'props'})
_PROP_FIELDS = frozenset({'id', 'model', 'x', 'y', 'yaw'})
_ROOM_FIELDS = ('x_min', 'x_max', 'y_min', 'y_max')
_PLACEMENT_FIELDS = ('backdrop_gap_m', 'dx_step_m')
_SCENE_FIELDS = frozenset({'dx_m', 'anchor', 'drops'})
_ANCHOR_FIELDS = frozenset({'target_index', 'backdrop', 'gap_m'})

Point = tuple[float, float]
# A convex polygon, its corners in order.
Outline = tuple[Point, ...]


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

    def outline(self) -> Outline:
        return ((self.x_max, self.y_max), (self.x_min, self.y_max),
                (self.x_min, self.y_min), (self.x_max, self.y_min))


@dataclass(frozen=True)
class Placement:
    """How a layout places each scene: the backdrop gap it aims for, on a dx grid."""

    backdrop_gap_m: float
    dx_step_m: float


@dataclass(frozen=True)
class BackgroundLayout:
    path: str
    sha256: str
    world: str
    room: Room
    placement: Placement
    props: tuple[BackgroundProp, ...]

    @property
    def models(self) -> frozenset[str]:
        return frozenset(prop.model for prop in self.props)


@dataclass(frozen=True)
class Anchor:
    """The target a scene is placed by, and what stands behind it."""

    target_index: int
    backdrop: str
    gap_m: float

    def as_record(self) -> dict[str, Any]:
        return {'target_index': self.target_index, 'backdrop': self.backdrop,
                'gap_m': self.gap_m}


@dataclass(frozen=True)
class ScenePlacement:
    """One scene's shift along world x, its anchor, and the props it drops."""

    dx_m: float
    anchor: Anchor
    drops: tuple[str, ...]

    def as_record(self) -> dict[str, Any]:
        return {'dx_m': self.dx_m, 'anchor': self.anchor.as_record(),
                'drops': list(self.drops)}


@dataclass(frozen=True)
class BackgroundDrops:
    """The generated per-scene placements and drop lists of one background."""

    path: str
    sha256: str
    background: str
    layout_file: str
    layout_sha256: str
    rules: Mapping[str, Any]
    scenarios: Mapping[str, tuple[str, Mapping[str, ScenePlacement]]]


@dataclass(frozen=True)
class SceneBackground:
    """One run's background: the layout and each scene's placement and drops."""

    name: str
    layout: BackgroundLayout
    drops: BackgroundDrops
    placements: Mapping[str, ScenePlacement]

    @property
    def scene_drops(self) -> dict[str, tuple[str, ...]]:
        return {scene_id: placement.drops for scene_id, placement in self.placements.items()}

    def dx_m(self, scene_id: str) -> float:
        return self.placements[scene_id].dx_m

    def anchor(self, scene_id: str) -> Anchor:
        return self.placements[scene_id].anchor

    def kept(self, scene_id: str) -> tuple[BackgroundProp, ...]:
        dropped = set(self.placements[scene_id].drops)
        return tuple(prop for prop in self.layout.props if prop.id not in dropped)

    def provenance(self) -> dict[str, Any]:
        return {
            'name': self.name,
            'layout': {'path': self.layout.path, 'sha256': self.layout.sha256},
            'drops': {'path': self.drops.path, 'sha256': self.drops.sha256},
            'props': len(self.layout.props),
            'backdrop_gap_m': self.layout.placement.backdrop_gap_m,
            'dx_step_m': self.layout.placement.dx_step_m,
            'rules': dict(self.drops.rules),
            'scenes': {
                scene_id: placement.as_record()
                for scene_id, placement in self.placements.items()},
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


def _check_schema(document: Mapping[str, Any], expected: str, what: str, path: str) -> None:
    schema = document.get('schema')
    if schema == expected:
        return
    if isinstance(schema, str) and schema == expected[:-2] + _SCHEMA_V1_SUFFIX:
        raise BackgroundError(
            f'{what} {path} uses schema "{schema}", which placed every scene with the '
            f'robot at the world origin; this runner places each scene by its dx. '
            f'Regenerate it with tools/benchmark/build_background.py.')
    raise BackgroundError(
        f'{what} {path}: unsupported schema {schema!r}; expected "{expected}".')


def parse_background_layout(
    document: Any,
    *,
    path: str = '<memory>',
    sha256: str = '',
) -> BackgroundLayout:
    document = _mapping(document, f'Background layout {path}')
    _check_schema(document, LAYOUT_SCHEMA, 'Background layout', path)
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
    raw_placement = _mapping(document.get('placement'), f'{path}: "placement"')
    unknown = sorted(set(raw_placement) - set(_PLACEMENT_FIELDS))
    if unknown:
        raise BackgroundError(f'{path}: placement has unknown fields: {", ".join(unknown)}.')
    placement = Placement(*(
        _number(raw_placement.get(field), f'{path}: placement.{field}')
        for field in _PLACEMENT_FIELDS))
    if placement.backdrop_gap_m <= 0.0 or placement.dx_step_m <= 0.0:
        raise BackgroundError(f'Background layout {path}: placement values must be positive.')

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
        if prop_id == WALL:
            raise BackgroundError(f'{where}: "{WALL}" is reserved for the room wall.')
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
        path=path, sha256=sha256, world=world.strip(), room=room, placement=placement,
        props=tuple(props))


def load_background_layout(path: str) -> BackgroundLayout:
    path = os.path.abspath(path)
    with open(path, 'rb') as handle:
        raw = handle.read()
    return parse_background_layout(yaml.safe_load(raw), path=path, sha256=sha256_bytes(raw))


def _parse_scene_placement(raw: Any, where: str) -> ScenePlacement:
    raw = _mapping(raw, where)
    unknown = sorted(set(raw) - _SCENE_FIELDS)
    if unknown:
        raise BackgroundError(f'{where} has unknown fields: {", ".join(unknown)}.')
    anchor = _mapping(raw.get('anchor'), f'{where}.anchor')
    unknown = sorted(set(anchor) - _ANCHOR_FIELDS)
    if unknown:
        raise BackgroundError(f'{where}.anchor has unknown fields: {", ".join(unknown)}.')
    index = anchor.get('target_index')
    if isinstance(index, bool) or not isinstance(index, int) or index < 0:
        raise BackgroundError(f'{where}.anchor.target_index must be a non-negative integer.')
    drops = raw.get('drops')
    if not isinstance(drops, list):
        raise BackgroundError(f'{where}.drops must be a list.')
    return ScenePlacement(
        dx_m=_number(raw.get('dx_m'), f'{where}.dx_m'),
        anchor=Anchor(
            target_index=index,
            backdrop=str(anchor.get('backdrop', '')),
            gap_m=_number(anchor.get('gap_m'), f'{where}.anchor.gap_m'),
        ),
        drops=tuple(str(prop_id) for prop_id in drops),
    )


def parse_background_drops(
    document: Any,
    *,
    path: str = '<memory>',
    sha256: str = '',
) -> BackgroundDrops:
    document = _mapping(document, f'Background drop file {path}')
    _check_schema(document, DROPS_SCHEMA, 'Background drop file', path)
    layout = _mapping(document.get('layout'), f'{path}: "layout"')
    scenarios = {}
    for name, raw in _mapping(document.get('scenarios'), f'{path}: "scenarios"').items():
        raw = _mapping(raw, f'{path}: scenarios.{name}')
        scenes = {
            str(scene_id): _parse_scene_placement(scene, f'{path}: {name}: {scene_id}')
            for scene_id, scene in _mapping(
                raw.get('scenes'), f'{path}: scenarios.{name}.scenes').items()
        }
        scenarios[str(name)] = (str(raw.get('sha256', '')), scenes)
    return BackgroundDrops(
        path=path,
        sha256=sha256,
        background=str(document.get('background', '')),
        layout_file=str(layout.get('file', '')),
        layout_sha256=str(layout.get('sha256', '')),
        rules=dict(_mapping(document.get('rules', {}), f'{path}: "rules"')),
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
    """Load a packaged background and the placement of every scene to run.

    Refuses a layout or scenario file that changed since the drop file was
    generated, and any scene the drop file does not cover: either would place
    the robot or props where the rules never checked them.
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
        prop_id for scene_id in scene_ids for prop_id in scenes[scene_id].drops
        if prop_id not in prop_ids})
    unknown += sorted({
        scenes[scene_id].anchor.backdrop for scene_id in scene_ids
        if scenes[scene_id].anchor.backdrop not in prop_ids | {WALL}})
    if unknown:
        raise BackgroundError(
            f'Background drop file {drops_path} names props the layout lacks: '
            f'{", ".join(unknown)}.')
    return SceneBackground(
        name=name,
        layout=layout,
        drops=drops,
        placements={scene_id: scenes[scene_id] for scene_id in scene_ids},
    )


# --- Geometry ----------------------------------------------------------------

def place_polygon(points: Sequence[Point], x: float, y: float, yaw: float) -> Outline:
    """World-frame corners of a polygon given in a body frame placed at (x, y, yaw)."""

    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in points)


def placed_outline(footprint: Footprint, x: float, y: float, yaw: float) -> Outline:
    """World-frame corners of a footprint placed at (x, y, yaw)."""

    return place_polygon(
        ((footprint.x1, footprint.y1), (footprint.x0, footprint.y1),
         (footprint.x0, footprint.y0), (footprint.x1, footprint.y0)),
        x, y, yaw)


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
    for a, b in _edges(outline):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if sign is None:
            sign = cross > 0.0
        elif (cross > 0.0) != sign:
            return False
    return True


def _edges(outline: Outline) -> Iterable[tuple[Point, Point]]:
    return ((outline[i], outline[(i + 1) % len(outline)]) for i in range(len(outline)))


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
    """Distance between two convex outlines; zero when they touch or one holds the other."""

    if any(_inside(p, second) for p in first):
        return 0.0
    return min(outline_segment_distance(first, a, b) for a, b in _edges(second))


def bounding_circle(outline: Outline) -> tuple[Point, float]:
    """A circle holding the outline: its corners' mean and farthest corner."""

    cx = sum(x for x, _ in outline) / len(outline)
    cy = sum(y for _, y in outline) / len(outline)
    return (cx, cy), max(math.hypot(x - cx, y - cy) for x, y in outline)


def camera_xy(dx_m: float) -> Point:
    """The camera's planar position for a scene placed at ``dx_m``."""

    return (dx_m + CAMERA_MOUNT_XY[0], CAMERA_MOUNT_XY[1])


def planar_range(origin: Point, p: Point) -> float:
    return math.hypot(p[0] - origin[0], p[1] - origin[1])


def planar_bearing(origin: Point, p: Point) -> float:
    return math.atan2(p[1] - origin[1], p[0] - origin[0])


def nearest_in_cone(
    outline: Outline,
    origin: Point,
    bearing: float,
    half_width: float,
    beyond: float = 0.0,
) -> float | None:
    """The smallest range from ``origin`` of an outline point inside a cone.

    The cone opens from ``origin`` along ``bearing`` by ``half_width`` to each
    side. Only points farther than ``beyond`` count; when the outline crosses
    that range inside the cone the result is ``beyond`` itself. None when no
    outline point lies in the cone beyond that range. Exact: each edge is
    clipped to the cone, not sampled.
    """

    c, s = math.cos(bearing), math.sin(bearing)
    slope = math.tan(half_width)
    local = [((p[0] - origin[0]) * c + (p[1] - origin[1]) * s,
              -(p[0] - origin[0]) * s + (p[1] - origin[1]) * c) for p in outline]
    best = None
    for i in range(len(local)):
        (u0, v0), (u1, v1) = local[i], local[(i + 1) % len(local)]
        t0, t1 = 0.0, 1.0
        # Inside the cone: v - u * slope <= 0 and -v - u * slope <= 0.
        for f0, f1 in ((v0 - u0 * slope, v1 - u1 * slope), (-v0 - u0 * slope, -v1 - u1 * slope)):
            if f0 > 0.0 and f1 > 0.0:
                t0, t1 = 1.0, 0.0
                break
            if f0 > 0.0:
                t0 = max(t0, f0 / (f0 - f1))
            elif f1 > 0.0:
                t1 = min(t1, f0 / (f0 - f1))
        if t0 > t1:
            continue
        a = (u0 + (u1 - u0) * t0, v0 + (v1 - v0) * t0)
        b = (u0 + (u1 - u0) * t1, v0 + (v1 - v0) * t1)
        nearest = _point_segment_distance((0.0, 0.0), a, b)
        if nearest > beyond:
            candidate = nearest
        elif max(math.hypot(*a), math.hypot(*b)) > beyond:
            candidate = beyond
        else:
            continue
        best = candidate if best is None else min(best, candidate)
    return best


# --- The drop rule -------------------------------------------------------------

@dataclass(frozen=True)
class SceneFrame:
    """One scene's bodies in the world frame, the scene placed at ``dx_m``."""

    dx_m: float
    camera: Point
    robot: Outline
    targets: tuple[Point, ...]
    object_centres: tuple[Point, ...]
    objects: tuple[Outline, ...]
    # Per target with a designed object behind it: the cone (bearing,
    # half-width) and the range of that object's nearest point inside it.
    behind_cones: tuple[tuple[float, float, float], ...]
    # Bounding circles of the robot and each object, for cheap rejection.
    robot_circle: tuple[Point, float]
    object_circles: tuple[tuple[Point, float], ...]


def scene_frame(
    scene: Any,
    dx_m: float,
    robot_footprint: Sequence[Point],
    object_footprints: Mapping[str, Footprint],
) -> SceneFrame:
    camera = camera_xy(dx_m)
    targets = tuple((robot.x + dx_m, robot.y) for robot in scene.robots)
    objects = tuple(
        placed_outline(object_footprints[obj.model], obj.x + dx_m, obj.y, obj.yaw)
        for obj in scene.objects)
    behind_cones = []
    for target in targets:
        target_range = planar_range(camera, target)
        bearing = planar_bearing(camera, target)
        half_width = math.atan((TARGET_RADIUS_M + MARGIN_M) / target_range)
        ranges = [
            nearest_in_cone(outline, camera, bearing, half_width, beyond=target_range)
            for outline in objects]
        ranges = [value for value in ranges if value is not None]
        if ranges:
            behind_cones.append((bearing, half_width, max(ranges)))
    robot = place_polygon(robot_footprint, dx_m, 0.0, 0.0)
    return SceneFrame(
        dx_m=dx_m,
        camera=camera,
        robot=robot,
        targets=targets,
        object_centres=tuple((obj.x + dx_m, obj.y) for obj in scene.objects),
        objects=objects,
        behind_cones=tuple(behind_cones),
        robot_circle=bounding_circle(robot),
        object_circles=tuple(bounding_circle(outline) for outline in objects),
    )


def drop_reasons(outline: Outline, frame: SceneFrame) -> tuple[str, ...]:
    """Why a prop with this outline cannot stay in the placed scene; empty when it can.

    A prop is dropped when it comes within the prop clearance of a target's
    spawn point, within the margin of an object's or the robot's footprint,
    within the corridor of a camera-to-target or camera-to-object sight line,
    or -- for a target with a designed object behind it -- inside that
    target's cone (its width plus margin as seen from the camera) nearer than
    the object.
    """

    # Each test first rejects on the prop's bounding circle, which can only
    # be farther from a body than the outline itself is.
    reasons = set()
    camera = frame.camera
    centre, radius = bounding_circle(outline)
    for target in frame.targets:
        if (planar_range(centre, target) - radius < TARGET_PROP_CLEARANCE_M
                and outline_segment_distance(outline, target, target)
                < TARGET_PROP_CLEARANCE_M):
            reasons.add(TARGET_FOOTPRINT)
        if (_point_segment_distance(centre, camera, target) - radius < SIGHTLINE_CORRIDOR_M
                and outline_segment_distance(outline, camera, target) < SIGHTLINE_CORRIDOR_M):
            reasons.add(TARGET_SIGHTLINE)
    for object_centre, object_outline, (circle_centre, circle_radius) in zip(
            frame.object_centres, frame.objects, frame.object_circles):
        if (planar_range(centre, circle_centre) - radius - circle_radius < MARGIN_M
                and outline_distance(outline, object_outline) < MARGIN_M):
            reasons.add(OBJECT_FOOTPRINT)
        if (_point_segment_distance(centre, camera, object_centre) - radius
                < SIGHTLINE_CORRIDOR_M
                and outline_segment_distance(outline, camera, object_centre)
                < SIGHTLINE_CORRIDOR_M):
            reasons.add(OBJECT_SIGHTLINE)
    robot_centre, robot_radius = frame.robot_circle
    if (planar_range(centre, robot_centre) - radius - robot_radius < MARGIN_M
            and outline_distance(outline, frame.robot) < MARGIN_M):
        reasons.add(ROBOT_FOOTPRINT)
    for bearing, half_width, limit in frame.behind_cones:
        if planar_range(camera, centre) - radius >= limit:
            continue
        nearest = nearest_in_cone(outline, camera, bearing, half_width)
        if nearest is not None and nearest < limit:
            reasons.add(BEHIND_TARGET_OBJECT)
    return tuple(sorted(reasons))


def prop_outlines(
    layout: BackgroundLayout,
    prop_footprints: Mapping[str, Footprint],
) -> dict[str, Outline]:
    return {
        prop.id: placed_outline(prop_footprints[prop.model], prop.x, prop.y, prop.yaw)
        for prop in layout.props
    }


def frame_drops(
    outlines: Mapping[str, Outline],
    frame: SceneFrame,
) -> dict[str, tuple[str, ...]]:
    """The props the placed scene drops, in layout order, each with its reasons."""

    dropped = {}
    for prop_id, outline in outlines.items():
        reasons = drop_reasons(outline, frame)
        if reasons:
            dropped[prop_id] = reasons
    return dropped


# --- Placement -----------------------------------------------------------------

def feasible_dx(
    room: Room,
    scene: Any,
    robot_footprint: Sequence[Point],
    object_footprints: Mapping[str, Footprint],
) -> tuple[float, float]:
    """The dx interval that keeps the robot, every target and every object in the room.

    Each body keeps the margin from the walls: the robot's footprint and each
    object's footprint, and each target's radius. y is never shifted, so a
    body outside the room's y range has no placement at all.
    """

    # (x extent at dx = 0, y extent, clearance) per body.
    bodies = [(
        (min(x for x, _ in robot_footprint), max(x for x, _ in robot_footprint)),
        (min(y for _, y in robot_footprint), max(y for _, y in robot_footprint)),
        MARGIN_M, 'robot')]
    for index, robot in enumerate(scene.robots):
        reach = TARGET_RADIUS_M + MARGIN_M
        bodies.append(((robot.x - reach, robot.x + reach), (robot.y - reach, robot.y + reach),
                       0.0, f'target {index}'))
    for index, obj in enumerate(scene.objects):
        outline = placed_outline(object_footprints[obj.model], obj.x, obj.y, obj.yaw)
        bodies.append(((min(x for x, _ in outline), max(x for x, _ in outline)),
                       (min(y for _, y in outline), max(y for _, y in outline)),
                       MARGIN_M, f'object {index} ({obj.model})'))
    low, high = -math.inf, math.inf
    for (x0, x1), (y0, y1), clearance, name in bodies:
        if y0 - clearance < room.y_min or y1 + clearance > room.y_max:
            raise BackgroundError(
                f'Scene "{scene.id}": the {name} leaves the room in y at any placement.')
        low = max(low, room.x_min + clearance - x0)
        high = min(high, room.x_max - clearance - x1)
    if low > high:
        raise BackgroundError(f'Scene "{scene.id}" does not fit in the room at any dx.')
    return low, high


def dx_candidates(low: float, high: float, step: float) -> list[float]:
    """Every multiple of ``step`` in [low, high], as the decimal it is written as."""

    first = math.ceil(low / step - 1e-9)
    last = math.floor(high / step + 1e-9)
    return [round(k * step, 6) for k in range(first, last + 1)]


def anchor_target(scene: Any) -> int:
    """The target with the largest camera range; the lowest index on ties."""

    if not scene.robots:
        raise BackgroundError(f'Scene "{scene.id}" has no target to place it by.')
    camera = camera_xy(0.0)
    ranges = [planar_range(camera, (robot.x, robot.y)) for robot in scene.robots]
    return max(range(len(ranges)), key=lambda index: (ranges[index], -index))


def backdrop(
    frame: SceneFrame,
    target_index: int,
    kept: Mapping[str, Outline],
    room: Room,
) -> tuple[str, float]:
    """What the anchor is seen against, and how far behind it that is.

    The backdrop is the kept prop with the nearest outline point inside the
    anchor's silhouette cone and farther from the camera than the anchor (the
    earliest in layout order on ties); without one, the room wall.
    """

    target = frame.targets[target_index]
    target_range = planar_range(frame.camera, target)
    bearing = planar_bearing(frame.camera, target)
    half_width = math.atan(TARGET_RADIUS_M / target_range)
    best_id, best_range = None, math.inf
    for prop_id, outline in kept.items():
        centre, radius = bounding_circle(outline)
        distance = planar_range(frame.camera, centre)
        if distance - radius >= best_range or distance + radius <= target_range:
            continue
        nearest = nearest_in_cone(outline, frame.camera, bearing, half_width, beyond=target_range)
        if nearest is not None and nearest < best_range:
            best_id, best_range = prop_id, nearest
    if best_id is None:
        wall = nearest_in_cone(
            room.outline(), frame.camera, bearing, half_width, beyond=target_range)
        return WALL, wall - target_range
    return best_id, best_range - target_range


@dataclass(frozen=True)
class PlacedScene:
    """The chosen placement of one scene, with the reasons behind each drop."""

    placement: ScenePlacement
    reasons: Mapping[str, tuple[str, ...]]
    frame: SceneFrame


def place_scene(
    layout: BackgroundLayout,
    prop_footprints: Mapping[str, Footprint],
    robot_footprint: Sequence[Point],
    scene: Any,
    object_footprints: Mapping[str, Footprint],
    *,
    backdrop_gap_m: float | None = None,
) -> PlacedScene:
    """Choose a scene's dx: the backdrop gap nearest the layout's aim.

    Among the candidates whose anchor has a prop behind it, the one whose gap
    is nearest ``backdrop_gap_m``, then the fewest drops, then the smallest
    |dx|, then the smaller dx. Only when no candidate has a prop backdrop do
    the wall backdrops compete, by the same criterion.
    """

    aim = layout.placement.backdrop_gap_m if backdrop_gap_m is None else backdrop_gap_m
    outlines = prop_outlines(layout, prop_footprints)
    low, high = feasible_dx(layout.room, scene, robot_footprint, object_footprints)
    candidates = dx_candidates(low, high, layout.placement.dx_step_m)
    if not candidates:
        raise BackgroundError(
            f'Scene "{scene.id}" fits in the room only between dx {low:.4f} and '
            f'{high:.4f}, which holds no multiple of {layout.placement.dx_step_m}.')
    target_index = anchor_target(scene)
    evaluated = []
    for dx_m in candidates:
        frame = scene_frame(scene, dx_m, robot_footprint, object_footprints)
        dropped = frame_drops(outlines, frame)
        kept = {prop_id: outline for prop_id, outline in outlines.items()
                if prop_id not in dropped}
        backdrop_id, gap = backdrop(frame, target_index, kept, layout.room)
        miss = round(abs(gap - aim) / GAP_RESOLUTION_M)
        key = (backdrop_id == WALL, miss, len(dropped), abs(dx_m), dx_m)
        evaluated.append((key, dx_m, frame, dropped, backdrop_id, gap))
    _key, dx_m, frame, dropped, backdrop_id, gap = min(evaluated, key=lambda item: item[0])
    return PlacedScene(
        placement=ScenePlacement(
            dx_m=dx_m,
            anchor=Anchor(target_index=target_index, backdrop=backdrop_id, gap_m=gap),
            drops=tuple(dropped),
        ),
        reasons=dropped,
        frame=frame,
    )


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
