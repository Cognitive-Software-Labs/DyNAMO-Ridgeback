"""Geometric audit of benchmark scenes against the robot's camera and LiDAR.

Every scene in a scenario YAML is rebuilt as triangles -- the world's static
models, one G1 mesh per target and every object occluder, all read from the
simulator asset SDFs -- and seen through the sensor geometry resolved from the
robot description: a pinhole z-buffer for the camera and a planar ray cast for
the front 2D LiDAR. Pure numpy; no ROS runtime, no simulator.

Coordinates
-----------
World frame as in the scenario YAML: x forward, y left, z up. The sensor robot's
``base_link`` sits at the world origin with zero yaw, ``base_link_world_z``
above z = 0. Targets and objects stand at ``(x, y, spawn_z)`` rotated by their
yaw about +z. Camera points are expressed in the optical frame (+Z forward,
+X image-right, +Y image-down).

Pixel convention
----------------
Pixel column ``i`` / row ``j`` samples the single ray through the continuous
image coordinate ``(u, v) = (i, j)``, where ``u = fx * X / Z + cx`` and
``v = fy * Y / Z + cy``. That is the convention of
``ridgeback_localization.core.intrinsics`` (``deproject_pixel`` takes a pixel
index as ``u``; ``project_points`` rounds to the nearest index). A pixel is
covered by a triangle when its sample point lies inside the projected triangle,
edges inclusive; the nearest covering triangle (smallest optical Z, interpolated
perspective-correctly) owns the pixel. ``pixel_center_offset`` shifts every
sample by a constant (0.5 models a renderer whose principal point is the exact
image centre while ``cx = W / 2``); the effect is at most half a pixel on any
boundary.

Near plane
----------
Triangles are clipped against the plane ``Z = near_clip_m`` in camera space
before projection (one-in/two-out and two-in/one-out cases split into new
triangles), so geometry crossing the camera is drawn exactly up to that plane.

Metrics (per target, per scene, per camera profile)
---------------------------------------------------
``visible_fraction``
    Target pixels visible in the full scene / target pixels when the target is
    rendered with ALL other geometry removed (floor, walls, other targets and
    objects), on the same image bounds. ``null`` when the denominator is 0.
``visible_fraction_floor_kept``
    The same, with the floor kept in the denominator render, so pixels of a
    model sunk into the floor do not count as occluded. Equal to
    ``visible_fraction`` for models standing on the floor. Computed exactly from
    the floor's top plane.
``in_frame_fraction``
    Target pixels inside the image / target pixels on an extended image plane
    with the same fx, fy and principal point but a canvas large enough to hold
    the whole projected target, all other geometry removed. ``null`` when the
    extended count is 0.
``center_ratio``
    ``|u_c - cx| / cx``, where ``u_c`` is the projection of the target's torso
    centre: the point on the model's vertical yaw axis (model-frame x = y = 0)
    at the height of the centre of the ``torso_link`` visual's bounding box.
    Being on the yaw axis makes ``u_c`` a function of placement only, never of
    the target's yaw; with a level camera any height on that axis gives the
    same ``u_c``. Values above 1 put the centre outside the frame.
``bbox``
    Inclusive pixel bounds ``[u_min, v_min, u_max, v_max]`` of the visible
    target pixels (``bbox_alone``: unoccluded, inside the image;
    ``bbox_extended``: on the extended plane, image coordinates).
``range`` / ``bearing``
    From the camera optical origin to the target origin, horizontally, and to
    the torso centre in 3D; bearing is ``atan2(dy, dx)`` in the world/base
    frame, left positive. ``depth_z_m`` is the optical Z of the torso centre.

LiDAR metrics (front 2D LiDAR, laser plane)
-------------------------------------------
Triangles are sliced by the horizontal laser plane into 2D segments and every
beam (``angle_min + k * increment``) is intersected with them; the nearest hit
within ``[range_min, range_max]`` owns the beam.

``legs[side].beams`` / ``beams_alone``
    Beams whose nearest hit is that leg in the full scene / with every other
    triangle removed. ``width_deg`` / ``width_alone_deg`` are the same counts
    measured with ``FINE_RAY_STEP_DEG`` rays, i.e. the visible angular width.
    ``width_ratio`` is their quotient; the scene criterion is >= 0.60.
``objects[j]``
    Beams on the object, its horizontal range from the laser, the image column
    span of the parts that cross the laser plane, and per target the inset of
    that span inside the target's detection box, taken as ``bbox_alone`` (the
    unoccluded target inside the image, so a thin pole hiding the outermost
    columns cannot shrink the box it is measured against): the
    ``min(left, right)`` margin in px, positive = inside. For cylindrical
    poles additionally the clearance to each target mesh (nearest mesh vertex
    within the shaft's height, minus the radius), to every other pole and to
    the sensor-robot footprint, and which nearer poles shadow it from the
    laser (angular overlap).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np


SCHEMA = 'ridgeback_scene_audit/1'
DEFAULT_CYLINDER_SEGMENTS = 64
FINE_RAY_STEP_DEG = 0.01
LEG_WIDTH_CRITERION = 0.60
EXTENDED_CANVAS_LIMIT_PX = 8192
# Fragments evaluated per rasterizer batch; bounds peak memory (~0.5 GB).
RASTER_BATCH_CANDIDATES = 1 << 21

KIND_WORLD = 'world'
KIND_TARGET = 'target'
KIND_OBJECT = 'object'
_PART_BITS = 8
_LEG_PART = re.compile(r'^(left|right)_(hip_[a-z]+|knee|ankle_[a-z]+)_link$')


# --------------------------------------------------------------------------- #
# Triangles
# --------------------------------------------------------------------------- #

def read_stl(path: str | Path) -> np.ndarray:
    """Triangles ``(N, 3, 3)`` of a binary or ASCII STL file."""

    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = struct.unpack('<I', data[80:84])[0]
        if len(data) == 84 + 50 * count:
            records = np.frombuffer(
                data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                count=count, offset=84)
            return records['v'].astype(np.float64)
    text = data.decode('ascii', errors='replace')
    values = [
        [float(x) for x in line.split()[1:4]]
        for line in text.splitlines() if line.strip().startswith('vertex')
    ]
    if not values or len(values) % 3:
        raise ValueError(f'{path}: not a binary or ASCII STL with whole triangles')
    return np.asarray(values, dtype=np.float64).reshape(-1, 3, 3)


def box_triangles(size: Sequence[float]) -> np.ndarray:
    """12 triangles of an axis-aligned box centred on the origin."""

    hx, hy, hz = (0.5 * float(s) for s in size)
    corners = np.array([
        [sx * hx, sy * hy, sz * hz]
        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
    ])
    faces = [
        (0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),  # -x, +x
        (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),  # -y, +y
        (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3),  # -z, +z
    ]
    return corners[np.array(faces)]


def cylinder_triangles(radius: float, length: float, segments: int) -> np.ndarray:
    """Closed z-axis cylinder centred on the origin, ``4 * segments`` triangles.

    The side is the inscribed ``segments``-gon prism; its silhouette is short of
    the true radius by at most ``radius * (1 - cos(pi / segments))``.
    """

    angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ring = np.stack((radius * np.cos(angles), radius * np.sin(angles)), axis=1)
    nxt = np.roll(ring, -1, axis=0)
    hz = 0.5 * float(length)
    lo = np.column_stack((ring, np.full(segments, -hz)))
    hi = np.column_stack((ring, np.full(segments, hz)))
    lo2 = np.column_stack((nxt, np.full(segments, -hz)))
    hi2 = np.column_stack((nxt, np.full(segments, hz)))
    top_c = np.tile([0.0, 0.0, hz], (segments, 1))
    bot_c = np.tile([0.0, 0.0, -hz], (segments, 1))
    return np.concatenate([
        np.stack((lo, lo2, hi2), axis=1),
        np.stack((lo, hi2, hi), axis=1),
        np.stack((top_c, hi, hi2), axis=1),
        np.stack((bot_c, lo2, lo), axis=1),
    ])


def transform_triangles(triangles: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Apply a 4x4 rigid transform to ``(N, 3, 3)`` triangles."""

    return triangles @ matrix[:3, :3].T + matrix[:3, 3]


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF/URDF fixed-axis roll-pitch-yaw: ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_matrix(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(roll, pitch, yaw)
    matrix[:3, 3] = (x, y, z)
    return matrix


def planar_pose(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    return pose_matrix(x, y, z, 0.0, 0.0, yaw)


# --------------------------------------------------------------------------- #
# SDF models
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Part:
    """One visual of a model, triangulated in the model frame."""

    name: str
    kind: str  # 'mesh' | 'box' | 'cylinder'
    triangles: np.ndarray
    params: Mapping[str, float] = field(default_factory=dict)
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))

    @property
    def z_range(self) -> tuple[float, float]:
        z = self.triangles[:, :, 2]
        return float(z.min()), float(z.max())


@dataclass(frozen=True)
class ModelGeometry:
    name: str
    source: str
    parts: tuple[Part, ...]
    notes: tuple[str, ...] = ()

    @property
    def triangle_count(self) -> int:
        return int(sum(len(part.triangles) for part in self.parts))

    def part(self, name: str) -> Part:
        return next(part for part in self.parts if part.name == name)


def _sdf_pose(element: ET.Element | None) -> np.ndarray:
    if element is None:
        return np.eye(4)
    node = element.find('pose')
    if node is None or not (node.text or '').strip():
        return np.eye(4)
    values = [float(v) for v in node.text.split()]
    values += [0.0] * (6 - len(values))
    return pose_matrix(*values[:6])


def _geometry_triangles(
    geometry: ET.Element,
    base_dir: Path,
    cylinder_segments: int,
) -> tuple[str, np.ndarray, dict]:
    box = geometry.find('box')
    if box is not None:
        size = [float(v) for v in box.find('size').text.split()]
        return 'box', box_triangles(size), {'sx': size[0], 'sy': size[1], 'sz': size[2]}
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.find('radius').text)
        length = float(cylinder.find('length').text)
        return ('cylinder', cylinder_triangles(radius, length, cylinder_segments),
                {'radius': radius, 'length': length, 'segments': cylinder_segments})
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = mesh.find('uri').text.strip()
        if uri.startswith('model://'):
            raise ValueError(f'{base_dir}: model:// mesh URIs are not resolved: {uri}')
        if uri.startswith('file://'):
            uri = uri[len('file://'):]
        path = Path(uri) if Path(uri).is_absolute() else base_dir / uri
        triangles = read_stl(path)
        scale = mesh.find('scale')
        if scale is not None:
            triangles = triangles * np.array([float(v) for v in scale.text.split()])
        return 'mesh', triangles, {}
    raise ValueError(f'unsupported SDF geometry in {base_dir}: {ET.tostring(geometry)[:80]!r}')


def _model_parts(
    model: ET.Element,
    base_dir: Path,
    cylinder_segments: int,
    element: str,
    model_pose: np.ndarray,
) -> list[Part]:
    parts = []
    for link in model.findall('link'):
        link_pose = model_pose @ _sdf_pose(link)
        for item in link.findall(element):
            pose = link_pose @ _sdf_pose(item)
            kind, local, params = _geometry_triangles(
                item.find('geometry'), base_dir, cylinder_segments)
            name = item.get('name') or element
            for suffix in ('_visual', '_collision'):
                if name.endswith(suffix):
                    name = name[: -len(suffix)]
            parts.append(Part(name, kind, transform_triangles(local, pose), params, pose))
    return parts


def load_sdf_model(
    path: str | Path,
    *,
    cylinder_segments: int = DEFAULT_CYLINDER_SEGMENTS,
    element: str = 'visual',
) -> ModelGeometry:
    """Triangulate one SDF model's ``visual`` (or ``collision``) geometry.

    Poses compose model, link and visual ``<pose>``; mesh URIs resolve against
    the SDF's directory. The note records how collision geometry differs.
    """

    path = Path(path)
    root = ET.parse(path).getroot()
    model = root.find('model')
    parts = _model_parts(model, path.parent, cylinder_segments, element, _sdf_pose(model))

    def signature(kind: str) -> dict[str, tuple]:
        return {
            (item.get('name') or '').replace('visual', '').replace('collision', ''): (
                ' '.join(ET.tostring(item.find('geometry'), encoding='unicode').split()),
                (item.findtext('pose') or '').split())
            for link in model.findall('link') for item in link.findall(kind)
        }

    visuals, collisions = signature('visual'), signature('collision')
    notes = []
    missing = sorted(set(visuals) - set(collisions))
    if missing:
        notes.append(f'{len(missing)} of {len(visuals)} visuals have no collision '
                     f'counterpart: {", ".join(missing)}')
    differing = sorted(k for k in set(visuals) & set(collisions) if visuals[k] != collisions[k])
    if differing:
        notes.append(f'visual and collision geometry differ for: {", ".join(differing)}')
    if not notes:
        notes.append('visual and collision geometry identical')
    return ModelGeometry(model.get('name'), str(path), tuple(parts), tuple(notes))


def load_world_models(
    path: str | Path,
    *,
    cylinder_segments: int = DEFAULT_CYLINDER_SEGMENTS,
) -> list[ModelGeometry]:
    """Every inline ``<model>`` of a world SDF, triangulated in the world frame."""

    path = Path(path)
    world = ET.parse(path).getroot().find('world')
    models = []
    for model in world.findall('model'):
        parts = _model_parts(model, path.parent, cylinder_segments, 'visual', _sdf_pose(model))
        if parts:
            models.append(ModelGeometry(model.get('name'), str(path), tuple(parts)))
    return models


# --------------------------------------------------------------------------- #
# Sensors
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera on a ``width x height`` pixel grid."""

    name: str
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in ('name', 'fx', 'fy', 'cx', 'cy', 'width', 'height')}


def profile_intrinsics(profile_name: str) -> Intrinsics:
    from ridgeback_common.camera_profiles import resolve_camera_profile

    profile = resolve_camera_profile(profile_name)
    return Intrinsics(profile.name, profile.focal_length_px, profile.focal_length_px,
                      profile.cx, profile.cy, profile.width, profile.height)


@dataclass(frozen=True)
class LaserSpec:
    """Planar scan geometry in the laser frame."""

    angle_min: float
    angle_increment: float
    bins: int
    range_min: float
    range_max: float

    def angles(self) -> np.ndarray:
        return self.angle_min + self.angle_increment * np.arange(self.bins)

    def as_dict(self) -> dict:
        return {
            'angle_min_rad': self.angle_min,
            'angle_increment_rad': self.angle_increment,
            'bins': self.bins,
            'range_min_m': self.range_min,
            'range_max_m': self.range_max,
        }


def contract_laser_spec() -> LaserSpec:
    """The shared UST-10LX raw-scan contract (``ridgeback_common.lidar_contract``)."""

    from ridgeback_common import lidar_contract as c

    return LaserSpec(c.RAW_ANGLE_MIN, c.RAW_ANGLE_INCREMENT, c.RAW_SCAN_BINS,
                     c.RAW_RANGE_MIN, c.RAW_RANGE_MAX)


@dataclass(frozen=True)
class SensorGeometry:
    """Everything the audit needs to know about the sensor robot."""

    name: str
    base_link_world_z: float
    camera_in_base: np.ndarray  # 4x4 optical frame in base_link
    laser_in_base: np.ndarray  # 4x4 laser frame in base_link
    intrinsics: tuple[Intrinsics, ...]
    laser: LaserSpec
    near_clip_m: float
    pixel_center_offset: float = 0.0
    footprint_xy: tuple[float, float, float, float] | None = None  # xmin, xmax, ymin, ymax
    provenance: Mapping[str, object] = field(default_factory=dict)

    @property
    def base_in_world(self) -> np.ndarray:
        return planar_pose(0.0, 0.0, self.base_link_world_z, 0.0)

    @property
    def camera_in_world(self) -> np.ndarray:
        return self.base_in_world @ self.camera_in_base

    @property
    def laser_in_world(self) -> np.ndarray:
        return self.base_in_world @ self.laser_in_base

    def as_dict(self) -> dict:
        from ridgeback_autonomy.robot_description import matrix_rpy

        def frame(matrix):
            return {
                'xyz_in_base_link': [round(float(v), 9) for v in matrix[:3, 3]],
                'rpy_in_base_link': [round(float(v), 12) for v in matrix_rpy(matrix)],
                'matrix_in_base_link': np.round(matrix, 12).tolist(),
            }

        return {
            'name': self.name,
            'base_link_world_z_m': self.base_link_world_z,
            'camera_optical': frame(self.camera_in_base),
            'laser': {**frame(self.laser_in_base),
                      'plane_world_z_m': float(self.laser_in_world[2, 3]),
                      **self.laser.as_dict()},
            'intrinsics': [intr.as_dict() for intr in self.intrinsics],
            'near_clip_m': self.near_clip_m,
            'pixel_center_offset_px': self.pixel_center_offset,
            'sensor_robot_footprint_xy_m': self.footprint_xy,
            'provenance': dict(self.provenance),
        }


def level_optical_frame(x: float, y: float, z: float) -> np.ndarray:
    """Optical frame at ``(x, y, z)`` in base_link looking along +x, image-right = -y."""

    matrix = np.eye(4)
    matrix[:3, :3] = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    matrix[:3, 3] = (x, y, z)
    return matrix


# Gazebo spawns the robot above the floor and lets it settle on its wheel
# collision cylinders: base_link ends up at floor top (0.05) + wheel radius
# (0.0759) - axle height (0.05). Isaac seats it at floor + 0.02617.
GAZEBO_BASE_LINK_WORLD_Z = 0.05 + 0.0759 - 0.05
ISAAC_BASE_LINK_WORLD_Z = 0.05 + 0.02617

LEGACY_PRESETS = {
    # 2026-07-26 pixel certifications: one-shot script, box-figure target.
    'pixel': {
        'camera_xyz': (0.25, 0.0, 0.85),
        'laser_xyz': (0.392, 0.0, 0.342),
        'intrinsics': Intrinsics('legacy-640x480', 443.53, 443.53, 319.5, 239.5, 640, 480),
    },
    # 2026-08-23 in-box clutter banner: then-live URDF camera, same fx.
    'interfere': {
        'camera_xyz': (0.011, 0.018, 1.028),
        'laser_xyz': (0.392, 0.0, 0.342),
        'intrinsics': Intrinsics('legacy-640x480', 443.53, 443.53, 319.5, 239.5, 640, 480),
    },
}


def legacy_sensor_geometry(
    preset: str = 'pixel',
    *,
    base_link_world_z: float = GAZEBO_BASE_LINK_WORLD_Z,
    near_clip_m: float = 0.05,
) -> SensorGeometry:
    spec = LEGACY_PRESETS[preset]
    laser = np.eye(4)
    laser[:3, 3] = spec['laser_xyz']
    return SensorGeometry(
        name=f'legacy-{preset}',
        base_link_world_z=base_link_world_z,
        camera_in_base=level_optical_frame(*spec['camera_xyz']),
        laser_in_base=laser,
        intrinsics=(spec['intrinsics'],),
        laser=contract_laser_spec(),
        near_clip_m=near_clip_m,
        provenance={'source': f'legacy preset {preset!r} (historical YAML comments)'},
    )


# --------------------------------------------------------------------------- #
# Rasterizer
# --------------------------------------------------------------------------- #

def clip_near(triangles: np.ndarray, codes: np.ndarray, near: float):
    """Clip camera-space triangles to ``Z >= near``; returns new arrays."""

    inside = triangles[:, :, 2] >= near
    count = inside.sum(axis=1)
    out_tris = [triangles[count == 3]]
    out_codes = [codes[count == 3]]

    def cut(a, b):
        s = (near - a[:, 2]) / (b[:, 2] - a[:, 2])
        return a + s[:, None] * (b - a)

    rows = np.nonzero(count == 1)[0]
    if len(rows):
        first = np.argmax(inside[rows], axis=1)
        t = triangles[rows]
        r = np.arange(len(rows))
        a, b, c = t[r, first], t[r, (first + 1) % 3], t[r, (first + 2) % 3]
        out_tris.append(np.stack((a, cut(a, b), cut(a, c)), axis=1))
        out_codes.append(codes[rows])
    rows = np.nonzero(count == 2)[0]
    if len(rows):
        outside = np.argmin(inside[rows], axis=1)
        t = triangles[rows]
        r = np.arange(len(rows))
        c, a, b = t[r, outside], t[r, (outside + 1) % 3], t[r, (outside + 2) % 3]
        p, q = cut(a, c), cut(b, c)
        out_tris.append(np.stack((a, b, q), axis=1))
        out_tris.append(np.stack((a, q, p), axis=1))
        out_codes += [codes[rows], codes[rows]]
    return np.concatenate(out_tris), np.concatenate(out_codes)


@dataclass(frozen=True)
class Canvas:
    """A pixel grid: intrinsics plus where its pixel (0, 0) sits in image coordinates."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    u0: int = 0
    v0: int = 0

    @classmethod
    def image(cls, intr: Intrinsics) -> 'Canvas':
        return cls(intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height)

    def image_window(self, intr: Intrinsics) -> tuple[slice, slice]:
        """Rows/columns of this canvas that are the image."""

        return (slice(-self.v0, -self.v0 + intr.height), slice(-self.u0, -self.u0 + intr.width))


def project(points_cam: np.ndarray, canvas: Canvas, offset: float = 0.0):
    """Pixel-index coordinates on ``canvas`` (sample points are integers)."""

    z = points_cam[..., 2]
    u = canvas.fx * points_cam[..., 0] / z + canvas.cx - canvas.u0 - offset
    v = canvas.fy * points_cam[..., 1] / z + canvas.cy - canvas.v0 - offset
    return u, v


def rasterize(
    triangles_cam: np.ndarray,
    codes: np.ndarray,
    canvas: Canvas,
    *,
    near: float,
    offset: float = 0.0,
    batch_candidates: int = RASTER_BATCH_CANDIDATES,
) -> tuple[np.ndarray, np.ndarray]:
    """Z-buffer camera-space triangles; returns ``(depth, code)`` images.

    ``depth`` is optical Z (``inf`` where empty); ``code`` the owning
    triangle's code (``-1`` where empty).
    """

    width, height = canvas.width, canvas.height
    depth = np.full(width * height, np.inf)
    owner = np.full(width * height, -1, dtype=np.int64)
    if len(triangles_cam) == 0:
        return depth.reshape(height, width), owner.reshape(height, width)

    # Cheap reject before clipping: entirely behind the near plane.
    keep = triangles_cam[:, :, 2].max(axis=1) >= near
    tris, tcodes = clip_near(triangles_cam[keep], np.asarray(codes)[keep], near)
    if len(tris) == 0:
        return depth.reshape(height, width), owner.reshape(height, width)
    u, v = project(tris, canvas, offset)
    zs = tris[:, :, 2]
    umin = np.maximum(np.ceil(u.min(axis=1)), 0)
    umax = np.minimum(np.floor(u.max(axis=1)), width - 1)
    vmin = np.maximum(np.ceil(v.min(axis=1)), 0)
    vmax = np.minimum(np.floor(v.max(axis=1)), height - 1)
    area = (u[:, 1] - u[:, 0]) * (v[:, 2] - v[:, 0]) - (u[:, 2] - u[:, 0]) * (v[:, 1] - v[:, 0])
    live = (umin <= umax) & (vmin <= vmax) & (np.abs(area) > 1e-12)
    if not live.any():
        return depth.reshape(height, width), owner.reshape(height, width)
    u, v, zs, area, tcodes = u[live], v[live], zs[live], area[live], tcodes[live]
    umin, umax = umin[live].astype(np.int64), umax[live].astype(np.int64)
    vmin, vmax = vmin[live].astype(np.int64), vmax[live].astype(np.int64)

    bw = umax - umin + 1
    bh = vmax - vmin + 1
    kw = np.ceil(np.log2(bw)).astype(np.int64)
    kh = np.ceil(np.log2(bh)).astype(np.int64)
    key = kw * 64 + kh
    order = np.argsort(key, kind='stable')
    key_sorted = key[order]
    bounds = np.flatnonzero(np.diff(key_sorted)) + 1
    groups = np.split(order, bounds)

    eps = 1e-9
    for group in groups:
        gw = 1 << int(kw[group[0]])
        gh = 1 << int(kh[group[0]])
        per = gw * gh
        step = max(1, batch_candidates // per)
        du = np.arange(gw)[None, None, :]
        dv = np.arange(gh)[None, :, None]
        for start in range(0, len(group), step):
            idx = group[start:start + step]
            px = umin[idx][:, None, None] + du
            py = vmin[idx][:, None, None] + dv
            valid = (px <= umax[idx][:, None, None]) & (py <= vmax[idx][:, None, None])
            u0, u1, u2 = (u[idx, k][:, None, None] for k in range(3))
            v0, v1, v2 = (v[idx, k][:, None, None] for k in range(3))
            a = area[idx][:, None, None]
            w0 = ((u1 - px) * (v2 - py) - (u2 - px) * (v1 - py)) / a
            w1 = ((u2 - px) * (v0 - py) - (u0 - px) * (v2 - py)) / a
            w2 = 1.0 - w0 - w1
            inside = valid & (w0 >= -eps) & (w1 >= -eps) & (w2 >= -eps)
            if not inside.any():
                continue
            tri_idx = np.broadcast_to(np.arange(len(idx))[:, None, None], inside.shape)[inside]
            iz = (w0 / zs[idx, 0][:, None, None] + w1 / zs[idx, 1][:, None, None]
                  + w2 / zs[idx, 2][:, None, None])[inside]
            z = 1.0 / iz
            pix = (py * width + px)[inside]
            frag_codes = tcodes[idx][tri_idx]
            # Nearest fragment per pixel within the batch, then against the buffer.
            sort = np.lexsort((z, pix))
            pix, z, frag_codes = pix[sort], z[sort], frag_codes[sort]
            first = np.ones(len(pix), dtype=bool)
            first[1:] = pix[1:] != pix[:-1]
            pix, z, frag_codes = pix[first], z[first], frag_codes[first]
            nearer = z < depth[pix]
            depth[pix[nearer]] = z[nearer]
            owner[pix[nearer]] = frag_codes[nearer]
    return depth.reshape(height, width), owner.reshape(height, width)


# --------------------------------------------------------------------------- #
# Planar LiDAR
# --------------------------------------------------------------------------- #

def _axis_clearance(triangles: np.ndarray, centre, z_span, step: float = 0.01) -> float | None:
    """Horizontal distance from a vertical axis to a mesh within ``z_span``.

    The nearest surface comes from horizontal slices every ``step`` metres; the
    nearest vertex covers features thinner than the step. An axis deeper inside
    a closed mesh than the caller's radius still reads as a positive distance.
    ``None`` when the mesh has no geometry in the span.
    """

    z = triangles[:, :, 2]
    lo, hi = max(z_span[0], z.min()), min(z_span[1], z.max())
    if lo > hi:
        return None
    best = math.inf
    point = np.asarray(centre, dtype=np.float64)
    for z_plane in np.arange(lo, hi + 0.5 * step, step):
        segments, _ = slice_triangles(triangles, np.zeros(len(triangles), dtype=int), z_plane)
        if len(segments) == 0:
            continue
        a, b = segments[:, 0], segments[:, 1]
        ab = b - a
        s = np.clip(((point - a) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-18), 0, 1)
        best = min(best, float(np.hypot(*(a + s[:, None] * ab - point).T).min()))
    pts = triangles.reshape(-1, 3)
    pts = pts[(pts[:, 2] >= z_span[0]) & (pts[:, 2] <= z_span[1])]
    if len(pts):
        best = min(best, float(np.hypot(pts[:, 0] - point[0], pts[:, 1] - point[1]).min()))
    return None if math.isinf(best) else best


def slice_triangles(triangles: np.ndarray, codes: np.ndarray, z_plane: float):
    """Intersect triangles with ``z = z_plane``: ``(M, 2, 2)`` xy segments + codes."""

    z = triangles[:, :, 2]
    candidate = (z.min(axis=1) <= z_plane) & (z.max(axis=1) >= z_plane)
    tris = triangles[candidate]
    tcodes = np.asarray(codes)[candidate]
    d = tris[:, :, 2] - z_plane
    above = d >= 0.0
    crossing = above.any(axis=1) & ~above.all(axis=1)
    tris, tcodes, d, above = tris[crossing], tcodes[crossing], d[crossing], above[crossing]
    points = []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        cut = above[:, i] != above[:, j]
        s = np.where(cut, d[:, i] / np.where(cut, d[:, i] - d[:, j], 1.0), 0.0)
        point = tris[:, i, :2] + s[:, None] * (tris[:, j, :2] - tris[:, i, :2])
        points.append((cut, point))
    cuts = np.stack([c for c, _ in points], axis=1)
    pts = np.stack([p for _, p in points], axis=1)
    # Exactly two edges change sign on a crossing triangle.
    first = np.argmax(cuts, axis=1)
    second = 2 - np.argmax(cuts[:, ::-1], axis=1)
    r = np.arange(len(tris))
    segments = np.stack((pts[r, first], pts[r, second]), axis=1)
    return segments, tcodes


def cast_rays(
    origin: Sequence[float],
    angles: np.ndarray,
    segments: np.ndarray,
    codes: np.ndarray,
    *,
    range_min: float,
    range_max: float,
    chunk: int = 256,
) -> tuple[np.ndarray, np.ndarray]:
    """Nearest hit per world-frame ray angle: ``(range, code)``; ``inf`` / ``-1`` on a miss.

    A hit closer than ``range_min`` or beyond ``range_max`` is no return.
    """

    angles = np.asarray(angles, dtype=np.float64)
    ranges = np.full(len(angles), np.inf)
    owner = np.full(len(angles), -1, dtype=np.int64)
    if len(segments) == 0 or len(angles) == 0:
        return ranges, owner
    p = segments[:, 0, :] - np.asarray(origin, dtype=np.float64)[:2]
    e = segments[:, 1, :] - segments[:, 0, :]
    codes = np.asarray(codes)
    for start in range(0, len(angles), chunk):
        a = angles[start:start + chunk]
        dx, dy = np.cos(a)[:, None], np.sin(a)[:, None]
        denom = dx * e[None, :, 1] - dy * e[None, :, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (p[None, :, 0] * e[None, :, 1] - p[None, :, 1] * e[None, :, 0]) / denom
            s = (p[None, :, 0] * dy - p[None, :, 1] * dx) / denom
        ok = (np.abs(denom) > 1e-15) & (t > 0.0) & (s >= 0.0) & (s <= 1.0)
        t = np.where(ok, t, np.inf)
        best = np.argmin(t, axis=1)
        tb = t[np.arange(len(a)), best]
        hit = np.isfinite(tb) & (tb >= range_min) & (tb <= range_max)
        ranges[start:start + chunk] = np.where(hit, tb, np.inf)
        owner[start:start + chunk] = np.where(hit, codes[best], -1)
    return ranges, owner


# --------------------------------------------------------------------------- #
# Scenes
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Entity:
    kind: str
    index: int  # within its kind
    name: str  # model name
    parts: tuple[str, ...]
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def label(self) -> str:
        return f'{self.kind}:{self.index}' if self.kind != KIND_WORLD else f'world:{self.name}'


@dataclass
class SceneGeometry:
    entities: list[Entity]
    triangles: np.ndarray  # (N, 3, 3) world
    codes: np.ndarray  # (N,) entity << _PART_BITS | part
    entity_rows: list[np.ndarray]  # triangle row slice per entity

    def ids(self, kind: str) -> list[int]:
        return [i for i, e in enumerate(self.entities) if e.kind == kind]


class SceneBuilder:
    """Turns scenario scenes into triangle soups using loaded asset geometry."""

    def __init__(
        self,
        *,
        target: ModelGeometry,
        objects: Mapping[str, ModelGeometry],
        world: Sequence[ModelGeometry],
        target_spawn_z: float = 0.0,
        object_spawn_z: Mapping[str, float] | None = None,
    ):
        self.target = target
        self.objects = dict(objects)
        self.world = list(world)
        self.target_spawn_z = target_spawn_z
        self.object_spawn_z = dict(object_spawn_z or {})
        self._target_tris, self._target_parts = self._stack(target)
        self._object_stacks = {name: self._stack(model) for name, model in self.objects.items()}

    @staticmethod
    def _stack(model: ModelGeometry):
        tris = np.concatenate([part.triangles for part in model.parts])
        parts = np.concatenate([
            np.full(len(part.triangles), i, dtype=np.int64) for i, part in enumerate(model.parts)])
        if len(model.parts) >= (1 << _PART_BITS):
            raise ValueError(f'{model.name} has too many parts')
        return tris, parts

    def build(self, scene) -> SceneGeometry:
        entities: list[Entity] = []
        chunks, code_chunks, rows = [], [], []
        cursor = 0

        def add(entity: Entity, tris: np.ndarray, parts: np.ndarray):
            nonlocal cursor
            eid = len(entities)
            entities.append(entity)
            chunks.append(tris)
            code_chunks.append((eid << _PART_BITS) | parts)
            rows.append(slice(cursor, cursor + len(tris)))
            cursor += len(tris)

        for model in self.world:
            tris, parts = self._stack(model)
            add(Entity(KIND_WORLD, len(entities), model.name,
                       tuple(p.name for p in model.parts)), tris, parts)
        for i, robot in enumerate(scene.robots):
            pose = planar_pose(robot.x, robot.y, self.target_spawn_z, robot.yaw)
            add(Entity(KIND_TARGET, i, self.target.name,
                       tuple(p.name for p in self.target.parts), (robot.x, robot.y, robot.yaw)),
                transform_triangles(self._target_tris, pose), self._target_parts)
        for j, obj in enumerate(scene.objects):
            model = self.objects[obj.model]
            tris, parts = self._object_stacks[obj.model]
            pose = planar_pose(obj.x, obj.y, self.object_spawn_z.get(obj.model, 0.0), obj.yaw)
            add(Entity(KIND_OBJECT, j, obj.model, tuple(p.name for p in model.parts),
                       (obj.x, obj.y, obj.yaw)),
                transform_triangles(tris, pose), parts)
        return SceneGeometry(entities, np.concatenate(chunks), np.concatenate(code_chunks), rows)


# --------------------------------------------------------------------------- #
# Audit
# --------------------------------------------------------------------------- #

def _bbox(mask: np.ndarray, u0: int = 0, v0: int = 0):
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    if len(rows) == 0:
        return None
    return [int(cols[0] + u0), int(rows[0] + v0), int(cols[-1] + u0), int(rows[-1] + v0)]


def _fraction(numerator: int, denominator: int):
    return None if denominator == 0 else numerator / denominator


def _round(value, digits=4):
    if value is None:
        return None
    return round(float(value), digits)


def torso_centre_height(target: ModelGeometry, part_name: str = 'torso_link') -> float:
    z = target.part(part_name).triangles[:, :, 2]
    return 0.5 * float(z.min() + z.max())


def leg_side(part_name: str) -> str | None:
    match = _LEG_PART.match(part_name)
    return match.group(1) if match else None


@dataclass
class ProfileRender:
    """Buffers kept for previews."""

    intrinsics: Intrinsics
    code: np.ndarray
    depth: np.ndarray
    entities: list[Entity]
    targets: list[dict]


class SceneAuditor:
    """Computes the camera and LiDAR metrics for scenes."""

    def __init__(self, builder: SceneBuilder, sensors: SensorGeometry):
        self.builder = builder
        self.sensors = sensors
        self.torso_z = torso_centre_height(builder.target)
        self.floor = self._floor_plane()
        target = builder.target
        self.target_leg_side = [leg_side(p.name) for p in target.parts]

    # -- geometry helpers ------------------------------------------------- #

    def _floor_plane(self):
        floors = [m for m in self.builder.world if 'floor' in m.name.lower()]
        if not floors:
            return None
        tris = np.concatenate([p.triangles for p in floors[0].parts])
        flat = tris.reshape(-1, 3)
        return {
            'name': floors[0].name,
            'z_top': float(flat[:, 2].max()),
            'x': (float(flat[:, 0].min()), float(flat[:, 0].max())),
            'y': (float(flat[:, 1].min()), float(flat[:, 1].max())),
        }

    def _to_camera(self, points_world: np.ndarray) -> np.ndarray:
        inv = np.linalg.inv(self.sensors.camera_in_world)
        return points_world @ inv[:3, :3].T + inv[:3, 3]

    def _floor_depth(self, canvas: Canvas, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        """Optical Z where each pixel ray meets the floor's top face (inf if never)."""

        if self.floor is None:
            return np.full(len(rows), np.inf)
        off = self.sensors.pixel_center_offset
        x = (cols + canvas.u0 + off - canvas.cx) / canvas.fx
        y = (rows + canvas.v0 + off - canvas.cy) / canvas.fy
        cam = self.sensors.camera_in_world
        direction = np.stack((x, y, np.ones_like(x)), axis=1) @ cam[:3, :3].T
        origin = cam[:3, 3]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (self.floor['z_top'] - origin[2]) / direction[:, 2]
        hit = origin[:2] + t[:, None] * direction[:, :2]
        ok = ((t > 0) & (hit[:, 0] >= self.floor['x'][0]) & (hit[:, 0] <= self.floor['x'][1])
              & (hit[:, 1] >= self.floor['y'][0]) & (hit[:, 1] <= self.floor['y'][1]))
        return np.where(ok, t, np.inf)

    def _extended_canvas(self, intr: Intrinsics, tris_cam: np.ndarray):
        near = self.sensors.near_clip_m
        crosses = bool((tris_cam[:, :, 2] < near).any())
        if crosses:
            # Size from the clipped triangles: vertices created on the near
            # plane can project beyond every unclipped vertex.
            clipped, _ = clip_near(tris_cam, np.zeros(len(tris_cam), dtype=int), near)
            front = clipped.reshape(-1, 3)
        else:
            front = tris_cam.reshape(-1, 3)
        if len(front) == 0:
            return Canvas.image(intr), crosses, False
        base = Canvas.image(intr)
        u, v = project(front, base, self.sensors.pixel_center_offset)
        lo_u = int(math.floor(min(0.0, u.min()))) - 2
        hi_u = int(math.ceil(max(intr.width - 1.0, u.max()))) + 2
        lo_v = int(math.floor(min(0.0, v.min()))) - 2
        hi_v = int(math.ceil(max(intr.height - 1.0, v.max()))) + 2
        limit = EXTENDED_CANVAS_LIMIT_PX
        capped = (hi_u - lo_u + 1 > limit) or (hi_v - lo_v + 1 > limit)
        lo_u, lo_v = max(lo_u, -limit // 2), max(lo_v, -limit // 2)
        hi_u = min(hi_u, intr.width + limit // 2)
        hi_v = min(hi_v, intr.height + limit // 2)
        canvas = Canvas(intr.fx, intr.fy, intr.cx, intr.cy,
                        hi_u - lo_u + 1, hi_v - lo_v + 1, lo_u, lo_v)
        return canvas, crosses, capped

    # -- camera ----------------------------------------------------------- #

    def camera_metrics(self, geometry: SceneGeometry, intr: Intrinsics) -> ProfileRender:
        sensors = self.sensors
        near, off = sensors.near_clip_m, sensors.pixel_center_offset
        tris_cam = self._to_camera(geometry.triangles)
        image = Canvas.image(intr)
        depth, code = rasterize(tris_cam, geometry.codes, image, near=near, offset=off)
        owner_entity = np.where(code >= 0, code >> _PART_BITS, -1)
        cam_origin = sensors.camera_in_world[:3, 3]

        targets = []
        for eid in geometry.ids(KIND_TARGET):
            entity = geometry.entities[eid]
            rows = geometry.entity_rows[eid]
            t_cam = tris_cam[rows]
            canvas, crosses_near, capped = self._extended_canvas(intr, t_cam)
            ext_depth, ext_code = rasterize(
                t_cam, geometry.codes[rows], canvas, near=near, offset=off)
            ext_mask = ext_code >= 0
            window = canvas.image_window(intr)
            alone_depth = ext_depth[window]
            alone = ext_mask[window]
            visible = owner_entity == eid
            n_alone = int(alone.sum())
            n_visible = int(visible.sum())
            n_ext = int(ext_mask.sum())

            ar, ac = np.nonzero(alone)
            floor_z = self._floor_depth(image, ar, ac)
            floor_hidden = int((floor_z < alone_depth[ar, ac]).sum())

            hidden = alone & ~visible
            occluders: dict[str, int] = {}
            if hidden.any():
                ids, counts = np.unique(owner_entity[hidden], return_counts=True)
                for other, count in zip(ids, counts):
                    label = 'none' if other < 0 else geometry.entities[int(other)].label
                    occluders[label] = int(count)

            x, y, yaw = entity.pose
            torso_world = np.array([x, y, self.builder.target_spawn_z + self.torso_z])
            torso_cam = self._to_camera(torso_world[None, :])[0]
            if torso_cam[2] > 0:
                u_c = intr.fx * torso_cam[0] / torso_cam[2] + intr.cx
                v_c = intr.fy * torso_cam[1] / torso_cam[2] + intr.cy
                centre_ratio = abs(u_c - intr.cx) / intr.cx
            else:
                u_c = v_c = centre_ratio = None
            bbox_ext = _bbox(ext_mask, canvas.u0, canvas.v0)
            clipped = []
            if bbox_ext is not None:
                if bbox_ext[0] < 0:
                    clipped.append('left')
                if bbox_ext[2] > intr.width - 1:
                    clipped.append('right')
                if bbox_ext[1] < 0:
                    clipped.append('top')
                if bbox_ext[3] > intr.height - 1:
                    clipped.append('bottom')
            dx, dy = x - cam_origin[0], y - cam_origin[1]
            targets.append({
                'index': entity.index,
                'pose': {'x': x, 'y': y, 'yaw': yaw},
                'pixels_visible': n_visible,
                'pixels_alone_in_image': n_alone,
                'pixels_extended': n_ext,
                'pixels_floor_hidden': floor_hidden,
                'visible_fraction': _round(_fraction(n_visible, n_alone)),
                'visible_fraction_floor_kept': _round(
                    _fraction(n_visible, n_alone - floor_hidden)),
                'in_frame_fraction': _round(_fraction(n_alone, n_ext)),
                'center_ratio': _round(centre_ratio),
                'center_px': None if u_c is None else [_round(u_c, 2), _round(v_c, 2)],
                'bbox': _bbox(visible),
                'bbox_alone': _bbox(alone),
                'bbox_extended': bbox_ext,
                'clipped_edges': clipped,
                'occluded_by': occluders,
                'range_horizontal_m': _round(math.hypot(dx, dy)),
                'range_torso_m': _round(float(np.linalg.norm(torso_world - cam_origin))),
                'depth_z_m': _round(float(torso_cam[2])),
                'bearing_deg': _round(math.degrees(math.atan2(dy, dx)), 3),
                'crosses_near_plane': crosses_near,
                'extended_canvas_capped': capped,
            })
        return ProfileRender(intr, code, depth, geometry.entities, targets)

    def object_columns(self, geometry: SceneGeometry, intr: Intrinsics, z_plane: float):
        """Image column span of each object's parts that cross the laser plane."""

        spans = {}
        near = self.sensors.near_clip_m
        for eid in geometry.ids(KIND_OBJECT):
            entity = geometry.entities[eid]
            model = self.builder.objects[entity.name]
            x, y, yaw = entity.pose
            pose = planar_pose(x, y, self.builder.object_spawn_z.get(entity.name, 0.0), yaw)
            pts = []
            for part in model.parts:
                lo, hi = part.z_range
                lo += pose[2, 3]
                hi += pose[2, 3]
                if lo <= z_plane <= hi:
                    pts.append(transform_triangles(part.triangles, pose).reshape(-1, 3))
            if not pts:
                spans[eid] = None
                continue
            cam = self._to_camera(np.concatenate(pts))
            cam = cam[cam[:, 2] >= near]
            if len(cam) == 0:
                spans[eid] = None
                continue
            # Sample-index space, like the bboxes: index i is coordinate i + offset.
            u = intr.fx * cam[:, 0] / cam[:, 2] + intr.cx - self.sensors.pixel_center_offset
            spans[eid] = (float(u.min()), float(u.max()))
        return spans

    # -- lidar ------------------------------------------------------------ #

    def lidar_metrics(self, geometry: SceneGeometry) -> dict:
        sensors = self.sensors
        laser = sensors.laser_in_world
        origin = laser[:3, 3]
        z_plane = float(origin[2])
        yaw = math.atan2(laser[1, 0], laser[0, 0])
        spec = sensors.laser
        segments, seg_codes = slice_triangles(geometry.triangles, geometry.codes, z_plane)
        seg_entity = seg_codes >> _PART_BITS
        angles = yaw + spec.angles()
        ranges, owner = cast_rays(origin, angles, segments, seg_codes,
                                  range_min=spec.range_min, range_max=spec.range_max)
        owner_entity = np.where(owner >= 0, owner >> _PART_BITS, -1)
        owner_part = np.where(owner >= 0, owner & ((1 << _PART_BITS) - 1), -1)
        fine_step = math.radians(FINE_RAY_STEP_DEG)

        targets = []
        for eid in geometry.ids(KIND_TARGET):
            entity = geometry.entities[eid]
            mine = seg_entity == eid
            own_segments, own_codes = segments[mine], seg_codes[mine]
            sides = np.array([self.target_leg_side[c & ((1 << _PART_BITS) - 1)] or ''
                              for c in own_codes])
            alone_r, alone_o = cast_rays(origin, angles, own_segments, own_codes,
                                         range_min=spec.range_min, range_max=spec.range_max)
            legs = {}
            fine = None
            if len(own_segments):
                rel = own_segments.reshape(-1, 2) - origin[:2]
                seg_angles = np.arctan2(rel[:, 1], rel[:, 0])
                lo = seg_angles.min() - math.radians(0.2)
                hi = seg_angles.max() + math.radians(0.2)
                fine = np.arange(lo, hi + fine_step, fine_step)
            if fine is not None:
                fr, fo = cast_rays(origin, fine, segments, seg_codes,
                                   range_min=spec.range_min, range_max=spec.range_max)
                ar, ao = cast_rays(origin, fine, own_segments, own_codes,
                                   range_min=spec.range_min, range_max=spec.range_max)
            for side in ('left', 'right'):
                leg_parts = {i for i, s in enumerate(self.target_leg_side) if s == side}

                def on_leg(owner_codes, entity_id=eid):
                    ent = np.where(owner_codes >= 0, owner_codes >> _PART_BITS, -1)
                    part = np.where(owner_codes >= 0, owner_codes & ((1 << _PART_BITS) - 1), -1)
                    return (ent == entity_id) & np.isin(part, list(leg_parts))

                beams = on_leg(owner)
                beams_alone = on_leg(alone_o)
                width = width_alone = 0.0
                if fine is not None:
                    width = float(on_leg(fo).sum()) * FINE_RAY_STEP_DEG
                    width_alone = float(on_leg(ao).sum()) * FINE_RAY_STEP_DEG
                leg_ranges = ranges[beams]
                legs[side] = {
                    'beams': int(beams.sum()),
                    'beams_alone': int(beams_alone.sum()),
                    'width_deg': _round(width, 3),
                    'width_alone_deg': _round(width_alone, 3),
                    'width_ratio': _round(_fraction(width, width_alone)),
                    'range_median_m': (_round(float(np.median(leg_ranges)))
                                       if len(leg_ranges) else None),
                    'sliced': bool((sides == side).any()),
                }
            ratios = [legs[s]['width_ratio'] for s in legs if legs[s]['width_ratio'] is not None]
            any_leg = [i for i, s in enumerate(self.target_leg_side) if s]
            all_leg = ranges[(owner_entity == eid) & np.isin(owner_part, any_leg)]
            targets.append({
                'index': entity.index,
                'beams_total': int((owner_entity == eid).sum()),
                'legs': legs,
                'leg_range_median_m': _round(float(np.median(all_leg))) if len(all_leg) else None,
                'min_leg_width_ratio': min(ratios) if ratios else None,
                'legs_meet_criterion': (all(r >= LEG_WIDTH_CRITERION for r in ratios)
                                        if len(ratios) == 2 else None),
            })

        objects = []
        for eid in geometry.ids(KIND_OBJECT):
            entity = geometry.entities[eid]
            on_obj = owner_entity == eid
            x, y, _ = entity.pose
            objects.append({
                'index': entity.index,
                'model': entity.name,
                'beams': int(on_obj.sum()),
                'range_from_laser_m': _round(math.hypot(x - origin[0], y - origin[1])),
                'beam_range_median_m': (_round(float(np.median(ranges[on_obj])))
                                        if on_obj.any() else None),
                'bearing_from_laser_deg': _round(
                    math.degrees(math.atan2(y - origin[1], x - origin[0])), 3),
            })
        return {
            'plane_world_z_m': _round(z_plane, 5),
            'plane_target_model_z_m': _round(z_plane - self.builder.target_spawn_z, 5),
            'segments': int(len(segments)),
            'targets': targets,
            'objects': objects,
            '_ranges': ranges,
            '_owner_entity': owner_entity,
            '_angles': angles,
            '_segments': segments,
            '_segment_entity': seg_entity,
        }

    # -- poles ------------------------------------------------------------ #

    def pole_metrics(
        self,
        geometry: SceneGeometry,
        lidar: dict,
        camera: Mapping[str, ProfileRender],
    ) -> dict:
        """Fill object entries with clearance/shadow/column-span data."""

        origin = self.sensors.laser_in_world[:3, 3]
        z_plane = lidar['plane_world_z_m']
        poles = {}
        for eid in geometry.ids(KIND_OBJECT):
            entity = geometry.entities[eid]
            model = self.builder.objects[entity.name]
            shaft = None
            for part in model.parts:
                lo, hi = part.z_range
                spawn = self.builder.object_spawn_z.get(entity.name, 0.0)
                if part.kind == 'cylinder' and lo + spawn <= z_plane <= hi + spawn:
                    shaft = part
            if shaft is not None:
                x, y, yaw = entity.pose
                centre = planar_pose(x, y, 0.0, yaw) @ shaft.pose[:, 3]
                lo, hi = shaft.z_range
                poles[eid] = {
                    'xy': centre[:2], 'radius': shaft.params['radius'],
                    'z': (lo + self.builder.object_spawn_z.get(entity.name, 0.0),
                          hi + self.builder.object_spawn_z.get(entity.name, 0.0)),
                }

        by_index = {geometry.entities[eid].index: eid for eid in geometry.ids(KIND_OBJECT)}
        foot = self.sensors.footprint_xy
        spans_by_profile = {
            name: self.object_columns(geometry, render.intrinsics, z_plane)
            for name, render in camera.items()
        }
        for entry in lidar['objects']:
            eid = by_index[entry['index']]
            pole = poles.get(eid)
            entry['is_pole'] = pole is not None
            # Column span per profile and inset in every target's visible bbox.
            entry['camera'] = {}
            for name, render in camera.items():
                span = spans_by_profile[name].get(eid)
                insets = []
                for target in render.targets:
                    box = target['bbox_alone']
                    inset = None
                    if span is not None and box is not None:
                        inset = min(span[0] - box[0], box[2] - span[1])
                    insets.append({'target': target['index'],
                                   'inset_px': _round(inset, 2)})
                entry['camera'][name] = {
                    'column_span_px': None if span is None else [_round(span[0], 2),
                                                                 _round(span[1], 2)],
                    'inset_in_target_bbox': insets,
                }
            if pole is None:
                continue
            centre, radius = pole['xy'], pole['radius']
            entry['pole_axis_xy'] = [_round(centre[0]), _round(centre[1])]
            entry['pole_radius_m'] = radius
            clear_targets = []
            for tid in geometry.ids(KIND_TARGET):
                d = _axis_clearance(geometry.triangles[geometry.entity_rows[tid]],
                                    centre, pole['z'])
                clear_targets.append({'target': geometry.entities[tid].index,
                                      'clearance_m': None if d is None else _round(d - radius)})
            entry['clearance_to_targets'] = clear_targets
            others = []
            for other_eid, other in poles.items():
                if other_eid == eid:
                    continue
                d = float(np.hypot(*(centre - other['xy']))) - radius - other['radius']
                others.append({'object': geometry.entities[other_eid].index,
                               'clearance_m': _round(d)})
            entry['clearance_to_other_poles'] = others
            if foot is not None:
                dx = max(foot[0] - centre[0], 0.0, centre[0] - foot[1])
                dy = max(foot[2] - centre[1], 0.0, centre[1] - foot[3])
                entry['clearance_to_sensor_robot_m'] = _round(math.hypot(dx, dy) - radius)
            # Shadow: an interval of bearings from the laser nearer poles cover.
            dist = float(np.hypot(*(centre - origin[:2])))
            bearing = math.atan2(centre[1] - origin[1], centre[0] - origin[0])
            half = math.asin(min(1.0, radius / dist))
            shadows = []
            for other_eid, other in poles.items():
                if other_eid == eid:
                    continue
                od = float(np.hypot(*(other['xy'] - origin[:2])))
                if od >= dist:
                    continue
                ob = math.atan2(other['xy'][1] - origin[1], other['xy'][0] - origin[0])
                oh = math.asin(min(1.0, other['radius'] / od))
                overlap = min(bearing + half, ob + oh) - max(bearing - half, ob - oh)
                if overlap > 0:
                    shadows.append({'object': geometry.entities[other_eid].index,
                                    'overlap_fraction': _round(overlap / (2 * half))})
            entry['shadowed_by'] = shadows
            entry['angular_width_deg'] = _round(math.degrees(2 * half), 4)
        return lidar

    # -- entry point ------------------------------------------------------ #

    def audit_scene(self, scene) -> tuple[dict, dict[str, ProfileRender], dict]:
        geometry = self.builder.build(scene)
        renders = {intr.name: self.camera_metrics(geometry, intr)
                   for intr in self.sensors.intrinsics}
        lidar = self.lidar_metrics(geometry)
        self.pole_metrics(geometry, lidar, renders)
        result = {
            'id': scene.id,
            'targets': len(scene.robots),
            'objects': [o.model for o in scene.objects],
            'triangles': int(len(geometry.triangles)),
            'camera': {name: {'targets': r.targets} for name, r in renders.items()},
            'lidar': {k: v for k, v in lidar.items() if not k.startswith('_')},
        }
        return result, renders, {'geometry': geometry, **lidar}


def render_self(
    triangles_base: np.ndarray,
    sensors: SensorGeometry,
    *,
    near: float,
) -> dict[str, int]:
    """Pixels the sensor robot's own body covers, per camera profile."""

    tris_cam = transform_triangles(triangles_base, np.linalg.inv(sensors.camera_in_base))
    codes = np.zeros(len(tris_cam), dtype=np.int64)
    result = {}
    for intr in sensors.intrinsics:
        _, code = rasterize(tris_cam, codes, Canvas.image(intr), near=near,
                            offset=sensors.pixel_center_offset)
        result[intr.name] = int((code >= 0).sum())
    return result
