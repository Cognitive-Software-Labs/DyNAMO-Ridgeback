"""RViz markers for the LiDAR beams polar profiling reduces to a distance.

Answers "which rays produced this number, and where are they" -- the question the
2D overlay panel cannot, since a dot on the camera image says nothing about range.
Three layers, each its own marker namespace so RViz renders one checkbox per layer
in the display's ``Namespaces`` subtree; that subtree is the whole enable/disable
mechanism, so no node parameters are involved.

``used``     the near-band survivors: exactly the beams the estimate medians over.
``dropped``  selected by the mask but discarded by the range segmentation. Drawn
             because without them a frame where the estimator latched onto an
             occluder looks identical to a frame where nothing was there.
``wedge``    the bearing span of the beams inside the detection box, so the rays
             can be read against the region they were drawn from.

Everything is built in the SCAN's own frame, where the LiDAR sits at the origin
and a beam is just ``(angle_min + i*angle_increment, ranges[i])`` in polar form.
That keeps the geometry to one cos/sin and leaves the placement to TF, rather than
duplicating the extrinsics the measurement node already applies. ``z`` is 0, i.e.
markers land on the real scan plane rather than a guessed height.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


NS_USED = 'polar/used'
NS_DROPPED = 'polar/dropped'
NS_WEDGE = 'polar/wedge'

# (r, g, b, a). The survivors are the loud layer; dropped beams stay quiet so a
# cluttered frame still reads at a glance, and the wedge is a wash the rays sit on.
COLOR_USED = (1.0, 0.65, 0.0, 1.0)
COLOR_DROPPED = (0.55, 0.55, 0.58, 0.55)
COLOR_WEDGE = (0.20, 0.75, 0.60, 0.18)

RAY_WIDTH_USED_M = 0.02
RAY_WIDTH_DROPPED_M = 0.008

# One detection is drawn per frame, so each namespace holds a single marker.
_MARKER_ID = 0

# Arc resolution of the wedge fan. The spans here are a few degrees, so this is
# far more than enough for the edge to read as straight-sided.
WEDGE_SEGMENTS = 24


@dataclass(frozen=True)
class PolarBeamRecord:
    """The beam sets for one detection, all indexing the original scan array.

    ``selected`` comes from the ``PolarProfilingAttempt`` and ``merged`` from its
    ``PolarProfilingResult``, which is why a miss still fills the first: the
    attempt has a selection whether or not an estimate came out of it.
    ``in_bbox`` is a superset of ``selected`` under a silhouette gate and equal to
    it under a box gate -- the difference is what the silhouette removed.

    ``detection_index`` says which detection in the batch these beams belong to,
    so the caller can pick the record for the instance being drawn. It is not a
    marker id: only one record is drawn per frame.
    """

    detection_index: int
    selected: np.ndarray
    merged: np.ndarray
    in_bbox: np.ndarray


def _color(rgba: tuple[float, float, float, float]) -> ColorRGBA:
    color = ColorRGBA()
    color.r, color.g, color.b, color.a = (float(component) for component in rgba)
    return color


def _point(x: float, y: float) -> Point:
    point = Point()
    point.x, point.y, point.z = float(x), float(y), 0.0
    return point


def beam_polar(scan, beams: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``(angles, ranges)`` for ``beams``, dropping anything not finite.

    A LaserScan reports out-of-range returns as inf or NaN. Those carry no
    endpoint, so drawing them would put a ray at an arbitrary distance.
    """

    beams = np.asarray(beams, dtype=np.intp).ravel()
    if beams.size == 0:
        return np.empty(0), np.empty(0)

    ranges = np.asarray(scan.ranges, dtype=np.float64)
    beams = beams[(beams >= 0) & (beams < ranges.size)]
    if beams.size == 0:
        return np.empty(0), np.empty(0)

    beam_ranges = ranges[beams]
    usable = np.isfinite(beam_ranges) & (beam_ranges > 0.0)
    beams, beam_ranges = beams[usable], beam_ranges[usable]
    angles = float(scan.angle_min) + beams * float(scan.angle_increment)
    return angles, beam_ranges


def build_ray_marker(
    scan,
    beams: np.ndarray,
    namespace: str,
    marker_id: int,
    color: tuple[float, float, float, float],
    width_m: float,
    stamp,
    lifetime,
) -> Marker | None:
    """One LINE_LIST holding every beam in ``beams`` as an origin-to-hit segment.

    A single marker rather than one per beam: a scan can contribute dozens of
    segments per detection, and RViz charges per marker, not per point.
    """

    angles, ranges = beam_polar(scan, beams)
    if angles.size == 0:
        return None

    marker = Marker()
    marker.header.frame_id = scan.header.frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width_m)
    marker.color = _color(color)
    marker.lifetime = lifetime

    points: list[Point] = []
    for angle, beam_range in zip(angles, ranges):
        points.append(_point(0.0, 0.0))
        points.append(_point(beam_range * math.cos(angle), beam_range * math.sin(angle)))
    marker.points = points
    return marker


def build_wedge_marker(
    scan,
    beams: np.ndarray,
    marker_id: int,
    stamp,
    lifetime,
    segments: int = WEDGE_SEGMENTS,
) -> Marker | None:
    """A flat TRIANGLE_LIST fan spanning the bearings of ``beams``.

    A 2D LiDAR's returns are coplanar, so the region containing them is a wedge
    at the scan plane, not a cone. Radius is the farthest beam in the set, so the
    wedge reaches everything it claims to contain. Returns ``None`` for fewer than
    two distinct bearings -- a one-beam box has no span worth shading, and the ray
    layer already shows it.
    """

    angles, ranges = beam_polar(scan, beams)
    if angles.size < 2:
        return None
    start, end = float(angles.min()), float(angles.max())
    if end <= start:
        return None
    radius = float(ranges.max())

    marker = Marker()
    marker.header.frame_id = scan.header.frame_id
    marker.header.stamp = stamp
    marker.ns = NS_WEDGE
    marker.id = int(marker_id)
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color = _color(COLOR_WEDGE)
    marker.lifetime = lifetime

    edges = np.linspace(start, end, int(segments) + 1)
    points: list[Point] = []
    for near, far in zip(edges[:-1], edges[1:]):
        points.append(_point(0.0, 0.0))
        points.append(_point(radius * math.cos(near), radius * math.sin(near)))
        points.append(_point(radius * math.cos(far), radius * math.sin(far)))
    marker.points = points
    return marker


def build_polar_ray_markers(
    scan,
    record,
    stamp,
    lifetime,
) -> list[Marker]:
    """Every marker for one frame, for the one detection being drawn.

    The three namespaces keep the layers from colliding and each holds a single
    fixed id, so a frame overwrites the previous one instead of accumulating.
    Keying the id off the detection index instead would strand the old
    instance's rays on screen for a full lifetime every time the drawn detection
    changes. Markers carry a lifetime rather than being explicitly deleted, so
    they expire when detections stop -- matching how the estimate rings behave.
    """

    dropped = np.setdiff1d(
        np.asarray(record.selected, dtype=np.intp),
        np.asarray(record.merged, dtype=np.intp),
    )
    candidates = (
        build_wedge_marker(scan, record.in_bbox, _MARKER_ID, stamp, lifetime),
        build_ray_marker(
            scan, dropped, NS_DROPPED, _MARKER_ID,
            COLOR_DROPPED, RAY_WIDTH_DROPPED_M, stamp, lifetime),
        build_ray_marker(
            scan, record.merged, NS_USED, _MARKER_ID,
            COLOR_USED, RAY_WIDTH_USED_M, stamp, lifetime),
    )
    return [marker for marker in candidates if marker is not None]
