from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
from visualization_msgs.msg import Marker

from ridgeback_autonomy.common.markers import (
    NS_DROPPED,
    NS_USED,
    NS_WEDGE,
    PolarBeamRecord,
    beam_polar,
    build_polar_ray_markers,
    build_ray_marker,
    build_wedge_marker,
)


ANGLE_MIN = -math.pi / 4
ANGLE_INCREMENT = math.radians(1.0)
STAMP = SimpleNamespace(sec=3, nanosec=7)
LIFETIME = SimpleNamespace(sec=1, nanosec=500_000_000)


def make_scan(ranges, frame_id: str = 'lidar2d_0_laser') -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=STAMP),
        angle_min=ANGLE_MIN,
        angle_increment=ANGLE_INCREMENT,
        ranges=list(ranges),
    )


def expected_endpoint(beam: int, beam_range: float) -> tuple[float, float]:
    angle = ANGLE_MIN + beam * ANGLE_INCREMENT
    return beam_range * math.cos(angle), beam_range * math.sin(angle)


def test_ray_endpoints_match_scan_polar_geometry() -> None:
    # The whole point of building in the scan frame is that an endpoint is
    # recoverable from angle_min/increment alone -- assert that literally.
    scan = make_scan([2.0] * 10)
    marker = build_ray_marker(
        scan, np.array([3, 7]), NS_USED, 0, (1.0, 0.0, 0.0, 1.0), 0.02, STAMP, LIFETIME)

    assert marker is not None
    assert marker.type == Marker.LINE_LIST
    assert marker.header.frame_id == 'lidar2d_0_laser'
    # LINE_LIST consumes points pairwise: origin, hit, origin, hit.
    assert len(marker.points) == 4
    for index, beam in enumerate((3, 7)):
        origin, hit = marker.points[index * 2], marker.points[index * 2 + 1]
        assert (origin.x, origin.y, origin.z) == (0.0, 0.0, 0.0)
        want_x, want_y = expected_endpoint(beam, 2.0)
        assert math.isclose(hit.x, want_x, abs_tol=1e-9)
        assert math.isclose(hit.y, want_y, abs_tol=1e-9)
        assert hit.z == 0.0


def test_non_finite_returns_are_not_drawn() -> None:
    # A LaserScan reports out-of-range beams as inf/NaN. Those have no endpoint,
    # so drawing them would plant a ray at an arbitrary distance.
    scan = make_scan([float('inf'), 2.0, float('nan'), 0.0, 3.0])
    angles, ranges = beam_polar(scan, np.array([0, 1, 2, 3, 4]))

    assert ranges.tolist() == [2.0, 3.0]
    assert angles.size == 2


def test_out_of_bounds_beam_indices_are_ignored() -> None:
    scan = make_scan([1.0, 1.0, 1.0])
    angles, _ = beam_polar(scan, np.array([-1, 1, 99]))

    assert angles.size == 1


def test_empty_beam_set_yields_no_marker() -> None:
    scan = make_scan([1.0, 1.0])

    assert build_ray_marker(
        scan, np.array([], dtype=int), NS_USED, 0,
        (1.0, 0.0, 0.0, 1.0), 0.02, STAMP, LIFETIME) is None


def test_wedge_spans_the_bearing_extent_and_apexes_at_the_lidar() -> None:
    scan = make_scan([2.0, 2.0, 5.0, 2.0, 2.0])
    marker = build_wedge_marker(scan, np.array([1, 2, 3]), 0, STAMP, LIFETIME, segments=4)

    assert marker is not None
    assert marker.type == Marker.TRIANGLE_LIST
    assert marker.ns == NS_WEDGE
    # A fan of `segments` triangles, each apexed at the LiDAR origin.
    assert len(marker.points) == 4 * 3
    for triangle in range(4):
        apex = marker.points[triangle * 3]
        assert (apex.x, apex.y, apex.z) == (0.0, 0.0, 0.0)

    # Radius reaches the farthest beam, so the wedge contains what it claims to.
    rim = marker.points[1]
    assert math.isclose(math.hypot(rim.x, rim.y), 5.0, abs_tol=1e-9)
    # Span runs between the extreme bearings of the given beams, not the scan's.
    bearings = [math.atan2(p.y, p.x) for p in marker.points if (p.x, p.y) != (0.0, 0.0)]
    assert math.isclose(min(bearings), ANGLE_MIN + 1 * ANGLE_INCREMENT, abs_tol=1e-9)
    assert math.isclose(max(bearings), ANGLE_MIN + 3 * ANGLE_INCREMENT, abs_tol=1e-9)


def test_single_bearing_produces_no_wedge() -> None:
    # One beam has no span worth shading; the ray layer already shows it.
    scan = make_scan([2.0, 2.0, 2.0])

    assert build_wedge_marker(scan, np.array([1]), 0, STAMP, LIFETIME) is None


def test_bbox_beams_with_no_finite_return_produce_no_wedge() -> None:
    # A detection box that does not span the scan plane row selects beams that
    # never came back -- the correct picture is no wedge, not a degenerate one.
    scan = make_scan([float('inf')] * 4)

    assert build_wedge_marker(scan, np.array([0, 1, 2, 3]), 0, STAMP, LIFETIME) is None


def test_dropped_layer_is_selected_minus_merged() -> None:
    scan = make_scan([2.0] * 8)
    record = PolarBeamRecord(
        detection_index=0,
        selected=np.array([1, 2, 3, 4, 5]),
        merged=np.array([2, 3]),
        in_bbox=np.array([1, 2, 3, 4, 5]),
    )

    markers = build_polar_ray_markers(scan, record, STAMP, LIFETIME)
    by_namespace = {marker.ns: marker for marker in markers}

    assert set(by_namespace) == {NS_USED, NS_DROPPED, NS_WEDGE}
    assert len(by_namespace[NS_USED].points) == 2 * 2       # beams 2, 3
    assert len(by_namespace[NS_DROPPED].points) == 3 * 2    # beams 1, 4, 5


def test_ids_are_fixed_so_a_change_of_instance_leaves_no_orphans() -> None:
    # Two robots in frame, drawn on consecutive frames as the nearer one changes.
    # Both frames have to land on the same ids, or the robot that stopped being
    # nearest keeps its rays on screen until the marker lifetime runs out.
    scan = make_scan([2.0] * 12)
    first = PolarBeamRecord(0, np.array([1, 2, 3]), np.array([2]), np.array([1, 2, 3]))
    second = PolarBeamRecord(1, np.array([7, 8, 9]), np.array([8]), np.array([7, 8, 9]))

    keys = [
        {(marker.ns, marker.id) for marker in build_polar_ray_markers(
            scan, record, STAMP, LIFETIME)}
        for record in (first, second)
    ]

    assert keys[0] == keys[1]
    assert {(NS_USED, 0), (NS_DROPPED, 0), (NS_WEDGE, 0)} == keys[0]


def test_fully_merged_selection_draws_no_dropped_layer() -> None:
    # Nothing discarded means nothing to show; an empty LINE_LIST would render
    # as a phantom entry in the Namespaces tree.
    scan = make_scan([2.0] * 6)
    record = PolarBeamRecord(0, np.array([1, 2, 3]), np.array([1, 2, 3]), np.array([1, 2, 3]))

    namespaces = {marker.ns for marker in build_polar_ray_markers(scan, record, STAMP, LIFETIME)}

    assert NS_DROPPED not in namespaces
    assert NS_USED in namespaces
