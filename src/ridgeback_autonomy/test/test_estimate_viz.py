from __future__ import annotations

import html
import math
import re
from types import MethodType, SimpleNamespace

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PointStamped

from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_LABELS,
    MASK_ESTIMATORS,
    PUBLIC_ESTIMATOR_ORDER,
    TRUTH_MAX_AGE_S,
    display_bearing,
    nearest_instance_index,
    parse_estimators,
    truth_reading,
)
from ridgeback_autonomy.benchmarking.g1_estimate_viz_node import (
    BORROWED_BEARING_LINE_WIDTH_M,
    ESTIMATOR_COLOURS,
    G1EstimateVizNode,
    HUD_AGED_LUMINANCE,
    HUD_MIN_LUMINANCE,
    MARKER_LIFETIME_SEC,
    MAX_OBSERVATION_AGE_S,
    RING_LINE_WIDTH_M,
    _ESTIMATOR_ID_BASE,
    _hud_line,
    _LUMA_WEIGHTS,
    colour_at_luminance,
    estimator_reading,
    hud_section_text,
    hud_text_colour,
    hud_truth_header,
    merged_position_reader,
    nearest_detection_index,
    partition_measurements,
    world_marker_point,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.geometry import (
    ROBOT_FRONT_OFFSET_M,
    planar_measurement_from_vehicle_front,
)


def test_every_registered_estimator_has_a_colour() -> None:
    # A ring with no colour raises a KeyError mid-callback, so the registry and
    # the palette have to stay in step -- including the three mask paths, which
    # had no entry before and therefore drew nothing.
    assert set(ESTIMATOR_COLOURS) == set(PUBLIC_ESTIMATOR_ORDER)


def test_mask_and_legacy_families_are_split_by_colour_temperature() -> None:
    # Nothing on screen labels the two families, so the warm/cold split is the
    # whole of the distinction: a legacy row that drifted cold would silently
    # read as a mask path. Red over blue is warm, blue over red cold.
    for estimator, (red, _, blue, _) in ESTIMATOR_COLOURS.items():
        if estimator in MASK_ESTIMATORS:
            assert blue > red, f'{estimator} is a mask path but reads warm'
        else:
            assert red > blue, f'{estimator} is a legacy path but reads cold'


def test_marker_id_bases_are_distinct() -> None:
    bases = [_ESTIMATOR_ID_BASE[name] for name in PUBLIC_ESTIMATOR_ORDER]

    assert len(set(bases)) == len(bases)
    # An estimator owns a ring at its base and a dot at base + 1, so two bases
    # one apart would have one estimator overwriting another's dot.
    assert min(abs(a - b) for a in bases for b in bases if a != b) >= 2


def test_mask_estimator_positions_are_read_from_the_message() -> None:
    msg = G1Measurements()
    msg.detected = True
    msg.count = 1
    msg.polar_profiling_forward_m = [3.0]
    msg.polar_profiling_lateral_m = [0.5]
    msg.polar_profiling_distance_m = [3.04]

    forward, lateral, distance = estimator_reading(msg, 'polar_profiling', 0)

    # The message fields are float32, so compare with tolerance.
    assert (forward, lateral) == (3.0, 0.5)
    assert distance == pytest.approx(3.04, abs=1e-6)


def test_absent_estimator_slots_read_as_none() -> None:
    # Every producer publishes the same message type, so the fields belonging to
    # other nodes are simply empty -- that must read as "no value", not crash.
    msg = G1Measurements()
    msg.detected = True
    msg.count = 1

    assert estimator_reading(msg, 'lidar', 0) == (None, None, None)


def test_nan_slots_read_as_none() -> None:
    msg = G1Measurements()
    msg.detected = True
    msg.count = 1
    msg.rgb_distance_m = [float('nan')]

    assert estimator_reading(msg, 'rgb', 0)[2] is None


def reader_from(table):
    """``(estimator, index) -> distance`` from a ``{index: {estimator: value}}``."""

    return lambda estimator, index: table.get(index, {}).get(estimator)


def test_nearest_instance_index_picks_the_closest_detection() -> None:
    table = {0: {'rgb': 4.2}, 1: {'rgb': 2.1}}

    assert nearest_instance_index(2, reader_from(table)) == 1


def test_canonical_order_decides_which_estimator_speaks_for_a_detection() -> None:
    # Detection 1 holds the smallest number in the frame, but only on an
    # estimator that never gets consulted: rgb answers for both detections, so
    # detection 0 is the nearer one. Ranking by the minimum across estimators
    # instead would flip this, and would flip it differently in a node that does
    # not compute rgb.
    table = {0: {'rgb': 1.0, 'lidar': 5.0}, 1: {'rgb': 2.0, 'lidar': 0.5}}

    assert nearest_instance_index(2, reader_from(table)) == 0


def test_equal_distances_tie_break_to_the_lower_index() -> None:
    table = {0: {'rgb': 3.0}, 1: {'rgb': 3.0}}

    assert nearest_instance_index(2, reader_from(table)) == 0


def test_a_frame_no_estimator_placed_ranks_to_nothing() -> None:
    assert nearest_instance_index(2, reader_from({})) is None


def measurements(count: int, *, stamp_ns: int = 0, **fields) -> G1Measurements:
    msg = G1Measurements()
    msg.detected = count > 0
    msg.count = count
    msg.header.stamp = TimeMsg(
        sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    for name, values in fields.items():
        setattr(msg, name, values)
    return msg


def test_nearest_detection_index_ranks_across_every_producer() -> None:
    # The camera node fills rgb, the mask node fills polar profiling, and the
    # indices are the same detections in both. Detection 1 is the closer robot.
    camera = measurements(2, rgb_distance_m=[4.0, 2.0])
    mask = measurements(2, polar_profiling_distance_m=[4.1, 2.2])

    assert nearest_detection_index([camera, mask]) == 1


def test_nearest_detection_index_falls_back_to_the_first_detection() -> None:
    # Boxes but no distances: nothing is rankable, and the frame has to keep
    # rendering its all-miss HUD rather than crash on a None index.
    assert nearest_detection_index([measurements(2)]) == 0


def luminance(colour) -> float:
    return sum(w * c for w, c in zip(_LUMA_WEIGHTS, colour))


def test_every_hud_colour_clears_the_legibility_floor() -> None:
    # The HUD sits on a half-opaque black panel, so a dark ring colour is
    # unreadable as text. Red and depth_anything's rose are the ones that fail
    # without the lift.
    for estimator in ESTIMATOR_COLOURS:
        assert luminance(hud_text_colour(estimator)) >= HUD_MIN_LUMINANCE - 1e-9


def test_bright_colours_are_left_exactly_alone() -> None:
    # lidar's chartreuse is already well above the floor; lifting it would drift
    # the text away from the ring for no reason.
    assert hud_text_colour('lidar') == ESTIMATOR_COLOURS['lidar'][:3]


def test_lift_preserves_hue_ordering_so_rings_stay_identifiable() -> None:
    # Blending toward white must not reorder the channels -- rose that came out
    # red-dominant with blue second has to stay that way, or it no longer reads
    # as the same estimator as its ring.
    red, green, blue = hud_text_colour('depth_anything')
    original_r, original_g, original_b = ESTIMATOR_COLOURS['depth_anything'][:3]

    assert red > blue > green
    assert (original_r > original_b > original_g)


# --- aged rows --------------------------------------------------------------


def test_every_aged_colour_lands_on_the_aged_luminance() -> None:
    # One target for all eight, not a dim multiplier: red and depth_anything's
    # rose sit AT the legibility floor after their lift, so a multiply-then-clamp
    # would hand them back their fresh appearance and the state would be
    # invisible on exactly the two rows that most need it.
    for estimator in ESTIMATOR_COLOURS:
        assert luminance(hud_text_colour(estimator, aged=True)) == pytest.approx(
            HUD_AGED_LUMINANCE, abs=1e-9)


def test_aged_is_strictly_dimmer_than_fresh_for_every_estimator() -> None:
    for estimator in ESTIMATOR_COLOURS:
        assert luminance(hud_text_colour(estimator, aged=True)) < luminance(
            hud_text_colour(estimator))


def test_dimming_preserves_hue_ordering_so_an_aged_row_keeps_its_estimator() -> None:
    # The mirror of the lift test, and it matters more here: an aged row draws no
    # ring, so its colour is the only thing left tying it to an estimator.
    red, green, blue = hud_text_colour('depth_anything', aged=True)

    assert red > blue > green


def test_dimming_preserves_saturation_not_just_hue() -> None:
    # Scaling all three channels by one factor holds their ratios exactly, so a
    # dimmed row is the same colour rather than a washed-out one. Blending toward
    # black would do the same; blending toward grey would not.
    original = ESTIMATOR_COLOURS['euclidean_reconstruction'][:3]
    dimmed = hud_text_colour('euclidean_reconstruction', aged=True)

    scales = [d / o for d, o in zip(dimmed, original) if o != 0.0]
    assert scales == pytest.approx([scales[0]] * len(scales), abs=1e-9)


def test_colour_at_luminance_lifts_a_colour_that_starts_below_the_target() -> None:
    # red's 0.2126 is under the aged target too, so it is the one aged row that
    # gets brighter rather than darker -- the reason this is a target and not a
    # dimming.
    lifted = colour_at_luminance(ESTIMATOR_COLOURS['rgb'][:3], HUD_AGED_LUMINANCE)

    assert luminance(lifted) == pytest.approx(HUD_AGED_LUMINANCE, abs=1e-9)
    assert luminance(lifted) > luminance(ESTIMATOR_COLOURS['rgb'][:3])


def test_colour_at_luminance_leaves_a_colour_already_on_target_untouched() -> None:
    colour = ESTIMATOR_COLOURS['lidar'][:3]

    assert colour_at_luminance(colour, luminance(colour)) == colour


def test_hud_line_escapes_before_padding() -> None:
    # Padding first would leave the escaper rewriting its own ampersands into
    # "&amp;nbsp;", which renders as literal text instead of a space.
    line = _hud_line('a  b', (1.0, 1.0, 1.0))

    assert '&nbsp;&nbsp;' in line
    assert '&amp;nbsp;' not in line


def test_hud_line_escapes_markup_in_the_body() -> None:
    line = _hud_line('a <b> & c', (1.0, 1.0, 1.0))

    assert '&lt;b&gt;' in line
    assert '&amp;' in line


def test_hud_line_emits_an_rgb_span() -> None:
    line = _hud_line('x', (1.0, 0.0, 0.5))

    assert line.startswith('<span style="color: rgb(255, 0, 128)">')
    assert line.endswith('</span>')


def test_depth_only_estimators_have_distance_but_no_position() -> None:
    # sensor_depth and depth_anything report a planar distance and nothing else;
    # they intentionally have no entry in the position registry, which is why
    # their rings have to borrow a bearing to be drawn at all.
    msg = G1Measurements()
    msg.detected = True
    msg.count = 1
    msg.sensor_depth_distance_m = [2.5]

    forward, lateral, distance = estimator_reading(msg, 'sensor_depth', 0)

    assert (forward, lateral) == (None, None)
    assert distance == 2.5


# --- ring placement ---------------------------------------------------------
#
# These assert where a ring actually lands in the world. Nothing did before, and
# the gap let every ring sit ROBOT_FRONT_OFFSET_M short of the distance printed
# beside it: the measurement is referenced to the robot front, the pose TF
# returns is the base origin, and the two were added together directly.


def _place(
    estimator: str,
    forward_m: float | None,
    lateral_m: float | None,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_yaw: float = 0.0,
    distance_m: float | None = None,
    bearing_rad: float | None = None,
) -> list:
    """Markers from the real node method, with no ROS context to stand up.

    ``_add_estimator_markers`` only reaches ``self`` for the marker lifetime, so
    an unbound call against a stub exercises the shipped placement path rather
    than a copy of the arithmetic that could drift from it.
    """

    markers: list = []
    G1EstimateVizNode._add_estimator_markers(
        SimpleNamespace(marker_lifetime=1.5),
        markers,
        estimator,
        forward_m,
        lateral_m,
        robot_x,
        robot_y,
        robot_yaw,
        TimeMsg(),
        'map',
        distance_m=distance_m,
        bearing_rad=bearing_rad,
    )
    return markers


def _dot_xy(markers: list) -> tuple[float, float]:
    ring, dot = markers
    # The ring is a circle of points around the same centre the dot sits on, so
    # averaging them recovers that centre and a ring drawn away from its own dot
    # cannot slip through the dot-only assertions below. The last point repeats
    # the first to close the line strip and is dropped, or it drags the mean.
    assert (ring.points[0].x, ring.points[0].y) == pytest.approx(
        (ring.points[-1].x, ring.points[-1].y), abs=1e-9)
    arc = ring.points[:-1]
    centre_x = sum(p.x for p in arc) / len(arc)
    centre_y = sum(p.y for p in arc) / len(arc)
    assert (centre_x, centre_y) == pytest.approx(
        (dot.pose.position.x, dot.pose.position.y), abs=1e-6)
    return dot.pose.position.x, dot.pose.position.y


def test_ring_lands_on_the_world_point_the_measurement_came_from() -> None:
    # The round trip a producer and this node together perform: world offset ->
    # front-referenced measurement -> back to the world. Yaw is deliberately not
    # axis-aligned, so an offset applied along the wrong axis cannot pass.
    robot_x, robot_y, robot_yaw = 2.0, -1.5, math.radians(37.0)
    target_x, target_y = 5.4, 0.9

    forward_m, lateral_m, _ = planar_measurement_from_vehicle_front(
        target_x - robot_x, target_y - robot_y, robot_yaw)
    markers = _place('lidar', forward_m, lateral_m, robot_x, robot_y, robot_yaw)

    assert _dot_xy(markers) == pytest.approx((target_x, target_y), abs=1e-9)


def test_ring_is_referenced_to_the_robot_front_not_the_base_origin() -> None:
    # A robot facing +X: a 3.00 m reading is 3.25 m from the base origin, since
    # the producer already subtracted the front offset from it.
    markers = _place('rgb', 3.0, 0.0)

    assert _dot_xy(markers) == pytest.approx((3.0 + ROBOT_FRONT_OFFSET_M, 0.0), abs=1e-9)


def test_the_front_offset_turns_with_the_robot() -> None:
    # Same reading with the robot facing +Y. The offset has to rotate with the
    # base, so it lands on the y axis -- adding it in the world frame would
    # leave the dot off-axis and still pass the yaw=0 case above.
    markers = _place('rgb', 3.0, 0.0, robot_yaw=math.pi / 2.0)

    assert _dot_xy(markers) == pytest.approx((0.0, 3.0 + ROBOT_FRONT_OFFSET_M), abs=1e-9)


def test_depth_only_ring_takes_the_bearing_another_row_measured() -> None:
    # An off-axis target: 2.0 m away on a bearing of -30 degrees. The depth row
    # owns the radius, the borrowed bearing owns the direction.
    bearing = math.radians(-30.0)
    markers = _place(
        'sensor_depth', None, None, distance_m=2.0, bearing_rad=bearing)

    dot_x, dot_y = _dot_xy(markers)
    from_front = math.hypot(dot_x - ROBOT_FRONT_OFFSET_M, dot_y)
    assert from_front == pytest.approx(2.0, abs=1e-9)
    assert math.atan2(dot_y, dot_x - ROBOT_FRONT_OFFSET_M) == pytest.approx(
        bearing, abs=1e-9)
    # The old behaviour put this on the boresight, a full metre out.
    assert dot_y == pytest.approx(-1.0, abs=1e-9)


def test_depth_only_ring_keeps_the_boresight_when_no_one_placed_the_detection() -> None:
    # Nothing else measured a position, so there is no bearing to borrow. A ring
    # on the boresight is still worth more than no ring: the radius is right.
    markers = _place('sensor_depth', None, None, distance_m=2.0)

    assert _dot_xy(markers) == pytest.approx((2.0 + ROBOT_FRONT_OFFSET_M, 0.0), abs=1e-9)


def test_a_row_with_neither_a_position_nor_a_distance_draws_nothing() -> None:
    assert _place('lidar', None, None) == []


def test_borrowed_bearing_rings_are_drawn_thinner() -> None:
    # The width is the only cue that a ring's direction is not its own answer.
    borrowed, _ = _place('sensor_depth', None, None, distance_m=2.0, bearing_rad=0.1)
    measured, _ = _place('lidar', 2.0, 0.1)

    assert borrowed.scale.x == BORROWED_BEARING_LINE_WIDTH_M
    assert measured.scale.x == RING_LINE_WIDTH_M
    assert BORROWED_BEARING_LINE_WIDTH_M < RING_LINE_WIDTH_M


def test_world_marker_point_is_a_plain_rotation_about_the_given_origin() -> None:
    # Guards the convention the placement rests on: +forward is base +X and
    # +lateral is base +Y (left-positive, REP-103), not the other way round.
    assert world_marker_point(1.0, 0.0, 0.0, 0.0, math.pi / 2.0) == pytest.approx(
        (0.0, 1.0), abs=1e-9)
    assert world_marker_point(0.0, 1.0, 0.0, 0.0, 0.0) == pytest.approx(
        (0.0, 1.0), abs=1e-9)


# --- borrowed bearings ------------------------------------------------------


def test_display_bearing_follows_the_canonical_estimator_order() -> None:
    # rgb precedes lidar in PUBLIC_ESTIMATOR_ORDER, so its bearing is the one
    # borrowed -- the same first-usable rule display_distance applies, so the
    # direction drawn and the distance printed cannot come from different rows.
    positions = {'rgb': (3.0, 3.0), 'lidar': (3.0, -3.0)}

    bearing = display_bearing(lambda estimator, index: positions.get(estimator), 0)

    assert bearing == pytest.approx(math.radians(45.0), abs=1e-9)


def test_display_bearing_skips_rows_that_placed_nothing() -> None:
    positions = {'lidar': (0.0, -2.0)}

    bearing = display_bearing(lambda estimator, index: positions.get(estimator), 0)

    assert bearing == pytest.approx(math.radians(-90.0), abs=1e-9)


def test_display_bearing_is_none_when_no_row_placed_the_detection() -> None:
    assert display_bearing(lambda estimator, index: None, 0) is None


def test_display_bearing_rejects_a_degenerate_origin_reading() -> None:
    # atan2(0, 0) is 0.0, an answer indistinguishable from a real dead-ahead
    # bearing; a row that placed the target on the robot itself has not measured
    # a direction, so the next row gets the chance.
    positions = {'rgb': (0.0, 0.0), 'lidar': (0.0, 2.0)}

    bearing = display_bearing(lambda estimator, index: positions.get(estimator), 0)

    assert bearing == pytest.approx(math.radians(90.0), abs=1e-9)


def test_merged_position_reader_finds_a_position_in_any_producer() -> None:
    # Each producer fills only its own rows, so a bearing has to be looked for
    # across all of them -- the same reason merged_distance_reader exists.
    camera = G1Measurements()
    camera.detected = True
    camera.count = 1
    camera.rgb_forward_m = [4.0]
    camera.rgb_lateral_m = [1.0]

    mask = G1Measurements()
    mask.detected = True
    mask.count = 1
    mask.polar_profiling_forward_m = [4.1]
    mask.polar_profiling_lateral_m = [1.1]

    read_position = merged_position_reader([camera, mask])

    assert read_position('rgb', 0) == pytest.approx((4.0, 1.0), abs=1e-6)
    assert read_position('polar_profiling', 0) == pytest.approx((4.1, 1.1), abs=1e-6)
    assert read_position('lidar', 0) is None


def _truth_message(distance_m: float, trial_id: str, stamp_ns: int) -> PointStamped:
    msg = PointStamped()
    msg.header.frame_id = trial_id
    msg.header.stamp = TimeMsg(
        sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    msg.point.x = 0.0
    msg.point.y = distance_m
    msg.point.z = distance_m
    return msg


def test_truth_reading_expires_on_stamp_age_not_on_arrival() -> None:
    # The regression: the truth line used to expire on when the message reached
    # the callback, so a message delayed behind a full subscription queue read
    # as fresh. That pinned the PREVIOUS trial's truth under the current
    # trial's scene for a whole capture window -- the estimator rows moved on,
    # the truth number did not, and the difference showed up as sensor error.
    now_ns = 500 * 1_000_000_000
    stale_ns = now_ns - int((TRUTH_MAX_AGE_S + 1.0) * 1_000_000_000)

    assert truth_reading(_truth_message(1.929, 'multi_visible', stale_ns), now_ns) is None


def test_truth_reading_unpacks_the_planar_convention_and_its_trial() -> None:
    now_ns = 500 * 1_000_000_000
    msg = _truth_message(3.25, 'bed_occluder_single', now_ns)
    msg.point.x = -0.75

    reading = truth_reading(msg, now_ns)

    assert reading.lateral_m == pytest.approx(-0.75)
    assert reading.forward_m == pytest.approx(3.25)
    assert reading.distance_m == pytest.approx(3.25)
    assert reading.trial_id == 'bed_occluder_single'


def test_truth_reading_is_none_before_any_message_arrives() -> None:
    assert truth_reading(None, 0) is None


def test_hud_truth_header_names_the_trial_the_number_belongs_to() -> None:
    # A bare number cannot be checked against the scene on screen; the trial id
    # is what turns "the estimators disagree" into "that is the wrong target".
    reading = truth_reading(
        _truth_message(3.25, 'bed_occluder_single', 0), 0)

    assert hud_truth_header(reading) == (
        'G1 DISTANCES   truth 3.250 m  [bed_occluder_single]')


def test_hud_truth_header_drops_the_truth_entirely_when_it_has_expired() -> None:
    assert hud_truth_header(None) == 'G1 DISTANCES'


def test_hud_truth_header_omits_empty_brackets_for_an_unnamed_trial() -> None:
    reading = truth_reading(_truth_message(2.25, '', 0), 0)

    assert hud_truth_header(reading) == 'G1 DISTANCES   truth 2.250 m'


def hud_text_from(
    same_batch: list,
    nearest: int,
    truth,
    aged: list | None = None,
    estimators: tuple[str, ...] | None = None,
) -> str:
    """The shipped section renderer, with no ROS context to stand up.

    ``hud_section_text`` is module-level and pure, so the error column -- which
    subtracts a field off the reading rather than the reading itself -- is
    assertable directly. Omitting ``estimators`` exercises the default, which is
    what the exploration entrypoint gets.
    """

    if estimators is None:
        return hud_section_text(same_batch, aged or [], nearest, truth)
    return hud_section_text(same_batch, aged or [], nearest, truth, estimators)


def test_hud_scores_every_row_against_the_truth_it_displays() -> None:
    fresh = [measurements(1, rgb_distance_m=[3.885], lidar_distance_m=[1.050])]
    truth = truth_reading(_truth_message(3.25, 'bed_occluder_single', 0), 0)

    text = hud_text_from(fresh, 0, truth)

    assert 'truth&nbsp;3.250&nbsp;m&nbsp;&nbsp;[bed_occluder_single]' in text
    # 3.885 - 3.25 and 1.050 - 3.25, off the SAME trial's truth.
    assert '+0.635' in text
    assert '-2.200' in text


def test_hud_drops_the_error_column_when_the_truth_has_expired() -> None:
    # No truth means no reference, and a distance printed beside a stale error
    # is worse than one printed beside none.
    fresh = [measurements(1, rgb_distance_m=[3.885])]

    text = hud_text_from(fresh, 0, None)

    assert 'truth' not in text
    assert '+0.635' not in text
    assert '3.885' in text


# --- freshness: liveness, validity and the two row states -------------------
#
# The mask rows used to blink: their message is stamped with the DETECTION
# instant, so inference, segmentation and the depth lookup are all charged
# against it, and a single stamp gate against the marker lifetime threw the
# reading away while camera and lidar -- same stamp, no pipeline behind it --
# repainted the row as "-- miss" at detector rate.


SECOND_NS = 1_000_000_000


def cached(msg, receipt_ns: int):
    return (msg, receipt_ns)


def test_a_message_older_than_the_lifetime_but_received_now_still_renders() -> None:
    # The regression. 2.0 s of pipeline latency, arrived this instant: the
    # producer is plainly running, and the old gate dropped it anyway.
    now = 100 * SECOND_NS
    lagging = measurements(1, stamp_ns=now - 2 * SECOND_NS,
                           polar_profiling_distance_m=[2.4])

    same_batch, aged = partition_measurements(
        [cached(lagging, now)], now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert same_batch == [lagging]
    assert aged == []


def test_a_message_received_long_ago_is_dropped_however_recent_its_stamp() -> None:
    # Liveness is the receipt question: this producer has said nothing for two
    # lifetimes, so its last word is not evidence that it is still running.
    now = 100 * SECOND_NS
    quiet = measurements(1, stamp_ns=now, rgb_distance_m=[2.4])

    same_batch, aged = partition_measurements(
        [cached(quiet, now - 3 * SECOND_NS)],
        now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert (same_batch, aged) == ([], [])


def test_a_message_past_the_observation_age_is_dropped_however_recent_its_receipt() -> None:
    # Validity is the stamp question, and it is what keeps the cross-trial
    # protection the old single gate provided: an observation from before the
    # target teleported describes a scene that no longer exists.
    now = 100 * SECOND_NS
    ancient = measurements(
        1, stamp_ns=now - int((MAX_OBSERVATION_AGE_S + 1.0) * SECOND_NS),
        polar_profiling_distance_m=[2.4])

    same_batch, aged = partition_measurements(
        [cached(ancient, now)], now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert (same_batch, aged) == ([], [])


def test_only_the_newest_stamp_counts_as_the_current_batch() -> None:
    now = 100 * SECOND_NS
    camera = measurements(1, stamp_ns=now, rgb_distance_m=[2.4])
    lidar = measurements(1, stamp_ns=now, lidar_distance_m=[2.5])
    lagging_mask = measurements(1, stamp_ns=now - int(1.2 * SECOND_NS),
                                polar_profiling_distance_m=[2.6])

    same_batch, aged = partition_measurements(
        [cached(camera, now), cached(lidar, now), cached(lagging_mask, now)],
        now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert same_batch == [camera, lidar]
    assert [msg for msg, _ in aged] == [lagging_mask]
    assert aged[0][1] == pytest.approx(1.2, abs=1e-9)


def test_a_lone_lagging_producer_is_its_own_current_batch() -> None:
    # A mask-only run has nothing newer to contradict it, so its rings are still
    # the best claim available and must keep being drawn.
    now = 100 * SECOND_NS
    mask = measurements(1, stamp_ns=now - int(1.2 * SECOND_NS),
                        polar_profiling_distance_m=[2.6])

    same_batch, aged = partition_measurements(
        [cached(mask, now)], now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert same_batch == [mask]
    assert aged == []


def test_an_empty_cache_partitions_to_nothing() -> None:
    assert partition_measurements(
        [None, None, None], 0, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S) == ([], [])


def test_an_aged_reading_renders_with_an_age_column_instead_of_a_miss() -> None:
    # The whole point: the mask row keeps its number and says how old it is,
    # rather than being repainted as "-- miss" by the camera row's stamp.
    camera = measurements(1, rgb_distance_m=[3.885])
    mask = measurements(1, polar_profiling_distance_m=[2.431])

    text = hud_text_from([camera], 0, None, aged=[(mask, 1.2)])

    assert '2.431' in text
    assert '1.2s' in text
    assert 'Polar&nbsp;Profiling' in text
    # The camera row is current and carries no age column.
    assert '3.885' in text


def test_an_aged_row_is_coloured_at_the_aged_luminance() -> None:
    mask = measurements(1, polar_profiling_distance_m=[2.431])
    dimmed = hud_text_colour('polar_profiling', aged=True)
    expected = ', '.join(str(int(round(channel * 255))) for channel in dimmed)

    text = hud_text_from([measurements(1, rgb_distance_m=[3.885])], 0, None,
                         aged=[(mask, 1.2)])

    assert f'rgb({expected})' in text


def test_a_current_reading_wins_over_an_aged_one_for_the_same_estimator() -> None:
    # Nothing publishes the same estimator from two batches today, but if it
    # did, the current answer is the one the rings are drawn from.
    current = measurements(1, polar_profiling_distance_m=[2.431])
    stale = measurements(1, polar_profiling_distance_m=[9.999])

    text = hud_text_from([current], 0, None, aged=[(stale, 1.2)])

    assert '2.431' in text
    assert '9.999' not in text
    assert '1.2s' not in text


def test_an_aged_message_is_ranked_against_itself_not_the_current_batch() -> None:
    # Two robots, and the batches disagree about which is nearer. The aged
    # message's own ranking picks its index 1 (2.0 m); borrowing the current
    # batch's nearest would print 5.0 m against the wrong robot.
    camera = measurements(2, rgb_distance_m=[1.0, 6.0])
    mask = measurements(2, polar_profiling_distance_m=[5.0, 2.0])

    text = hud_text_from([camera], 0, None, aged=[(mask, 1.2)])

    assert '2.000' in text
    assert '5.000' not in text


def test_a_nan_reading_still_renders_a_miss_distinct_from_an_aged_one() -> None:
    # "miss" has to mean the estimator ran and returned nothing, or the word is
    # a lie about the estimator every time the HUD is merely out of date.
    missed = hud_text_from([measurements(1, rgb_distance_m=[float('nan')])], 0, None)
    aged = hud_text_from(
        [measurements(1, lidar_distance_m=[2.0])], 0, None,
        aged=[(measurements(1, polar_profiling_distance_m=[2.431]), 1.2)])

    assert 'miss' in missed
    assert 'nan' not in missed.lower()
    assert 'miss' in aged and '2.431' in aged and '1.2s' in aged


def test_aged_messages_never_reach_the_ranking_the_rings_are_drawn_from() -> None:
    # A stale position is a false claim about where a robot is now, in a way a
    # stale number is not -- and an older batch's index 0 may not even be the
    # same robot. _publish_markers ranks and draws from same_batch alone, so the
    # aged mask message here must not pull the nearest index onto its own robot.
    now = 100 * SECOND_NS
    # The camera placed only detection 0 this batch; the aged mask message has
    # its own detection 1 much nearer.
    camera = measurements(2, stamp_ns=now, rgb_distance_m=[1.0, float('nan')])
    mask = measurements(2, stamp_ns=now - int(1.2 * SECOND_NS),
                        polar_profiling_distance_m=[9.0, 0.5])

    same_batch, aged = partition_measurements(
        [cached(camera, now), cached(mask, now)],
        now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert [msg for msg, _ in aged] == [mask]
    assert nearest_detection_index(same_batch) == 0
    # Folding the aged message back in flips every ring onto a robot no current
    # message measured -- which is what the split exists to prevent.
    assert nearest_detection_index(same_batch + [msg for msg, _ in aged]) == 1


def test_the_hud_renders_nothing_at_all_when_nothing_is_cached() -> None:
    # hud_node drops falsy panels, so the empty string collapses the section.
    # Returning early instead is what latched the last numbers on screen under a
    # floor plan whose rings had all correctly expired.
    assert hud_section_text([], [], 0, None) == ''


def test_the_hud_still_renders_when_the_truth_has_gone_but_readings_have_not() -> None:
    # The timer re-evaluates the truth gate with no measurement traffic needed;
    # the rows outlive the truth line rather than disappearing with it.
    text = hud_section_text([measurements(1, rgb_distance_m=[3.885])], [], 0, None)

    assert text.count('<br/>') == len(PUBLIC_ESTIMATOR_ORDER)
    assert 'G1&nbsp;DISTANCES' in text
    assert 'truth' not in text


# --- the run's estimator set decides the rows -------------------------------
#
# A row for a path the run never launched can only ever print "-- miss", which
# is the same thing the panel prints for an estimator that ran and found
# nothing. The benchmark's own default selects the five legacy rows, so the
# unfiltered panel carried three permanently dead mask rows.


def hud_labels(text: str) -> list[str]:
    """The estimator label from each row, in the order the panel prints them.

    Every row body starts with the label in a fixed 24-column field
    (``_reading_body``, and the miss branch beside it), so the label is that
    slice once the span markup is stripped and the ``&nbsp;`` padding is turned
    back into ordinary spaces. Splitting on whitespace instead would cut
    two-word labels like ``Sensor Depth`` in half.
    """

    rows = text.split('<br/>')[1:]  # drop the truth header
    return [
        html.unescape(re.sub(r'<[^>]+>', '', row))
        .replace('\xa0', ' ')[:24]
        .strip()
        for row in rows
    ]


def test_only_the_selected_estimators_get_a_row() -> None:
    same_batch = [measurements(1, lidar_distance_m=[2.430])]

    text = hud_text_from(same_batch, 0, None, estimators=('lidar', 'polar_profiling'))

    assert hud_labels(text) == ['LiDAR', 'Polar Profiling']


def test_rows_keep_canonical_order_however_the_argument_named_them() -> None:
    # parse_estimators sorts into PUBLIC_ESTIMATOR_ORDER, so "polar,lidar" and
    # "lidar,polar" have to render identically -- the HUD's order is the
    # registry's, not the launch line's.
    selected = parse_estimators('polar_profiling,rgb,lidar')
    same_batch = [measurements(1, rgb_distance_m=[3.885], lidar_distance_m=[2.430])]

    text = hud_text_from(same_batch, 0, None, estimators=selected)

    assert hud_labels(text) == ['RGB', 'LiDAR', 'Polar Profiling']


def test_an_unselected_estimator_that_is_publishing_still_gets_no_row() -> None:
    # The only way this arises: a producer that fills more rows than the run
    # asked for. Selection has to be enforced where it is rendered, not merely
    # assumed of the upstream node.
    same_batch = [measurements(
        1,
        polar_profiling_distance_m=[2.455],
        projective_ranging_distance_m=[2.431],
    )]

    text = hud_text_from(same_batch, 0, None, estimators=('polar_profiling',))

    assert hud_labels(text) == ['Polar Profiling']
    assert '2.431' not in text


def test_a_selected_estimator_that_reported_nothing_still_reads_as_a_miss() -> None:
    # Filtering must not swallow the selected-but-silent case: that row is the
    # one place "-- miss" is now the honest word.
    same_batch = [measurements(1, lidar_distance_m=[2.430])]

    text = hud_text_from(same_batch, 0, None, estimators=('lidar', 'polar_profiling'))

    assert hud_labels(text) == ['LiDAR', 'Polar Profiling']
    assert 'miss' in text


def test_an_aged_row_keeps_its_age_column_when_the_set_is_restricted() -> None:
    same_batch = [measurements(1, lidar_distance_m=[2.430])]
    aged = [(measurements(1, polar_profiling_distance_m=[2.455]), 1.2)]

    text = hud_text_from(
        same_batch, 0, None, aged=aged, estimators=('lidar', 'polar_profiling'))

    assert '2.455' in text
    assert '1.2s' in text


def test_omitting_the_set_renders_every_registered_estimator() -> None:
    # The exploration entrypoint declares no estimators argument at all, so the
    # parameter default has to leave that panel exactly as it was.
    text = hud_text_from([measurements(1, rgb_distance_m=[3.885])], 0, None)

    assert hud_labels(text) == [ESTIMATOR_LABELS[e] for e in PUBLIC_ESTIMATOR_ORDER]


def test_the_parameter_default_resolves_to_every_registered_estimator() -> None:
    # 'all' is the declared default on the node, and the launch file hands over
    # a comma-joined list; both have to come back in canonical order.
    assert parse_estimators('all') == PUBLIC_ESTIMATOR_ORDER
    assert parse_estimators(','.join(('lidar', 'rgb'))) == ('rgb', 'lidar')


# --- the same set decides the rings -----------------------------------------
#
# The panel and the floor plan are one claim in two renderings, so a ring with
# no row beside it to name it is unreadable -- the colour is the only label a
# ring has. Filtering both from the one set makes them agree by construction.


def _published_markers(
    same_batch: list,
    estimators: tuple[str, ...] | None = None,
    robot_yaw: float = 0.0,
) -> list:
    """Markers from the real ``_publish_markers``, with no ROS context.

    Unbound against a stub, the same discipline ``_place`` uses: the TF lookup
    and the message cache are the only things replaced, so the selection, the
    ranking and the bearing borrow are all the shipped ones.
    """

    published: list = []
    stub = SimpleNamespace(
        marker_lifetime=1.5,
        estimators=PUBLIC_ESTIMATOR_ORDER if estimators is None else estimators,
        _partitioned_messages=lambda: (same_batch, []),
        _robot_pose_in_world=lambda _time: (0.0, 0.0, robot_yaw, 'map'),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=TimeMsg)),
        _pub=SimpleNamespace(publish=lambda ma: published.extend(ma.markers)),
    )
    stub._add_estimator_markers = MethodType(
        G1EstimateVizNode._add_estimator_markers, stub)

    G1EstimateVizNode._publish_markers(stub)
    return published


def _marker_estimators(markers: list) -> set[str]:
    return {marker.ns.removeprefix('g1_estimates/') for marker in markers}


def test_only_the_selected_estimators_get_a_ring() -> None:
    same_batch = [measurements(
        1,
        rgb_forward_m=[3.8], rgb_lateral_m=[0.4], rgb_distance_m=[3.821],
        lidar_forward_m=[2.4], lidar_lateral_m=[0.3], lidar_distance_m=[2.419],
    )]

    markers = _published_markers(same_batch, estimators=('lidar',))

    assert _marker_estimators(markers) == {'lidar'}


def test_an_unselected_estimator_that_is_publishing_draws_nothing() -> None:
    # The mask-row case: the mask node carries all three status arrays, so a
    # field filled for a path the run did not select would otherwise get a ring
    # that no HUD row explains.
    same_batch = [measurements(
        1,
        polar_profiling_forward_m=[2.45], polar_profiling_lateral_m=[0.12],
        polar_profiling_distance_m=[2.455],
        projective_ranging_forward_m=[2.42], projective_ranging_lateral_m=[0.10],
        projective_ranging_distance_m=[2.431],
    )]

    markers = _published_markers(same_batch, estimators=('polar_profiling',))

    assert _marker_estimators(markers) == {'polar_profiling'}
    assert not [m for m in markers if 'projective_ranging' in m.ns]


def test_omitting_the_set_draws_every_estimator_that_reported() -> None:
    # The exploration entrypoint again: nothing selected, nothing filtered.
    same_batch = [measurements(
        1,
        rgb_forward_m=[3.8], rgb_lateral_m=[0.4], rgb_distance_m=[3.821],
        lidar_forward_m=[2.4], lidar_lateral_m=[0.3], lidar_distance_m=[2.419],
        sensor_depth_distance_m=[2.401],
    )]

    markers = _published_markers(same_batch)

    assert _marker_estimators(markers) == {'rgb', 'lidar', 'sensor_depth'}


def test_a_selected_depth_row_still_borrows_a_bearing_from_an_unselected_one() -> None:
    # display_bearing reads every estimator that placed the detection, not just
    # the selected ones. Filtering it too would drop a depth-only ring back onto
    # the boresight -- wrong by the whole lateral component, and silently.
    # Both rows agree on the range, so any gap between the two dots below is the
    # bearing alone rather than the two ring radii differing.
    planar_m = math.hypot(2.4, 1.2)
    same_batch = [measurements(
        1,
        lidar_forward_m=[2.4], lidar_lateral_m=[1.2], lidar_distance_m=[planar_m],
        sensor_depth_distance_m=[planar_m],
    )]

    markers = _published_markers(same_batch, estimators=('sensor_depth',))

    assert _marker_estimators(markers) == {'sensor_depth'}
    borrowed = _dot_xy(markers)
    # Same distance down the same borrowed bearing, so the depth ring lands on
    # the lidar dot rather than straight ahead of the robot.
    assert borrowed == pytest.approx(
        _dot_xy(_place('lidar', 2.4, 1.2)), abs=1e-6)
    assert borrowed[1] != pytest.approx(0.0, abs=1e-3)
