from __future__ import annotations

import html
import math
import re
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PointStamped

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    ESTIMATOR_LABELS,
    ESTIMATOR_POSITION_ATTRS,
    ESTIMATOR_SHORT_LABELS,
    MASK_ESTIMATORS,
    PUBLIC_ESTIMATOR_ORDER,
    nearest_instance_index,
    parse_estimators,
)
from ridgeback_autonomy.perception.target_localization.hud_rendering import (
    HUD_AGED_LUMINANCE,
    HUD_LAYOUTS,
    HUD_LAYOUT_ROWS,
    HUD_MIN_LUMINANCE,
    HUD_WIDE_CELL_COLUMNS,
    colour_at_luminance,
    hud_line as _hud_line,
    hud_section_text,
    hud_text_colour,
    hud_truth_header,
    hud_wide_section_text,
    hud_wide_text,
    parse_hud_layout,
)
from ridgeback_autonomy.perception.target_localization.marker_rendering import (
    append_estimator_markers,
    world_marker_point,
)
from ridgeback_autonomy.perception.target_localization.visualization_node import (
    MARKER_LIFETIME_SEC,
    MAX_OBSERVATION_AGE_S,
    TargetVisualizationNode,
)
from ridgeback_autonomy.perception.target_localization.visualization_readings import (
    batch_messages,
    collect_readings,
    estimator_reading,
    nearest_detection_index,
    partition_measurements,
)
from ridgeback_autonomy.perception.target_localization.visualization_style import (
    ESTIMATOR_COLOURS,
    ESTIMATOR_MARKER_ID_BASES,
    RING_LINE_WIDTH_M,
)
from ridgeback_autonomy.msg import TargetMeasurements
from visualization_msgs.msg import Marker
from ridgeback_autonomy.perception.target_localization.core.vehicle_frame import (
    ROBOT_FRONT_OFFSET_M,
    planar_measurement_from_vehicle_front,
)
from ridgeback_autonomy.perception.target_localization.ground_truth import TRUTH_MAX_AGE_S, truth_reading


def test_every_registered_estimator_has_a_colour() -> None:
    # A ring with no colour raises a KeyError mid-callback, so the registry and
    # the palette have to stay in step -- including the three mask paths, which
    # had no entry before and therefore drew nothing.
    assert set(ESTIMATOR_COLOURS) == set(PUBLIC_ESTIMATOR_ORDER)


def test_every_estimator_reports_a_planar_position() -> None:
    # Licenses the removal of the borrowed-bearing path: a distance-only
    # estimator would silently draw its ring on the boresight.
    assert set(ESTIMATOR_POSITION_ATTRS) == set(PUBLIC_ESTIMATOR_ORDER)


def test_mask_and_pointcloud_families_are_split_by_colour_temperature() -> None:
    # Nothing on screen labels the two families, so the warm/cold split is the
    # whole of the distinction: a non-mask row that drifted cold would silently
    # read as a mask path. Red over blue is warm, blue over red cold.
    for estimator, (red, _, blue, _) in ESTIMATOR_COLOURS.items():
        if estimator in MASK_ESTIMATORS:
            assert blue > red, f'{estimator} is a mask path but reads warm'
        else:
            assert red > blue, f'{estimator} is not a mask path but reads cold'


def test_marker_id_bases_are_distinct() -> None:
    bases = [ESTIMATOR_MARKER_ID_BASES[name] for name in PUBLIC_ESTIMATOR_ORDER]

    assert len(set(bases)) == len(bases)
    # An estimator owns a ring at its base and a dot at base + 1, so two bases
    # one apart would have one estimator overwriting another's dot.
    assert min(abs(a - b) for a in bases for b in bases if a != b) >= 2


def test_mask_estimator_positions_are_read_from_the_message() -> None:
    msg = TargetMeasurements()
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
    msg = TargetMeasurements()
    msg.detected = True
    msg.count = 1

    assert estimator_reading(msg, 'pointcloud', 0) == (None, None, None)


def test_nan_slots_read_as_none() -> None:
    msg = TargetMeasurements()
    msg.detected = True
    msg.count = 1
    msg.pointcloud_distance_m = [float('nan')]

    assert estimator_reading(msg, 'pointcloud', 0)[2] is None


def reader_from(table):
    """``(estimator, index) -> distance`` from a ``{index: {estimator: value}}``."""

    return lambda estimator, index: table.get(index, {}).get(estimator)


def test_nearest_instance_index_picks_the_closest_detection() -> None:
    table = {0: {'pointcloud': 4.2}, 1: {'pointcloud': 2.1}}

    assert nearest_instance_index(2, reader_from(table)) == 1


def test_canonical_order_decides_which_estimator_speaks_for_a_detection() -> None:
    # Detection 1 holds the smallest number in the frame, but only on an
    # estimator that never gets consulted: pointcloud answers for both
    # detections, so detection 0 is the nearer one. Ranking by the minimum
    # across estimators instead would flip this, and would flip it differently
    # in a node that does not compute pointcloud.
    table = {
        0: {'pointcloud': 1.0, 'polar_profiling': 5.0},
        1: {'pointcloud': 2.0, 'polar_profiling': 0.5},
    }

    assert nearest_instance_index(2, reader_from(table)) == 0


def test_equal_distances_tie_break_to_the_lower_index() -> None:
    table = {0: {'pointcloud': 3.0}, 1: {'pointcloud': 3.0}}

    assert nearest_instance_index(2, reader_from(table)) == 0


def test_a_frame_no_estimator_placed_ranks_to_nothing() -> None:
    assert nearest_instance_index(2, reader_from({})) is None


def measurements(count: int, *, stamp_ns: int = 0, **fields) -> TargetMeasurements:
    msg = TargetMeasurements()
    msg.detected = count > 0
    msg.count = count
    msg.header.stamp = TimeMsg(
        sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    for name, values in fields.items():
        setattr(msg, name, values)
    return msg


def test_nearest_detection_index_ranks_across_every_producer() -> None:
    # The pointcloud node fills pointcloud, the mask node fills polar profiling,
    # and the indices are the same detections in both. Detection 1 is closer.
    pointcloud = measurements(2, pointcloud_distance_m=[4.0, 2.0])
    mask = measurements(2, polar_profiling_distance_m=[4.1, 2.2])

    assert nearest_detection_index([pointcloud, mask]) == 1


def test_nearest_detection_index_falls_back_to_the_first_detection() -> None:
    # Boxes but no distances: nothing is rankable, and the frame has to keep
    # rendering its all-miss HUD rather than crash on a None index.
    assert nearest_detection_index([measurements(2)]) == 0


def luminance(colour) -> float:
    from ridgeback_autonomy.perception.target_localization.hud_rendering import (
        LUMA_WEIGHTS,
    )

    return sum(w * c for w, c in zip(LUMA_WEIGHTS, colour))


def test_every_hud_colour_clears_the_legibility_floor() -> None:
    # The HUD sits on a half-opaque black panel, so a dark ring colour is
    # unreadable as text. Every registered colour clears the floor unlifted
    # today; the lift is the guard for one added later.
    for estimator in ESTIMATOR_COLOURS:
        assert luminance(hud_text_colour(estimator)) >= HUD_MIN_LUMINANCE - 1e-9


def test_bright_colours_are_left_exactly_alone() -> None:
    # The pointcloud amber is already well above the floor; lifting it would
    # drift the text away from the ring for no reason.
    assert hud_text_colour('pointcloud') == ESTIMATOR_COLOURS['pointcloud'][:3]


def test_lift_preserves_hue_ordering_so_rings_stay_identifiable() -> None:
    # Blending toward white must not reorder the channels, or a lifted colour no
    # longer reads as the same estimator as its ring. Asserted on a synthetic
    # dark colour, since no registered one is currently below the floor.
    dark = (0.4, 0.0, 0.2)
    red, green, blue = colour_at_luminance(dark, HUD_MIN_LUMINANCE)

    assert red > blue > green


# --- aged rows --------------------------------------------------------------


def test_every_aged_colour_lands_on_the_aged_luminance() -> None:
    # One target for every row, not a dim multiplier: a colour sitting AT the
    # legibility floor after its lift would be handed back its fresh appearance
    # by a multiply-then-clamp, and the state would be invisible on exactly the
    # rows that most need it.
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
    red, green, blue = hud_text_colour('polar_profiling', aged=True)

    assert blue > red > green


def test_dimming_preserves_saturation_not_just_hue() -> None:
    # Scaling all three channels by one factor holds their ratios exactly, so a
    # dimmed row is the same colour rather than a washed-out one. Blending toward
    # black would do the same; blending toward grey would not.
    original = ESTIMATOR_COLOURS['euclidean_reconstruction'][:3]
    dimmed = hud_text_colour('euclidean_reconstruction', aged=True)

    scales = [d / o for d, o in zip(dimmed, original) if o != 0.0]
    assert scales == pytest.approx([scales[0]] * len(scales), abs=1e-9)


def test_colour_at_luminance_lifts_a_colour_that_starts_below_the_target() -> None:
    # A colour under the aged target gets BRIGHTER rather than darker -- the
    # reason this is a target and not a dimming. No registered colour is that
    # dark today, so the case is asserted on a synthetic one.
    dark = (0.2, 0.0, 0.1)
    lifted = colour_at_luminance(dark, HUD_AGED_LUMINANCE)

    assert luminance(lifted) == pytest.approx(HUD_AGED_LUMINANCE, abs=1e-9)
    assert luminance(lifted) > luminance(dark)


def test_colour_at_luminance_leaves_a_colour_already_on_target_untouched() -> None:
    colour = ESTIMATOR_COLOURS['pointcloud'][:3]

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
) -> list:
    """Markers from the shipped construction helper, with no node to stand up."""

    markers: list = []
    append_estimator_markers(
        markers,
        estimator,
        forward_m,
        lateral_m,
        robot_x,
        robot_y,
        robot_yaw,
        TimeMsg(),
        'map',
        1.5,
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
    markers = _place(
        'polar_profiling', forward_m, lateral_m, robot_x, robot_y, robot_yaw)

    assert _dot_xy(markers) == pytest.approx((target_x, target_y), abs=1e-9)


def test_ring_is_referenced_to_the_robot_front_not_the_base_origin() -> None:
    # A robot facing +X: a 3.00 m reading is 3.25 m from the base origin, since
    # the producer already subtracted the front offset from it.
    markers = _place('pointcloud', 3.0, 0.0)

    assert _dot_xy(markers) == pytest.approx((3.0 + ROBOT_FRONT_OFFSET_M, 0.0), abs=1e-9)


def test_the_front_offset_turns_with_the_robot() -> None:
    # Same reading with the robot facing +Y. The offset has to rotate with the
    # base, so it lands on the y axis -- adding it in the world frame would
    # leave the dot off-axis and still pass the yaw=0 case above.
    markers = _place('pointcloud', 3.0, 0.0, robot_yaw=math.pi / 2.0)

    assert _dot_xy(markers) == pytest.approx((0.0, 3.0 + ROBOT_FRONT_OFFSET_M), abs=1e-9)


def test_a_row_that_placed_nothing_draws_nothing() -> None:
    assert _place('pointcloud', None, None) == []


def test_every_ring_is_drawn_at_the_one_line_width() -> None:
    # There is no second width any more: the thin ring marked a borrowed
    # bearing, and every registered estimator now measures its own.
    ring, _ = _place('polar_profiling', 2.0, 0.1)

    assert ring.scale.x == RING_LINE_WIDTH_M


def test_world_marker_point_is_a_plain_rotation_about_the_given_origin() -> None:
    # Guards the convention the placement rests on: +forward is base +X and
    # +lateral is base +Y (left-positive, REP-103), not the other way round.
    assert world_marker_point(1.0, 0.0, 0.0, 0.0, math.pi / 2.0) == pytest.approx(
        (0.0, 1.0), abs=1e-9)
    assert world_marker_point(0.0, 1.0, 0.0, 0.0, 0.0) == pytest.approx(
        (0.0, 1.0), abs=1e-9)


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
        'TARGET DISTANCES   truth 3.250 m  [bed_occluder_single]')


def test_hud_truth_header_drops_the_truth_entirely_when_it_has_expired() -> None:
    assert hud_truth_header(None) == 'TARGET DISTANCES'


def test_hud_truth_header_omits_empty_brackets_for_an_unnamed_trial() -> None:
    reading = truth_reading(_truth_message(2.25, '', 0), 0)

    assert hud_truth_header(reading) == 'TARGET DISTANCES   truth 2.250 m'


FRESH_AGE_S = 0.1


def dated(same_batch: list, age_s: float = FRESH_AGE_S) -> list:
    """Plain messages as the ``(message, age_seconds)`` entries the HUD takes.

    ``partition_measurements`` returns an age for every live message, not only
    the ones that fell out of the current batch: a same-batch reading's age is
    the pipeline latency behind that row, which the wide layout prints.
    """

    return [(msg, age_s) for msg in same_batch]


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
    what a caller with no run configuration to hand gets.
    """

    if estimators is None:
        return hud_section_text(dated(same_batch), aged or [], nearest, truth)
    return hud_section_text(dated(same_batch), aged or [], nearest, truth, estimators)


def hud_wide_text_from(
    same_batch: list,
    nearest: int = 0,
    aged: list | None = None,
    estimators: tuple[str, ...] | None = None,
    fresh_age_s: float = FRESH_AGE_S,
) -> str:
    """The wide renderer, same discipline. No truth argument: it has no truth row."""

    entries = dated(same_batch, fresh_age_s)
    if estimators is None:
        return hud_wide_section_text(entries, aged or [], nearest)
    return hud_wide_section_text(entries, aged or [], nearest, estimators)


def test_hud_scores_every_row_against_the_truth_it_displays() -> None:
    fresh = [measurements(
        1, pointcloud_distance_m=[3.885], polar_profiling_distance_m=[1.050])]
    truth = truth_reading(_truth_message(3.25, 'bed_occluder_single', 0), 0)

    text = hud_text_from(fresh, 0, truth)

    assert 'truth&nbsp;3.250&nbsp;m&nbsp;&nbsp;[bed_occluder_single]' in text
    # 3.885 - 3.25 and 1.050 - 3.25, off the SAME trial's truth.
    assert '+0.635' in text
    assert '-2.200' in text


def test_hud_drops_the_error_column_when_the_truth_has_expired() -> None:
    # No truth means no reference, and a distance printed beside a stale error
    # is worse than one printed beside none.
    fresh = [measurements(1, pointcloud_distance_m=[3.885])]

    text = hud_text_from(fresh, 0, None)

    assert 'truth' not in text
    assert '+0.635' not in text
    assert '3.885' in text


# --- freshness: liveness, validity and the two row states -------------------
#
# The mask rows used to blink: their message is stamped with the DETECTION
# instant, so inference, segmentation and the depth lookup are all charged
# against it, and a single stamp gate against the marker lifetime threw the
# reading away while the pointcloud row -- same stamp, no pipeline behind it --
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

    assert batch_messages(same_batch) == [lagging]
    # Two seconds of pipeline latency, reported as such rather than discarded.
    assert same_batch[0][1] == pytest.approx(2.0, abs=1e-9)
    assert aged == []


def test_a_message_received_long_ago_is_dropped_however_recent_its_stamp() -> None:
    # Liveness is the receipt question: this producer has said nothing for two
    # lifetimes, so its last word is not evidence that it is still running.
    now = 100 * SECOND_NS
    quiet = measurements(1, stamp_ns=now, pointcloud_distance_m=[2.4])

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
    pointcloud = measurements(1, stamp_ns=now, pointcloud_distance_m=[2.4])
    projective = measurements(
        1, stamp_ns=now, projective_ranging_distance_m=[2.5])
    lagging_mask = measurements(1, stamp_ns=now - int(1.2 * SECOND_NS),
                                polar_profiling_distance_m=[2.6])

    same_batch, aged = partition_measurements(
        [cached(pointcloud, now), cached(projective, now), cached(lagging_mask, now)],
        now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert batch_messages(same_batch) == [pointcloud, projective]
    assert batch_messages(aged) == [lagging_mask]
    assert aged[0][1] == pytest.approx(1.2, abs=1e-9)


def test_a_lone_lagging_producer_is_its_own_current_batch() -> None:
    # A mask-only run has nothing newer to contradict it, so its rings are still
    # the best claim available and must keep being drawn.
    now = 100 * SECOND_NS
    mask = measurements(1, stamp_ns=now - int(1.2 * SECOND_NS),
                        polar_profiling_distance_m=[2.6])

    same_batch, aged = partition_measurements(
        [cached(mask, now)], now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert batch_messages(same_batch) == [mask]
    assert aged == []


def test_an_empty_cache_partitions_to_nothing() -> None:
    assert partition_measurements(
        [None, None], 0, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S) == ([], [])


def test_an_aged_reading_renders_with_an_age_column_instead_of_a_miss() -> None:
    # The whole point: the mask row keeps its number and says how old it is,
    # rather than being repainted as "-- miss" by the pointcloud row's stamp.
    pointcloud = measurements(1, pointcloud_distance_m=[3.885])
    mask = measurements(1, polar_profiling_distance_m=[2.431])

    text = hud_text_from([pointcloud], 0, None, aged=[(mask, 1.2)])

    assert '2.431' in text
    assert '1.2s' in text
    assert 'Polar&nbsp;Profiling' in text
    # The pointcloud row is current and carries no age column.
    assert '3.885' in text


def test_an_aged_row_is_coloured_at_the_aged_luminance() -> None:
    mask = measurements(1, polar_profiling_distance_m=[2.431])
    dimmed = hud_text_colour('polar_profiling', aged=True)
    expected = ', '.join(str(int(round(channel * 255))) for channel in dimmed)

    text = hud_text_from([measurements(1, pointcloud_distance_m=[3.885])], 0, None,
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
    pointcloud = measurements(2, pointcloud_distance_m=[1.0, 6.0])
    mask = measurements(2, polar_profiling_distance_m=[5.0, 2.0])

    text = hud_text_from([pointcloud], 0, None, aged=[(mask, 1.2)])

    assert '2.000' in text
    assert '5.000' not in text


def test_a_nan_reading_still_renders_a_miss_distinct_from_an_aged_one() -> None:
    # "miss" has to mean the estimator ran and returned nothing, or the word is
    # a lie about the estimator every time the HUD is merely out of date.
    missed = hud_text_from(
        [measurements(1, pointcloud_distance_m=[float('nan')])], 0, None)
    aged = hud_text_from(
        [measurements(1, pointcloud_distance_m=[2.0])], 0, None,
        aged=[(measurements(1, polar_profiling_distance_m=[2.431]), 1.2)])

    assert 'miss' in missed
    assert 'nan' not in missed.lower()
    assert 'miss' in aged and '2.431' in aged and '1.2s' in aged


def test_an_aged_message_never_joins_the_batch_ranking() -> None:
    # An aged message does get a ring now, but never off the BATCH's index: it
    # comes from an earlier detector batch, so its index 0 may be a different
    # robot. It is ranked against itself instead, which is what keeps the two
    # sets of indices from being silently mixed.
    now = 100 * SECOND_NS
    # The pointcloud row placed only detection 0 this batch; the aged mask
    # message has its own detection 1 much nearer.
    pointcloud = measurements(
        2, stamp_ns=now, pointcloud_distance_m=[1.0, float('nan')])
    mask = measurements(2, stamp_ns=now - int(1.2 * SECOND_NS),
                        polar_profiling_distance_m=[9.0, 0.5])

    same_batch, aged = partition_measurements(
        [cached(pointcloud, now), cached(mask, now)],
        now, MARKER_LIFETIME_SEC, MAX_OBSERVATION_AGE_S)

    assert batch_messages(aged) == [mask]
    assert nearest_detection_index(batch_messages(same_batch)) == 0
    # Folding the aged message into the batch ranking would move the pointcloud
    # ring onto a robot no current message measured -- which is what the split
    # exists to prevent. The aged message's own ranking picks that robot for its
    # own ring, and only for its own.
    assert nearest_detection_index(batch_messages(same_batch + aged)) == 1
    assert nearest_detection_index(batch_messages(aged)) == 1


def test_the_hud_renders_nothing_at_all_when_nothing_is_cached() -> None:
    # hud_node drops falsy panels, so the empty string collapses the section.
    # Returning early instead is what latched the last numbers on screen under a
    # floor plan whose rings had all correctly expired.
    assert hud_section_text([], [], 0, None) == ''


def test_the_hud_still_renders_when_the_truth_has_gone_but_readings_have_not() -> None:
    # The timer re-evaluates the truth gate with no measurement traffic needed;
    # the rows outlive the truth line rather than disappearing with it.
    text = hud_section_text(
        dated([measurements(1, pointcloud_distance_m=[3.885])]), [], 0, None)

    assert text.count('<br/>') == len(PUBLIC_ESTIMATOR_ORDER)
    assert 'TARGET&nbsp;DISTANCES' in text
    assert 'truth' not in text


# --- the run's estimator set decides the rows -------------------------------
#
# A row for a path the run never launched can only ever print "-- miss", which
# is the same thing the panel prints for an estimator that ran and found
# nothing -- so the two are indistinguishable unless the panel is filtered to
# the set the run actually launched.


def hud_labels(text: str) -> list[str]:
    """The estimator label from each row, in the order the panel prints them.

    Every row body starts with the label in a fixed 24-column field
    (``_reading_body``, and the miss branch beside it), so the label is that
    slice once the span markup is stripped and the ``&nbsp;`` padding is turned
    back into ordinary spaces. Splitting on whitespace instead would cut
    two-word labels like ``Point Cloud`` in half.
    """

    rows = text.split('<br/>')[1:]  # drop the truth header
    return [
        html.unescape(re.sub(r'<[^>]+>', '', row))
        .replace('\xa0', ' ')[:24]
        .strip()
        for row in rows
    ]


def test_only_the_selected_estimators_get_a_row() -> None:
    same_batch = [measurements(1, pointcloud_distance_m=[2.430])]

    text = hud_text_from(
        same_batch, 0, None, estimators=('pointcloud', 'polar_profiling'))

    assert hud_labels(text) == ['Point Cloud', 'Polar Profiling']


def test_rows_keep_canonical_order_however_the_argument_named_them() -> None:
    # parse_estimators sorts into PUBLIC_ESTIMATOR_ORDER, so "polar,pointcloud"
    # and "pointcloud,polar" have to render identically -- the HUD's order is
    # the registry's, not the launch line's.
    selected = parse_estimators('polar_profiling,pointcloud,projective_ranging')
    same_batch = [measurements(
        1, pointcloud_distance_m=[3.885], projective_ranging_distance_m=[2.430])]

    text = hud_text_from(same_batch, 0, None, estimators=selected)

    assert hud_labels(text) == [
        'Point Cloud', 'Projective Ranging', 'Polar Profiling']


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
    same_batch = [measurements(1, pointcloud_distance_m=[2.430])]

    text = hud_text_from(
        same_batch, 0, None, estimators=('pointcloud', 'polar_profiling'))

    assert hud_labels(text) == ['Point Cloud', 'Polar Profiling']
    assert 'miss' in text


def test_an_aged_row_keeps_its_age_column_when_the_set_is_restricted() -> None:
    same_batch = [measurements(1, pointcloud_distance_m=[2.430])]
    aged = [(measurements(1, polar_profiling_distance_m=[2.455]), 1.2)]

    text = hud_text_from(
        same_batch, 0, None, aged=aged,
        estimators=('pointcloud', 'polar_profiling'))

    assert '2.455' in text
    assert '1.2s' in text


def test_omitting_the_set_renders_every_registered_estimator() -> None:
    # The exploration entrypoint declares no estimators argument at all, so the
    # parameter default has to leave that panel exactly as it was.
    text = hud_text_from([measurements(1, pointcloud_distance_m=[3.885])], 0, None)

    assert hud_labels(text) == [ESTIMATOR_LABELS[e] for e in PUBLIC_ESTIMATOR_ORDER]


def test_the_parameter_default_resolves_to_every_registered_estimator() -> None:
    # 'all' is the declared default on the node, and the launch file hands over
    # a comma-joined list; both have to come back in canonical order.
    assert parse_estimators('all') == PUBLIC_ESTIMATOR_ORDER
    assert parse_estimators(','.join(('polar_profiling', 'pointcloud'))) == (
        'pointcloud', 'polar_profiling')


# --- the wide layout --------------------------------------------------------
#
# Exploration's shape: estimator names across the top, distances under them,
# ages under those. No truth source there, so no truth line and no error column
# -- and, unlike the row layout, an age on every column rather than only on the
# ones that fell out of the current batch.


def wide_cells(text: str) -> list[list[str]]:
    """Each line's cells with their padding intact, markup stripped.

    One ``<span>`` per cell is what lets a column keep its own ring colour, so
    the spans are also the cell boundaries -- splitting on whitespace would
    merge a blank age cell into its neighbour and hide exactly the case the
    ``--`` column exists to show.
    """

    return [
        [
            html.unescape(re.sub(r'<[^>]+>', '', cell)).replace('\xa0', ' ')
            for cell in re.findall(r'<span[^>]*>.*?</span>', line)
        ]
        for line in text.split('<br/>')
    ]


def wide_row(text: str, index: int) -> list[str]:
    return [cell.strip() for cell in wide_cells(text)[index]]


def wide_colours(text: str) -> list[list[str]]:
    return [re.findall(r'rgb\([^)]*\)', line) for line in text.split('<br/>')]


def test_every_registered_estimator_has_a_column_header() -> None:
    # A missing short label is a KeyError mid-render, and the wide layout is the
    # exploration HUD's only shape -- the panel would simply never appear.
    assert set(ESTIMATOR_SHORT_LABELS) == set(PUBLIC_ESTIMATOR_ORDER)
    for estimator in PUBLIC_ESTIMATOR_ORDER:
        assert len(ESTIMATOR_SHORT_LABELS[estimator]) <= HUD_WIDE_CELL_COLUMNS


def test_the_wide_layout_is_names_then_distances_then_ages() -> None:
    same_batch = [measurements(
        1,
        pointcloud_distance_m=[3.310],
        projective_ranging_distance_m=[3.262],
        euclidean_reconstruction_distance_m=[3.255],
        polar_profiling_distance_m=[3.180],
    )]

    text = hud_wide_text_from(same_batch, fresh_age_s=0.14)

    assert text.count('<br/>') == 2
    assert wide_row(text, 0) == [
        ESTIMATOR_SHORT_LABELS[e] for e in PUBLIC_ESTIMATOR_ORDER]
    assert wide_row(text, 1) == ['3.310', '3.262', '3.255', '3.180']
    assert wide_row(text, 2) == ['0.1s', '0.1s', '0.1s', '0.1s']


def test_a_fresh_column_carries_an_age_too() -> None:
    # The row layout prints an age only for an aged reading. Here every column
    # has one, because a column with a blank age is how a miss is shown -- a
    # fresh reading left blank would be indistinguishable from one.
    text = hud_wide_text_from(
        [measurements(1, pointcloud_distance_m=[3.310])],
        estimators=('pointcloud',), fresh_age_s=1.24)

    assert wide_row(text, 2) == ['1.2s']


def test_an_aged_column_prints_its_own_age_and_dims_whole() -> None:
    # The header dims with the number: the column is what is stale, and a bright
    # header over a dimmed reading reads as the layout, not as the state.
    fresh = [measurements(1, pointcloud_distance_m=[3.310])]
    aged = [(measurements(1, polar_profiling_distance_m=[3.180]), 1.2)]
    dim = hud_text_colour('polar_profiling', aged=True)
    expected = 'rgb(' + ', '.join(
        str(int(round(channel * 255))) for channel in dim) + ')'

    text = hud_wide_text_from(
        fresh, aged=aged, estimators=('pointcloud', 'polar_profiling'))

    assert wide_row(text, 1) == ['3.310', '3.180']
    assert wide_row(text, 2) == ['0.1s', '1.2s']
    # Header, distance and age of that one column, all at the aged luminance.
    assert [line[1] for line in wide_colours(text)] == [expected] * 3


def test_a_column_nothing_reported_prints_a_dash_over_a_blank_age() -> None:
    text = hud_wide_text_from(
        [measurements(1, pointcloud_distance_m=[3.310])],
        estimators=('pointcloud', 'polar_profiling'))

    assert wide_row(text, 1) == ['3.310', '--']
    assert wide_row(text, 2) == ['0.1s', '']


def test_a_missing_column_keeps_its_fresh_colour_not_the_aged_one() -> None:
    # A miss is not a stale reading: nothing about it is out of date, and
    # dimming it would spend the one signal the layout has for staleness.
    text = hud_wide_text_from(
        [measurements(1, pointcloud_distance_m=[3.310])],
        estimators=('polar_profiling',))

    fresh = hud_text_colour('polar_profiling')
    expected = 'rgb(' + ', '.join(
        str(int(round(channel * 255))) for channel in fresh) + ')'
    assert wide_colours(text)[0] == [expected]


def test_wide_columns_are_one_fixed_width_so_the_panel_cannot_ragged() -> None:
    # Rich text collapses runs of spaces, so the columns line up only because
    # every cell is padded to the same width with &nbsp;. A cell wider than the
    # rest also pushes the last column past overlay_width, where it is clipped
    # rather than wrapped -- which looks like that estimator never reporting.
    text = hud_wide_text_from(
        [measurements(1, pointcloud_distance_m=[12.345])],
        fresh_age_s=12.3)

    for line in wide_cells(text):
        assert [len(cell) for cell in line] == (
            [HUD_WIDE_CELL_COLUMNS] * len(PUBLIC_ESTIMATOR_ORDER))


def test_the_wide_layout_renders_nothing_at_all_when_nothing_is_cached() -> None:
    # Same load-bearing contract as the row layout: hud_node drops falsy panels,
    # and a latched panel of numbers under expired rings reads as a broken ring
    # layer rather than as a dead detector.
    assert hud_wide_section_text([], [], 0) == ''


def test_wide_columns_follow_the_selected_set_in_canonical_order() -> None:
    selected = parse_estimators('polar_profiling,pointcloud')
    same_batch = [measurements(
        1, pointcloud_distance_m=[3.310], projective_ranging_distance_m=[3.262])]

    text = hud_wide_text_from(same_batch, estimators=selected)

    assert wide_row(text, 0) == ['Cloud', 'Polar']
    assert '3.262' not in text


def test_the_hud_layout_parameter_default_is_the_row_layout() -> None:
    # The benchmark passes nothing and must keep the layout with the truth and
    # error columns; only exploration asks for the other one.
    assert parse_hud_layout(HUD_LAYOUT_ROWS) == HUD_LAYOUT_ROWS
    assert parse_hud_layout('') == HUD_LAYOUT_ROWS
    assert parse_hud_layout(None) == HUD_LAYOUT_ROWS
    assert set(HUD_LAYOUTS) == {'rows', 'wide'}


def test_an_unknown_hud_layout_is_rejected_rather_than_defaulted() -> None:
    # Falling back would ship the row layout into a panel sized for columns,
    # which looks like a clipping fault rather than a misspelled parameter.
    with pytest.raises(ValueError):
        parse_hud_layout('columns')


# --- the same set decides the rings -----------------------------------------
#
# The panel and the floor plan are one claim in two renderings, so a ring with
# no row beside it to name it is unreadable -- the colour is the only label a
# ring has. Filtering both from the one set makes them agree by construction.


def _published_markers(
    same_batch: list,
    estimators: tuple[str, ...] | None = None,
    robot_yaw: float = 0.0,
    aged: list | None = None,
    pose_at=None,
) -> list:
    """Markers from the real ``_publish_markers``, with no ROS context.

    Unbound against a stub, the same discipline ``_place`` uses: the TF lookup
    and the message cache are the only things replaced, so the selection and the
    ranking are the shipped ones.

    ``pose_at`` receives the ``rclpy.time.Time`` the node asked TF for, so a test
    can return a different pose per stamp -- which is the whole point of placing
    an aged reading against the pose it was taken from. It defaults to one fixed
    pose regardless of stamp.
    """

    published: list = []
    selected = PUBLIC_ESTIMATOR_ORDER if estimators is None else estimators

    def lookup(stamp):
        if pose_at is None:
            return (0.0, 0.0, robot_yaw, 'map')
        return pose_at(stamp)

    stub = SimpleNamespace(
        marker_lifetime=1.5,
        estimators=selected,
        _robot_pose_in_world=lookup,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=TimeMsg)),
        _pub=SimpleNamespace(publish=lambda ma: published.extend(ma.markers)),
    )
    # The same snapshot the node's tick builds, so the markers under test come
    # from the shipped path rather than from a second reading of the messages.
    entries = dated(same_batch)
    aged_entries = aged or []
    nearest = (
        nearest_detection_index(batch_messages(entries)) if entries else 0
    )
    readings = collect_readings(entries, aged_entries, nearest, selected)

    TargetVisualizationNode._publish_markers(stub, readings)
    return published


def _added(markers: list) -> list:
    """Only the markers that draw something, dropping the deletes.

    Every estimator with no reading now gets an explicit DELETE so its ring
    cannot outlive its HUD column, so the raw list always names the full
    selected set.
    """

    return [marker for marker in markers if marker.action == Marker.ADD]


def _marker_estimators(markers: list) -> set[str]:
    return {marker.ns.removeprefix('target_estimates/') for marker in _added(markers)}


def _deleted_estimators(markers: list) -> set[str]:
    return {
        marker.ns.removeprefix('target_estimates/')
        for marker in markers if marker.action == Marker.DELETE
    }


def test_only_the_selected_estimators_get_a_ring() -> None:
    same_batch = [measurements(
        1,
        pointcloud_forward_m=[3.8], pointcloud_lateral_m=[0.4],
        pointcloud_distance_m=[3.821],
        polar_profiling_forward_m=[2.4], polar_profiling_lateral_m=[0.3],
        polar_profiling_distance_m=[2.419],
    )]

    markers = _published_markers(same_batch, estimators=('polar_profiling',))

    assert _marker_estimators(markers) == {'polar_profiling'}


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
        pointcloud_forward_m=[3.8], pointcloud_lateral_m=[0.4],
        pointcloud_distance_m=[3.821],
        polar_profiling_forward_m=[2.4], polar_profiling_lateral_m=[0.3],
        polar_profiling_distance_m=[2.419],
    )]

    markers = _published_markers(same_batch)

    assert _marker_estimators(markers) == {'pointcloud', 'polar_profiling'}


# --- aged messages draw rings, placed at their own stamp --------------------
#
# Excluding aged messages did not cost the mask rows an occasional ring, it cost
# them EVERY ring: a mask measurement carries the detection instant and only
# arrives after inference, segmentation and the depth lookup, so it is never the
# newest stamp while the pointcloud row is also running. Measured in
# exploration: 19 of 19 detected mask messages landed in the aged remainder.


def _mask_reading(stamp_ns: int, forward: float, lateral: float):
    return measurements(
        1, stamp_ns=stamp_ns,
        polar_profiling_forward_m=[forward],
        polar_profiling_lateral_m=[lateral],
        polar_profiling_distance_m=[math.hypot(forward, lateral)],
    )


def test_an_aged_message_still_draws_its_ring() -> None:
    # The regression this whole path exists for. The pointcloud row is current
    # and the mask row is a batch behind, which is the steady state in
    # exploration -- not an edge case.
    pointcloud = measurements(
        1, stamp_ns=2 * SECOND_NS,
        pointcloud_forward_m=[3.8], pointcloud_lateral_m=[0.4],
        pointcloud_distance_m=[3.821])
    mask = _mask_reading(1 * SECOND_NS, 2.4, 0.3)

    markers = _published_markers([pointcloud], aged=[(mask, 1.2)])

    assert _marker_estimators(markers) == {'pointcloud', 'polar_profiling'}


def test_an_aged_ring_is_placed_at_the_pose_its_measurement_came_from() -> None:
    # The robot drove 2 m along +X between the detection and the message
    # arriving. Placing the reading against the pose NOW would put the ring 2 m
    # past the target; against the pose at its own stamp it lands on the target.
    def pose_at(stamp):
        return (0.0, 0.0, 0.0, 'map') if stamp.nanoseconds == SECOND_NS \
            else (2.0, 0.0, 0.0, 'map')

    mask = _mask_reading(1 * SECOND_NS, 3.0, 0.0)

    markers = _published_markers(
        [], aged=[(mask, 1.2)], estimators=('polar_profiling',), pose_at=pose_at)

    # 3.0 m off the robot front, from an origin at x=0, not x=2.
    assert _dot_xy(markers) == pytest.approx((3.0 + ROBOT_FRONT_OFFSET_M, 0.0), abs=1e-9)


def test_the_aged_ring_turns_with_the_pose_it_was_taken_from() -> None:
    # Rotation is what makes the latest-pose shortcut worst: the robot has since
    # turned 90 degrees, so borrowing the current yaw would swing a 3 m reading
    # onto the wrong axis entirely.
    def pose_at(stamp):
        return (0.0, 0.0, 0.0, 'map') if stamp.nanoseconds == SECOND_NS \
            else (0.0, 0.0, math.pi / 2.0, 'map')

    mask = _mask_reading(1 * SECOND_NS, 3.0, 0.0)

    markers = _published_markers(
        [], aged=[(mask, 1.2)], estimators=('polar_profiling',), pose_at=pose_at)

    assert _dot_xy(markers) == pytest.approx((3.0 + ROBOT_FRONT_OFFSET_M, 0.0), abs=1e-9)


def test_a_message_whose_pose_lookup_fails_is_skipped_not_the_whole_frame() -> None:
    # An aged stamp can fall off the back of the TF buffer. That must cost that
    # one message's ring, not every ring in the frame -- the old code returned
    # early on a failed lookup because there was only ever one to do.
    pointcloud = measurements(
        1, stamp_ns=2 * SECOND_NS,
        pointcloud_forward_m=[3.8], pointcloud_lateral_m=[0.4],
        pointcloud_distance_m=[3.821])
    mask = _mask_reading(1 * SECOND_NS, 2.4, 0.3)

    def pose_at(stamp):
        if stamp.nanoseconds == SECOND_NS:
            return (None, None, None, None)
        return (0.0, 0.0, 0.0, 'map')

    markers = _published_markers([pointcloud], aged=[(mask, 1.2)], pose_at=pose_at)

    assert _marker_estimators(markers) == {'pointcloud'}


def test_an_aged_message_is_ranked_against_itself_for_rings_too() -> None:
    # Same rule the HUD applies: the aged message comes from a different
    # detector batch, so the batch's nearest index would name a different robot.
    pointcloud = measurements(
        2, stamp_ns=2 * SECOND_NS,
        pointcloud_forward_m=[1.0, float('nan')],
        pointcloud_lateral_m=[0.0, float('nan')],
        pointcloud_distance_m=[1.0, float('nan')])
    mask = measurements(
        2, stamp_ns=1 * SECOND_NS,
        polar_profiling_forward_m=[9.0, 2.0],
        polar_profiling_lateral_m=[0.0, 0.0],
        polar_profiling_distance_m=[9.0, 2.0])

    markers = _published_markers(
        [pointcloud], aged=[(mask, 1.2)], estimators=('polar_profiling',))

    # Its own index 1 (2.0 m), not the batch's index 0 (9.0 m).
    assert _dot_xy(markers) == pytest.approx((2.0 + ROBOT_FRONT_OFFSET_M, 0.0), abs=1e-9)


def test_an_estimator_with_no_reading_has_its_ring_deleted() -> None:
    # lifetime alone could not keep the two surfaces in step: it starts at the
    # last publish, so a ring outlived its own column by up to a full lifetime
    # after a producer went quiet. The delete retires it on the same tick the
    # column turns to "--".
    pointcloud = measurements(
        1, pointcloud_forward_m=[3.8], pointcloud_lateral_m=[0.4],
        pointcloud_distance_m=[3.821])

    markers = _published_markers([pointcloud])

    assert _marker_estimators(markers) == {'pointcloud'}
    assert _deleted_estimators(markers) == {
        'projective_ranging', 'euclidean_reconstruction', 'polar_profiling'}


def test_a_path_ring_and_its_column_always_agree() -> None:
    # The invariant the shared snapshot exists for: whatever the panel prints a
    # number for has a ring, and whatever has a ring is printed. Here all three
    # states are live at once -- pointcloud fresh, polar aged, the other two
    # silent -- which is exactly the exploration steady state.
    pointcloud = measurements(
        1, stamp_ns=2 * SECOND_NS,
        pointcloud_forward_m=[3.8], pointcloud_lateral_m=[0.4],
        pointcloud_distance_m=[3.821])
    mask = _mask_reading(1 * SECOND_NS, 2.4, 0.3)

    entries, aged_entries = dated([pointcloud]), [(mask, 1.2)]
    readings = collect_readings(entries, aged_entries, 0, PUBLIC_ESTIMATOR_ORDER)

    markers = _published_markers([pointcloud], aged=aged_entries)
    text = hud_wide_text(readings, PUBLIC_ESTIMATOR_ORDER)

    ringed = _marker_estimators(markers)
    printed = {
        estimator
        for estimator, cell in zip(PUBLIC_ESTIMATOR_ORDER, wide_row(text, 1))
        if cell != '--'
    }

    assert ringed == printed == {'pointcloud', 'polar_profiling'}
    # And the silent two are retired from both surfaces, not merely absent.
    assert _deleted_estimators(markers) == {
        'projective_ranging', 'euclidean_reconstruction'}


def test_a_row_with_only_a_distance_draws_no_ring() -> None:
    # No borrowed bearing any more: a row that published a distance but no
    # position has no direction of its own, and a ring on the boresight would be
    # wrong by the whole lateral component with nothing marking it as a guess.
    same_batch = [measurements(
        1,
        pointcloud_forward_m=[2.4], pointcloud_lateral_m=[1.2],
        pointcloud_distance_m=[math.hypot(2.4, 1.2)],
        polar_profiling_distance_m=[math.hypot(2.4, 1.2)],
    )]

    markers = _published_markers(same_batch)

    assert _marker_estimators(markers) == {'pointcloud'}
