from __future__ import annotations

import pytest

from ridgeback_autonomy.benchmarking.estimators import PUBLIC_ESTIMATOR_ORDER
from ridgeback_autonomy.benchmarking.g1_estimate_viz_node import (
    ESTIMATOR_COLOURS,
    HUD_MIN_LUMINANCE,
    _ESTIMATOR_ID_BASE,
    _hud_line,
    _LUMA_WEIGHTS,
    estimator_reading,
    hud_text_colour,
)
from ridgeback_autonomy.msg import G1Measurements


def test_every_registered_estimator_has_a_colour() -> None:
    # A ring with no colour raises a KeyError mid-callback, so the registry and
    # the palette have to stay in step -- including the three mask paths, which
    # had no entry before and therefore drew nothing.
    assert set(ESTIMATOR_COLOURS) == set(PUBLIC_ESTIMATOR_ORDER)


def test_marker_id_bases_are_distinct_and_leave_room_for_instances() -> None:
    bases = [_ESTIMATOR_ID_BASE[name] for name in PUBLIC_ESTIMATOR_ORDER]

    assert len(set(bases)) == len(bases)
    # Each estimator burns 2 ids per detection, so the spacing caps instances
    # per frame; 100 is far above anything a scene produces.
    assert min(abs(a - b) for a in bases for b in bases if a != b) >= 100


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


def luminance(colour) -> float:
    return sum(w * c for w, c in zip(_LUMA_WEIGHTS, colour))


def test_every_hud_colour_clears_the_legibility_floor() -> None:
    # The HUD sits on a half-opaque black panel, so a dark ring colour is
    # unreadable as text. Red and depth_anything's purple are the ones that fail
    # without the lift.
    for estimator in ESTIMATOR_COLOURS:
        assert luminance(hud_text_colour(estimator)) >= HUD_MIN_LUMINANCE - 1e-9


def test_bright_colours_are_left_exactly_alone() -> None:
    # lidar (0, 0.9, 0) is already well above the floor; lifting it would drift
    # the text away from the ring for no reason.
    assert hud_text_colour('lidar') == ESTIMATOR_COLOURS['lidar'][:3]


def test_lift_preserves_hue_ordering_so_rings_stay_identifiable() -> None:
    # Blending toward white must not reorder the channels -- purple that came
    # out blue-dominant has to stay blue-dominant, or it no longer reads as the
    # same estimator as its ring.
    red, green, blue = hud_text_colour('depth_anything')
    original_r, original_g, original_b = ESTIMATOR_COLOURS['depth_anything'][:3]

    assert blue > red > green
    assert (original_b > original_r > original_g)


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
    # sensor_depth and depth_anything are scalar ranges along the boresight;
    # they intentionally have no entry in the position registry.
    msg = G1Measurements()
    msg.detected = True
    msg.count = 1
    msg.sensor_depth_distance_m = [2.5]

    forward, lateral, distance = estimator_reading(msg, 'sensor_depth', 0)

    assert (forward, lateral) == (None, None)
    assert distance == 2.5
