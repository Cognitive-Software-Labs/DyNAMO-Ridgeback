from __future__ import annotations

import math

from ridgeback_localization.core.vehicle_frame import (
    ROBOT_FRONT_OFFSET_M,
    apply_vehicle_front_offset,
    planar_measurement_from_vehicle_front,
    remove_vehicle_front_offset,
)


def test_ground_truth_components_share_the_front_reference() -> None:
    """Forward, lateral, and distance must agree; each is measured off the front.

    A mixed tuple (origin-relative forward next to a front-relative distance)
    reads as a target that is farther ahead than it is away, and inflates every
    benchmark association cost by the offset.
    """

    forward_m, lateral_m, distance_m = planar_measurement_from_vehicle_front(
        5.02, -1.11, 0.0)

    assert math.isclose(forward_m, 5.02 - ROBOT_FRONT_OFFSET_M, rel_tol=1e-9)
    assert math.isclose(lateral_m, -1.11, rel_tol=1e-9)
    assert math.isclose(distance_m, math.hypot(forward_m, lateral_m), rel_tol=1e-12)
    assert distance_m >= abs(forward_m)


def test_front_offset_round_trips() -> None:
    """The two directions must stay exact inverses.

    A one-sided change to the offset leaves every ring a constant shift from the
    distance printed beside it, which reads as sensor error rather than as a
    plotting fault.
    """

    lateral_m, forward_m = apply_vehicle_front_offset(-1.11, 5.02)
    assert math.isclose(forward_m, 5.02 - ROBOT_FRONT_OFFSET_M, rel_tol=1e-12)

    lateral_m, forward_m = remove_vehicle_front_offset(lateral_m, forward_m)
    assert math.isclose(lateral_m, -1.11, rel_tol=1e-12)
    assert math.isclose(forward_m, 5.02, rel_tol=1e-12)


def test_hardware_front_reference_round_trip_keeps_base_point():
    lateral, forward = apply_vehicle_front_offset(-.6, 2.0, front_offset_m=.465)
    assert math.isclose(forward, 1.535)
    lateral, forward = remove_vehicle_front_offset(lateral, forward, front_offset_m=.465)
    assert math.isclose(lateral, -.6)
    assert math.isclose(forward, 2.0)
