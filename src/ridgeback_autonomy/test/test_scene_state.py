"""The pre-capture state check and its run-independent fingerprint."""

from __future__ import annotations

import math

import pytest

from ridgeback_autonomy.benchmarking.scene_state import (
    ExpectedPose,
    check_scene_state,
    state_fingerprint,
)


def _pose(x: float, y: float, yaw: float) -> dict:
    return {'position': {'x': x, 'y': y, 'z': 0.05},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}}


def _check(poses, expected):
    return check_scene_state(poses, expected, entity_prefix='bench_',
                             position_tolerance_m=0.005, yaw_tolerance_rad=0.002)


def test_the_fingerprint_ignores_run_names_and_order() -> None:
    first = _check(
        {'r/robot': _pose(1.0, 0.0, 0.0), 'bench_run1_bg_desk': _pose(8.0, 2.0, 1.5)},
        {'r/robot': ExpectedPose('robot', 1.0, 0.0, 0.0),
         'bench_run1_bg_desk': ExpectedPose('bg_desk', 8.0, 2.0, 1.5)})
    second = _check(
        {'bench_run2_bg_desk': _pose(8.0004, 2.0, 1.5), 'r/robot': _pose(1.0, 0.0, 0.0)},
        {'bench_run2_bg_desk': ExpectedPose('bg_desk', 8.0, 2.0, 1.5),
         'r/robot': ExpectedPose('robot', 1.0, 0.0, 0.0)})

    assert first.ok and second.ok
    assert first.state_hash == second.state_hash


def test_the_fingerprint_moves_with_a_millimetre_and_not_with_the_sign_of_zero() -> None:
    assert state_fingerprint({'a': (1.0, 0.0, -0.0001)}) == state_fingerprint({'a': (1.0, 0.0, 0.0)})
    assert state_fingerprint({'a': (1.0, 0.0, 0.0)}) != state_fingerprint({'a': (1.001, 0.0, 0.0)})
    # A yaw of pi and of -pi is the same heading.
    assert state_fingerprint({'a': (0.0, 0.0, math.pi)}) == state_fingerprint(
        {'a': (0.0, 0.0, -math.pi)})


def test_missing_unexpected_and_displaced_entities_are_each_named() -> None:
    check = _check(
        {'r/robot': _pose(1.0, 0.0, 0.003), 'bench_x_target_0': _pose(4.0, 0.0, 3.1),
         'bench_old_bg_desk': _pose(0.0, 0.0, 0.0)},
        {'r/robot': ExpectedPose('robot', 1.0, 0.0, 0.0),
         'bench_x_target_0': ExpectedPose('target_0', 4.0, 0.0, 3.1),
         'bench_x_obj_0': ExpectedPose('obj_0', 3.0, 0.0, 0.0)})

    assert not check.ok
    assert check.differences[0] == 'missing obj_0'
    assert check.differences[1] == 'unexpected bench_old_bg_desk'
    assert check.differences[2].startswith('robot at (1.0000, 0.0000, yaw 0.0030)')
    assert check.max_yaw_error_rad == pytest.approx(0.003)
    assert check.as_record()['reason'] == 'state_mismatch'
