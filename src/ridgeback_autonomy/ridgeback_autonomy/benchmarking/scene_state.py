"""What the simulation must hold before a trial is captured, and its fingerprint.

After a scene settles, one state snapshot is compared with what the trial
commanded: exactly the expected benchmark entities, each at its commanded
planar pose, and the robot at its placement. A trial whose room differs is not
captured, so no result is ever scored in a room other than the one its scene
defines.

The fingerprint hashes the measured poses under run-independent labels, so two
runs -- or two orders of the same scenes -- that built the same room hash the
same, whatever their entity names.

Kept pure (no ROS): the runner and the tests share it.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import math
from typing import Any

from ridgeback_localization.core.vehicle_frame import yaw_from_quaternion


# Rounding of the fingerprinted poses: 1 mm and 1 mrad.
HASH_DECIMALS = 3
STATE_MISMATCH = 'state_mismatch'


@dataclass(frozen=True)
class ExpectedPose:
    """A commanded planar pose under a label that does not depend on the run."""

    label: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StateCheck:
    ok: bool
    differences: tuple[str, ...]
    state_hash: str
    max_position_error_m: float
    max_yaw_error_rad: float

    def as_record(self) -> dict[str, Any]:
        return {
            'ok': self.ok,
            **({} if self.ok else {'reason': STATE_MISMATCH}),
            'differences': list(self.differences),
            'state_hash': self.state_hash,
            'max_position_error_m': self.max_position_error_m,
            'max_yaw_error_rad': self.max_yaw_error_rad,
        }


class SceneStateMismatch(RuntimeError):
    """The settled scene is not the one the trial commanded."""

    def __init__(self, check: StateCheck) -> None:
        self.check = check
        super().__init__(f'{STATE_MISMATCH}: {"; ".join(check.differences)}')


def _planar(pose: Mapping[str, Any]) -> tuple[float, float, float]:
    orientation = pose['orientation']
    return (
        pose['position']['x'],
        pose['position']['y'],
        yaw_from_quaternion(
            orientation['x'], orientation['y'], orientation['z'], orientation['w']),
    )


def _wrapped(angle: float) -> float:
    """``angle`` in (-pi, pi]."""

    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return math.pi if wrapped == -math.pi else wrapped


def _rounded(value: float) -> str:
    # Adding 0.0 turns a rounded -0.0 into 0.0, so the sign of a near-zero
    # value cannot change the fingerprint.
    return f'{round(value, HASH_DECIMALS) + 0.0:.{HASH_DECIMALS}f}'


def state_fingerprint(measured: Mapping[str, tuple[float, float, float]]) -> str:
    """SHA-256 of the sorted (label, x, y, yaw) rows, rounded to 1 mm and 1 mrad."""

    rows = ''.join(
        f'{label}\t{_rounded(x)}\t{_rounded(y)}\t{_rounded(_wrapped(yaw))}\n'
        for label, (x, y, yaw) in sorted(measured.items()))
    return hashlib.sha256(rows.encode('utf-8')).hexdigest()


def check_scene_state(
    poses: Mapping[str, Mapping[str, Any]],
    expected: Mapping[str, ExpectedPose],
    *,
    entity_prefix: str,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> StateCheck:
    """Compare one snapshot with the commanded scene.

    ``poses`` holds the snapshot's normalized poses: every entity whose name
    starts with ``entity_prefix``, plus whatever else ``expected`` names (the
    robot). The prefixed entities present must be exactly the prefixed ones
    expected; every expected entity must sit within the tolerances of its
    commanded x, y and yaw.
    """

    differences = []
    present = {name for name in poses if name.startswith(entity_prefix)}
    wanted = {name for name in expected if name.startswith(entity_prefix)}
    missing = sorted((wanted - present) | {
        name for name in expected if name not in poses and name not in wanted})
    unexpected = sorted(present - wanted)
    if missing:
        differences.append(f'missing {", ".join(expected[name].label for name in missing)}')
    if unexpected:
        differences.append(f'unexpected {", ".join(unexpected)}')

    measured = {}
    worst_position = worst_yaw = 0.0
    for name, target in expected.items():
        if name not in poses:
            continue
        x, y, yaw = _planar(poses[name])
        measured[target.label] = (x, y, yaw)
        position_error = math.hypot(x - target.x, y - target.y)
        yaw_error = abs(_wrapped(yaw - target.yaw))
        worst_position = max(worst_position, position_error)
        worst_yaw = max(worst_yaw, yaw_error)
        if position_error > position_tolerance_m or yaw_error > yaw_tolerance_rad:
            differences.append(
                f'{target.label} at ({x:.4f}, {y:.4f}, yaw {yaw:.4f}), '
                f'{position_error:.4f} m and {yaw_error:.4f} rad from '
                f'({target.x:.4f}, {target.y:.4f}, yaw {target.yaw:.4f})')
    for name in unexpected:
        measured[name] = _planar(poses[name])
    return StateCheck(
        ok=not differences,
        differences=tuple(differences),
        state_hash=state_fingerprint(measured),
        max_position_error_m=worst_position,
        max_yaw_error_rad=worst_yaw,
    )
