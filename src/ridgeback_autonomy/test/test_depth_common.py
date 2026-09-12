from __future__ import annotations

import math

import pytest
import yaml

from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    DEPTH_GATE_DISABLED,
    resolve_depth_gate,
    valid_depth,
)


def test_positive_gate_passes_through() -> None:
    assert resolve_depth_gate(10.0) == 10.0
    assert resolve_depth_gate(18.0) == 18.0


@pytest.mark.parametrize('written', [DEPTH_GATE_DISABLED, 0.0, -1.0])
def test_non_positive_gate_means_unbounded(written: float) -> None:
    # Not merely "large": the value is combined with a source ceiling by min(),
    # where anything finite would be a real gate and a 0 would cull every pixel.
    assert resolve_depth_gate(written) == math.inf


def test_unbounded_gate_still_rejects_the_invalid_pixel_encodings() -> None:
    # Removing the gate must not weaken the frame contract. Out-of-range
    # readings arrive as 0/NaN/inf, and those stay excluded with no ceiling.
    depths = [0.0, math.nan, math.inf, -1.0, 0.5, 12.0, 95.0]

    keep = valid_depth(depths, resolve_depth_gate(DEPTH_GATE_DISABLED))

    assert list(keep) == [False, False, False, False, True, True, True]


def test_disabled_sentinel_survives_a_launch_substitution() -> None:
    # The gate reaches the node as a launch substitution, which is parsed as
    # YAML into a double. Bare ``inf`` is why the sentinel is 0 and not
    # infinity: YAML reads it as the string "inf", which fails the type check.
    assert isinstance(yaml.safe_load(str(DEPTH_GATE_DISABLED)), float)
    assert not isinstance(yaml.safe_load('inf'), float)
