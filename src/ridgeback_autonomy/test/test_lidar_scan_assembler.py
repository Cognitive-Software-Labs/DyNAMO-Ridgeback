"""Contract tests for the Isaac RTX-lidar scan assembler (no rclpy needed).

Covers the arc-edge mask added for `docs/isaac/open-issues.md` §1: while the
robot moves, phantom returns at 0.40-0.55 m appear at +129..+135 deg and trip
`collision_monitor`'s `min_points: 6`, pinning `cmd_vel` at zero. The
mechanism is still unknown (a self-occlusion explanation was retracted — the
body is rigid and a static ray-cast self-occludes 0/1081 bins), so the mask is
a mitigation with an empirically chosen width. What these tests pin is the
mask's shape, not its justification: it must cover the measured band on BOTH
ends, stay symmetric, and not eat into the arc the robot navigates by.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROS_IO = (_REPO_ROOT / "src" / "ridgeback_autonomy" / "sim" / "isaac"
           / "ros_io.py")

spec = importlib.util.spec_from_file_location("isaac_ros_io", _ROS_IO)
ros_io = importlib.util.module_from_spec(spec)
sys.modules["isaac_ros_io"] = ros_io
spec.loader.exec_module(ros_io)

A = ros_io.LidarScanAssembler


def bin_degrees():
    return np.degrees(A.ANGLE_MIN + A.ANGLE_INC * np.arange(A.N_BINS))


def test_contract_scan_geometry_is_the_ust_10lx_window():
    assert A.N_BINS == 1081
    assert math.isclose(math.degrees(A.ANGLE_MIN), -135.0, abs_tol=1e-9)
    assert math.isclose(math.degrees(A.ANGLE_INC), 0.25, abs_tol=1e-9)
    # last bin lands exactly on +135, so the window is closed, not off-by-one
    assert math.isclose(bin_degrees()[-1], 135.0, abs_tol=1e-9)


def test_edge_mask_covers_the_measured_phantom_band_on_both_ends():
    mask = A.edge_mask()
    deg = bin_degrees()
    # measured artefact reached inward to 126.5 deg at 0.4 rad/s (it thins
    # inward rather than ending sharply); mirror the band for the far end
    for lo, hi in ((126.5, 135.0), (-135.0, -126.5)):
        band = (deg >= lo) & (deg <= hi)
        assert band.any(), f"no bins in {lo}..{hi}"
        assert mask[band].all(), f"phantom band {lo}..{hi} deg not masked"


def test_edge_mask_is_symmetric_and_only_touches_the_arc_ends():
    mask = A.edge_mask()
    assert np.array_equal(mask, mask[::-1]), "mask must be symmetric"
    deg = bin_degrees()
    keep = 135.0 - A.EDGE_MASK_DEG
    assert not mask[np.abs(deg) <= keep - 1e-9].any(), \
        "mask reaches inside the retained arc"
    # the forward/lateral arc the robot navigates by must survive untouched
    assert not mask[np.abs(deg) <= 120.0].any()


def test_edge_mask_cost_stays_small():
    mask = A.edge_mask()
    # 10 deg per end at 0.25 deg/bin = 40 bins per end
    assert mask.sum() == 80, mask.sum()
    assert mask.sum() / A.N_BINS < 0.08


def test_masked_sector_is_inside_the_other_lidar_arc():
    """Each masked sector must be covered by the opposite, back-to-back unit,
    so nothing in the merged scan (or either costmap) goes blind."""
    kept_half_width = 135.0 - A.EDGE_MASK_DEG
    # the other unit is yawed 180 deg, so it sees bearings |theta - 180| <= kept
    # a masked bearing b (|b| > kept) maps to |b - 180| = 180 - |b| < 180 - kept
    worst = 180.0 - 135.0          # bearing +-135 maps to 45 deg off the rear
    assert worst < kept_half_width, "rear unit cannot cover the masked sector"
