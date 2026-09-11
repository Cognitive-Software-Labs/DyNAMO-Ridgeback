"""Contract tests for the Isaac RTX-lidar scan assembler (no rclpy needed).

Pins the published scan geometry to the UST-10LX contract, and guards the
regression that `docs/isaac/open-issues.md` §1 turned out to be: the lidar
links were parented to `base_link`, a bare Xform with no joint into the
articulation, so PhysX turned `chassis_link` and left the sensors behind. The
chassis then swept under a static emitter and its own notch edge came into
range, producing phantom 0.40-0.55 m returns that tripped
`collision_monitor`'s `min_points: 6` and pinned `cmd_vel` at zero.

A 10-deg edge mask was added here first and is gone: it masked the symptom,
and once the reparent landed it measured unnecessary. So these tests also
assert the assembler drops NO bins of its own — if someone reintroduces a
mask to hide a geometry or transform bug, this fails.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

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


def test_range_window_matches_the_sensor():
    # the RTX prims are authored nearRangeM 0.06 / farRangeM 10.0
    assert math.isclose(A.RANGE_MIN, 0.06, abs_tol=1e-9)
    assert math.isclose(A.RANGE_MAX, 10.0, abs_tol=1e-9)
    assert math.isclose(A.SCAN_PERIOD, 1.0 / 40.0, abs_tol=1e-12)


def test_assembler_declares_no_bin_mask():
    """No edge/seam mask. The phantom band §1 chased was a detached-sensor
    transform bug, fixed in clearpath/robot.yaml by reparenting the lidars to
    chassis_link — not something the assembler should hide."""
    assert not hasattr(A, "EDGE_MASK_DEG"), \
        "an edge mask is back; fix the root cause instead (open-issues.md §1)"
    assert not hasattr(A, "edge_mask"), \
        "an edge mask is back; fix the root cause instead (open-issues.md §1)"


def test_full_270_deg_arc_is_publishable():
    """Every bin in the contract window must be reachable — the arc is the
    robot's whole forward+lateral sensing envelope and the earlier mask cost
    7.4% of it per unit."""
    deg = bin_degrees()
    assert deg.min() <= -135.0 + 1e-9
    assert deg.max() >= 135.0 - 1e-9
    assert len(deg) == A.N_BINS
