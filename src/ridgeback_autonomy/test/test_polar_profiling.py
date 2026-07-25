from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.core.mask import MaskPrecision, mask_from_array, rasterize_bbox
from ridgeback_autonomy.perception.core.polar_profiling import (
    localize_polar_profiling,
    merge_near_band,
    scan_points_optical,
    segment_range_profile,
)


HEIGHT, WIDTH = 60, 80
INTRINSICS = CameraIntrinsics(fx=100.0, fy=100.0, cx=40.0, cy=30.0, width=WIDTH, height=HEIGHT)

# The scan plane sits 0.3 m below the camera (optical Y is down), so leg
# points at Z = 2 m project to v = 45 and wall points at Z = 5 m to v = 36.
SCAN_PLANE_Y_M = 0.3
LEG_Z_M = 2.0
WALL_Z_M = 5.0
# Rows cover both projections; cols 30:51 admit bearings within ~±5.7 deg.
MASK_BBOX = (30, 33, 51, 49)

# LiDAR frame (x forward, y left, z up) -> camera optical (X right, Y down,
# Z forward), plus the mounting offset expressed in the optical frame.
ROTATION_LIDAR_TO_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])
TRANSLATION_LIDAR_TO_OPTICAL = np.array([0.0, 0.65, 0.09])


def profile_points(bearings_deg: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    """Synthetic camera-optical scan points: one per bearing, at depth Z."""

    bearings_rad = np.radians(np.asarray(bearings_deg, dtype=np.float64))
    z = np.asarray(z_m, dtype=np.float64)
    x = z * np.tan(bearings_rad)
    y = np.full_like(z, SCAN_PLANE_Y_M)
    return np.stack((x, y, z), axis=1)


def two_legs_profile() -> np.ndarray:
    """Legs at |bearing| in [2, 4] deg (Z = 2 m); wall everywhere else (Z = 5 m).

    The wall appears both through the gap between the legs (parallax: those
    beams still project inside the mask) and beside them (neighbor
    contamination admitted by the rect window).
    """

    bearings_deg = np.arange(-6.0, 6.25, 0.25)
    is_leg = (np.abs(bearings_deg) >= 2.0) & (np.abs(bearings_deg) <= 4.0)
    z_m = np.where(is_leg, LEG_Z_M, WALL_Z_M)
    return profile_points(bearings_deg, z_m)


def test_two_legs_merge_drops_parallax_wall() -> None:
    points = two_legs_profile()
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, reason = localize_polar_profiling(
        points, np.ones(points.shape[0], dtype=bool), mask, INTRINSICS)

    assert result is not None
    assert reason is MissReason.OK
    # Both legs merged: X medians to the body center, Z to the leg depth.
    assert np.allclose(result.xz_optical, (0.0, LEG_Z_M), atol=1e-6)
    # Exactly the 9-beam legs survive (both sides); every wall beam is gone.
    assert result.foreground_points.shape == (18, 2)
    assert result.ray_count > 18  # wall beams did enter the mask select


def test_unequal_legs_median_stays_on_object() -> None:
    # One leg partially occluded (fewer beams): the pinned per-axis median
    # may snap toward the better-sampled leg but never off the object, and
    # never toward the wall — the robustness the median was chosen for.
    bearings_deg = np.arange(-6.0, 6.25, 0.25)
    left_leg = (bearings_deg >= -4.0) & (bearings_deg <= -2.0)
    right_leg = (bearings_deg >= 3.0) & (bearings_deg <= 4.0)  # 5 beams vs 9
    z_m = np.where(left_leg | right_leg, LEG_Z_M, WALL_Z_M)
    points = profile_points(bearings_deg, z_m)
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, _ = localize_polar_profiling(
        points, np.ones(points.shape[0], dtype=bool), mask, INTRINSICS)

    assert result is not None
    assert result.xz_optical[1] == LEG_Z_M
    # X lies within the legs' lateral span (leaning left, the fatter leg).
    left_x = LEG_Z_M * np.tan(np.radians(-4.0))
    right_x = LEG_Z_M * np.tan(np.radians(4.0))
    assert left_x <= result.xz_optical[0] <= right_x
    assert result.foreground_points.shape == (14, 2)


def test_tight_tag_runs_identical_recovery() -> None:
    # Same selector, tight tag: no fork exists in polar profiling, so the result is
    # bit-identical to the rect run (polar_profiling.md Section 2.5).
    points = two_legs_profile()
    valid = np.ones(points.shape[0], dtype=bool)
    rect = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)
    tight = mask_from_array(rect.data.copy(), MaskPrecision.TIGHT)

    rect_result, _ = localize_polar_profiling(points, valid, rect, INTRINSICS)
    tight_result, _ = localize_polar_profiling(points, valid, tight, INTRINSICS)

    assert rect_result is not None and tight_result is not None
    assert np.array_equal(rect_result.xz_optical, tight_result.xz_optical)
    assert rect_result.ray_count == tight_result.ray_count


def test_wall_behind_single_object_rejected() -> None:
    # One solid object run flanked by wall runs inside the mask window.
    bearings_deg = np.arange(-5.0, 5.25, 0.25)
    z_m = np.where(np.abs(bearings_deg) <= 1.5, LEG_Z_M, WALL_Z_M)
    points = profile_points(bearings_deg, z_m)
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, _ = localize_polar_profiling(
        points, np.ones(points.shape[0], dtype=bool), mask, INTRINSICS)

    assert result is not None
    assert np.allclose(result.xz_optical, (0.0, LEG_Z_M), atol=1e-6)
    assert result.foreground_points.shape[0] == 13


def test_no_beam_in_mask_returns_too_few_rays_selected() -> None:
    # Beams are in view, but none fall inside the (empty) mask.
    points = two_legs_profile()
    empty_mask = mask_from_array(np.zeros((HEIGHT, WIDTH), dtype=bool), MaskPrecision.TIGHT)

    result, reason = localize_polar_profiling(
        points, np.ones(points.shape[0], dtype=bool), empty_mask, INTRINSICS)

    assert result is None
    assert reason is MissReason.TOO_FEW_RAYS_SELECTED


def test_all_invalid_beams_return_no_beams_in_view() -> None:
    points = two_legs_profile()
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, reason = localize_polar_profiling(
        points, np.zeros(points.shape[0], dtype=bool), mask, INTRINSICS)

    assert result is None
    assert reason is MissReason.NO_BEAMS_IN_VIEW


def test_sparse_rays_return_too_few_rays_selected() -> None:
    # A single beam in the mask is below min_valid_rays.
    points = profile_points(np.array([0.0]), np.array([LEG_Z_M]))
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, reason = localize_polar_profiling(points, np.ones(1, dtype=bool), mask, INTRINSICS)

    assert result is None
    assert reason is MissReason.TOO_FEW_RAYS_SELECTED


def test_merge_leaves_too_few_returns_too_few_rays_merged() -> None:
    # Two in-mask beams at far-apart ranges: the near-band merge keeps only the
    # nearest, dropping below min_valid_rays after the merge (a distinct reason).
    points = profile_points(np.array([-1.0, 1.0]), np.array([LEG_Z_M, WALL_Z_M]))
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, reason = localize_polar_profiling(points, np.ones(2, dtype=bool), mask, INTRINSICS)

    assert result is None
    assert reason is MissReason.TOO_FEW_RAYS_MERGED


def test_segment_range_profile_splits_on_jump_and_gap() -> None:
    beam_indices = np.array([10, 11, 12, 13, 17, 18])
    planar_range_m = np.array([2.0, 2.02, 5.0, 5.01, 5.02, 5.03])

    runs = segment_range_profile(beam_indices, planar_range_m)

    # Range jump splits after position 1; the beam gap (13 -> 17) after 3.
    assert [run.tolist() for run in runs] == [[0, 1], [2, 3], [4, 5]]


def test_merge_near_band_keeps_both_legs_only() -> None:
    planar_range_m = np.array([2.0, 2.01, 5.0, 5.0, 2.2, 2.21, 3.5])
    runs = [np.array([0, 1]), np.array([2, 3]), np.array([4, 5]), np.array([6])]

    merged = merge_near_band(runs, planar_range_m)

    # Nearest run is 2.0; the 2.2 leg merges (band 0.35), 3.5 and 5.0 drop.
    assert merged.tolist() == [0, 1, 4, 5]


def test_scan_points_optical_transform_and_validity() -> None:
    scan = SimpleNamespace(
        ranges=[2.0, float('nan'), 0.01, 20.0, 3.0],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.02,
        range_max=15.0,
    )

    points, valid = scan_points_optical(
        scan, ROTATION_LIDAR_TO_OPTICAL, TRANSLATION_LIDAR_TO_OPTICAL,
    )

    assert points.shape == (5, 3)
    # NaN, below the range floor, and beyond the 10 m cap are invalid.
    assert valid.tolist() == [True, False, False, False, True]
    # Beam 0: straight ahead at 2 m -> optical (0, 0.65, 2.09).
    assert np.allclose(points[0], (0.0, 0.65, 2.09))
    # Beam 4: bearing 0.4 rad left at 3 m -> optical X is negative (left).
    expected = (
        -3.0 * math.sin(0.4),
        0.65,
        3.0 * math.cos(0.4) + 0.09,
    )
    assert np.allclose(points[4], expected)


def test_scan_points_optical_empty_scan_raises() -> None:
    scan = SimpleNamespace(
        ranges=[], angle_min=0.0, angle_increment=0.1, range_min=0.02, range_max=15.0,
    )
    try:
        scan_points_optical(scan, ROTATION_LIDAR_TO_OPTICAL, TRANSLATION_LIDAR_TO_OPTICAL)
    except ValueError:
        return
    raise AssertionError('expected ValueError on an empty scan')
