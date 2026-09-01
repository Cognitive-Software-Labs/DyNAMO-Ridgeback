from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.target_localization.core.mask import MaskPrecision, mask_from_array, rasterize_bbox
from ridgeback_autonomy.perception.target_localization.core import polar_profiling
from ridgeback_autonomy.perception.target_localization.core.polar_profiling import (
    beams_in_bbox,
    localize_projected_polar_profiling,
    localize_polar_profiling,
    merge_near_band,
    project_in_view,
    project_scan_to_image,
    scan_points_optical,
    segment_range_profile,
    select_bbox_beams,
    select_beams,
    select_mask_beams,
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


def test_reported_beams_index_the_original_scan() -> None:
    # The beam sets are what a visualization draws, so they have to index the
    # scan the caller passed in -- not the selected subset the merge works on.
    # Indexing the input array by merged_beams must reproduce the foreground.
    points = two_legs_profile()
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    result, _ = localize_polar_profiling(
        points, np.ones(points.shape[0], dtype=bool), mask, INTRINSICS)

    assert result is not None
    assert np.array_equal(points[result.merged_beams][:, (0, 2)], result.foreground_points)
    assert set(result.merged_beams.tolist()) <= set(result.selected_beams.tolist())
    assert result.selected_beams.size == result.ray_count

    # The gap between the two sets is exactly the parallax wall the segmentation
    # discarded -- the beams a "dropped" overlay exists to show.
    dropped = np.setdiff1d(result.selected_beams, result.merged_beams)
    assert dropped.size > 0
    assert np.allclose(points[dropped][:, 2], WALL_Z_M)
    assert np.allclose(points[result.merged_beams][:, 2], LEG_Z_M)


def test_bbox_beams_equal_mask_beams_under_a_box_gate() -> None:
    # Under a box gate the mask IS the rasterized box, so the wedge's beam set
    # and the estimator's selection must coincide -- if they ever diverge here,
    # the two are being derived by different rules.
    points = two_legs_profile()
    valid = np.ones(points.shape[0], dtype=bool)
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)

    selected = select_beams(points, valid, mask, INTRINSICS)
    in_bbox = beams_in_bbox(points, valid, MASK_BBOX, INTRINSICS)

    assert np.array_equal(selected, in_bbox)


def test_bbox_beams_are_a_superset_of_a_tighter_silhouette() -> None:
    # A silhouette that drops the middle columns keeps the box beams unchanged;
    # the difference is exactly what the segmentation removed.
    points = two_legs_profile()
    valid = np.ones(points.shape[0], dtype=bool)
    silhouette_data = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH).data.copy()
    silhouette_data[:, 38:43] = False
    silhouette = mask_from_array(silhouette_data, MaskPrecision.TIGHT)

    selected = select_beams(points, valid, silhouette, INTRINSICS)
    in_bbox = beams_in_bbox(points, valid, MASK_BBOX, INTRINSICS)

    assert set(selected.tolist()) < set(in_bbox.tolist())


def test_scan_image_projection_preserves_full_uv_and_compact_original_indices() -> None:
    points = np.vstack((
        two_legs_profile(),
        # These rows must remain in the full UV array even though neither is
        # selectable: one is beyond the image edge, the other behind camera.
        np.array([[10.0, SCAN_PLANE_Y_M, 1.0], [0.0, SCAN_PLANE_Y_M, -1.0]]),
    ))
    valid = np.ones(points.shape[0], dtype=bool)
    valid[0] = False

    projection = project_scan_to_image(points, valid, INTRINSICS)
    legacy_indices, legacy_uv = project_in_view(points, valid, INTRINSICS)

    assert projection.uv.shape == (points.shape[0], 2)
    assert np.array_equal(projection.uv, legacy_uv, equal_nan=True)
    assert np.array_equal(projection.beam_indices, legacy_indices)
    assert np.array_equal(
        projection.u_px, np.rint(projection.uv[legacy_indices, 0]).astype(np.intp))
    assert np.array_equal(
        projection.v_px, np.rint(projection.uv[legacy_indices, 1]).astype(np.intp))
    assert {0, points.shape[0] - 2, points.shape[0] - 1}.isdisjoint(
        projection.beam_indices.tolist())


def test_distinct_masks_select_independently_from_one_projection() -> None:
    points = two_legs_profile()
    projection = project_scan_to_image(
        points, np.ones(points.shape[0], dtype=bool), INTRINSICS)
    left = mask_from_array(
        rasterize_bbox((30, 33, 40, 49), HEIGHT, WIDTH).data, MaskPrecision.TIGHT)
    right = mask_from_array(
        rasterize_bbox((40, 33, 51, 49), HEIGHT, WIDTH).data, MaskPrecision.TIGHT)

    left_beams = select_mask_beams(projection, left)
    right_beams = select_mask_beams(projection, right)

    assert left_beams.size > 0 and right_beams.size > 0
    assert not np.array_equal(left_beams, right_beams)
    assert set(left_beams.tolist()).isdisjoint(right_beams.tolist())
    assert np.array_equal(select_bbox_beams(projection, MASK_BBOX),
                          beams_in_bbox(points, np.ones(points.shape[0], dtype=bool),
                                        MASK_BBOX, INTRINSICS))


def test_projected_localization_matches_legacy_and_retains_failed_selection() -> None:
    points = two_legs_profile()
    valid = np.ones(points.shape[0], dtype=bool)
    mask = rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH)
    projection = project_scan_to_image(points, valid, INTRINSICS)

    attempt = localize_projected_polar_profiling(projection, mask)
    legacy_result, legacy_reason = localize_polar_profiling(points, valid, mask, INTRINSICS)

    assert attempt.reason is legacy_reason is MissReason.OK
    assert np.array_equal(attempt.result.xz_optical, legacy_result.xz_optical)
    assert np.array_equal(attempt.result.selected_beams, legacy_result.selected_beams)
    assert np.array_equal(attempt.result.merged_beams, legacy_result.merged_beams)

    sparse_points = profile_points(
        np.array([-1.0, 1.0]), np.array([LEG_Z_M, WALL_Z_M]))
    sparse_projection = project_scan_to_image(
        sparse_points, np.ones(2, dtype=bool), INTRINSICS)
    failed = localize_projected_polar_profiling(
        sparse_projection, rasterize_bbox(MASK_BBOX, HEIGHT, WIDTH))

    assert failed.result is None
    assert failed.reason is MissReason.TOO_FEW_RAYS_MERGED
    assert failed.selected_beams.tolist() == [0, 1]


def test_box_above_the_scan_row_contains_no_beams() -> None:
    # The vertical test is load-bearing: a target occluded at scan height still
    # has a detection box, but no ray in it. No beams means no wedge downstream.
    points = two_legs_profile()
    valid = np.ones(points.shape[0], dtype=bool)
    above_scan_row = (30, 0, 51, 10)

    assert beams_in_bbox(points, valid, above_scan_row, INTRINSICS).size == 0


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
    # bit-identical to the rect run (docs/target_localization/polar_profiling.md Section 2.5).
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


def test_scan_points_optical_reuses_immutable_unit_directions() -> None:
    polar_profiling._scan_unit_directions.cache_clear()
    scan = SimpleNamespace(
        ranges=[1.0, 2.0, 3.0],
        angle_min=-0.1,
        angle_increment=0.1,
        range_min=0.02,
        range_max=15.0,
    )

    scan_points_optical(scan, np.eye(3), np.zeros(3))
    scan.ranges = [3.0, 2.0, 1.0]
    scan_points_optical(scan, np.eye(3), np.zeros(3))

    cache_info = polar_profiling._scan_unit_directions.cache_info()
    assert cache_info.hits == 1
    assert cache_info.misses == 1
    assert cache_info.maxsize == polar_profiling._SCAN_GEOMETRY_CACHE_SIZE

    directions = polar_profiling._scan_unit_directions(3, -0.1, 0.1)
    assert directions.dtype == np.float64
    assert not directions.flags.writeable
    with pytest.raises(ValueError):
        directions[0, 0] = 0.0


def test_scan_points_optical_cache_keys_every_geometry_field() -> None:
    polar_profiling._scan_unit_directions.cache_clear()

    geometries = (
        (3, -0.1, 0.1),
        (4, -0.1, 0.1),
        (3, -0.2, 0.1),
        (3, -0.1, 0.2),
    )
    directions = [
        polar_profiling._scan_unit_directions(*geometry)
        for geometry in geometries
    ]

    assert len({id(value) for value in directions}) == len(geometries)
    assert directions[0].shape != directions[1].shape
    assert not np.array_equal(directions[0], directions[2])
    assert not np.array_equal(directions[0], directions[3])
    assert polar_profiling._scan_unit_directions.cache_info().currsize == len(geometries)


def test_scan_points_optical_current_ranges_and_transform_are_not_cached() -> None:
    polar_profiling._scan_unit_directions.cache_clear()
    scan = SimpleNamespace(
        ranges=[1.0, 2.0, 3.0],
        angle_min=-0.2,
        angle_increment=0.2,
        range_min=0.02,
        range_max=15.0,
    )
    first_rotation = np.eye(3)
    first_translation = np.zeros(3)

    first_points, first_valid = scan_points_optical(
        scan, first_rotation, first_translation,
    )
    first_angles = scan.angle_min + np.arange(3, dtype=np.float64) * scan.angle_increment
    first_expected = np.stack((
        np.asarray(scan.ranges) * np.cos(first_angles),
        np.asarray(scan.ranges) * np.sin(first_angles),
        np.zeros(3),
    ), axis=1)

    scan.ranges = [3.0, 2.0, 1.0]
    second_rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    second_translation = np.array([0.5, -0.25, 1.0])
    second_points, second_valid = scan_points_optical(
        scan, second_rotation, second_translation,
    )
    second_scan_points = np.stack((
        np.asarray(scan.ranges) * np.cos(first_angles),
        np.asarray(scan.ranges) * np.sin(first_angles),
        np.zeros(3),
    ), axis=1)
    second_expected = second_scan_points @ second_rotation.T + second_translation

    assert np.array_equal(first_valid, np.ones(3, dtype=bool))
    assert np.array_equal(second_valid, np.ones(3, dtype=bool))
    assert np.allclose(first_points, first_expected)
    assert np.allclose(second_points, second_expected)
    assert not np.array_equal(first_points, second_points)
    assert polar_profiling._scan_unit_directions.cache_info().hits == 1


def test_scan_points_optical_empty_scan_raises() -> None:
    scan = SimpleNamespace(
        ranges=[], angle_min=0.0, angle_increment=0.1, range_min=0.02, range_max=15.0,
    )
    try:
        scan_points_optical(scan, ROTATION_LIDAR_TO_OPTICAL, TRANSLATION_LIDAR_TO_OPTICAL)
    except ValueError:
        return
    raise AssertionError('expected ValueError on an empty scan')
