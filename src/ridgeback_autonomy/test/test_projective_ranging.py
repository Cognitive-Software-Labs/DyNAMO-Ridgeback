from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import otsu_foreground
from ridgeback_autonomy.perception.target_localization.core.mask import MaskPrecision, mask_from_array, rasterize_bbox
from ridgeback_autonomy.perception.target_localization.core import projective_ranging
from ridgeback_autonomy.perception.target_localization.core.projective_ranging import localize_projective_ranging


HEIGHT, WIDTH = 60, 80
INTRINSICS = CameraIntrinsics(fx=100.0, fy=100.0, cx=40.0, cy=30.0, width=WIDTH, height=HEIGHT)
# The ceiling these fixtures run under. Stated because the entry points
# take no default one -- the trustworthy range belongs to the depth source
# and the operator gate, not to a module constant.
DEPTH_GATE_M = 10.0

OBJECT_DEPTH_M = 2.0
BACKGROUND_DEPTH_M = 4.0
# Object plate rows 20:40, cols 30:50 -- foreground centroid (u, v) = (39.5, 29.5).
OBJECT_SLICE = (slice(20, 40), slice(30, 50))
OBJECT_CENTROID_UV = (39.5, 29.5)
# Its exact deprojection at the aggregated depth.
OBJECT_XYZ = (-0.01, -0.01, 2.0)
# Rect box oversizing the object asymmetrically: the raw box center (47.5,
# 32.5) sits on background, far from the object centroid.
RECT_BBOX = (25, 15, 70, 50)


def build_depth() -> np.ndarray:
    depth = np.full((HEIGHT, WIDTH), BACKGROUND_DEPTH_M, dtype=np.float32)
    depth[OBJECT_SLICE] = OBJECT_DEPTH_M
    return depth


def object_mask_data() -> np.ndarray:
    data = np.zeros((HEIGHT, WIDTH), dtype=bool)
    data[OBJECT_SLICE] = True
    return data


def test_rect_mask_recovers_object_coordinate() -> None:
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, reason = localize_projective_ranging(build_depth(), mask, INTRINSICS,
        depth_max=DEPTH_GATE_M)

    assert result is not None
    assert reason is MissReason.OK
    assert result.depth_m == OBJECT_DEPTH_M
    assert np.allclose(result.xyz_optical, OBJECT_XYZ)
    assert result.foreground_pixel_count == 400


def test_rect_representative_pixel_is_foreground_centroid_not_box_center() -> None:
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, _ = localize_projective_ranging(
        build_depth(), mask, INTRINSICS, depth_max=DEPTH_GATE_M)

    assert result is not None
    assert np.allclose(result.representative_uv, OBJECT_CENTROID_UV)
    x1, y1, x2, y2 = RECT_BBOX
    box_center_uv = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    assert not np.allclose(result.representative_uv, box_center_uv)


def test_rect_survives_invalid_pixels_in_the_box() -> None:
    depth = build_depth()
    depth[20, 30] = 0.0
    depth[21, 31] = np.nan
    depth[16, 26] = np.inf
    depth[17, 27] = 15.0  # out of range
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, _ = localize_projective_ranging(depth, mask, INTRINSICS, depth_max=DEPTH_GATE_M)

    assert result is not None
    assert result.depth_m == OBJECT_DEPTH_M
    # The two invalidated object pixels drop out of the foreground.
    assert result.foreground_pixel_count == 398


def test_rect_accepts_explicit_isolation_recipe() -> None:
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, _ = localize_projective_ranging(build_depth(), mask, INTRINSICS,
        isolation=otsu_foreground, depth_max=DEPTH_GATE_M)

    assert result is not None
    assert result.depth_m == OBJECT_DEPTH_M
    assert np.allclose(result.representative_uv, OBJECT_CENTROID_UV)


def test_rect_uses_precomputed_valid_mask_without_recleaning(
    monkeypatch,
) -> None:
    depth = build_depth()
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)
    valid_masked = mask.data & np.isfinite(depth) & (depth > 0.0)
    received = []

    def isolation(depth_arg, mask_arg, *, depth_max, valid_masked):
        received.append(valid_masked)
        return valid_masked

    monkeypatch.setattr(
        projective_ranging,
        'valid_depth',
        lambda *args: pytest.fail('precomputed validity should be reused'),
    )

    result, reason = localize_projective_ranging(
        depth,
        mask,
        INTRINSICS,
        isolation=isolation,
        valid_masked=valid_masked, depth_max=DEPTH_GATE_M)

    assert result is not None
    assert reason is MissReason.OK
    assert len(received) == 1
    assert received[0] is valid_masked


def test_rect_branch_respects_custom_depth_max() -> None:
    # The depth cutoff must reach the isolation recipe, not only a caller-side
    # pre-clean: an object entirely past a tightened depth_max drops out.
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)  # invalid background
    depth[OBJECT_SLICE] = 6.0                            # object plate at 6 m
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    kept, reason = localize_projective_ranging(depth, mask, INTRINSICS, depth_max=DEPTH_GATE_M)
    assert kept is not None
    assert reason is MissReason.OK
    assert kept.depth_m == 6.0
    assert kept.foreground_pixel_count == 400

    # Tighten the cutoff below the object: the gate cleans it away and the
    # selection is left with too few pixels, so the call returns None. The
    # shortfall is the gate's, not the recipe's -- which never ran.
    capped, capped_reason = localize_projective_ranging(depth, mask, INTRINSICS, depth_max=4.0)
    assert capped is None
    assert capped_reason is MissReason.TOO_FEW_VALID_PIXELS


def test_tight_mask_takes_the_median_directly() -> None:
    mask = mask_from_array(object_mask_data(), MaskPrecision.TIGHT)

    result, reason = localize_projective_ranging(build_depth(), mask, INTRINSICS,
        depth_max=DEPTH_GATE_M)

    assert result is not None
    assert reason is MissReason.OK
    assert result.depth_m == OBJECT_DEPTH_M
    assert np.allclose(result.representative_uv, OBJECT_CENTROID_UV)
    assert np.allclose(result.xyz_optical, OBJECT_XYZ)
    assert result.foreground_pixel_count == 400


def test_all_invalid_depth_blames_the_selection_not_the_recipe() -> None:
    # Nothing valid ever entered, so the recipe rejected nothing; reporting its
    # name here sent anyone reading the tally to tune a recipe that never ran.
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, reason = localize_projective_ranging(depth, mask, INTRINSICS, depth_max=DEPTH_GATE_M)

    assert result is None
    assert reason is MissReason.TOO_FEW_VALID_PIXELS


def test_sparse_rect_mask_reports_the_same_shortfall_as_a_sparse_tight_one() -> None:
    # A 3x3 rect box holds fewer valid pixels than min_valid_pixels. The mask
    # tag decides which recipe runs, never what a shortfall is called.
    mask = rasterize_bbox((10, 10, 13, 13), HEIGHT, WIDTH)

    result, reason = localize_projective_ranging(build_depth(), mask, INTRINSICS,
        depth_max=DEPTH_GATE_M)

    assert result is None
    assert reason is MissReason.TOO_FEW_VALID_PIXELS


def test_sparse_tight_mask_returns_too_few_valid_pixels() -> None:
    # The tight branch skips isolation, so its shortfall is a distinct reason.
    data = np.zeros((HEIGHT, WIDTH), dtype=bool)
    data[10:13, 10:13] = True  # 9 pixels < min_valid_pixels
    mask = mask_from_array(data, MaskPrecision.TIGHT)

    result, reason = localize_projective_ranging(build_depth(), mask, INTRINSICS,
        depth_max=DEPTH_GATE_M)

    assert result is None
    assert reason is MissReason.TOO_FEW_VALID_PIXELS
