from __future__ import annotations

import numpy as np

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.core.isolation_3d import RangeBand
from ridgeback_autonomy.perception.core.mask import MaskPrecision, mask_from_array, rasterize_bbox
from ridgeback_autonomy.perception.core.euclidean_reconstruction import localize_euclidean_reconstruction


HEIGHT, WIDTH = 60, 80
INTRINSICS = CameraIntrinsics(fx=100.0, fy=100.0, cx=40.0, cy=30.0, width=WIDTH, height=HEIGHT)

OBJECT_DEPTH_M = 2.0
BACKGROUND_DEPTH_M = 4.0
# Object plate rows 20:40, cols 30:50; centroid of its deprojection is exact
# because Z is constant over the plate.
OBJECT_SLICE = (slice(20, 40), slice(30, 50))
OBJECT_CENTROID_XYZ = (-0.01, -0.01, 2.0)
RECT_BBOX = (25, 15, 55, 45)


def build_depth() -> np.ndarray:
    depth = np.full((HEIGHT, WIDTH), BACKGROUND_DEPTH_M, dtype=np.float32)
    depth[OBJECT_SLICE] = OBJECT_DEPTH_M
    return depth


def object_mask_data() -> np.ndarray:
    data = np.zeros((HEIGHT, WIDTH), dtype=bool)
    data[OBJECT_SLICE] = True
    return data


def test_rect_mask_default_chain_recovers_object_centroid() -> None:
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, reason = localize_euclidean_reconstruction(build_depth(), mask, INTRINSICS)

    assert result is not None
    assert reason is MissReason.OK
    assert np.allclose(result.xyz_optical, OBJECT_CENTROID_XYZ)
    # By-product: exactly the object's points survive the chain.
    assert result.foreground_points.shape == (400, 3)


def test_rect_accepts_explicit_isolation_recipe() -> None:
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, _ = localize_euclidean_reconstruction(build_depth(), mask, INTRINSICS, isolation=RangeBand())

    assert result is not None
    assert np.allclose(result.xyz_optical, OBJECT_CENTROID_XYZ)
    assert result.foreground_points.shape == (400, 3)


def test_tight_mask_mad_pass_drops_edge_bleed() -> None:
    depth = build_depth()
    # Five tight-mask pixels bleed onto the far background.
    bleed = [(20, 30), (20, 49), (39, 30), (39, 49), (29, 40)]
    for row, col in bleed:
        depth[row, col] = 6.0
    mask = mask_from_array(object_mask_data(), MaskPrecision.TIGHT)

    result, _ = localize_euclidean_reconstruction(depth, mask, INTRINSICS)

    assert result is not None
    # The bleed points (range ~6 m) are gone; some legitimate far-corner
    # plate points may fall with them, so bound the count instead of pinning it.
    assert result.foreground_points[:, 2].max() < 3.0
    assert 300 <= result.foreground_points.shape[0] <= 395
    assert np.allclose(result.xyz_optical, OBJECT_CENTROID_XYZ, atol=0.02)


def test_all_invalid_depth_returns_too_few_valid_points() -> None:
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result, reason = localize_euclidean_reconstruction(depth, mask, INTRINSICS)

    assert result is None
    assert reason is MissReason.TOO_FEW_VALID_POINTS


def test_sparse_mask_returns_too_few_valid_points() -> None:
    mask = rasterize_bbox((10, 10, 13, 13), HEIGHT, WIDTH)

    result, reason = localize_euclidean_reconstruction(build_depth(), mask, INTRINSICS)

    assert result is None
    assert reason is MissReason.TOO_FEW_VALID_POINTS


def test_isolation_dropping_all_points_returns_isolation_empty() -> None:
    # Enough valid points enter, but the isolation recipe keeps none.
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    def drop_all(points: np.ndarray) -> np.ndarray:
        return np.zeros(points.shape[0], dtype=bool)

    result, reason = localize_euclidean_reconstruction(
        build_depth(), mask, INTRINSICS, isolation=drop_all)

    assert result is None
    assert reason is MissReason.ISOLATION_EMPTY
