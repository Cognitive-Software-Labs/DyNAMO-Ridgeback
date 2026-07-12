from __future__ import annotations

import numpy as np

from ridgeback_autonomy.perception.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.core.isolation_3d import RangeBand
from ridgeback_autonomy.perception.core.mask import MaskPrecision, mask_from_array, rasterize_bbox
from ridgeback_autonomy.perception.core.path_b import localize_path_b


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

    result = localize_path_b(build_depth(), mask, INTRINSICS)

    assert result is not None
    assert np.allclose(result.xyz_optical, OBJECT_CENTROID_XYZ)
    # Median camera-frame range of a plate at Z = 2 m is barely above 2 m.
    assert 2.0 <= result.distance_m <= 2.03
    # By-product: exactly the object's points survive the chain.
    assert result.foreground_points.shape == (400, 3)


def test_rect_accepts_explicit_isolation_recipe() -> None:
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    result = localize_path_b(build_depth(), mask, INTRINSICS, isolation=RangeBand())

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

    result = localize_path_b(depth, mask, INTRINSICS)

    assert result is not None
    # The bleed points (range ~6 m) are gone; some legitimate far-corner
    # plate points may fall with them, so bound the count instead of pinning it.
    assert result.foreground_points[:, 2].max() < 3.0
    assert 300 <= result.foreground_points.shape[0] <= 395
    assert np.allclose(result.xyz_optical, OBJECT_CENTROID_XYZ, atol=0.02)
    assert abs(result.distance_m - OBJECT_DEPTH_M) < 0.03


def test_all_invalid_depth_returns_none() -> None:
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)

    assert localize_path_b(depth, mask, INTRINSICS) is None


def test_sparse_mask_returns_none() -> None:
    mask = rasterize_bbox((10, 10, 13, 13), HEIGHT, WIDTH)

    assert localize_path_b(build_depth(), mask, INTRINSICS) is None
