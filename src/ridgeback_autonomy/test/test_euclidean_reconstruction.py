from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core import euclidean_reconstruction
from ridgeback_autonomy.perception.target_localization.core.intrinsics import CameraIntrinsics
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    RangeBand,
    build_isolation_3d,
)
from ridgeback_autonomy.perception.target_localization.core.mask import MaskPrecision, mask_from_array, rasterize_bbox
from ridgeback_autonomy.perception.target_localization.core.euclidean_reconstruction import localize_euclidean_reconstruction


HEIGHT, WIDTH = 60, 80
INTRINSICS = CameraIntrinsics(fx=100.0, fy=100.0, cx=40.0, cy=30.0, width=WIDTH, height=HEIGHT)

OBJECT_DEPTH_M = 2.0
BACKGROUND_DEPTH_M = 4.0
# Object plate rows 20:40, cols 30:50; centroid of its deprojection is exact
# because Z is constant over the plate.
OBJECT_SLICE = (slice(20, 40), slice(30, 50))
OBJECT_CENTROID_XYZ = (-0.01, -0.01, 2.0)
RECT_BBOX = (25, 15, 55, 45)

# This robot's level mount, camera 1.1855 m above the floor. The scene is not
# floor: its lowest pixels reach optical Y = 0.56, well above the 1.1355 m crop
# plane, so the height crop keeps everything and only the range band selects.
CAMERA_HEIGHT_M = 1.1855
LEVEL_DOWN_OPTICAL = (0.0, 1.0, 0.0)


def default_isolation():
    return build_isolation_3d(
        ISOLATION_3D_DEFAULT, CAMERA_HEIGHT_M, LEVEL_DOWN_OPTICAL)


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

    result, reason = localize_euclidean_reconstruction(
        build_depth(), mask, INTRINSICS, isolation=default_isolation())

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


def test_precomputed_valid_mask_avoids_recleaning(monkeypatch) -> None:
    depth = build_depth()
    mask = rasterize_bbox(RECT_BBOX, HEIGHT, WIDTH)
    valid_masked = mask.data & np.isfinite(depth) & (depth > 0.0)
    monkeypatch.setattr(
        euclidean_reconstruction,
        'valid_depth',
        lambda *args: pytest.fail('precomputed validity should be reused'),
    )

    result, reason = localize_euclidean_reconstruction(
        depth,
        mask,
        INTRINSICS,
        isolation=default_isolation(),
        valid_masked=valid_masked,
    )

    assert result is not None
    assert reason is MissReason.OK


def test_tight_mask_mad_pass_drops_edge_bleed() -> None:
    depth = build_depth()
    # Five tight-mask pixels bleed onto the far background.
    bleed = [(20, 30), (20, 49), (39, 30), (39, 49), (29, 40)]
    for row, col in bleed:
        depth[row, col] = 6.0
    mask = mask_from_array(object_mask_data(), MaskPrecision.TIGHT)

    result, _ = localize_euclidean_reconstruction(depth, mask, INTRINSICS)

    assert result is not None
    # Exactly the bleed goes: 400 plate pixels in, the 5 at 6 m out, and no
    # legitimate plate point with them. Pinned rather than bounded because the
    # margin is the whole point -- against a raw MAD the same cut also took 6
    # real far-corner points, and a loose bound hid that.
    assert result.foreground_points[:, 2].max() < 3.0
    assert result.foreground_points.shape[0] == 400 - len(bleed)
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
