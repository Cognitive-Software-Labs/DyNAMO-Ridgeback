from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.perception.core.mask import (
    Mask,
    MaskPrecision,
    mask_from_array,
    masked_rgb,
    rasterize_batch,
    rasterize_bbox,
    rasterize_detection,
)


def test_rasterize_bbox_shape_dtype_and_tag() -> None:
    mask = rasterize_bbox((10, 20, 40, 50), image_height=100, image_width=200)

    assert mask.data.shape == (100, 200)
    assert mask.data.dtype == np.bool_
    assert mask.height == 100
    assert mask.width == 200
    assert mask.precision is MaskPrecision.RECT
    assert mask.precision == 'rect'


def test_rasterize_bbox_fills_exact_half_open_region() -> None:
    x1, y1, x2, y2 = 10, 20, 40, 50
    mask = rasterize_bbox((x1, y1, x2, y2), 100, 200)

    assert int(np.count_nonzero(mask.data)) == (x2 - x1) * (y2 - y1)
    # Corners inside the box are True.
    assert mask.data[y1, x1]
    assert mask.data[y2 - 1, x2 - 1]
    # Half-open: the row y2 and column x2 are excluded.
    assert not mask.data[y2, x1]
    assert not mask.data[y1, x2]
    # Just outside the top-left is False.
    assert not mask.data[y1 - 1, x1]
    assert not mask.data[y1, x1 - 1]


def test_rasterize_bbox_full_frame_all_true() -> None:
    mask = rasterize_bbox((0, 0, 8, 6), 6, 8)

    assert mask.data.all()


def test_rasterize_bbox_single_pixel() -> None:
    mask = rasterize_bbox((3, 4, 4, 5), 10, 10)

    assert int(np.count_nonzero(mask.data)) == 1
    assert mask.data[4, 3]


def test_rasterize_bbox_clamps_out_of_bounds() -> None:
    mask = rasterize_bbox((-5, -5, 999, 999), 6, 8)

    assert mask.data.all()


def test_rasterize_bbox_degenerate_box_is_empty() -> None:
    assert not rasterize_bbox((10, 10, 10, 20), 30, 30).data.any()  # x2 == x1
    assert not rasterize_bbox((10, 10, 20, 10), 30, 30).data.any()  # y2 == y1
    assert not rasterize_bbox((20, 10, 10, 20), 30, 30).data.any()  # x2 < x1


def test_rasterize_detection_uses_bbox() -> None:
    detection = Detection(bbox_xyxy=(1, 2, 5, 6), label='humanoid robot', score=0.9)
    mask = rasterize_detection(detection, 10, 10)

    assert int(np.count_nonzero(mask.data)) == (5 - 1) * (6 - 2)
    assert mask.precision is MaskPrecision.RECT


def test_rasterize_batch_union_disjoint_boxes() -> None:
    batch = DetectionBatch(
        image_width=20,
        image_height=20,
        detections=[
            Detection(bbox_xyxy=(0, 0, 4, 4), label='a', score=0.9),
            Detection(bbox_xyxy=(10, 10, 14, 14), label='b', score=0.8),
        ],
    )
    mask = rasterize_batch(batch)

    assert mask.data.shape == (20, 20)
    assert int(np.count_nonzero(mask.data)) == 16 + 16


def test_rasterize_batch_union_overlapping_no_double_count() -> None:
    batch = DetectionBatch(
        image_width=20,
        image_height=20,
        detections=[
            Detection(bbox_xyxy=(0, 0, 10, 10), label='a', score=0.9),
            Detection(bbox_xyxy=(5, 5, 15, 15), label='b', score=0.8),
        ],
    )
    mask = rasterize_batch(batch)

    # union = area(A) + area(B) - area(intersection)
    assert int(np.count_nonzero(mask.data)) == 100 + 100 - 25


def test_rasterize_batch_empty_is_all_false() -> None:
    batch = DetectionBatch(image_width=8, image_height=6, detections=[])
    mask = rasterize_batch(batch)

    assert mask.data.shape == (6, 8)
    assert not mask.data.any()
    assert mask.precision is MaskPrecision.RECT


def test_masked_rgb_keeps_inside_zeros_outside() -> None:
    rgb = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    mask = rasterize_bbox((2, 1, 5, 4), 6, 8)
    out = masked_rgb(rgb, mask)

    assert out.shape == rgb.shape
    assert out.dtype == rgb.dtype
    assert np.array_equal(out[mask.data], rgb[mask.data])
    assert not out[~mask.data].any()


def test_masked_rgb_empty_mask_all_black() -> None:
    rgb = np.full((4, 4, 3), 200, dtype=np.uint8)
    mask = Mask(data=np.zeros((4, 4), dtype=bool), precision=MaskPrecision.RECT)
    out = masked_rgb(rgb, mask)

    assert not out.any()


def test_mask_from_array_builds_tight_mask_from_arbitrary_blob() -> None:
    # A non-rectangular (diagonal) blob -- what a segmentation front-end emits.
    blob = np.eye(5, dtype=bool)
    mask = mask_from_array(blob, MaskPrecision.TIGHT)

    assert mask.precision is MaskPrecision.TIGHT
    assert mask.precision == 'tight'
    assert mask.height == 5
    assert mask.width == 5
    assert int(np.count_nonzero(mask.data)) == 5


def test_masked_rgb_works_on_non_rect_tight_mask() -> None:
    rgb = np.arange(5 * 5 * 3, dtype=np.uint8).reshape(5, 5, 3)
    mask = mask_from_array(np.eye(5, dtype=bool), MaskPrecision.TIGHT)
    out = masked_rgb(rgb, mask)

    # Diagonal pixels keep RGB; everything off-diagonal is black.
    assert np.array_equal(out[mask.data], rgb[mask.data])
    assert not out[~mask.data].any()


def test_mask_contract_rejects_non_2d_and_non_bool() -> None:
    with pytest.raises(ValueError):
        Mask(data=np.zeros((4, 4, 3), dtype=bool), precision=MaskPrecision.RECT)
    with pytest.raises(ValueError):
        Mask(data=np.zeros((4, 4), dtype=np.uint8), precision=MaskPrecision.RECT)
    with pytest.raises(ValueError):
        mask_from_array(np.zeros((4, 4), dtype=np.float32), MaskPrecision.TIGHT)
