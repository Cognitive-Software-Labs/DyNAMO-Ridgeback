"""Contract and pixel geometry of the ROI-native ``MaskRegion``.

Storage is local, meaning is global: every test here is ultimately asking
whether a region and the full-grid mask it materializes to select the same
pixels, and whether the window ever leaks into a coordinate that leaves it.
Numerical parity of the estimators reading these regions lives in
``test_mask_region_parity.py``.
"""

from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.common.models import Detection
from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    MaskRegion,
    as_mask_region,
    clamp_box,
    empty_region,
    mask_from_array,
    masked_rgb,
    rasterize_bbox,
    region_from_bbox,
    region_from_blob,
    region_from_detection,
)


def blob_with_holes_and_islands(height: int = 40, width: int = 60) -> np.ndarray:
    """A silhouette that is nothing like its bounding box."""

    data = np.zeros((height, width), dtype=bool)
    data[10:25, 20:40] = True
    data[15:18, 26:31] = False       # hole
    data[30:33, 46:50] = True        # disconnected island, down and to the right
    data[12, 5] = True               # lone pixel, to the LEFT of the main body
    return data


# --- region contract -------------------------------------------------------


def test_bbox_region_stores_only_the_box_and_is_all_true() -> None:
    region = region_from_bbox((20, 10, 50, 30), image_height=40, image_width=60)

    assert region.roi_shape == (20, 30)
    assert region.image_shape == (40, 60)
    assert (region.origin_u, region.origin_v) == (20, 10)
    assert region.data.all()
    assert region.precision is MaskPrecision.RECT


def test_region_payload_is_owned_and_read_only() -> None:
    blob = blob_with_holes_and_islands()
    region = region_from_blob(blob, MaskPrecision.TIGHT)

    # Not a view of the model's full-frame output: the frame is free to expire.
    assert region.data.base is None
    assert not region.data.flags.writeable
    blob[:] = False
    assert region.data.any(), 'region must not alias the producer buffer'

    with pytest.raises(ValueError):
        region.data[0, 0] = True


def test_bbox_region_payload_is_not_an_alias_of_another_region() -> None:
    first = region_from_bbox((0, 0, 4, 4), 10, 10)
    second = region_from_bbox((0, 0, 4, 4), 10, 10)

    assert first.data is not second.data


def test_canonical_empty_region_is_a_selector_not_none() -> None:
    region = empty_region(40, 60, MaskPrecision.TIGHT)

    assert region is not None
    assert region.roi_shape == (0, 0)
    assert (region.origin_u, region.origin_v) == (0, 0)
    assert region.image_shape == (40, 60)
    assert region.precision is MaskPrecision.TIGHT
    assert region.is_empty
    assert not region.to_full_array().any()


@pytest.mark.parametrize('bbox', [
    (10, 10, 10, 20),   # x2 == x1
    (10, 10, 20, 10),   # y2 == y1
    (20, 10, 10, 20),   # inverted
    (99, 99, 120, 120),  # entirely outside
])
def test_degenerate_boxes_collapse_to_the_canonical_empty_region(bbox) -> None:
    region = region_from_bbox(bbox, 30, 30)

    assert region.roi_shape == (0, 0)
    assert region.is_empty


def test_all_false_payload_is_empty_but_keeps_its_window() -> None:
    """An all-``False`` crop is still a selector with a real window."""

    region = MaskRegion(
        data=np.zeros((3, 4), dtype=bool),
        origin_u=5, origin_v=6, image_width=20, image_height=20,
        precision=MaskPrecision.TIGHT,
    )

    assert region.is_empty
    assert region.roi_shape == (3, 4)


def test_precision_is_carried_never_inferred_from_an_all_true_crop() -> None:
    """A rectangular silhouette stays TIGHT -- it must not become a box."""

    rectangular_silhouette = np.zeros((40, 60), dtype=bool)
    rectangular_silhouette[10:20, 20:30] = True
    region = region_from_blob(rectangular_silhouette, MaskPrecision.TIGHT)

    assert region.data.all()
    assert region.precision is MaskPrecision.TIGHT


def test_region_rejects_a_window_outside_the_grid() -> None:
    with pytest.raises(ValueError, match='does not fit'):
        MaskRegion(
            data=np.ones((5, 5), dtype=bool),
            origin_u=58, origin_v=0, image_width=60, image_height=40,
            precision=MaskPrecision.RECT,
        )


@pytest.mark.parametrize('kwargs', [
    {'data': np.ones((2, 2), dtype=np.float32)},
    {'data': np.ones((2, 2, 2), dtype=bool)},
    {'origin_u': -1},
    {'origin_v': -1},
    {'origin_u': 1.5},
])
def test_region_validates_its_payload_and_bounds(kwargs) -> None:
    fields = dict(
        data=np.ones((2, 2), dtype=bool),
        origin_u=0, origin_v=0, image_width=10, image_height=10,
        precision=MaskPrecision.RECT,
    )
    fields.update(kwargs)

    with pytest.raises(ValueError):
        MaskRegion(**fields)


def test_bbox_region_matches_the_full_grid_rasterizer_pixel_for_pixel() -> None:
    for bbox in [(10, 20, 40, 50), (-5, -5, 999, 999), (0, 0, 8, 6), (3, 4, 4, 5)]:
        region = region_from_bbox(bbox, 60, 80)
        assert np.array_equal(
            region.to_full_array(), rasterize_bbox(bbox, 60, 80).data), bbox


def test_fractional_box_coordinates_truncate_the_same_way_in_both_forms() -> None:
    bbox = (10.7, 20.2, 40.9, 50.4)

    assert clamp_box(bbox, 60, 80) == (10, 20, 40, 50)
    assert np.array_equal(
        region_from_bbox(bbox, 60, 80).to_full_array(),
        rasterize_bbox(bbox, 60, 80).data)


def test_region_from_detection_uses_the_detection_box() -> None:
    detection = Detection(label='x', score=0.9, bbox_xyxy=(2, 3, 6, 8))

    region = region_from_detection(detection, 10, 10)

    assert (region.origin_u, region.origin_v) == (2, 3)
    assert region.roi_shape == (5, 4)


# --- tight-mask cropping ---------------------------------------------------


def test_blob_crops_to_its_own_extent_not_to_any_box() -> None:
    blob = blob_with_holes_and_islands()

    region = region_from_blob(blob, MaskPrecision.TIGHT)

    rows, cols = np.nonzero(blob)
    assert (region.origin_v, region.origin_u) == (int(rows.min()), int(cols.min()))
    assert region.roi_shape == (
        int(rows.max()) - int(rows.min()) + 1,
        int(cols.max()) - int(cols.min()) + 1,
    )


def test_blob_crop_keeps_holes_islands_and_pixels_outside_the_main_body() -> None:
    blob = blob_with_holes_and_islands()

    region = region_from_blob(blob, MaskPrecision.TIGHT)

    assert np.array_equal(region.to_full_array(), blob)
    assert int(region.data.sum()) == int(blob.sum())


def test_blob_crop_is_smaller_than_the_frame_it_came_from() -> None:
    blob = blob_with_holes_and_islands(240, 320)
    blob[:] = False
    blob[100:120, 150:170] = True

    region = region_from_blob(blob, MaskPrecision.TIGHT)

    assert region.data.nbytes < blob.nbytes / 100


def test_foreground_beyond_the_prompt_box_survives_cropping() -> None:
    """A padded prompt can return silhouette pixels outside the detector box."""

    detector_box = (20, 10, 40, 25)
    blob = np.zeros((40, 60), dtype=bool)
    blob[10:25, 20:40] = True
    blob[26:29, 41:45] = True  # below-right of the box: real, must be kept

    region = region_from_blob(blob, MaskPrecision.TIGHT)
    box_region = region_from_bbox(detector_box, 40, 60)

    assert region.data.sum() == blob.sum()
    assert region.contains_pixels(np.array([43]), np.array([27]))[0]
    assert not box_region.contains_pixels(np.array([43]), np.array([27]))[0]


def test_all_false_blob_yields_the_canonical_empty_region() -> None:
    region = region_from_blob(np.zeros((40, 60), dtype=bool), MaskPrecision.TIGHT)

    assert region.roi_shape == (0, 0)
    assert region.image_shape == (40, 60)


@pytest.mark.parametrize('blob', [
    np.ones((4, 4), dtype=np.float32),
    np.ones((4, 4, 4), dtype=bool),
])
def test_blob_constructor_validates_its_input(blob) -> None:
    with pytest.raises(ValueError):
        region_from_blob(blob, MaskPrecision.TIGHT)


# --- pixel geometry --------------------------------------------------------


def test_membership_matches_the_materialized_mask_everywhere() -> None:
    blob = blob_with_holes_and_islands()
    region = region_from_blob(blob, MaskPrecision.TIGHT)
    rows, cols = np.mgrid[0:40, 0:60]

    membership = region.contains_pixels(cols.ravel(), rows.ravel())

    assert np.array_equal(membership.reshape(40, 60), blob)


def test_membership_rejects_coordinates_outside_the_window() -> None:
    region = region_from_bbox((20, 10, 40, 25), 40, 60)

    # Left of and above the window; a raw local index would wrap to the far edge.
    assert not region.contains_pixels(np.array([19, 0, 5]), np.array([12, 12, 9])).any()
    # Right of and below it.
    assert not region.contains_pixels(np.array([40, 59]), np.array([12, 24])).any()
    # The four corners of the half-open window are inside.
    assert region.contains_pixels(
        np.array([20, 39, 20, 39]), np.array([10, 10, 24, 24])).all()


def test_membership_never_wraps_a_negative_index() -> None:
    """A window at the far edge: a negative local index would read a True pixel."""

    region = region_from_bbox((56, 36, 60, 40), 40, 60)

    assert not region.contains_pixels(np.array([-1]), np.array([-1]))[0]
    assert not region.contains_pixels(np.array([55]), np.array([36]))[0]
    assert region.contains_pixels(np.array([56]), np.array([36]))[0]


def test_membership_of_an_empty_region_is_all_false() -> None:
    region = empty_region(40, 60, MaskPrecision.TIGHT)

    assert not region.contains_pixels(np.array([0, 5]), np.array([0, 5])).any()


@pytest.mark.parametrize('bbox,expected_shape', [
    ((0, 0, 1, 1), (1, 1)),          # one pixel, at the origin
    ((59, 39, 60, 40), (1, 1)),      # one pixel, at the far corner
    ((0, 0, 60, 40), (40, 60)),      # the whole image
    ((0, 10, 60, 20), (10, 60)),     # a full-width band
])
def test_edge_windows_round_trip_through_materialization(bbox, expected_shape) -> None:
    region = region_from_bbox(bbox, 40, 60)

    assert region.roi_shape == expected_shape
    assert np.array_equal(
        region_from_blob(region.to_full_array(), MaskPrecision.RECT).data,
        region.data)


def test_global_pixels_offsets_rows_and_columns_by_the_origin() -> None:
    region = region_from_bbox((20, 10, 40, 25), 40, 60)
    local_rows, local_cols = np.nonzero(region.data)

    rows, cols = region.global_pixels(local_rows, local_cols)

    assert np.array_equal(rows, local_rows + 10)
    assert np.array_equal(cols, local_cols + 20)
    # And those global coordinates index exactly the materialized mask.
    full = region.to_full_array()
    assert full[rows, cols].all()
    assert int(full.sum()) == rows.size


def test_slice_image_aligns_with_the_payload() -> None:
    image = np.arange(40 * 60, dtype=np.float32).reshape(40, 60)
    region = region_from_bbox((20, 10, 40, 25), 40, 60)

    window = region.slice_image(image)

    assert window.shape == region.roi_shape
    assert window[0, 0] == image[10, 20]
    assert window[-1, -1] == image[24, 39]


def test_global_pixels_returns_its_inputs_when_the_origin_is_zero() -> None:
    region = region_from_bbox((0, 0, 10, 10), 40, 60)
    local_rows, local_cols = np.nonzero(region.data)

    rows, cols = region.global_pixels(local_rows, local_cols)

    assert rows is local_rows
    assert cols is local_cols


def test_slice_image_returns_the_array_itself_for_a_full_grid_window() -> None:
    image = np.zeros((40, 60), dtype=bool)
    region = region_from_bbox((0, 0, 60, 40), 40, 60)

    assert region.slice_image(image) is image


def test_slice_image_rejects_an_already_cropped_array() -> None:
    """Guards the offset-twice bug: cropped input plus an origin offset."""

    region = region_from_bbox((20, 10, 40, 25), 40, 60)
    cropped = np.zeros(region.roi_shape, dtype=np.float32)

    with pytest.raises(ValueError, match='full-grid'):
        region.slice_image(cropped)


def test_blit_writes_only_the_selected_pixels() -> None:
    blob = blob_with_holes_and_islands()
    region = region_from_blob(blob, MaskPrecision.TIGHT)
    destination = np.zeros((40, 60), dtype=np.uint8)

    region.blit_into(destination, 255)

    assert np.array_equal(destination == 255, blob)


def test_blitting_several_regions_builds_their_union() -> None:
    left = region_from_bbox((5, 5, 15, 15), 40, 60)
    right = region_from_bbox((10, 5, 25, 12), 40, 60)
    destination = np.zeros((40, 60), dtype=np.uint8)

    left.blit_into(destination, 255)
    right.blit_into(destination, 255)

    expected = left.to_full_array() | right.to_full_array()
    assert np.array_equal(destination == 255, expected)


def test_to_mask_materializes_the_same_selector_with_the_same_tag() -> None:
    blob = blob_with_holes_and_islands()
    region = region_from_blob(blob, MaskPrecision.TIGHT)

    mask = region.to_mask()

    assert np.array_equal(mask.data, blob)
    assert mask.precision is MaskPrecision.TIGHT
    assert (mask.height, mask.width) == region.image_shape


# --- the compatibility crossing --------------------------------------------


def test_as_mask_region_wraps_a_full_grid_mask_without_copying() -> None:
    blob = blob_with_holes_and_islands()
    mask = mask_from_array(blob, MaskPrecision.TIGHT)

    region = as_mask_region(mask)

    assert region.data is mask.data
    assert region.covers_full_grid
    assert (region.origin_u, region.origin_v) == (0, 0)
    assert region.image_shape == (40, 60)
    assert region.precision is MaskPrecision.TIGHT


def test_as_mask_region_passes_a_region_through_unchanged() -> None:
    region = region_from_bbox((1, 2, 3, 4), 10, 10)

    assert as_mask_region(region) is region


def test_mask_and_region_round_trip_pixel_for_pixel() -> None:
    blob = blob_with_holes_and_islands()

    once = region_from_blob(blob, MaskPrecision.TIGHT)
    twice = as_mask_region(once.to_mask())

    assert np.array_equal(twice.to_full_array(), blob)
    assert np.array_equal(
        region_from_blob(twice.to_full_array(), MaskPrecision.TIGHT).data, once.data)


def test_masked_rgb_agrees_between_the_two_representations() -> None:
    blob = blob_with_holes_and_islands()
    rgb = np.arange(40 * 60 * 3, dtype=np.uint8).reshape(40, 60, 3)

    from_region = masked_rgb(rgb, region_from_blob(blob, MaskPrecision.TIGHT))
    from_mask = masked_rgb(rgb, mask_from_array(blob, MaskPrecision.TIGHT))

    assert np.array_equal(from_region, from_mask)
    assert np.array_equal(from_region[blob], rgb[blob])
    assert not from_region[~blob].any()
