"""Structural proof that production stays region-sized.

These are allocation-size assertions, not timings: ``tracemalloc``'s peak
counter covers numpy's own data allocations, including temporaries that are
freed again before the call returns, so it can see a full-frame array that was
built and thrown away. Nothing here asserts a speedup -- smaller storage is not
the same claim as lower end-to-end latency.

The bounds are stated relative to one full-frame float64 array, which is the
size of the thing being ruled out, rather than as tuned constants.
"""

from __future__ import annotations

import tracemalloc

import numpy as np
import pytest

from ridgeback_autonomy.common.models import Detection, DetectionBatch
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    prepare_depth_region,
    valid_depth,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import (
    CameraIntrinsics,
    deproject_masked,
)
from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    MaskRegion,
    region_from_bbox,
    region_from_blob,
)
from ridgeback_autonomy.perception.target_localization.measurement_pipeline import (
    encode_mask_debug_image,
    fill_path_measurements,
)


HEIGHT, WIDTH = 480, 640
FRAME_FLOAT64_BYTES = HEIGHT * WIDTH * 8
# A production-representative detection: well under the oversized-box gate.
BOX = (300, 200, 380, 360)
INTRINSICS = CameraIntrinsics(
    fx=600.0, fy=600.0, cx=WIDTH / 2, cy=HEIGHT / 2, width=WIDTH, height=HEIGHT)
LEVEL_OPTICAL_TO_BASE = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)
ZERO_TRANSLATION = np.zeros(3, dtype=np.float64)


def peak_bytes(call) -> int:
    """Peak bytes allocated during ``call``, numpy data buffers included."""

    tracemalloc.start()
    try:
        tracemalloc.reset_peak()
        call()
        return tracemalloc.get_traced_memory()[1]
    finally:
        tracemalloc.stop()


def build_depth() -> np.ndarray:
    depth = np.full((HEIGHT, WIDTH), 6.0, dtype=np.float32)
    depth[200:360, 300:380] = 2.4
    return depth


def build_batch(count: int = 1) -> DetectionBatch:
    return DetectionBatch(
        image_width=WIDTH,
        image_height=HEIGHT,
        detections=[
            Detection(bbox_xyxy=BOX, label='humanoid robot', score=0.9)
            for _ in range(count)
        ],
    )


def run_fill(masks, depth, batch) -> None:
    fill_path_measurements(
        batch, masks, INTRINSICS, depth, None,
        camera_rotation=LEVEL_OPTICAL_TO_BASE,
        camera_translation=ZERO_TRANSLATION,
        front_offset_m=0.25,
        isolation_2d=None,
        isolation_3d=None,
    )


# --- storage ---------------------------------------------------------------


def test_box_region_stores_only_its_own_box() -> None:
    region = region_from_bbox(BOX, HEIGHT, WIDTH)
    full_mask_bytes = HEIGHT * WIDTH  # one bool per pixel

    assert region.data.nbytes == (BOX[3] - BOX[1]) * (BOX[2] - BOX[0])
    assert region.data.nbytes < full_mask_bytes / 20


def test_tight_region_does_not_retain_the_model_output_as_backing_storage() -> None:
    blob = np.zeros((HEIGHT, WIDTH), dtype=bool)
    blob[200:360, 300:380] = True

    region = region_from_blob(blob, MaskPrecision.TIGHT)

    assert region.data.base is None
    assert region.data.nbytes < blob.nbytes / 20


def test_cropping_a_blob_costs_at_most_one_extra_crop() -> None:
    blob = np.zeros((HEIGHT, WIDTH), dtype=bool)
    blob[200:360, 300:380] = True
    crop_bytes = 160 * 80

    peak = peak_bytes(lambda: region_from_blob(blob, MaskPrecision.TIGHT))

    # The extent scan produces per-axis reductions, not another full frame.
    assert peak < blob.nbytes / 2
    assert peak >= crop_bytes


# --- per-detection temporaries ---------------------------------------------


def test_prepared_region_arrays_are_region_sized() -> None:
    depth = build_depth()
    region = region_from_bbox(BOX, HEIGHT, WIDTH)

    prepared = prepare_depth_region(region, depth, valid_depth(depth, np.inf))

    assert prepared.roi_depth.shape == region.roi_shape
    assert prepared.valid_masked.shape == region.roi_shape
    # The frame itself is retained by reference, never copied.
    assert prepared.depth_full is depth
    assert prepared.roi_depth.base is depth


def test_deproject_masked_does_not_widen_the_whole_frame() -> None:
    depth = build_depth()
    rows, cols = np.nonzero(region_from_bbox(BOX, HEIGHT, WIDTH).to_full_array())

    helper = peak_bytes(lambda: deproject_masked(depth, rows, cols, INTRINSICS))
    cast_first = peak_bytes(
        lambda: np.asarray(depth, dtype=np.float64)[rows, cols])

    # Casting first really does cost a whole float64 frame, per call. The
    # helper -- which production hands the WHOLE frame plus global indices --
    # stays below that, so the selection is what it pays for.
    assert cast_first > FRAME_FLOAT64_BYTES
    assert helper < FRAME_FLOAT64_BYTES
    assert helper >= rows.size * 3 * 8


# --- the production pipeline -----------------------------------------------


def test_fill_never_materializes_a_full_mask_per_detection(monkeypatch) -> None:
    def forbidden(self):
        raise AssertionError('production must not materialize a full-grid mask')

    monkeypatch.setattr(MaskRegion, 'to_full_array', forbidden)
    monkeypatch.setattr(MaskRegion, 'to_mask', forbidden)
    batch = build_batch(count=3)
    masks = [region_from_bbox(BOX, HEIGHT, WIDTH) for _ in range(3)]

    run_fill(masks, build_depth(), batch)

    for detection in batch.detections:
        assert detection.projective_ranging_distance_m is not None
        assert detection.euclidean_reconstruction_distance_m is not None


@pytest.mark.parametrize('count', [1, 8])
def test_fill_allocation_does_not_grow_by_a_frame_per_detection(count) -> None:
    depth = build_depth()
    batch = build_batch(count=count)
    masks = [region_from_bbox(BOX, HEIGHT, WIDTH) for _ in range(count)]

    peak = peak_bytes(lambda: run_fill(masks, depth, batch))

    # One shared frame-wide validity image (bool) is allowed; a float64 frame
    # per detection is what this rules out.
    assert peak < FRAME_FLOAT64_BYTES


def test_debug_union_allocates_one_output_not_one_mask_per_detection() -> None:
    class Header:
        pass

    masks = [region_from_bbox(BOX, HEIGHT, WIDTH) for _ in range(8)]
    union_bytes = HEIGHT * WIDTH

    peak = peak_bytes(
        lambda: encode_mask_debug_image(masks, HEIGHT, WIDTH, Header()))

    # One uint8 output plus its serialized bytes; eight full-grid bool masks
    # would add another 8 * union_bytes on top.
    assert peak < 4 * union_bytes
