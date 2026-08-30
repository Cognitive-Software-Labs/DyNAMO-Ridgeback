from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.perception.target_localization.core.depth_common import valid_depth
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
    nearest_mode_histogram,
    otsu_foreground,
)


HEIGHT, WIDTH = 60, 80
OBJECT_DEPTH_M = 2.0
BACKGROUND_DEPTH_M = 4.0
# Object plate and the (oversized) rect ROI around it, as index slices.
OBJECT_SLICE = (slice(20, 40), slice(30, 50))
MASK_SLICE = (slice(15, 45), slice(25, 55))


def build_scene() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Bimodal ROI: object plate at 2 m over a 4 m background, plus invalids.

    Returns ``(depth, mask, object_region)`` where ``object_region`` marks the
    *valid* object pixels the recipes should recover.
    """

    depth = np.full((HEIGHT, WIDTH), BACKGROUND_DEPTH_M, dtype=np.float32)
    depth[OBJECT_SLICE] = OBJECT_DEPTH_M

    # Invalid pixels inside the ROI: two on the object, four on background.
    depth[20, 30] = 0.0
    depth[21, 31] = np.nan
    depth[16, 26] = 0.0
    depth[17, 27] = np.nan
    depth[18, 28] = np.inf
    depth[19, 29] = 15.0  # out of range

    mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
    mask[MASK_SLICE] = True

    object_region = np.zeros((HEIGHT, WIDTH), dtype=bool)
    object_region[OBJECT_SLICE] = True
    object_region &= valid_depth(depth)
    return depth, mask, object_region


@pytest.mark.parametrize('recipe', [nearest_mode_histogram, otsu_foreground])
def test_recipe_recovers_exactly_the_object_pixels(recipe) -> None:
    depth, mask, object_region = build_scene()

    foreground = recipe(depth, mask)

    assert foreground.shape == mask.shape
    assert foreground.dtype == np.bool_
    assert np.array_equal(foreground, object_region)


@pytest.mark.parametrize('recipe', [nearest_mode_histogram, otsu_foreground])
def test_recipe_output_is_subset_of_valid_masked_pixels(recipe) -> None:
    depth, mask, _ = build_scene()

    foreground = recipe(depth, mask)

    assert not np.any(foreground & ~(mask & valid_depth(depth)))


@pytest.mark.parametrize('recipe', [nearest_mode_histogram, otsu_foreground])
def test_unimodal_roi_keeps_all_valid_masked_pixels(recipe) -> None:
    # A tight-fitting box over the object only: one mode, nothing to isolate.
    depth = np.full((HEIGHT, WIDTH), OBJECT_DEPTH_M, dtype=np.float32)
    mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
    mask[OBJECT_SLICE] = True

    foreground = recipe(depth, mask)

    assert np.array_equal(foreground, mask)


def test_nearest_mode_dispersed_histogram_picks_nearest_not_global_mode() -> None:
    # Regression: when the histogram is so dispersed that NO bin
    # clears the significance floor, the recipe must fall back to the NEAREST
    # non-empty bin, not the global argmax. Here a thin near cluster (~2 m,
    # fewer pixels) sits in front of a larger dispersed far wall (~5 m): the
    # global mode is a far-wall bin, but the subject is the near cluster.
    near_rows = 10
    depth = np.empty((HEIGHT, WIDTH), dtype=np.float32)
    # Near: 1 row per 0.05 m bin (~80 px/bin). Far: 2 rows per bin (~160 px/bin)
    # -> the far wall is the taller (global) mode, yet neither clears the 5%
    # floor (240 px), so the dispersed fallback is what runs.
    depth[:near_rows] = np.linspace(2.0, 2.5, near_rows, dtype=np.float32)[:, np.newaxis]
    depth[near_rows:] = np.linspace(
        5.0, 6.25, HEIGHT - near_rows, dtype=np.float32)[:, np.newaxis]
    mask = np.ones((HEIGHT, WIDTH), dtype=bool)

    # Precondition: the far wall really is the more populated (global) mode, so
    # this test would fail under the old argmax fallback.
    values = depth[mask]
    hist, _ = np.histogram(
        values, bins=int(np.ceil((values.max() - values.min()) / 0.05)))
    assert hist.max() < 0.05 * values.size  # no bin clears the floor -> fallback

    foreground = nearest_mode_histogram(depth, mask)

    assert foreground.shape == mask.shape
    assert foreground.any()
    assert not np.any(foreground & ~valid_depth(depth))
    # The near cluster is isolated; the far wall is never selected.
    assert depth[foreground].max() < 3.0
    assert not np.any(foreground & (depth > 3.0))


@pytest.mark.parametrize('recipe', [nearest_mode_histogram, otsu_foreground])
def test_all_invalid_roi_yields_empty_foreground(recipe) -> None:
    depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)
    mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
    mask[MASK_SLICE] = True

    foreground = recipe(depth, mask)

    assert foreground.shape == mask.shape
    assert not foreground.any()


def test_registry_exposes_both_recipes_and_a_default() -> None:
    assert ISOLATION_2D_RECIPES['nearest_mode_histogram'] is nearest_mode_histogram
    assert ISOLATION_2D_RECIPES['otsu'] is otsu_foreground
    assert ISOLATION_2D_DEFAULT in ISOLATION_2D_RECIPES
