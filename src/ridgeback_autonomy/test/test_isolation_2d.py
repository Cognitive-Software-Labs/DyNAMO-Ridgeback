from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.perception.core.depth_common import valid_depth
from ridgeback_autonomy.perception.core.isolation_2d import (
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
