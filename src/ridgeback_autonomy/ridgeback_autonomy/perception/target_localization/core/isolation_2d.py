"""2D foreground isolation recipes for projective ranging's ``rect`` branch.

Contract (``docs/target_localization/projective_ranging.md`` Section 2.3): input the aligned depth frame and
a mask, both ``H x W`` on the color grid; output the **foreground pixels** as
an ``H x W`` boolean array -- always a subset of the valid masked pixels. An
empty (all-``False``) result is allowed and means the recipe found no
foreground.

Each recipe is documented with its owning estimator in
``docs/target_localization/projective_ranging.md``.
They are registered in ``ISOLATION_2D_RECIPES`` so choosing one is a config
choice, and the benchmark swaps recipes behind this contract to compare them
on identical input.
"""

from __future__ import annotations

import functools
import inspect
from typing import Callable

import numpy as np

from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
    nearest_significant_mode,
    valid_depth,
)

# Depth kept either side of the near-surface anchor. Its job is to span from the
# anchor to the far side of whatever the anchor landed on, so the bound that
# matters is the occluder standoff (0.62 m in the shipped scene set), not the
# target's own 0.4457 m depth extent: when an occluder takes the anchor, only a
# band wider than the standoff can reach the target behind it.
#
# 0.35 is therefore measured **wrong** -- 0.75 removes 88% of the error tail at
# no cost (docs/history/projective_parameter_sensitivity.md). It is left at the
# shipped value here because moving it is a behaviour change with goldens to
# regenerate, not part of the cleanup that gave it its own home.
#
# It lived in ``ranging_defaults`` until the polar path, which read the same
# constant as a lidar run-merge distance, was given its own. The two were equal
# by history, not by intent.
NEAREST_MODE_BAND_M_DEFAULT = 0.35

OTSU_BIN_WIDTH_M_DEFAULT = 0.05


def _valid_masked(
    depth_m: np.ndarray,
    mask: np.ndarray,
    depth_max: float | None,
    valid_masked: np.ndarray | None = None,
):
    """Shared select+clean prologue: the valid masked pixels and their depths.

    ``depth_max`` is read only when ``valid_masked`` is absent, so a caller
    that already cleaned its selection passes ``None``. Neither has a default:
    between them they decide which pixels a recipe may see, and inheriting that
    from a module constant is how a ceiling belonging to another estimator used
    to reach this one.
    """

    depth_m = np.asarray(depth_m)
    if valid_masked is None:
        if depth_max is None:
            raise ValueError(
                'A recipe needs a depth_max when valid_masked is not '
                'precomputed; there is no default ceiling to fall back on.')
        valid = np.asarray(mask, dtype=bool) & valid_depth(depth_m, depth_max)
    else:
        valid = np.asarray(valid_masked)
        if valid.dtype != np.bool_ or valid.shape != depth_m.shape:
            raise ValueError(
                'valid_masked must be a boolean array matching depth_m; '
                f'got dtype={valid.dtype}, shape={valid.shape}, '
                f'depth_shape={depth_m.shape}')
    return valid, depth_m[valid]


def nearest_mode_histogram(
    depth_m: np.ndarray,
    mask: np.ndarray,
    *,
    bin_width_m: float = NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    band_m: float = NEAREST_MODE_BAND_M_DEFAULT,
    min_bin_fraction: float = NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
    depth_max: float | None,
    valid_masked: np.ndarray | None = None,
) -> np.ndarray:
    """Catalogue #1 (baseline): nearest significant depth mode, fixed band.

    Histogram the valid masked depths, take the nearest bin holding at least
    ``min_bin_fraction`` of them as the object's near surface, and keep the
    pixels within ``band_m`` of that peak. Assumes the subject is the nearest
    coherent surface in the box. The anchor itself is
    ``depth_common.nearest_significant_mode``, shared with the point-domain
    twin so both domains place the near surface identically.
    """

    valid, values = _valid_masked(depth_m, mask, depth_max, valid_masked)
    if values.size == 0:
        return np.zeros_like(valid)

    peak_m = nearest_significant_mode(
        values, bin_width_m=bin_width_m, min_bin_fraction=min_bin_fraction)

    foreground = np.zeros_like(valid)
    foreground[valid] = np.abs(values - peak_m) <= band_m
    return foreground


def otsu_foreground(
    depth_m: np.ndarray,
    mask: np.ndarray,
    *,
    bin_width_m: float = OTSU_BIN_WIDTH_M_DEFAULT,
    depth_max: float | None,
    valid_masked: np.ndarray | None = None,
) -> np.ndarray:
    """Catalogue #2: Otsu threshold on the masked depth histogram, keep the near side.

    Picks the depth threshold maximizing the between-class variance of the
    (assumed bimodal) subject/background distribution; foreground is
    everything at or below it. Degenerates gracefully: a unimodal ROI whose
    depths fit one bin keeps every valid masked pixel.
    """

    valid, values = _valid_masked(depth_m, mask, depth_max, valid_masked)
    if values.size == 0:
        return np.zeros_like(valid)

    low = float(values.min())
    high = float(values.max())
    num_bins = int(np.ceil((high - low) / bin_width_m))
    if num_bins < 2:
        return valid.copy()

    hist, edges = np.histogram(
        values, bins=num_bins, range=(low, low + num_bins * bin_width_m))
    centers = 0.5 * (edges[:-1] + edges[1:])
    weights = hist.astype(np.float64) / values.size
    omega = np.cumsum(weights)
    mu = np.cumsum(weights * centers)
    mu_total = mu[-1]
    with np.errstate(divide='ignore', invalid='ignore'):
        sigma_between = (mu_total * omega - mu) ** 2 / (omega * (1.0 - omega))
    sigma_between[~np.isfinite(sigma_between)] = 0.0

    threshold_m = float(edges[int(np.argmax(sigma_between)) + 1])
    foreground = np.zeros_like(valid)
    foreground[valid] = values <= threshold_m
    return foreground


# The config swap point: recipes keyed by name, all satisfying the contract
# above. ``ISOLATION_2D_DEFAULT`` is what projective ranging uses when none is chosen.
ISOLATION_2D_RECIPES: dict[str, Callable[..., np.ndarray]] = {
    'nearest_mode_histogram': nearest_mode_histogram,
    'otsu': otsu_foreground,
}
ISOLATION_2D_DEFAULT = 'nearest_mode_histogram'

def build_isolation_2d(
    name: str,
    *,
    bin_width_m: float | None = None,
    band_m: float | None = None,
    min_bin_fraction: float | None = None,
) -> Callable[..., np.ndarray]:
    """Build the named recipe with runtime values in place of its own defaults.

    The 2D counterpart of ``isolation_3d.build_isolation_3d``, and the reason
    the numbers above are reachable from a launch argument at all. The recipes
    are plain functions rather than dataclasses, so the binding is a
    ``functools.partial`` instead of a constructor call.

    Recipes take different subsets of these settings -- ``otsu`` bins but has no
    band or significance floor -- so each value is bound only if that recipe
    accepts it. Passing one it does not is silently ignored rather than a
    ``TypeError``: one launch argument spans every recipe, and a sweep that
    varies the bin width across both must not fail on the recipe with fewer
    knobs. ``None`` means "leave the recipe's own default", so a caller that
    sets nothing gets exactly ``ISOLATION_2D_RECIPES[name]``.
    """

    if name not in ISOLATION_2D_RECIPES:
        supported = ', '.join(sorted(ISOLATION_2D_RECIPES))
        raise ValueError(f'Unknown isolation_2d recipe "{name}". Expected one of: {supported}')
    recipe = ISOLATION_2D_RECIPES[name]

    requested = {
        'bin_width_m': bin_width_m,
        'band_m': band_m,
        'min_bin_fraction': min_bin_fraction,
    }
    accepted = inspect.signature(recipe).parameters
    bound = {
        key: float(value)
        for key, value in requested.items()
        if value is not None and key in accepted
    }
    if not bound:
        return recipe
    return functools.partial(recipe, **bound)
