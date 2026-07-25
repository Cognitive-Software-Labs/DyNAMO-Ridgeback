"""2D foreground isolation recipes for projective ranging's ``rect`` branch.

Contract (``projective_ranging.md`` Section 2.3): input the aligned depth frame and
a mask, both ``H x W`` on the color grid; output the **foreground pixels** as
an ``H x W`` boolean array -- always a subset of the valid masked pixels. An
empty (all-``False``) result is allowed and means the recipe found no
foreground.

Each recipe is one entry of the catalogue in ``foreground_isolation_2d.md``.
They are registered in ``ISOLATION_2D_RECIPES`` so choosing one is a config
choice, and the benchmark swaps recipes behind this contract to compare them
on identical input.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from ridgeback_autonomy.perception.core.depth_common import (
    DEPTH_MAX_METERS_DEFAULT,
    valid_depth,
)


NEAREST_MODE_BIN_WIDTH_M_DEFAULT = 0.05
NEAREST_MODE_BAND_M_DEFAULT = 0.35
NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT = 0.05

OTSU_BIN_WIDTH_M_DEFAULT = 0.05


def _valid_masked(depth_m: np.ndarray, mask: np.ndarray, depth_max: float):
    """Shared select+clean prologue: the valid masked pixels and their depths."""

    valid = np.asarray(mask, dtype=bool) & valid_depth(depth_m, depth_max)
    return valid, np.asarray(depth_m)[valid]


def nearest_mode_histogram(
    depth_m: np.ndarray,
    mask: np.ndarray,
    *,
    bin_width_m: float = NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    band_m: float = NEAREST_MODE_BAND_M_DEFAULT,
    min_bin_fraction: float = NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
) -> np.ndarray:
    """Catalogue #1 (baseline): nearest significant depth mode, fixed band.

    Histogram the valid masked depths, take the nearest bin holding at least
    ``min_bin_fraction`` of them as the object's near surface, and keep the
    pixels within ``band_m`` of that peak. Assumes the subject is the nearest
    coherent surface in the box.
    """

    valid, values = _valid_masked(depth_m, mask, depth_max)
    if values.size == 0:
        return np.zeros_like(valid)

    low = float(values.min())
    high = float(values.max())
    num_bins = max(1, int(np.ceil((high - low) / bin_width_m)))
    hist, edges = np.histogram(
        values, bins=num_bins, range=(low, low + num_bins * bin_width_m))

    significant = np.flatnonzero(hist >= max(1.0, min_bin_fraction * values.size))
    if significant.size > 0:
        nearest = int(significant[0])
    else:
        # Dispersed distribution: no bin clears the significance floor (the
        # object's depth spread plus the floor ramp can dilute every bin at
        # range). Fall back to the NEAREST non-empty bin, not the global mode:
        # at range the biggest coherent bin is the background wall, so
        # ``argmax`` would confidently isolate the wall instead of the subject.
        # ``values.size > 0`` guarantees at least one non-empty bin.
        nearest = int(np.flatnonzero(hist >= 1)[0])
    peak_m = 0.5 * (edges[nearest] + edges[nearest + 1])

    foreground = np.zeros_like(valid)
    foreground[valid] = np.abs(values - peak_m) <= band_m
    return foreground


def otsu_foreground(
    depth_m: np.ndarray,
    mask: np.ndarray,
    *,
    bin_width_m: float = OTSU_BIN_WIDTH_M_DEFAULT,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
) -> np.ndarray:
    """Catalogue #2: Otsu threshold on the masked depth histogram, keep the near side.

    Picks the depth threshold maximizing the between-class variance of the
    (assumed bimodal) subject/background distribution; foreground is
    everything at or below it. Degenerates gracefully: a unimodal ROI whose
    depths fit one bin keeps every valid masked pixel.
    """

    valid, values = _valid_masked(depth_m, mask, depth_max)
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
