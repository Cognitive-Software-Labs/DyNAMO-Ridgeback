"""Projective ranging -- aggregate-then-deproject (``docs/target_localization/projective_ranging.md``).

The cheapest localization path: select the masked depths, clean them,
collapse them to one distance, and deproject exactly one representative
pixel. The aggregate step is the only place the mask's ``tight | rect`` tag
changes behavior: ``tight`` takes a robust median directly; ``rect`` first
isolates the foreground with a pluggable 2D recipe (``isolation_2d.py``).
That fork lives in ``select_foreground_pixels`` and nowhere else; the
reduction below it is single-sourced and never asks which tag it came from.

Two entry points, one implementation. ``localize_prepared_projective_ranging``
is the production one: it consumes a ``PreparedDepthRegion``, so the recipe
runs on ROI-sized arrays and only the surviving pixel coordinates are lifted
back onto the full grid. ``localize_projective_ranging`` is the standalone
full-frame signature -- same policy, same reduction, arrays the caller's own
shape, so a custom isolation callable still sees the absolute coordinates it
may depend on.

Runs per mask over a shared aligned depth frame; the caller loops masks
(1:1:1 hierarchy, ``docs/target_localization/mask_representation.md`` Section 6.1). A mask with too few
valid depth pixels is skipped by returning ``None`` -- the docs'
invalid-depth fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    PreparedDepthRegion,
    valid_depth,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import (
    CameraIntrinsics,
    deproject_pixel,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.target_localization.core.mask import Mask, MaskPrecision
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    MIN_VALID_SAMPLES as MIN_VALID_PIXELS_DEFAULT,
)


@dataclass(frozen=True)
class ProjectiveRangingResult:
    """One projective ranging localization: a point in the camera optical frame plus provenance."""

    xyz_optical: np.ndarray  # (3,) X right, Y down, Z forward, meters
    depth_m: float  # the aggregated (median) foreground depth
    representative_uv: tuple[float, float]  # centroid of the foreground pixels
    foreground_pixel_count: int


def select_foreground_pixels(
    depth: np.ndarray,
    mask_data: np.ndarray,
    precision: MaskPrecision,
    *,
    isolation: Callable[..., np.ndarray] | None,
    depth_max: float | None,
    min_valid_pixels: int,
    valid_masked: np.ndarray | None,
) -> tuple[np.ndarray | None, MissReason]:
    """The 2D foreground policy: the one place the precision tag is read.

    All three arrays share one grid -- either the full frame (standalone) or one
    mask's storage window (production). The recipes are shape-agnostic, and a
    window's valid pixels are the frame's valid pixels in the same row-major
    order, so both grids feed the histogram the identical value sequence.

    Returns ``(foreground, MissReason.OK)`` or ``(None, <reason>)``. The two
    shortfalls are split by *cause*, on both branches: a selection that never
    held enough valid depth is ``TOO_FEW_VALID_PIXELS``, and only a recipe that
    was given enough and returned too little is ``TOO_FEW_AFTER_ISOLATION``.
    The rect branch used to report the recipe's name for both, blaming an
    isolation that had rejected nothing, on the grounds that counting first
    meant a scan nothing else wanted. It does not: the validity array is
    computed here in either case -- the caller usually supplies it precomputed
    -- and is then handed to the recipe, which takes it rather than rescanning.
    Euclidean
    reconstruction has always split these, so this is the two paths agreeing
    about what they mean rather than a new policy.

    ``depth_max`` is the ceiling to clean against and is needed only when
    ``valid_masked`` is absent. A caller that already cleaned the selection has
    no second ceiling to apply and passes ``None``; there is no default,
    because the trustworthy range is a property of the depth source and the
    operator's gate rather than of this function.
    """

    if valid_masked is None and depth_max is None:
        raise ValueError(
            'select_foreground_pixels needs a depth_max when valid_masked is '
            'not precomputed; there is no default ceiling to fall back on.')

    # Sufficiency is judged on the selection before any recipe sees it, so the
    # answer means the same thing on both branches.
    if valid_masked is None:
        valid_masked = np.asarray(mask_data, dtype=bool) & valid_depth(depth, depth_max)
    if int(np.count_nonzero(valid_masked)) < min_valid_pixels:
        return None, MissReason.TOO_FEW_VALID_PIXELS

    if precision is MaskPrecision.TIGHT:
        # A tight silhouette already is the object: keep its valid depths.
        return valid_masked, MissReason.OK

    if isolation is None:
        isolation = ISOLATION_2D_RECIPES[ISOLATION_2D_DEFAULT]
    # The recipe is handed the validity array rather than the ceiling that
    # produced it, so it consumes the selection just counted instead of
    # rescanning for its own.
    foreground = isolation(
        depth, mask_data, depth_max=depth_max, valid_masked=valid_masked)
    if int(np.count_nonzero(foreground)) < min_valid_pixels:
        return None, MissReason.TOO_FEW_AFTER_ISOLATION
    return foreground, MissReason.OK


def _reduce_foreground(
    foreground_depths: np.ndarray,
    rows: np.ndarray,
    cols: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> ProjectiveRangingResult:
    """Collapse the foreground to one distance and one deprojected pixel.

    ``rows`` / ``cols`` are **full-grid** coordinates whichever entry point
    called: the intrinsics describe the color grid, so a window-local column
    would deproject to the wrong bearing. ``foreground_depths`` are those
    pixels' depths, already gathered.

    DEPROJECT the centroid of the foreground pixels -- never the raw box
    center, which can sit on a depth discontinuity. For a non-convex
    silhouette the centroid can land in a concavity (e.g. the gap between the
    legs), but that pixel's own depth is never read: Z is the median
    foreground depth, so distance stays robust wherever the centroid falls
    and (X, Y) is a body-center estimate. This is deliberate, not a gap to
    "fix" by snapping the pixel onto the mask.
    """

    aggregated_depth_m = float(np.median(foreground_depths))
    u = float(cols.mean())
    v = float(rows.mean())
    x, y, z = deproject_pixel(u, v, aggregated_depth_m, intrinsics)
    return ProjectiveRangingResult(
        xyz_optical=np.array((x, y, z), dtype=np.float64),
        depth_m=aggregated_depth_m,
        representative_uv=(u, v),
        foreground_pixel_count=int(rows.size),
    )


def localize_prepared_projective_ranging(
    prepared: PreparedDepthRegion,
    intrinsics: CameraIntrinsics,
    *,
    isolation: Callable[..., np.ndarray] | None = None,
    min_valid_pixels: int = MIN_VALID_PIXELS_DEFAULT,
) -> tuple[ProjectiveRangingResult | None, MissReason]:
    """Localize one prepared mask region -- the ROI-native production path.

    ``isolation`` receives **window-local** depth, mask and validity arrays, so
    it must be a built-in ``ISOLATION_2D_RECIPES`` entry or another recipe
    written against relative coordinates. A callable that reads absolute pixel
    positions belongs on ``localize_projective_ranging`` instead.

    There is no ``depth_max`` here: a ``PreparedDepthRegion`` arrives already
    cleaned against the caller's gate, so a ceiling on this signature would
    have nothing left to reject. It used to take one and thread it two levels
    down to be ignored. ``localize_prepared_euclidean_reconstruction`` never
    had one, and this now matches it.
    """

    region = prepared.region
    foreground, reason = select_foreground_pixels(
        prepared.roi_depth,
        region.data,
        region.precision,
        isolation=isolation,
        depth_max=None,
        min_valid_pixels=min_valid_pixels,
        valid_masked=prepared.valid_masked,
    )
    if foreground is None:
        return None, reason

    local_rows, local_cols = np.nonzero(foreground)
    # Depths come from the window; coordinates go back to the full grid.
    foreground_depths = prepared.roi_depth[local_rows, local_cols]
    rows, cols = region.global_pixels(local_rows, local_cols)
    return _reduce_foreground(foreground_depths, rows, cols, intrinsics), MissReason.OK


def localize_projective_ranging(
    depth_m: np.ndarray,
    mask: Mask,
    intrinsics: CameraIntrinsics,
    *,
    isolation: Callable[..., np.ndarray] | None = None,
    depth_max: float | None,
    min_valid_pixels: int = MIN_VALID_PIXELS_DEFAULT,
    valid_masked: np.ndarray | None = None,
) -> tuple[ProjectiveRangingResult | None, MissReason]:
    """Localize one full-grid mask against one aligned depth frame.

    The standalone signature: everything stays frame-shaped, so ``isolation``
    (an ``ISOLATION_2D_RECIPES`` entry, or any callable satisfying the same
    contract; default recipe when ``None``) sees the caller's own arrays --
    including a ``valid_masked`` it precomputed, passed through by identity.
    The ``tight`` branch never calls it. Returns ``(result, MissReason.OK)`` on
    success, or ``(None, <reason>)`` when fewer than ``min_valid_pixels`` valid
    (or, on the ``rect`` branch, foreground) pixels remain.
    """

    depth_m = np.asarray(depth_m)
    if valid_masked is not None:
        valid_masked = np.asarray(valid_masked)
        if valid_masked.dtype != np.bool_ or valid_masked.shape != depth_m.shape:
            raise ValueError(
                'valid_masked must be a boolean array matching depth_m; '
                f'got dtype={valid_masked.dtype}, shape={valid_masked.shape}, '
                f'depth_shape={depth_m.shape}')

    foreground, reason = select_foreground_pixels(
        depth_m,
        mask.data,
        mask.precision,
        isolation=isolation,
        depth_max=depth_max,
        min_valid_pixels=min_valid_pixels,
        valid_masked=valid_masked,
    )
    if foreground is None:
        return None, reason

    rows, cols = np.nonzero(foreground)
    return _reduce_foreground(
        depth_m[rows, cols], rows, cols, intrinsics), MissReason.OK
