"""Path A -- aggregate-then-deproject (``depth_based_A.md``).

The cheapest localization path: select the masked depths, clean them,
collapse them to one distance, and deproject exactly one representative
pixel. The aggregate step is the only place the mask's ``tight | rect`` tag
changes behavior: ``tight`` takes a robust median directly; ``rect`` first
isolates the foreground with a pluggable 2D recipe (``isolation_2d.py``).

Runs per mask over a shared aligned depth frame; the caller loops masks
(1:1:1 hierarchy, ``mask_component.md`` Section 6.1). A mask with too few
valid depth pixels is skipped by returning ``None`` -- the docs'
invalid-depth fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from ridgeback_autonomy.perception.core.depth_common import (
    DEPTH_MAX_METERS_DEFAULT,
    valid_depth,
)
from ridgeback_autonomy.perception.core.intrinsics import (
    CameraIntrinsics,
    deproject_pixel,
)
from ridgeback_autonomy.perception.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.core.mask import Mask, MaskPrecision


MIN_VALID_PIXELS_DEFAULT = 10  # mirrors POINTCLOUD_MIN_VALID_POINTS by value


@dataclass(frozen=True)
class PathAResult:
    """One Path A localization: a point in the camera optical frame plus provenance."""

    xyz_optical: np.ndarray  # (3,) X right, Y down, Z forward, meters
    depth_m: float  # the aggregated (median) foreground depth
    representative_uv: tuple[float, float]  # centroid of the foreground pixels
    foreground_pixel_count: int


def localize_path_a(
    depth_m: np.ndarray,
    mask: Mask,
    intrinsics: CameraIntrinsics,
    *,
    isolation: Callable[..., np.ndarray] | None = None,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
    min_valid_pixels: int = MIN_VALID_PIXELS_DEFAULT,
) -> PathAResult | None:
    """Localize one mask against one aligned depth frame.

    ``isolation`` is the ``rect``-branch foreground recipe (an
    ``ISOLATION_2D_RECIPES`` entry; default recipe when ``None``); the
    ``tight`` branch never calls it. Returns ``None`` when fewer than
    ``min_valid_pixels`` valid (or, on the ``rect`` branch, foreground)
    pixels remain.
    """

    depth_m = np.asarray(depth_m)

    # 1. SELECT + 2. CLEAN: the valid masked pixels.
    valid = mask.data & valid_depth(depth_m, depth_max)
    if int(np.count_nonzero(valid)) < min_valid_pixels:
        return None

    # 3. AGGREGATE -- the mask-tag fork. The foreground set is the single
    # source for both the median depth and the representative pixel.
    if mask.precision is MaskPrecision.TIGHT:
        foreground = valid
    else:
        if isolation is None:
            isolation = ISOLATION_2D_RECIPES[ISOLATION_2D_DEFAULT]
        # The recipe re-derives validity from the frame, so handing it the
        # already-cleaned selector keeps path and recipe on one clean rule.
        foreground = isolation(depth_m, valid)
        if int(np.count_nonzero(foreground)) < min_valid_pixels:
            return None

    rows, cols = np.nonzero(foreground)
    aggregated_depth_m = float(np.median(depth_m[rows, cols]))

    # 4. DEPROJECT the centroid of the foreground pixels -- never the raw box
    # center, which can sit on a depth discontinuity.
    u = float(cols.mean())
    v = float(rows.mean())
    x, y, z = deproject_pixel(u, v, aggregated_depth_m, intrinsics)
    return PathAResult(
        xyz_optical=np.array((x, y, z), dtype=np.float64),
        depth_m=aggregated_depth_m,
        representative_uv=(u, v),
        foreground_pixel_count=int(rows.size),
    )
