"""Path B -- deproject-then-aggregate (``depth_based_B.md``).

The point-domain localization path: deproject the valid masked pixels of the
aligned depth frame into camera-optical-frame points, isolate the foreground
in the point domain, and reduce the survivors to one coordinate. The
deprojected cloud is the canonical input -- the published cloud topic is
never consumed (provenance decision of ``pointcloud_provenance_test.md``).

The isolate step is the only place the mask's ``tight | rect`` tag changes
behavior: ``tight`` takes a statistical outlier pass (median +/- k*MAD on
range); ``rect`` runs a pluggable 3D recipe (``isolation_3d.py``, default:
height crop then range band).

Reduction convention: coordinate = centroid of the foreground points;
distance = median camera-frame range. Both read the same foreground set, so
they agree by construction. Runs per mask (1:1:1 hierarchy,
``mask_component.md`` Section 6.1); a mask with too few valid points is
skipped by returning ``None``.
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
    deproject_masked,
)
from ridgeback_autonomy.perception.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
    mad_outlier_removal,
    point_ranges,
)
from ridgeback_autonomy.perception.core.mask import Mask, MaskPrecision


MIN_VALID_POINTS_DEFAULT = 10  # mirrors POINTCLOUD_MIN_VALID_POINTS by value


@dataclass(frozen=True)
class PathBResult:
    """One Path B localization plus the foreground set it was reduced from."""

    xyz_optical: np.ndarray  # (3,) centroid, camera optical frame, meters
    distance_m: float  # median camera-frame range of the foreground points
    foreground_points: np.ndarray  # (M, 3) by-product (extent, orientation, ...)


def localize_path_b(
    depth_m: np.ndarray,
    mask: Mask,
    intrinsics: CameraIntrinsics,
    *,
    isolation: Callable[[np.ndarray], np.ndarray] | None = None,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
    min_valid_points: int = MIN_VALID_POINTS_DEFAULT,
) -> PathBResult | None:
    """Localize one mask against one aligned depth frame in the point domain.

    ``isolation`` is the ``rect``-branch keep-selector (an
    ``ISOLATION_3D_RECIPES`` entry; default recipe when ``None``); the
    ``tight`` branch uses MAD outlier removal instead. Returns ``None`` when
    fewer than ``min_valid_points`` points enter or survive isolation.
    """

    depth_m = np.asarray(depth_m)

    # 1./2. DEPROJECT + SELECT commute: deproject the valid masked pixels only.
    valid = mask.data & valid_depth(depth_m, depth_max)
    rows, cols = np.nonzero(valid)
    if rows.size < min_valid_points:
        return None
    points = deproject_masked(depth_m, rows, cols, intrinsics)

    # 3. ISOLATE -- the mask-tag fork.
    if mask.precision is MaskPrecision.TIGHT:
        keep = mad_outlier_removal(points)
    else:
        if isolation is None:
            isolation = ISOLATION_3D_RECIPES[ISOLATION_3D_DEFAULT]
        keep = isolation(points)

    foreground = points[keep]
    if foreground.shape[0] < min_valid_points:
        return None

    # 4. REDUCE: centroid for the coordinate, median range for the distance.
    centroid = foreground.mean(axis=0)
    distance_m = float(np.median(point_ranges(foreground)))
    return PathBResult(
        xyz_optical=centroid,
        distance_m=distance_m,
        foreground_points=foreground,
    )
