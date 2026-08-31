"""Euclidean reconstruction -- deproject-then-aggregate (``euclidean_reconstruction.md``).

The point-domain localization path: deproject the valid masked pixels of the
aligned depth frame into camera-optical-frame points, isolate the foreground
in the point domain, and reduce the survivors to one coordinate. The
deprojected cloud is the canonical input -- the published cloud topic is
never consumed (provenance decision of ``pointcloud_provenance_test.md``).

The isolate step is the only place the mask's ``tight | rect`` tag changes
behavior: ``tight`` takes a statistical outlier pass (median +/- k*MAD on
range); ``rect`` runs a pluggable 3D recipe (``isolation_3d.py``, default:
height crop then range band). That fork lives in ``select_foreground_points``
and nowhere else.

Two entry points, one implementation.
``localize_prepared_euclidean_reconstruction`` is the production one: the mask
selection is read off one mask's storage window, then lifted to full-grid
indices for the deprojection -- which still runs against the original depth
frame and the original color intrinsics, so the ROI never needs intrinsics of
its own. ``localize_euclidean_reconstruction`` is the standalone full-frame
signature.

The reduced coordinate is the centroid of the foreground points; the
published distance is derived from that coordinate downstream (planar
projection into the base frame), so coordinate and distance stay
self-consistent by resting on the same point. Runs per mask (1:1:1
hierarchy, ``mask_component.md`` Section 6.1); a mask with too few valid
points is skipped by returning ``None``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    DEPTH_MAX_METERS_DEFAULT,
    PreparedDepthRegion,
    valid_depth,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import (
    CameraIntrinsics,
    deproject_masked,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
    mad_outlier_removal,
)
from ridgeback_autonomy.perception.target_localization.core.mask import Mask, MaskPrecision
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    MIN_VALID_SAMPLES as MIN_VALID_POINTS_DEFAULT,
)


@dataclass(frozen=True)
class EuclideanReconstructionResult:
    """One euclidean reconstruction localization plus the foreground set it was reduced from."""

    xyz_optical: np.ndarray  # (3,) centroid, camera optical frame, meters
    foreground_points: np.ndarray  # (M, 3) by-product (extent, orientation, ...)


def select_foreground_points(
    points: np.ndarray,
    precision: MaskPrecision,
    *,
    isolation: Callable[[np.ndarray], np.ndarray] | None,
) -> np.ndarray:
    """The 3D filtering policy: the one place the precision tag is read.

    ``tight`` silhouettes carry only the object, so the remaining spread is
    noise and a MAD pass is the right tool; a ``rect`` box carries real
    background, which needs a recipe that knows where the object starts.
    """

    if precision is MaskPrecision.TIGHT:
        return mad_outlier_removal(points)
    if isolation is None:
        isolation = ISOLATION_3D_RECIPES[ISOLATION_3D_DEFAULT]
    return isolation(points)


def _reduce_points(
    points: np.ndarray,
    precision: MaskPrecision,
    isolation: Callable[[np.ndarray], np.ndarray] | None,
    min_valid_points: int,
) -> tuple[EuclideanReconstructionResult | None, MissReason]:
    """Isolate the foreground and reduce it to one coordinate.

    Steps 3 and 4, shared by both entry points: the deprojected points are
    already in the camera optical frame by the time they get here, so nothing
    below this line knows or cares whether the selection came off a window or
    the whole frame.
    """

    keep = select_foreground_points(points, precision, isolation=isolation)
    foreground = points[keep]
    if foreground.shape[0] < min_valid_points:
        return None, MissReason.ISOLATION_EMPTY

    # The coordinate is the foreground centroid. A robust median range is
    # recoverable from foreground_points if a consumer ever needs one.
    return EuclideanReconstructionResult(
        xyz_optical=foreground.mean(axis=0),
        foreground_points=foreground,
    ), MissReason.OK


def localize_prepared_euclidean_reconstruction(
    prepared: PreparedDepthRegion,
    intrinsics: CameraIntrinsics,
    *,
    isolation: Callable[[np.ndarray], np.ndarray] | None = None,
    min_valid_points: int = MIN_VALID_POINTS_DEFAULT,
) -> tuple[EuclideanReconstructionResult | None, MissReason]:
    """Localize one prepared mask region -- the ROI-native production path.

    The selection is read off the window and lifted to full-grid indices before
    deprojection, which then gathers from the original depth frame against the
    original color intrinsics. Compensating for the origin here *and* handing
    the deprojection a cropped frame would offset every point twice, so the
    window is deliberately left behind at this line.
    """

    region = prepared.region
    local_rows, local_cols = np.nonzero(prepared.valid_masked)
    if local_rows.size < min_valid_points:
        return None, MissReason.TOO_FEW_VALID_POINTS
    rows, cols = region.global_pixels(local_rows, local_cols)
    points = deproject_masked(prepared.depth_full, rows, cols, intrinsics)
    return _reduce_points(points, region.precision, isolation, min_valid_points)


def localize_euclidean_reconstruction(
    depth_m: np.ndarray,
    mask: Mask,
    intrinsics: CameraIntrinsics,
    *,
    isolation: Callable[[np.ndarray], np.ndarray] | None = None,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
    min_valid_points: int = MIN_VALID_POINTS_DEFAULT,
    valid_masked: np.ndarray | None = None,
) -> tuple[EuclideanReconstructionResult | None, MissReason]:
    """Localize one full-grid mask against one aligned depth frame.

    The standalone signature. ``isolation`` is the ``rect``-branch
    keep-selector (an ``ISOLATION_3D_RECIPES`` entry; default recipe when
    ``None``); the ``tight`` branch uses MAD outlier removal instead. Returns
    ``(result, MissReason.OK)`` on success, or ``(None, <reason>)`` when fewer
    than ``min_valid_points`` points enter or survive isolation. ``valid_masked``
    may carry the caller's already-cleaned ``mask & valid_depth`` array;
    omitting it computes validity here.
    """

    depth_m = np.asarray(depth_m)

    # 1./2. DEPROJECT + SELECT commute: deproject the valid masked pixels only.
    if valid_masked is None:
        valid = mask.data & valid_depth(depth_m, depth_max)
    else:
        valid = np.asarray(valid_masked)
        if valid.dtype != np.bool_ or valid.shape != depth_m.shape:
            raise ValueError(
                'valid_masked must be a boolean array matching depth_m; '
                f'got dtype={valid.dtype}, shape={valid.shape}, '
                f'depth_shape={depth_m.shape}')
    rows, cols = np.nonzero(valid)
    if rows.size < min_valid_points:
        return None, MissReason.TOO_FEW_VALID_POINTS
    points = deproject_masked(depth_m, rows, cols, intrinsics)
    return _reduce_points(points, mask.precision, isolation, min_valid_points)
