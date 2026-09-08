"""3D foreground isolation for euclidean reconstruction's ``rect`` branch (plus tight cleanup).

Contract (``docs/target_localization/euclidean_reconstruction.md``): input the masked point set as an
``(N, 3)`` float array in the camera optical frame (X right, Y down, Z
forward, meters); output a boolean keep-selector of shape ``(N,)`` marking the
foreground points.

Isolators are callables satisfying that contract. ``Chain`` composes them --
per the docs, one floor-remover (``HeightCrop``) plus one
background-separator (``RangeBand``) makes a complete isolator, and that chain
is the default recipe. The camera pose ``HeightCrop`` needs comes from TF at
runtime and from nowhere else: there is no static mount in this module, so a
recipe cannot be built without one.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
    nearest_significant_mode,
)
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    FRONT_PERCENTILE as RANGE_BAND_PERCENTILE_DEFAULT,
    INLIER_AHEAD_MARGIN_M as RANGE_BAND_AHEAD_M_DEFAULT,
    INLIER_BEHIND_MARGIN_M as RANGE_BAND_BEHIND_M_DEFAULT,
)


FLOOR_MARGIN_M_DEFAULT = 0.05
# base_link origin above the floor: stable chassis geometry, and the floor is
# not a TF frame, so this stays a measured constant rather than a lookup. Added
# to the TF camera-above-base height to get camera-above-floor.
BASE_ABOVE_FLOOR_M_DEFAULT = 0.026

MAD_K_DEFAULT = 3.0


def point_ranges(points: np.ndarray) -> np.ndarray:
    """Euclidean camera-frame range of each point, ``(N,)`` meters."""

    return np.linalg.norm(np.asarray(points, dtype=np.float64), axis=1)


@dataclass(frozen=True)
class HeightCrop:
    """Catalogue #1: extrinsic ground-plane crop -- a floor-remover.

    The camera's pose above the floor is known (calibrated extrinsics), so the
    floor is removed by prior knowledge, not estimation: project every point
    onto gravity-down and drop the ones at or below the floor plus a margin.
    Far background passes untouched -- pair with a background-separator
    (``RangeBand``) via ``Chain``.

    ``down_optical`` is the unit gravity-down direction expressed in the camera
    optical frame, straight from the extrinsic (``camera_floor_geometry``); the
    crop is then the half-space test ``points @ down_optical``, exact for any
    mount orientation including roll. Both pose fields are required -- there is
    no default mount, because a wrong one crops silently.

    The vector is a tuple rather than an ``ndarray`` so the frozen dataclass
    keeps a working generated ``__eq__``.
    """

    camera_height_m: float
    down_optical: tuple[float, float, float]
    floor_margin_m: float = FLOOR_MARGIN_M_DEFAULT

    def __call__(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        down_m = points @ np.asarray(self.down_optical, dtype=np.float64)
        height_above_floor_m = self.camera_height_m - down_m
        return height_above_floor_m > self.floor_margin_m


@dataclass(frozen=True)
class RangeBand:
    """Catalogue #2: percentile anchor + inlier window -- a background-separator.

    Port of the incumbent estimator's isolation, applied to camera-frame
    range: anchor at a low percentile of the ranges (the object's near
    surface), keep the points inside the asymmetric window around it. The
    window width is tied to the G1's body depth.
    """

    percentile: float = RANGE_BAND_PERCENTILE_DEFAULT
    ahead_m: float = RANGE_BAND_AHEAD_M_DEFAULT
    behind_m: float = RANGE_BAND_BEHIND_M_DEFAULT

    def __call__(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        if points.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        ranges = point_ranges(points)
        anchor_m = float(np.percentile(ranges, self.percentile))
        return (ranges >= anchor_m - self.ahead_m) & (ranges <= anchor_m + self.behind_m)


@dataclass(frozen=True)
class NearestModeBand:
    """Catalogue #2b: nearest-mode anchor + inlier window -- a background-separator.

    ``RangeBand`` with its anchor swapped: the same asymmetric window, placed
    at the nearest significant mode of the ranges instead of at a low
    percentile of them. The point-domain twin of the 2D nearest-mode
    histogram, and it shares that recipe's anchor function outright.

    The percentile anchor is a *proportion* statistic -- "how far in does the
    25% mark fall" -- so it moves whenever the background's share of the point
    set changes. That share is a property of the scene's depth extent, not of
    the object: a rect mask in a corridor collects far wall points that a rect
    mask in a small room does not, and past the point where the subject stops
    being the nearest quarter of the set, the anchor lands on the wall and the
    window follows it. Anchoring on the nearest coherent surface removes the
    dependence, because far samples can only add far bins.

    That makes this the recipe to reach for when the depth gate is widened:
    ``RangeBand``'s tolerance for background is bounded by how much background
    the gate lets through, so the two are coupled and this one is not.
    """

    ahead_m: float = RANGE_BAND_AHEAD_M_DEFAULT
    behind_m: float = RANGE_BAND_BEHIND_M_DEFAULT
    bin_width_m: float = NEAREST_MODE_BIN_WIDTH_M_DEFAULT
    min_bin_fraction: float = NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT

    def __call__(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        if points.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        ranges = point_ranges(points)
        anchor_m = nearest_significant_mode(
            ranges,
            bin_width_m=self.bin_width_m,
            min_bin_fraction=self.min_bin_fraction,
        )
        return (ranges >= anchor_m - self.ahead_m) & (ranges <= anchor_m + self.behind_m)


@dataclass(frozen=True)
class Chain:
    """Compose isolators: each step sees only the survivors of the previous one."""

    steps: tuple

    def __call__(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        keep = np.ones(points.shape[0], dtype=bool)
        for step in self.steps:
            active = np.flatnonzero(keep)
            if active.size == 0:
                break
            keep[active] = step(points[active])
        return keep


def mad_outlier_removal(points: np.ndarray, k: float = MAD_K_DEFAULT) -> np.ndarray:
    """Statistical cleanup for the ``tight`` branch: median +/- k*MAD on range.

    A tight mask's points are nearly all object; the stragglers are edge bleed
    onto the background, which shows up as range outliers. Zero spread (MAD of
    0) means there is nothing to remove, so everything is kept.
    """

    points = np.asarray(points, dtype=np.float64)
    if points.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    ranges = point_ranges(points)
    median_m = float(np.median(ranges))
    mad_m = float(np.median(np.abs(ranges - median_m)))
    if mad_m == 0.0:
        return np.ones(points.shape[0], dtype=bool)
    return np.abs(ranges - median_m) <= k * mad_m


# The config swap point: the selectable recipe names, all building isolators
# that satisfy the contract above. Names only, not instances -- every recipe
# containing a ``HeightCrop`` needs the live camera pose, so the only way to get
# one is ``build_isolation_3d`` (below). This set is what parameter validation
# and listing check against.
ISOLATION_3D_NAMES = frozenset({
    'height_crop',
    'range_band',
    'nearest_mode_band',
    'height_crop_range_band',
    'height_crop_nearest_mode_band',
})
# The mode-anchored chain is the default. The percentile chain it replaces is
# correct only while the object is the nearest quarter of the point set, and
# what kept it inside that regime was the depth gate bounding how much
# background could enter -- not anything about the recipe. With no ceiling on
# the stereo source and none on the gate, that protection is gone, so the
# anchor has to be one that does not move as background grows.
# ``RangeBand`` stays in the registry: it is what the legacy pointcloud
# estimator does, and the benchmark needs to be able to select it to measure
# the difference.
ISOLATION_3D_DEFAULT = 'height_crop_nearest_mode_band'


def camera_floor_geometry(
    rotation: np.ndarray,
    translation: np.ndarray,
    base_above_floor_m: float,
) -> tuple[float, tuple[float, float, float]]:
    """Camera height above the floor and gravity-down, from the optical->base transform.

    ``rotation`` / ``translation`` are the camera-optical -> base extrinsics
    (from TF). Height above the floor is the camera's height above the base
    origin (``translation[2]``) plus the fixed chassis offset of that origin
    above the floor (``base_above_floor_m``). The direction is the negated
    base-up row of the rotation, which is gravity-down expressed in the optical
    frame -- handed to ``HeightCrop`` as-is, with no angle in between to lose
    the X component to.
    """

    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    camera_height_m = float(translation[2]) + float(base_above_floor_m)
    down_optical = -rotation[2, :]
    return camera_height_m, (
        float(down_optical[0]), float(down_optical[1]), float(down_optical[2]))


def build_isolation_3d(
    name: str,
    camera_height_m: float,
    down_optical: tuple[float, float, float],
):
    """Build the named recipe with its floor crop set to a specific camera pose.

    The only constructor for an ``ISOLATION_3D_NAMES`` recipe: any
    ``HeightCrop`` step is built at the given pose (typically straight from
    ``camera_floor_geometry``), so no caller can end up cropping against a mount
    that is not the live one. The background-separators take no pose -- they
    work on rotation-invariant camera-frame ranges -- so they are unchanged.
    """

    height_crop = HeightCrop(
        camera_height_m=camera_height_m, down_optical=down_optical)
    if name == 'height_crop':
        return height_crop
    if name == 'range_band':
        return RangeBand()
    if name == 'nearest_mode_band':
        return NearestModeBand()
    if name == 'height_crop_range_band':
        return Chain((height_crop, RangeBand()))
    if name == 'height_crop_nearest_mode_band':
        return Chain((height_crop, NearestModeBand()))
    supported = ', '.join(sorted(ISOLATION_3D_NAMES))
    raise ValueError(f'Unknown isolation_3d recipe "{name}". Expected one of: {supported}')
