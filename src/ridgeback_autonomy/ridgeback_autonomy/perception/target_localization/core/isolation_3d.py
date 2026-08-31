"""3D foreground isolation for euclidean reconstruction's ``rect`` branch (plus tight cleanup).

Contract (``docs/localization/foreground_isolation_3d.md``): input the masked point set as an
``(N, 3)`` float array in the camera optical frame (X right, Y down, Z
forward, meters); output a boolean keep-selector of shape ``(N,)`` marking the
foreground points.

Isolators are callables satisfying that contract. ``Chain`` composes them --
per the docs, one floor-remover (``HeightCrop``) plus one
background-separator (``RangeBand``) makes a complete isolator, and that chain
is the default recipe. Camera geometry comes from TF at runtime; the constants
here are standalone/static defaults, not copies of the camera configuration.
"""

from __future__ import annotations

import math
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


CAMERA_HEIGHT_M_DEFAULT = 1.053  # static/test default only; at runtime the height comes from TF via camera_floor_geometry.
CAMERA_PITCH_DEG_DEFAULT = 0.0  # static/test default; runtime pitch is read from the TF rotation
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
    floor is removed by prior knowledge, not estimation: rotate to the
    gravity-aligned frame and drop every point at or below the floor plus a
    margin. Far background passes untouched -- pair with a
    background-separator (``RangeBand``) via ``Chain``.
    """

    camera_height_m: float = CAMERA_HEIGHT_M_DEFAULT
    pitch_deg: float = CAMERA_PITCH_DEG_DEFAULT
    floor_margin_m: float = FLOOR_MARGIN_M_DEFAULT

    def __call__(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        pitch = math.radians(self.pitch_deg)
        # Gravity-aligned down component: optical Y is straight down at zero
        # pitch; a downward pitch folds part of optical Z into it.
        down_m = math.cos(pitch) * points[:, 1] + math.sin(pitch) * points[:, 2]
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


# The config swap point: isolators keyed by name, all satisfying the contract
# above. ``ISOLATION_3D_DEFAULT`` is what euclidean reconstruction uses when none is chosen.
# These carry the static default pose; the node builds a pose-parameterized
# recipe per frame via ``build_isolation_3d`` (below). The registry stays for
# name validation, listing, and pure unit tests.
ISOLATION_3D_RECIPES: dict[str, object] = {
    'height_crop': HeightCrop(),
    'range_band': RangeBand(),
    'nearest_mode_band': NearestModeBand(),
    'height_crop_range_band': Chain((HeightCrop(), RangeBand())),
    'height_crop_nearest_mode_band': Chain((HeightCrop(), NearestModeBand())),
}
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
) -> tuple[float, float]:
    """Camera height above the floor and pitch, from the optical->base transform.

    ``rotation`` / ``translation`` are the camera-optical -> base extrinsics
    (from TF). Height above the floor is the camera's height above the base
    origin (``translation[2]``) plus the fixed chassis offset of that origin
    above the floor (``base_above_floor_m``). Pitch comes from the rotation:
    gravity-down expressed in the optical frame is the negated base-up row of
    the transform, and ``HeightCrop`` models that direction as
    ``[0, cos p, sin p]``, so ``p`` is the angle of its (Y, Z) components -- 0
    for a level mount.
    """

    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    camera_height_m = float(translation[2]) + float(base_above_floor_m)
    down_optical = -rotation[2, :]
    pitch_deg = math.degrees(math.atan2(float(down_optical[2]), float(down_optical[1])))
    return camera_height_m, pitch_deg


def build_isolation_3d(name: str, camera_height_m: float, pitch_deg: float):
    """Build the named recipe with its floor crop set to a specific camera pose.

    Mirrors ``ISOLATION_3D_RECIPES`` but constructs any ``HeightCrop`` step at
    the given height and pitch (typically from ``camera_floor_geometry``) rather
    than the static defaults. The background-separators take no pose -- they
    work on rotation-invariant camera-frame ranges -- so they are unchanged.
    """

    height_crop = HeightCrop(camera_height_m=camera_height_m, pitch_deg=pitch_deg)
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
    supported = ', '.join(sorted(ISOLATION_3D_RECIPES))
    raise ValueError(f'Unknown isolation_3d recipe "{name}". Expected one of: {supported}')
