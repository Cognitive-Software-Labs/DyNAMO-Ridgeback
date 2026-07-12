"""3D foreground isolation for Path B's ``rect`` branch (plus tight cleanup).

Contract (``foreground_isolation_3d.md``): input the masked point set as an
``(N, 3)`` float array in the camera optical frame (X right, Y down, Z
forward, meters); output a boolean keep-selector of shape ``(N,)`` marking the
foreground points.

Isolators are callables satisfying that contract. ``Chain`` composes them --
per the docs, one floor-remover (``HeightCrop``) plus one
background-separator (``RangeBand``) makes a complete isolator, and that chain
is the default recipe. Geometry constants mirror ``config/camera_config.json``
and the legacy ``geometry.py`` incumbent by value; the new stack deliberately
never imports from the legacy stack.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


CAMERA_HEIGHT_M_DEFAULT = 0.85  # mirrors config/camera_config.json height_m
CAMERA_PITCH_DEG_DEFAULT = 0.0  # mirrors config/camera_config.json pitch_deg
FLOOR_MARGIN_M_DEFAULT = 0.05

RANGE_BAND_PERCENTILE_DEFAULT = 25.0  # mirrors POINTCLOUD_FRONT_PERCENTILE
RANGE_BAND_AHEAD_M_DEFAULT = 0.10  # mirrors POINTCLOUD_INLIER_AHEAD_MARGIN_M
RANGE_BAND_BEHIND_M_DEFAULT = 0.35  # mirrors POINTCLOUD_INLIER_BEHIND_MARGIN_M

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
# above. ``ISOLATION_3D_DEFAULT`` is what Path B uses when none is chosen.
ISOLATION_3D_RECIPES: dict[str, object] = {
    'height_crop': HeightCrop(),
    'range_band': RangeBand(),
    'height_crop_range_band': Chain((HeightCrop(), RangeBand())),
}
ISOLATION_3D_DEFAULT = 'height_crop_range_band'
