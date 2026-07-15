"""Polar profiling -- project-then-segment (``lidar_based_path.md``).

The LiDAR localization path: transform the planar scan into the camera
optical frame, project the points onto the color grid, keep the points the
mask selects, segment the surviving 1D range profile into runs, merge the
runs within a range band of the nearest, and reduce the merged set to one
planar coordinate ``(X, Z)``. Y (height) is unobservable from a single-plane
LiDAR and is not emitted.

Unlike Paths A and B there is no ``tight | rect`` fork: parallax lets
occluded background points project inside even a pixel-precise mask
(``lidar_based_path.md`` Section 2.5), so both tags run the same recovery --
the tag only changes how wide the admitted window effectively is.

Runs per mask over shared per-scan work (``scan_points_optical``); the caller
loops masks (1:1:1 hierarchy, ``mask_component.md`` Section 6.1). A mask that
selects too few rays -- the scan plane missed the object, or it sits outside
the FoV overlap -- is skipped by returning ``None``: the structural fallback
to projective ranging / euclidean reconstruction, never an error.

Constants mirror the legacy ``geometry.py`` incumbent by value; the new stack
deliberately never imports from the legacy stack.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ridgeback_autonomy.perception.core.intrinsics import (
    CameraIntrinsics,
    project_points,
)
from ridgeback_autonomy.perception.core.mask import Mask


# Scan-validity clips (not accuracy knobs): drop physically impossible returns
# before anything else. Values mirror the legacy geometry.py lidar estimator.
LIDAR_RANGE_MIN_M_DEFAULT = 0.05  # drop sub-5cm self-hits (mirrors LIDAR_MIN_RANGE_METERS)
LIDAR_RANGE_MAX_M_DEFAULT = 10.0  # drop far-field noise (mirrors LIDAR_MAX_METERS)

# Foreground-isolation parameters -- the LiDAR analogue of the isolation_2d /
# isolation_3d recipes (``lidar_based_path.md`` Section 2.5). They decide which
# of the masked beams are the object vs. background, so these are the knobs to
# tune for accuracy. Each notes what it does and which way it fails.

# Run split: start a new run where consecutive beams jump in range by more than
# this. Sits between the object's own front-to-back depth (~0.2 m for the G1)
# and the object-to-background gap (metres). Too small shatters the object into
# slivers -> keeps only the nearest sliver -> reads too near; too large glues
# the background onto the object.
RANGE_JUMP_M_DEFAULT = 0.30

# Near-band merge width: after taking the nearest run, also keep runs whose
# median range is within this of it. Set to the object's depth so both legs and
# torso merge but the wall behind does not. Too small keeps one leg (lateral
# offset, leg-face range); too large admits parallax / neighbour background.
# (Value mirrors NEAREST_MODE_BAND_M, the Path A near-band width, by value.)
RANGE_BAND_M_DEFAULT = 0.35

# Run split on a bearing gap: start a new run where more than this many
# consecutive beams are missing (invalid, or projecting outside the mask), so
# two objects sharing a range across an empty gap are not glued together. Minor
# knob.
MAX_BEARING_GAP_BEAMS_DEFAULT = 2

# Sparse floor: return ``None`` (no estimate -> that trial is simply dropped
# from this path's benchmark row, no fallback) when fewer than this many beams
# survive the mask select or the near-band merge. A handful of beams gives a
# noisy median; raising it trades availability (fewer usable trials) for a
# tighter estimate. (mirrors LIDAR_MIN_VALID_RAYS by value.)
MIN_VALID_RAYS_DEFAULT = 2


@dataclass(frozen=True)
class PolarProfilingResult:
    """One polar profiling localization: a planar point in the camera optical frame."""

    xz_optical: np.ndarray  # (2,) X right, Z forward, meters; Y unobserved
    distance_m: float  # median planar range of the merged near-band set
    foreground_points: np.ndarray  # (M, 2) the merged (X, Z) set, by-product
    ray_count: int  # rays that survived the mask ∩ FoV select


def scan_points_optical(
    scan,
    rotation: np.ndarray,
    translation: np.ndarray,
    *,
    range_min_m: float = LIDAR_RANGE_MIN_M_DEFAULT,
    range_max_m: float = LIDAR_RANGE_MAX_M_DEFAULT,
) -> tuple[np.ndarray, np.ndarray]:
    """Per-scan work: validity clean + polar->Cartesian + extrinsic transform.

    ``scan`` is duck-typed on the ``sensor_msgs/LaserScan`` fields (``ranges``,
    ``angle_min``, ``angle_increment``, ``range_min``, ``range_max``) so pure
    tests never construct a ROS message. ``rotation`` / ``translation`` are the
    LiDAR->camera-optical extrinsics (TF at the matching timestamp).

    Returns ``(points, valid)``: an ``(N, 3)`` array in the camera optical
    frame and an ``(N,)`` validity mask. Invalid beams (non-finite, below the
    range floor, beyond the range cap) keep their slot -- beam index encodes
    bearing order, which the run segmentation needs -- with ``valid`` False
    and an undefined point. Shared by all masks of the frame (Section 3 of
    the doc); raises only on an empty scan.
    """

    ranges = np.asarray(scan.ranges, dtype=np.float64)
    if ranges.size == 0:
        raise ValueError('LaserScan contains no ranges.')

    valid = np.isfinite(ranges)
    safe_ranges = np.where(valid, ranges, 0.0)

    range_floor = max(float(scan.range_min), range_min_m)
    range_cap = float(scan.range_max)
    if not (math.isfinite(range_cap) and range_cap > 0.0):
        range_cap = range_max_m
    valid &= (safe_ranges >= range_floor) & (safe_ranges <= min(range_cap, range_max_m))

    angles = float(scan.angle_min) + (
        np.arange(ranges.size, dtype=np.float64) * float(scan.angle_increment)
    )
    points_scan = np.stack((
        safe_ranges * np.cos(angles),
        safe_ranges * np.sin(angles),
        np.zeros_like(safe_ranges),
    ), axis=1)
    points_optical = points_scan @ np.asarray(rotation, dtype=np.float64).T
    points_optical += np.asarray(translation, dtype=np.float64)
    return points_optical, valid


def segment_range_profile(
    beam_indices: np.ndarray,
    planar_range_m: np.ndarray,
    *,
    range_jump_m: float = RANGE_JUMP_M_DEFAULT,
    max_bearing_gap_beams: int = MAX_BEARING_GAP_BEAMS_DEFAULT,
) -> list[np.ndarray]:
    """Split the selected rays into contiguous runs (doc Section 2.5 step 1).

    ``beam_indices`` are the original scan indices of the selected rays, in
    scan order (= bearing order); ``planar_range_m`` their planar ranges. A
    run breaks where the range jumps by more than ``range_jump_m`` (the beam
    slipped from one surface to another) or the beam index gaps by more than
    ``max_bearing_gap_beams`` (the intervening beams hit something outside
    the mask or returned invalid). Returns position-index arrays into the
    selected-ray arrays.
    """

    count = int(beam_indices.size)
    if count == 0:
        return []
    breaks = (
        (np.diff(np.asarray(beam_indices, dtype=np.int64)) > max_bearing_gap_beams)
        | (np.abs(np.diff(np.asarray(planar_range_m, dtype=np.float64))) > range_jump_m)
    )
    return np.split(np.arange(count), np.flatnonzero(breaks) + 1)


def merge_near_band(
    runs: list[np.ndarray],
    planar_range_m: np.ndarray,
    *,
    range_band_m: float = RANGE_BAND_M_DEFAULT,
) -> np.ndarray:
    """Merge the runs within a range band of the nearest run (step 2).

    Convention (pinned, ``Object_Localization_Pipeline.md`` Section 5): on a
    legged object the nearest run alone would be one leg; merging every run
    whose median range lies within ``range_band_m`` of the nearest run's
    median averages both legs in range and bearing. Returns the merged
    position indices; background runs (parallax and neighbors) fall outside
    the band and are dropped.
    """

    planar_range_m = np.asarray(planar_range_m, dtype=np.float64)
    medians = [float(np.median(planar_range_m[run])) for run in runs]
    nearest_m = min(medians)
    kept = [run for run, median_m in zip(runs, medians) if median_m - nearest_m <= range_band_m]
    return np.concatenate(kept)


def localize_polar_profiling(
    points_optical: np.ndarray,
    valid: np.ndarray,
    mask: Mask,
    intrinsics: CameraIntrinsics,
    *,
    range_jump_m: float = RANGE_JUMP_M_DEFAULT,
    range_band_m: float = RANGE_BAND_M_DEFAULT,
    max_bearing_gap_beams: int = MAX_BEARING_GAP_BEAMS_DEFAULT,
    min_valid_rays: int = MIN_VALID_RAYS_DEFAULT,
) -> PolarProfilingResult | None:
    """Localize one mask against one scan already in the camera optical frame.

    ``points_optical`` / ``valid`` come from ``scan_points_optical`` (beam
    order = bearing order). Returns ``None`` when fewer than
    ``min_valid_rays`` rays survive the mask ∩ FoV select or the near-band
    merge -- the fallback-to-Path-A/B signal.
    """

    points_optical = np.asarray(points_optical, dtype=np.float64)

    # 3. PROJECT + 4. SELECT: the mask indexes the projected points exactly
    # as it indexes depth pixels in projective ranging, in sparse per-point form.
    uv, in_view = project_points(points_optical, intrinsics)
    selectable = np.asarray(valid, dtype=bool) & in_view
    beam_indices = np.flatnonzero(selectable)
    if beam_indices.size == 0:
        return None
    u_px = np.rint(uv[beam_indices, 0]).astype(np.intp)
    v_px = np.rint(uv[beam_indices, 1]).astype(np.intp)
    beam_indices = beam_indices[mask.data[v_px, u_px]]
    if beam_indices.size < min_valid_rays:
        return None

    # 5. RECOVER -- same for both mask tags (parallax survives even a tight
    # mask): segment the range profile, merge the near band, median-reduce.
    selected = points_optical[beam_indices]
    planar_range_m = np.hypot(selected[:, 0], selected[:, 2])
    runs = segment_range_profile(
        beam_indices,
        planar_range_m,
        range_jump_m=range_jump_m,
        max_bearing_gap_beams=max_bearing_gap_beams,
    )
    merged = merge_near_band(runs, planar_range_m, range_band_m=range_band_m)
    if merged.size < min_valid_rays:
        return None

    # The merged set is the foreground set -- single source for coordinate
    # and distance, so they agree by construction (doc Section 2.5).
    foreground = selected[merged][:, (0, 2)]
    xz_optical = np.median(foreground, axis=0)
    distance_m = float(np.median(planar_range_m[merged]))
    return PolarProfilingResult(
        xz_optical=xz_optical,
        distance_m=distance_m,
        foreground_points=foreground,
        ray_count=int(beam_indices.size),
    )
