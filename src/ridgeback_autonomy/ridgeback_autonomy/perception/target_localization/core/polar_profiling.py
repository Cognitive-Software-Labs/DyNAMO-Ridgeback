"""Polar profiling -- project-then-segment (``docs/target_localization/polar_profiling.md``).

The LiDAR localization path: transform the planar scan into the camera
optical frame, project the points onto the color grid, keep the points the
mask selects, segment the surviving 1D range profile into runs, merge the
runs within a range band of the nearest, and reduce the merged set to one
planar coordinate ``(X, Z)``. Y (height) is unobservable from a single-plane
LiDAR and is not emitted.

Unlike projective ranging and euclidean reconstruction there is no
``tight | rect`` fork: parallax lets occluded background points project inside
even a pixel-precise mask (``docs/target_localization/polar_profiling.md`` Section 2.5), so both tags
run the same recovery -- the tag only changes how wide the admitted window
effectively is.

Runs per mask over shared per-scan work (``scan_points_optical``); the caller
loops masks (1:1:1 hierarchy, ``docs/target_localization/mask_representation.md`` Section 6.1). A mask that
selects too few rays -- the scan plane missed the object, or it sits outside
the FoV overlap -- is skipped by returning ``None``: no estimate for that
mask, never an error.

Scan validity is the driver's to declare, not this module's to guess: the
``range_min`` / ``range_max`` the message carries are the only bounds applied.
Nothing here narrows them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.intrinsics import (
    CameraIntrinsics,
    project_points,
)
from ridgeback_autonomy.perception.target_localization.core.mask import as_mask_region
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    NEAR_SURFACE_BAND_M as RANGE_BAND_M_DEFAULT,
)


# Foreground-isolation parameters -- the LiDAR analogue of the isolation_2d /
# isolation_3d recipes (``docs/target_localization/polar_profiling.md`` Section 2.5). They decide which
# of the masked beams are the object vs. background, so these are the knobs to
# tune for accuracy. Each notes what it does and which way it fails.

# Run split: start a new run where consecutive beams jump in range by more than
# this. Sits between the object's own front-to-back depth (~0.2 m for the G1)
# and the object-to-background gap (metres). Too small shatters the object into
# slivers -> keeps only the nearest sliver -> reads too near; too large glues
# the background onto the object.
RANGE_JUMP_M_DEFAULT = 0.30

# Near-band merge width: after taking the nearest run, also keep runs whose
# median range is within this of it. Wants to span the object's own
# front-to-back depth (~0.2 m for the G1) so both legs and torso merge but the
# wall behind does not. Too small keeps one leg (lateral offset, leg-face
# range); too large admits parallax / neighbour background. The value is an
# untuned starting point shared with the projective-ranging near-band width
# through ranging_defaults, not a fit to the object depth. Its current ownership
# and validation boundary are documented in docs/target_localization/polar_profiling.md.

# Run split on a bearing gap: start a new run where the beam index gaps by
# more than this many increments -- i.e. once that many or more consecutive
# beams are missing (invalid, or projecting outside the mask) -- so two objects
# sharing a range across an empty gap are not glued together. Minor knob.
MAX_BEARING_GAP_BEAMS_DEFAULT = 2

# Sparse floor: return ``None`` (no estimate -> that trial is simply dropped
# from this path's benchmark row, no fallback) when fewer than this many beams
# survive the mask select or the near-band merge. A handful of beams gives a
# noisy median; raising it trades availability (fewer usable trials) for a
# tighter estimate.
MIN_VALID_RAYS_DEFAULT = 2

# A driver normally publishes one fixed scan geometry. Keep a few immutable
# direction tables so a runtime profile change is handled without rebuilding
# trigonometry on every detection batch or allowing stale profiles to grow the
# process indefinitely.
_SCAN_GEOMETRY_CACHE_SIZE = 4


@dataclass(frozen=True)
class PolarProfilingResult:
    """One polar profiling localization: a planar point in the camera optical frame."""

    xz_optical: np.ndarray  # (2,) X right, Z forward, meters; Y unobserved
    # Beam indices into the ORIGINAL scan array, so a consumer can map an estimate
    # back to the rays it came from without re-deriving the selection. Reporting
    # both sides is the point: the gap between them is what the range segmentation
    # threw away, which is how an occluder latch becomes visible.
    selected_beams: np.ndarray  # (N,) the mask ∩ FoV select
    merged_beams: np.ndarray  # (M,) the near-band survivors -- what the estimate reduces


@dataclass(frozen=True)
class ScanImageProjection:
    """One scan's camera-image mapping, prepared for one detection batch.

    ``points_optical`` stays indexed by original scan beam, which is what lets
    ``beam_indices`` be reported against the scan the caller passed in. The
    other three are the compact valid in-view subset every per-mask selection
    indexes.
    """

    points_optical: np.ndarray  # (N, 3), indexed by original scan beam
    beam_indices: np.ndarray  # (K,), valid and in-view original beam indices
    u_px: np.ndarray  # (K,), rounded compact image columns
    v_px: np.ndarray  # (K,), rounded compact image rows


@dataclass(frozen=True)
class PolarProfilingAttempt:
    """One localization attempt, retaining its selection even on a miss."""

    result: PolarProfilingResult | None
    reason: MissReason
    selected_beams: np.ndarray


@lru_cache(maxsize=_SCAN_GEOMETRY_CACHE_SIZE)
def _scan_unit_directions(
    beam_count: int,
    angle_min: float,
    angle_increment: float,
) -> np.ndarray:
    """Return immutable scan-frame unit directions for one beam geometry."""

    angles = angle_min + (
        np.arange(beam_count, dtype=np.float64) * angle_increment
    )
    directions = np.stack((
        np.cos(angles),
        np.sin(angles),
        np.zeros(beam_count, dtype=np.float64),
    ), axis=1)
    directions.flags.writeable = False
    return directions


def scan_points_optical(
    scan,
    rotation: np.ndarray,
    translation: np.ndarray,
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

    valid &= safe_ranges >= float(scan.range_min)
    # No ceiling of our own. A return past the object is background the
    # segmentation is there to reject (Section 2.5); silently invalidating it
    # instead would hide a parallax failure as a clean miss, and cap the path
    # to a range nobody chose. Only an unusable declared cap is ignored.
    range_cap = float(scan.range_max)
    if math.isfinite(range_cap) and range_cap > 0.0:
        valid &= safe_ranges <= range_cap

    unit_directions = _scan_unit_directions(
        int(ranges.size),
        float(scan.angle_min),
        float(scan.angle_increment),
    )
    points_scan = safe_ranges[:, np.newaxis] * unit_directions
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

    Convention (pinned, ``docs/target_localization/target_localization_pipeline.md`` Section 5): on a
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


def project_scan_to_image(
    points_optical: np.ndarray,
    valid: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> ScanImageProjection:
    """Project one scan once and retain its full and compact image mappings.

    This is batch-local work: the scan, its TF transform, and its timestamp can
    all change on the next batch. ``project_points`` is deliberately called
    exactly once here; per-mask helpers index this prepared mapping only.
    """

    uv, in_view = project_points(points_optical, intrinsics)
    selectable = np.asarray(valid, dtype=bool) & in_view
    beam_indices = np.flatnonzero(selectable)
    return ScanImageProjection(
        points_optical=points_optical,
        beam_indices=beam_indices,
        u_px=np.rint(uv[beam_indices, 0]).astype(np.intp),
        v_px=np.rint(uv[beam_indices, 1]).astype(np.intp),
    )


def select_mask_beams(projection: ScanImageProjection, mask) -> np.ndarray:
    """Original scan beam indices selected by one mask from a prepared mapping.

    Membership goes through the region geometry rather than a raw
    ``data[v_px, u_px]`` index, so a beam projecting outside a cropped mask's
    storage window reads as "not selected" instead of wrapping around to a pixel
    on the far edge of the payload. Both mask representations are accepted;
    ``project_points`` has already clipped every compact beam to the grid, so a
    full-grid mask selects exactly what it always did.
    """

    if projection.beam_indices.size == 0:
        return projection.beam_indices
    region = as_mask_region(mask)
    return projection.beam_indices[
        region.contains_pixels(projection.u_px, projection.v_px)]


def select_bbox_beams(
    projection: ScanImageProjection,
    bbox_xyxy,
) -> np.ndarray:
    """Original scan beam indices inside one half-open, pixel-rounded box."""

    if projection.beam_indices.size == 0:
        return projection.beam_indices
    x1, y1, x2, y2 = (int(value) for value in bbox_xyxy)
    inside = (
        (projection.u_px >= x1) & (projection.u_px < x2)
        & (projection.v_px >= y1) & (projection.v_px < y2)
    )
    return projection.beam_indices[inside]


def select_beams(
    points_optical: np.ndarray,
    valid: np.ndarray,
    mask,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Beam indices inside both the camera FoV and ``mask`` -- the SELECT step.

    Split out so a caller can recover the selection when localization returns no
    estimate: a miss still has beams worth showing, and re-deriving them
    elsewhere is how a visualization drifts from the estimator it depicts.
    """

    return select_mask_beams(
        project_scan_to_image(points_optical, valid, intrinsics), mask)


def beams_in_bbox(
    points_optical: np.ndarray,
    valid: np.ndarray,
    bbox_xyxy,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Beam indices projecting inside the detection BOX, whatever the mask is.

    Under a box gate this equals ``select_beams``; under a silhouette gate it is
    a superset, and the difference is what the segmentation removed. The vertical
    test matters as much as the horizontal one: a box that does not span the scan
    plane's image row contains no beams at all, which is the correct reading of a
    target fully occluded at scan height.

    Deliberately mirrors ``mask._fill_box``: round to a pixel as ``select_beams``
    does, then test the HALF-OPEN ``[x1, x2) x [y1, y2)`` against truncated
    bounds. Testing the raw float against the raw box instead disagrees by a beam
    at each edge, which would make the wedge and the rays contradict each other
    on a box gate, where they are the same set by definition.
    """

    return select_bbox_beams(
        project_scan_to_image(points_optical, valid, intrinsics), bbox_xyxy)


def localize_projected_polar_profiling(
    projection: ScanImageProjection,
    mask,
    *,
    range_jump_m: float = RANGE_JUMP_M_DEFAULT,
    range_band_m: float = RANGE_BAND_M_DEFAULT,
    max_bearing_gap_beams: int = MAX_BEARING_GAP_BEAMS_DEFAULT,
    min_valid_rays: int = MIN_VALID_RAYS_DEFAULT,
) -> PolarProfilingAttempt:
    """Localize one mask from an already-prepared scan image projection.

    The attempt exposes its selected original beam indices even on failure, so
    subscribed visualization reuses estimator intermediates rather than doing
    another projection or mask selection.
    """

    # 3. PROJECT + 4. SELECT: the mask indexes the projected points exactly
    # as it indexes depth pixels in projective ranging, in sparse per-point form.
    # The two stages stay separate because they carry different miss reasons:
    # nothing on the image at all, versus nothing of it under the mask.
    if projection.beam_indices.size == 0:
        return PolarProfilingAttempt(
            None, MissReason.NO_BEAMS_IN_VIEW,
            np.empty(0, dtype=np.intp))
    beam_indices = select_mask_beams(projection, mask)
    if beam_indices.size < min_valid_rays:
        return PolarProfilingAttempt(
            None, MissReason.TOO_FEW_RAYS_SELECTED, beam_indices)

    # 5. RECOVER -- same for both mask tags (parallax survives even a tight
    # mask): segment the range profile, merge the near band, median-reduce.
    selected = np.asarray(projection.points_optical, dtype=np.float64)[beam_indices]
    planar_range_m = np.hypot(selected[:, 0], selected[:, 2])
    runs = segment_range_profile(
        beam_indices,
        planar_range_m,
        range_jump_m=range_jump_m,
        max_bearing_gap_beams=max_bearing_gap_beams,
    )
    merged = merge_near_band(runs, planar_range_m, range_band_m=range_band_m)
    if merged.size < min_valid_rays:
        return PolarProfilingAttempt(
            None, MissReason.TOO_FEW_RAYS_MERGED, beam_indices)

    # The merged set is the foreground. The coordinate is its per-axis median
    # (X, Z); the published distance is derived from that coordinate downstream
    # (planar projection), so coordinate and distance rest on the same point.
    foreground = selected[merged][:, (0, 2)]
    xz_optical = np.median(foreground, axis=0)
    return PolarProfilingAttempt(
        PolarProfilingResult(
            xz_optical=xz_optical,
            selected_beams=beam_indices,
            # ``merged`` indexes into the selected set, not the scan, so map it back.
            merged_beams=beam_indices[merged],
        ),
        MissReason.OK,
        beam_indices,
    )


def localize_polar_profiling(
    points_optical: np.ndarray,
    valid: np.ndarray,
    mask,
    intrinsics: CameraIntrinsics,
    *,
    range_jump_m: float = RANGE_JUMP_M_DEFAULT,
    range_band_m: float = RANGE_BAND_M_DEFAULT,
    max_bearing_gap_beams: int = MAX_BEARING_GAP_BEAMS_DEFAULT,
    min_valid_rays: int = MIN_VALID_RAYS_DEFAULT,
) -> tuple[PolarProfilingResult | None, MissReason]:
    """Compatibility wrapper for one-off localization callers."""

    attempt = localize_projected_polar_profiling(
        project_scan_to_image(points_optical, valid, intrinsics),
        mask,
        range_jump_m=range_jump_m,
        range_band_m=range_band_m,
        max_bearing_gap_beams=max_bearing_gap_beams,
        min_valid_rays=min_valid_rays,
    )
    return attempt.result, attempt.reason
