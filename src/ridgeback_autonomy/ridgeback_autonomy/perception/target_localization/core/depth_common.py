"""Rules shared by every consumer of an aligned depth frame.

Three of them, all deliberately independent of which localization path is
calling:

- ``valid_depth`` -- the clean rule (``docs/target_localization/projective_ranging.md`` Section 2.2,
  applied verbatim by euclidean reconstruction): ``0`` means no depth return,
  ``NaN``/``inf`` are undefined, and values past a sane maximum are far-field
  noise.
- ``nearest_significant_mode`` -- the near-surface anchor, shared by the 2D and
  3D isolation catalogues so the pixel domain and the point domain agree on
  where the subject starts.
- ``prepare_depth_region`` -- the per-detection select+clean prologue both depth
  estimators consume, cut to the mask's own storage window.

Two depth ceilings live here, and they are not interchangeable:

- ``MASK_DEPTH_GATE_DEFAULT`` -- the mask stack's working gate. Unbounded, so
  the only ceiling on a mask measurement is whatever the depth source declares
  it can resolve. A finite value here is an operator choice about how much
  scene to admit, never a validity rule.
- ``DEPTH_MAX_METERS_DEFAULT`` -- 10 m, for the callers that still need a
  finite number: the depth colorizers, which normalize by it and would render a
  uniform frame given infinity, and the isolation catalogues' static defaults.
  It shares ``ranging_defaults.MAX_RANGE_M`` with the pointcloud path without
  either path importing the other estimator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ridgeback_autonomy.perception.target_localization.core.mask import MaskRegion
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    MAX_RANGE_M as DEPTH_MAX_METERS_DEFAULT,
)


MASK_DEPTH_GATE_DEFAULT = math.inf

# What an operator writes to mean "no gate". Not spelled ``inf``: the value
# arrives through a launch substitution, and YAML reads bare ``inf`` as the
# string "inf", which fails the double parameter's type check. Non-positive
# already means "unlimited" for this same parameter name in ``rendering.py``.
DEPTH_GATE_DISABLED = 0.0

NEAREST_MODE_BIN_WIDTH_M_DEFAULT = 0.05
NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT = 0.05


def resolve_depth_gate(depth_max_meters: float) -> float:
    """Gate parameter as written by an operator -> the ceiling to clean against.

    Non-positive means "no gate". Kept in one place because the encoding is a
    convention rather than arithmetic: passed straight into ``min()`` against a
    source ceiling, a 0 would read as the tightest gate possible and reject
    every pixel.
    """

    depth_max_meters = float(depth_max_meters)
    return depth_max_meters if depth_max_meters > 0.0 else math.inf


def valid_depth(
    depths: np.ndarray,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
) -> np.ndarray:
    """Boolean selector of the usable depth values: finite, positive, in range."""

    depths = np.asarray(depths)
    with np.errstate(invalid='ignore'):
        return np.isfinite(depths) & (depths > 0.0) & (depths <= depth_max)


@dataclass(frozen=True)
class PreparedDepthRegion:
    """One detection's depth selection, cut to its mask's storage window.

    The select+clean prologue both depth estimators used to run separately,
    computed once per detection and handed to whichever of them is enabled. The
    two must see the *same* selection: a row present for one estimator and
    absent for the other would make their misses incomparable.

    ``depth_full`` is retained by reference, not copied -- euclidean
    reconstruction deprojects with global indices against the original frame and
    the original color intrinsics, which is exactly what keeps the ROI from
    needing adjusted intrinsics of its own.
    """

    region: MaskRegion
    depth_full: np.ndarray  # the whole aligned depth frame, by reference
    roi_depth: np.ndarray  # ``depth_full`` restricted to the region's window
    valid_masked: np.ndarray  # region-local: selected by the mask AND valid depth


def prepare_depth_region(
    region: MaskRegion,
    depth_m: np.ndarray,
    frame_valid_depth: np.ndarray,
) -> PreparedDepthRegion:
    """Cut one shared frame-wide depth validity image down to one mask's window.

    ``frame_valid_depth`` is ``valid_depth(depth_m, gate)`` for the whole frame,
    computed at most once per batch by the caller: the gate is frame-wide, so
    recomputing it per detection would re-scan the frame N times to get the same
    answer. Slicing it is what makes the per-detection cost proportional to the
    detection rather than to the image.
    """

    return PreparedDepthRegion(
        region=region,
        depth_full=depth_m,
        roi_depth=region.slice_image(depth_m),
        valid_masked=region.data & region.slice_image(frame_valid_depth),
    )


def nearest_significant_mode(
    values: np.ndarray,
    *,
    bin_width_m: float = NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    min_bin_fraction: float = NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
) -> float:
    """Distance of the nearest significant mode in ``values``, meters.

    Histograms the values at ``bin_width_m`` and returns the center of the
    nearest bin holding at least ``min_bin_fraction`` of them: the subject's
    near surface, on the standing assumption that the subject is the nearest
    coherent surface in the set.

    The property worth having is that the answer depends on *where* the near
    surface sits, never on how much of the set is background. Adding far
    samples adds far bins and leaves the nearest significant bin where it was.
    A percentile anchor has no such invariance -- it asks how far in the k%
    mark falls, so it slides outward as background's share of the set grows,
    and that share is set by the scene's depth extent rather than by the
    object.

    ``values`` must be non-empty; callers already guard for that because an
    empty foreground is a miss they report separately.
    """

    values = np.asarray(values, dtype=np.float64)
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
    return 0.5 * float(edges[nearest] + edges[nearest + 1])
