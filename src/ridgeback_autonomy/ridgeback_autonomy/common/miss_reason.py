"""Why a mask estimator produced (or failed to produce) a value on a frame.

A single source imported by both the writer (``g1_mask_measurement_node``) and
the reader (the benchmark), so the codes never drift between the two. Stored on
the wire as ``uint8`` per detection, parallel to the ``*_distance_m`` arrays.
``OK`` means a value was produced; ``UNSET`` means the node never wrote a status
for that estimator on that detection.
"""

from __future__ import annotations

from enum import IntEnum


class MissReason(IntEnum):
    OK = 0

    # Frame-level: the whole frame was unusable before any per-detection work.
    NO_CAMERA_INFO = 1
    GRID_MISMATCH = 2
    NO_COLOR_FRAME = 3          # silhouette gate: the exact color frame never arrived
    TF_MISS_EXTRINSIC = 4       # camera-optical -> base transform unavailable

    # Per-detection: this detection's mask was rejected before any path ran.
    MASK_OVERSIZED_BOX = 10
    MASK_EMPTY_SEGMENTATION = 11

    # Input-missing: the source stream this path needs was not matched.
    NO_DEPTH_FRAME = 20         # projective + euclidean
    NO_SCAN = 21               # polar
    TF_MISS_SCAN = 22          # polar: scan -> optical transform unavailable
    SCAN_INVALID = 23          # polar: scan message present but malformed/undecodable

    # Path-internal (projective + euclidean).
    TOO_FEW_VALID_PIXELS = 30  # projective: fewer than min valid masked pixels
    TOO_FEW_VALID_POINTS = 31  # euclidean: fewer than min valid deprojected points
    ISOLATION_EMPTY = 32       # foreground isolation left too few pixels/points

    # Path-internal (polar).
    NO_BEAMS_IN_VIEW = 40      # no scan beam projects into the image
    TOO_FEW_RAYS_SELECTED = 41 # too few beams fall inside the mask
    TOO_FEW_RAYS_MERGED = 42   # near-band merge left too few beams

    UNSET = 255


def reason_name(code: int | None) -> str:
    """Human-readable name for a status code, tolerant of ``None``/unknown."""

    if code is None:
        return MissReason.UNSET.name
    try:
        return MissReason(int(code)).name
    except ValueError:
        return f'UNKNOWN_{int(code)}'
