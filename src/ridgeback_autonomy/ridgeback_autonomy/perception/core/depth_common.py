"""The shared clean rule for aligned depth frames.

Every consumer of an aligned depth frame filters invalid pixels the same way
before doing anything else (``depth_based_A.md`` Section 2.2, applied verbatim
by euclidean reconstruction): ``0`` means no depth return, ``NaN``/``inf`` are undefined, and
values past a sane maximum are far-field noise.

``DEPTH_MAX_METERS_DEFAULT`` mirrors the legacy stack's
``POINTCLOUD_MAX_METERS`` by value; the new stack deliberately never imports
from ``geometry.py``.
"""

from __future__ import annotations

import numpy as np


DEPTH_MAX_METERS_DEFAULT = 10.0


def valid_depth(
    depths: np.ndarray,
    depth_max: float = DEPTH_MAX_METERS_DEFAULT,
) -> np.ndarray:
    """Boolean selector of the usable depth values: finite, positive, in range."""

    depths = np.asarray(depths)
    with np.errstate(invalid='ignore'):
        return np.isfinite(depths) & (depths > 0.0) & (depths <= depth_max)
