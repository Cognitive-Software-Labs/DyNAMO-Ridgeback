"""Camera intrinsics and inverse pinhole deprojection for the localization paths.

Deprojection needs the intrinsics *of the grid the frame lives on*
(``depth_based_path.md`` Section 2.3): for an aligned depth frame that is the
color camera. ``intrinsics_from_camera_info`` reads them from that camera's
``CameraInfo`` -- no FoV constants.

Frame convention: all coordinates are in the **camera optical frame** --
X right, Y down, Z forward, meters (``Object_Localization_Pipeline.md``
Section 7). The TF cross-check of axes/handedness stays an open item of the
integration PR.

This module is deliberately independent of the legacy estimator stack
(``geometry.py`` / ``g1_camera_measurement_node``); it shares no code with it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole projection parameters of one pixel grid."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def intrinsics_from_camera_info(info) -> CameraIntrinsics:
    """Read the intrinsics out of a ``sensor_msgs/CameraInfo``-shaped object.

    Duck-typed on purpose (needs only ``k``, ``width``, ``height``) so pure
    tests never construct a ROS message. ``k`` is the row-major 3x3 camera
    matrix: ``fx = k[0]``, ``cx = k[2]``, ``fy = k[4]``, ``cy = k[5]``.
    """

    k = info.k
    return CameraIntrinsics(
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        width=int(info.width),
        height=int(info.height),
    )


def deproject_pixel(
    u: float,
    v: float,
    z_m: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float, float]:
    """Inverse pinhole projection of one pixel (Path A Section 2.4).

    ``(u, v)`` is a pixel coordinate on the grid, ``z_m`` its depth in meters;
    returns ``(X, Y, Z)`` in the camera optical frame.
    """

    x = (float(u) - intrinsics.cx) / intrinsics.fx * float(z_m)
    y = (float(v) - intrinsics.cy) / intrinsics.fy * float(z_m)
    return x, y, float(z_m)


def deproject_masked(
    depth_m: np.ndarray,
    rows: np.ndarray,
    cols: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Vectorized inverse pinhole over the masked pixels only.

    ``rows`` / ``cols`` are the pixel indices to deproject (typically
    ``np.nonzero`` of a cleaned mask). Restricting the multiply to the masked
    pixels is the production form of Path B's deproject step
    (``depth_based_B.md`` Section 2.1). Returns an ``(N, 3)`` float array in
    the camera optical frame.
    """

    z = np.asarray(depth_m, dtype=np.float64)[rows, cols]
    x = (np.asarray(cols, dtype=np.float64) - intrinsics.cx) / intrinsics.fx * z
    y = (np.asarray(rows, dtype=np.float64) - intrinsics.cy) / intrinsics.fy * z
    return np.stack((x, y, z), axis=-1)
