from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from ridgeback_autonomy.perception.core.intrinsics import (
    CameraIntrinsics,
    deproject_masked,
    deproject_pixel,
    intrinsics_from_camera_info,
)


INTRINSICS = CameraIntrinsics(fx=100.0, fy=110.0, cx=40.0, cy=30.0, width=80, height=60)


def test_intrinsics_from_camera_info_k_matrix_mapping() -> None:
    info = SimpleNamespace(
        k=(100.0, 0.0, 40.0, 0.0, 110.0, 30.0, 0.0, 0.0, 1.0),
        width=80,
        height=60,
    )
    intrinsics = intrinsics_from_camera_info(info)

    assert intrinsics == INTRINSICS


def test_deproject_pixel_principal_point_is_on_axis() -> None:
    x, y, z = deproject_pixel(INTRINSICS.cx, INTRINSICS.cy, 3.0, INTRINSICS)

    assert (x, y, z) == (0.0, 0.0, 3.0)


def test_deproject_pixel_round_trip_known_point() -> None:
    # Project a known optical-frame point through the pinhole model, then
    # deproject the resulting pixel back.
    x_true, y_true, z_true = 0.5, -0.25, 2.0
    u = INTRINSICS.fx * x_true / z_true + INTRINSICS.cx
    v = INTRINSICS.fy * y_true / z_true + INTRINSICS.cy

    x, y, z = deproject_pixel(u, v, z_true, INTRINSICS)

    assert np.allclose((x, y, z), (x_true, y_true, z_true))


def test_deproject_masked_equals_per_pixel_loop() -> None:
    rng = np.random.default_rng(7)
    depth = rng.uniform(0.5, 8.0, size=(6, 8)).astype(np.float32)
    rows, cols = np.nonzero(np.ones((6, 8), dtype=bool))

    points = deproject_masked(depth, rows, cols, INTRINSICS)

    assert points.shape == (48, 3)
    expected = np.array([
        deproject_pixel(u, v, depth[v, u], INTRINSICS)
        for v, u in zip(rows, cols)
    ])
    assert np.allclose(points, expected)


def test_deproject_masked_empty_selection() -> None:
    depth = np.full((6, 8), 2.0, dtype=np.float32)
    rows, cols = np.nonzero(np.zeros((6, 8), dtype=bool))

    points = deproject_masked(depth, rows, cols, INTRINSICS)

    assert points.shape == (0, 3)
