from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from ridgeback_autonomy.perception.core.intrinsics import (
    CameraIntrinsics,
    deproject_masked,
    deproject_pixel,
    intrinsics_from_camera_info,
    project_points,
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


def test_project_points_round_trips_deproject_pixel() -> None:
    pixels = [(40.0, 30.0, 3.0), (10.5, 50.25, 1.5), (79.0, 0.0, 6.0)]
    points = np.array([deproject_pixel(u, v, z, INTRINSICS) for u, v, z in pixels])

    uv, valid = project_points(points, INTRINSICS)

    assert valid.all()
    assert np.allclose(uv, [(u, v) for u, v, _ in pixels])


def test_project_points_invalidates_behind_and_out_of_bounds() -> None:
    points = np.array([
        (0.0, 0.0, 2.0),    # principal point, valid
        (0.0, 0.0, -2.0),   # behind the camera
        (0.0, 0.0, 0.0),    # on the camera plane (division blows up)
        (10.0, 0.0, 2.0),   # projects far right of the grid
        (0.0, -10.0, 2.0),  # projects far above the grid
    ])

    uv, valid = project_points(points, INTRINSICS)

    assert valid.tolist() == [True, False, False, False, False]
    assert np.allclose(uv[0], (INTRINSICS.cx, INTRINSICS.cy))


def test_project_points_rint_boundary() -> None:
    # u = 79.4 rounds inside the 80-wide grid; u = 79.6 rounds to 80, outside.
    inside = deproject_pixel(79.4, 30.0, 2.0, INTRINSICS)
    outside = deproject_pixel(79.6, 30.0, 2.0, INTRINSICS)

    _, valid = project_points(np.array([inside, outside]), INTRINSICS)

    assert valid.tolist() == [True, False]
