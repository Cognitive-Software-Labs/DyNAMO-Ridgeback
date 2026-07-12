from __future__ import annotations

import numpy as np

from ridgeback_autonomy.perception.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
    Chain,
    HeightCrop,
    RangeBand,
    mad_outlier_removal,
    point_ranges,
)


def object_points() -> np.ndarray:
    """A G1-like cluster around Z = 2 m, well above the floor."""

    x, y = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.3, 0.5, 10))
    z = np.full_like(x, 2.0)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def floor_points() -> np.ndarray:
    """Floor strip: camera height 0.85 m, pitch 0 => the floor is Y = +0.85."""

    x, z = np.meshgrid(np.linspace(-0.5, 0.5, 5), np.linspace(1.0, 3.5, 10))
    y = np.full_like(x, 0.85)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def wall_points() -> np.ndarray:
    """Background wall at Z = 4 m, above the floor."""

    x, y = np.meshgrid(np.linspace(-0.5, 0.5, 5), np.linspace(-0.5, 0.5, 10))
    z = np.full_like(x, 4.0)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def test_height_crop_removes_floor_keeps_object_and_wall() -> None:
    points = np.concatenate((object_points(), floor_points(), wall_points()))
    n_object, n_floor = len(object_points()), len(floor_points())

    keep = HeightCrop()(points)

    assert keep.shape == (points.shape[0],)
    assert keep.dtype == np.bool_
    assert keep[:n_object].all()
    assert not keep[n_object:n_object + n_floor].any()
    assert keep[n_object + n_floor:].all()


def test_range_band_removes_wall_keeps_object() -> None:
    points = np.concatenate((object_points(), wall_points()))
    n_object = len(object_points())

    keep = RangeBand()(points)

    assert keep[:n_object].all()
    assert not keep[n_object:].any()


def test_chain_leaves_exactly_the_object() -> None:
    # Floor first (its ranges overlap the object's, so the range band alone
    # cannot drop it), then the wall: the complete floor-remover +
    # background-separator recipe.
    points = np.concatenate((object_points(), floor_points(), wall_points()))
    n_object = len(object_points())

    keep = Chain((HeightCrop(), RangeBand()))(points)

    assert keep[:n_object].all()
    assert not keep[n_object:].any()


def test_mad_outlier_removal_drops_injected_outlier() -> None:
    # Ranges spread uniformly over 0.2 m, so the whole cluster sits inside
    # median +/- 3*MAD, while the injected point at 6 m is far outside it.
    z = np.linspace(1.9, 2.1, 50)
    cluster = np.stack((np.zeros_like(z), np.zeros_like(z), z), axis=-1)
    points = np.concatenate((cluster, [[0.0, 0.0, 6.0]]))

    keep = mad_outlier_removal(points)

    assert keep[:-1].all()
    assert not keep[-1]


def test_mad_outlier_removal_zero_spread_keeps_all() -> None:
    points = np.tile([[0.1, 0.2, 2.0]], (20, 1))

    assert mad_outlier_removal(points).all()


def test_isolators_accept_empty_input() -> None:
    empty = np.zeros((0, 3), dtype=np.float64)

    for isolator in (HeightCrop(), RangeBand(), Chain((HeightCrop(), RangeBand()))):
        assert isolator(empty).shape == (0,)
    assert mad_outlier_removal(empty).shape == (0,)


def test_point_ranges_is_euclidean_norm() -> None:
    points = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 2.0]])

    assert np.allclose(point_ranges(points), [5.0, 2.0])


def test_registry_default_is_the_full_chain() -> None:
    default = ISOLATION_3D_RECIPES[ISOLATION_3D_DEFAULT]

    assert isinstance(default, Chain)
    assert isinstance(default.steps[0], HeightCrop)
    assert isinstance(default.steps[1], RangeBand)
