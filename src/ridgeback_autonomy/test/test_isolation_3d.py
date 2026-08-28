from __future__ import annotations

import math

import numpy as np
import pytest

from ridgeback_autonomy.perception.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    CAMERA_HEIGHT_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_RECIPES,
    Chain,
    HeightCrop,
    NearestModeBand,
    RangeBand,
    build_isolation_3d,
    camera_floor_geometry,
    mad_outlier_removal,
    point_ranges,
)


LEVEL_OPTICAL_TO_BASE = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def _pitched_optical_to_base(pitch_deg: float) -> np.ndarray:
    """A level optical->base rotation tilted about the optical X axis.

    Built so ``camera_floor_geometry`` recovers ``pitch_deg`` back out of it.
    """

    beta = math.radians(-pitch_deg)
    # Cross-product matrix of the optical X axis expressed in base = (0, -1, 0).
    axis_k = np.array([
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ])
    rot_about_optical_x = (
        np.eye(3) + math.sin(beta) * axis_k + (1.0 - math.cos(beta)) * (axis_k @ axis_k))
    return rot_about_optical_x @ LEVEL_OPTICAL_TO_BASE


def object_points() -> np.ndarray:
    """A G1-like cluster around Z = 2 m, well above the floor."""

    x, y = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.3, 0.5, 10))
    z = np.full_like(x, 2.0)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def floor_points() -> np.ndarray:
    """Floor strip: camera height 1.053 m, pitch 0 => the floor is Y = +1.053."""

    x, z = np.meshgrid(np.linspace(-0.5, 0.5, 5), np.linspace(1.0, 3.5, 10))
    y = np.full_like(x, 1.053)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def wall_points() -> np.ndarray:
    """Background wall at Z = 4 m, above the floor."""

    x, y = np.meshgrid(np.linspace(-0.5, 0.5, 5), np.linspace(-0.5, 0.5, 10))
    z = np.full_like(x, 4.0)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def corridor_points(near_m: float = 4.0, far_m: float = 22.0, planes: int = 24) -> np.ndarray:
    """Background receding well past the object, as a corridor's walls do.

    Deliberately far larger than ``object_points()``: the whole question is
    what an anchor does once the subject stops being a large share of the set.
    """

    x, y = np.meshgrid(np.linspace(-0.5, 0.5, 5), np.linspace(-0.5, 0.5, 10))
    slabs = [
        np.stack((x.ravel(), y.ravel(), np.full(x.size, z)), axis=-1)
        for z in np.linspace(near_m, far_m, planes)
    ]
    return np.concatenate(slabs)


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


def test_nearest_mode_band_removes_wall_keeps_object() -> None:
    points = np.concatenate((object_points(), wall_points()))
    n_object = len(object_points())

    keep = NearestModeBand()(points)

    assert keep[:n_object].all()
    assert not keep[n_object:].any()


def test_nearest_mode_band_anchor_is_invariant_to_background_size() -> None:
    # The property the percentile anchor lacks: growing the background must not
    # move the anchor, because the subject's near surface has not moved.
    objects = object_points()
    small = np.concatenate((objects, corridor_points(planes=2)))
    large = np.concatenate((objects, corridor_points(planes=24)))

    for points in (small, large):
        keep = NearestModeBand()(points)
        assert keep[:len(objects)].all()
        assert not keep[len(objects):].any()


def test_range_band_anchor_slides_off_the_object_in_a_corridor() -> None:
    # Characterizes the incumbent's coupling to the depth gate rather than
    # asserting it is correct: once the subject is under the 25th percentile of
    # the set, the percentile anchor leaves it and the window follows.
    objects = object_points()
    points = np.concatenate((objects, corridor_points(planes=24)))

    keep = RangeBand()(points)

    assert not keep[:len(objects)].any(), 'expected the incumbent to lose the object here'
    assert keep[len(objects):].any()


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

    for isolator in (
        HeightCrop(),
        RangeBand(),
        NearestModeBand(),
        Chain((HeightCrop(), RangeBand())),
        Chain((HeightCrop(), NearestModeBand())),
    ):
        assert isolator(empty).shape == (0,)
    assert mad_outlier_removal(empty).shape == (0,)


def test_point_ranges_is_euclidean_norm() -> None:
    points = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 2.0]])

    assert np.allclose(point_ranges(points), [5.0, 2.0])


def test_registry_default_is_the_mode_anchored_full_chain() -> None:
    # The anchor, not just the chain: a percentile anchor is only correct while
    # the object is the nearest quarter of the point set, and nothing bounds
    # the background any more now that the gate defaults to no gate.
    default = ISOLATION_3D_RECIPES[ISOLATION_3D_DEFAULT]

    assert isinstance(default, Chain)
    assert isinstance(default.steps[0], HeightCrop)
    assert isinstance(default.steps[1], NearestModeBand)


def test_camera_floor_geometry_level_mount() -> None:
    # Camera 1.027 m above the base origin; the base origin sits the chassis
    # offset above the floor, so the camera is 1.053 m above the floor.
    height, pitch = camera_floor_geometry(
        LEVEL_OPTICAL_TO_BASE, np.array([0.011, 0.018, 1.027]),
        BASE_ABOVE_FLOOR_M_DEFAULT)

    assert height == pytest.approx(1.053)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_camera_floor_geometry_recovers_pitch() -> None:
    _, pitch = camera_floor_geometry(
        _pitched_optical_to_base(12.0), np.array([0.0, 0.0, 1.0]), 0.0)

    assert pitch == pytest.approx(12.0)


def test_build_isolation_3d_parameterizes_heightcrop() -> None:
    chain = build_isolation_3d('height_crop_range_band', camera_height_m=1.2, pitch_deg=3.0)

    assert isinstance(chain, Chain)
    height_crop, range_band = chain.steps
    assert isinstance(height_crop, HeightCrop)
    assert height_crop.camera_height_m == 1.2
    assert height_crop.pitch_deg == 3.0
    assert isinstance(range_band, RangeBand)

    lone = build_isolation_3d('height_crop', camera_height_m=1.2, pitch_deg=3.0)
    assert isinstance(lone, HeightCrop)
    assert lone.camera_height_m == 1.2


def test_build_isolation_3d_covers_every_registered_recipe() -> None:
    # The registry and the builder are two lists of the same names; a recipe
    # added to one and not the other only fails at runtime, on the node, when
    # someone finally passes that name as a parameter.
    for name in ISOLATION_3D_RECIPES:
        built = build_isolation_3d(name, camera_height_m=1.2, pitch_deg=3.0)
        assert type(built) is type(ISOLATION_3D_RECIPES[name])


def test_build_isolation_3d_parameterizes_heightcrop_in_the_mode_chain() -> None:
    chain = build_isolation_3d(
        'height_crop_nearest_mode_band', camera_height_m=1.2, pitch_deg=3.0)

    height_crop, band = chain.steps
    assert isinstance(height_crop, HeightCrop)
    assert height_crop.camera_height_m == 1.2
    assert height_crop.pitch_deg == 3.0
    assert isinstance(band, NearestModeBand)


def test_build_isolation_3d_rejects_unknown_name() -> None:
    with pytest.raises(ValueError, match='Unknown isolation_3d recipe'):
        build_isolation_3d('nope', camera_height_m=1.0, pitch_deg=0.0)


def test_tf_derived_crop_matches_static_at_measured_pose() -> None:
    # At the current level pose the TF-derived height reproduces the static
    # default, so the crop keeps exactly the same points -- no behavior change.
    translation = np.array(
        [0.011, 0.018, CAMERA_HEIGHT_M_DEFAULT - BASE_ABOVE_FLOOR_M_DEFAULT])
    height, pitch = camera_floor_geometry(
        LEVEL_OPTICAL_TO_BASE, translation, BASE_ABOVE_FLOOR_M_DEFAULT)
    derived = build_isolation_3d('height_crop_range_band', height, pitch)
    static = ISOLATION_3D_RECIPES[ISOLATION_3D_DEFAULT]

    points = np.concatenate((object_points(), floor_points(), wall_points()))
    np.testing.assert_array_equal(derived(points), static(points))
