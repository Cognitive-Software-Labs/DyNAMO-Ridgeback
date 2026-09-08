from __future__ import annotations

import math

import numpy as np
import pytest

from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_NAMES,
    MAD_K_DEFAULT,
    MAD_SIGMA_SCALE,
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

# This robot's mount, from the expanded robot description: the color optical
# frame sits this far above base_link, whose origin is the chassis offset above
# the floor. Nothing in the module carries a pose any more, so the tests own it.
CAMERA_ABOVE_BASE_M = 1.1595
CAMERA_HEIGHT_M = CAMERA_ABOVE_BASE_M + BASE_ABOVE_FLOOR_M_DEFAULT
# Optical Y points straight down on a level mount.
LEVEL_DOWN_OPTICAL = (0.0, 1.0, 0.0)


def level_height_crop() -> HeightCrop:
    """The floor crop at this robot's level mount."""

    return HeightCrop(CAMERA_HEIGHT_M, LEVEL_DOWN_OPTICAL)


def _pitched_optical_to_base(pitch_deg: float) -> np.ndarray:
    """A level optical->base rotation tilted about the optical X axis."""

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


def _rolled_optical_to_base(roll_deg: float) -> np.ndarray:
    """A level optical->base rotation rolled about the optical Z (viewing) axis.

    A roll puts part of gravity-down along optical X, which is the component a
    single pitch angle cannot carry.
    """

    alpha = math.radians(roll_deg)
    # Optical Z expressed in base is (1, 0, 0), so rolling the camera about its
    # own viewing axis is a base-frame rotation about X.
    roll_about_optical_z = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(alpha), -math.sin(alpha)],
        [0.0, math.sin(alpha), math.cos(alpha)],
    ])
    return roll_about_optical_z @ LEVEL_OPTICAL_TO_BASE


def object_points() -> np.ndarray:
    """A G1-like cluster around Z = 2 m, well above the floor."""

    x, y = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.3, 0.5, 10))
    z = np.full_like(x, 2.0)
    return np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)


def floor_points() -> np.ndarray:
    """Floor strip: on a level mount the floor is Y = +CAMERA_HEIGHT_M."""

    x, z = np.meshgrid(np.linspace(-0.5, 0.5, 5), np.linspace(1.0, 3.5, 10))
    y = np.full_like(x, CAMERA_HEIGHT_M)
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

    keep = level_height_crop()(points)

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

    keep = Chain((level_height_crop(), RangeBand()))(points)

    assert keep[:n_object].all()
    assert not keep[n_object:].any()


def test_mad_outlier_removal_drops_injected_outlier() -> None:
    # Ranges spread uniformly over 0.2 m, so the whole cluster sits inside
    # median +/- 3 sigma, while the injected point at 6 m is far outside it.
    z = np.linspace(1.9, 2.1, 50)
    cluster = np.stack((np.zeros_like(z), np.zeros_like(z), z), axis=-1)
    points = np.concatenate((cluster, [[0.0, 0.0, 6.0]]))

    keep = mad_outlier_removal(points)

    assert keep[:-1].all()
    assert not keep[-1]


def test_mad_threshold_is_scaled_to_sigma_not_raw_deviations() -> None:
    # The consistency constant is behavior, not decoration: it widens the cut
    # by ~48%, and dropping it silently trims target surface. This probe sits
    # in the gap -- 0.198 m from the median, outside the raw 3*MAD cut of
    # 0.159 m and inside the rescaled 0.236 m -- so it survives only while the
    # rescaling is applied.
    z = np.append(np.linspace(1.9, 2.1, 50), 2.20)
    points = np.stack((np.zeros_like(z), np.zeros_like(z), z), axis=-1)
    ranges = point_ranges(points)
    raw_mad_m = float(np.median(np.abs(ranges - np.median(ranges))))

    assert raw_mad_m * MAD_K_DEFAULT < 0.198 < raw_mad_m * MAD_SIGMA_SCALE * MAD_K_DEFAULT
    assert mad_outlier_removal(points).all()


def test_mad_outlier_removal_zero_spread_keeps_all() -> None:
    points = np.tile([[0.1, 0.2, 2.0]], (20, 1))

    assert mad_outlier_removal(points).all()


def test_isolators_accept_empty_input() -> None:
    empty = np.zeros((0, 3), dtype=np.float64)

    for isolator in (
        level_height_crop(),
        RangeBand(),
        NearestModeBand(),
        Chain((level_height_crop(), RangeBand())),
        Chain((level_height_crop(), NearestModeBand())),
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
    assert ISOLATION_3D_DEFAULT in ISOLATION_3D_NAMES
    default = build_isolation_3d(
        ISOLATION_3D_DEFAULT, CAMERA_HEIGHT_M, LEVEL_DOWN_OPTICAL)

    assert isinstance(default, Chain)
    assert isinstance(default.steps[0], HeightCrop)
    assert isinstance(default.steps[1], NearestModeBand)


def test_camera_floor_geometry_level_mount() -> None:
    # The base origin sits the chassis offset above the floor, so the camera's
    # height above the floor is its height above the base plus that offset.
    height, down_optical = camera_floor_geometry(
        LEVEL_OPTICAL_TO_BASE, np.array([0.011, 0.018, CAMERA_ABOVE_BASE_M]),
        BASE_ABOVE_FLOOR_M_DEFAULT)

    assert height == pytest.approx(CAMERA_HEIGHT_M)
    assert down_optical == pytest.approx(LEVEL_DOWN_OPTICAL, abs=1e-12)


def test_camera_floor_geometry_recovers_the_down_vector() -> None:
    # A pitched mount folds part of optical Z into gravity-down; the vector is
    # taken straight off the extrinsic, so it comes back exact.
    _, down_optical = camera_floor_geometry(
        _pitched_optical_to_base(12.0), np.array([0.0, 0.0, 1.0]), 0.0)

    pitch = math.radians(12.0)
    assert down_optical == pytest.approx(
        (0.0, math.cos(pitch), math.sin(pitch)), abs=1e-12)


def test_roll_aware_crop_drops_a_floor_point_a_pitch_angle_would_keep() -> None:
    # The case the (height, pitch) encoding could not express: under roll,
    # gravity-down has an X component, and collapsing the direction to
    # atan2(down_z, down_y) discarded it.
    height, down_optical = camera_floor_geometry(
        _rolled_optical_to_base(30.0),
        np.array([0.0, 0.0, CAMERA_ABOVE_BASE_M]),
        BASE_ABOVE_FLOOR_M_DEFAULT)

    assert down_optical[0] == pytest.approx(0.5)
    assert down_optical[2] == pytest.approx(0.0, abs=1e-12)

    # Exactly on the floor: height along gravity-down, plus an offset along the
    # viewing axis, which is perpendicular to down for a pure roll.
    floor_point = np.array(
        [[height * down_optical[0], height * down_optical[1], 2.0]])

    assert not HeightCrop(height, down_optical)(floor_point).any()
    # atan2(down_z, down_y) on this mount is 0, so the old encoding rebuilt it
    # as a level camera -- and the same floor point survives that crop.
    assert HeightCrop(height, LEVEL_DOWN_OPTICAL)(floor_point).all()


def test_build_isolation_3d_parameterizes_heightcrop() -> None:
    chain = build_isolation_3d(
        'height_crop_range_band', camera_height_m=1.2,
        down_optical=LEVEL_DOWN_OPTICAL)

    assert isinstance(chain, Chain)
    height_crop, range_band = chain.steps
    assert isinstance(height_crop, HeightCrop)
    assert height_crop.camera_height_m == 1.2
    assert height_crop.down_optical == LEVEL_DOWN_OPTICAL
    assert isinstance(range_band, RangeBand)

    lone = build_isolation_3d(
        'height_crop', camera_height_m=1.2, down_optical=LEVEL_DOWN_OPTICAL)
    assert isinstance(lone, HeightCrop)
    assert lone.camera_height_m == 1.2


def test_build_isolation_3d_covers_every_registered_name() -> None:
    # The name set and the builder are two lists of the same names; a recipe
    # added to one and not the other only fails at runtime, on the node, when
    # someone finally passes that name as a parameter.
    empty = np.zeros((0, 3), dtype=np.float64)
    for name in ISOLATION_3D_NAMES:
        built = build_isolation_3d(
            name, camera_height_m=1.2, down_optical=LEVEL_DOWN_OPTICAL)
        assert built(empty).shape == (0,)


def test_build_isolation_3d_parameterizes_heightcrop_in_the_mode_chain() -> None:
    chain = build_isolation_3d(
        'height_crop_nearest_mode_band', camera_height_m=1.2,
        down_optical=LEVEL_DOWN_OPTICAL)

    height_crop, band = chain.steps
    assert isinstance(height_crop, HeightCrop)
    assert height_crop.camera_height_m == 1.2
    assert height_crop.down_optical == LEVEL_DOWN_OPTICAL
    assert isinstance(band, NearestModeBand)


def test_build_isolation_3d_rejects_unknown_name() -> None:
    with pytest.raises(ValueError, match='Unknown isolation_3d recipe'):
        build_isolation_3d(
            'nope', camera_height_m=1.0, down_optical=LEVEL_DOWN_OPTICAL)
