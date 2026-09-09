"""Shared defaults must be imported from their owner, not copied by value."""

from __future__ import annotations

import ast
import inspect

import pytest

from ridgeback_autonomy.perception.target_localization.core import (
    depth_common,
    euclidean_reconstruction,
    pointcloud_ranging,
    projective_ranging,
    ranging_defaults,
)


@pytest.mark.parametrize('module_name,local_name,shared_name', [
    ('pointcloud_ranging', 'POINTCLOUD_FRONT_PERCENTILE', 'FRONT_PERCENTILE'),
    ('pointcloud_ranging', 'POINTCLOUD_INLIER_AHEAD_MARGIN_M', 'INLIER_AHEAD_MARGIN_M'),
    ('pointcloud_ranging', 'POINTCLOUD_INLIER_BEHIND_MARGIN_M', 'INLIER_BEHIND_MARGIN_M'),
    ('isolation_3d', 'RANGE_BAND_PERCENTILE_DEFAULT', 'FRONT_PERCENTILE'),
    ('isolation_3d', 'RANGE_BAND_AHEAD_M_DEFAULT', 'INLIER_AHEAD_MARGIN_M'),
    ('isolation_3d', 'RANGE_BAND_BEHIND_M_DEFAULT', 'INLIER_BEHIND_MARGIN_M'),
    ('projective_ranging', 'MIN_VALID_PIXELS_DEFAULT', 'MIN_VALID_SAMPLES'),
    ('euclidean_reconstruction', 'MIN_VALID_POINTS_DEFAULT', 'MIN_VALID_SAMPLES'),
    ('isolation_2d', 'NEAREST_MODE_BAND_M_DEFAULT', 'NEAR_SURFACE_BAND_M'),
    ('polar_profiling', 'RANGE_BAND_M_DEFAULT', 'NEAR_SURFACE_BAND_M'),
])
def test_shared_defaults_are_imported_from_one_owner(module_name, local_name, shared_name):
    import importlib

    module = importlib.import_module(
        f'ridgeback_autonomy.perception.target_localization.core.{module_name}')
    assert getattr(module, local_name) == getattr(ranging_defaults, shared_name)
    tree = ast.parse(inspect.getsource(module))
    assert any(
        isinstance(node, ast.ImportFrom)
        and node.module == ranging_defaults.__name__
        and any(alias.name == shared_name and (alias.asname or alias.name) == local_name
                for alias in node.names)
        for node in tree.body
    ), f'{module_name}.{local_name} must import its shared default'
    assert not any(
        isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == local_name
                for target in node.targets)
        for node in tree.body
    ), f'{module_name}.{local_name} must not redefine the default'


def test_pointcloud_owns_its_range_clamp() -> None:
    """The 10 m clamp belongs to one estimator and must not be shared again.

    It is a planar-distance limit on what the pointcloud row will report, not a
    per-pixel validity rule, so the mask paths have no use for it -- they gate
    on optical depth against the node's ``effective_depth_max()``. While the
    two lived in one shared constant a coincidence looked like a decision, and
    a ceiling from an estimator that was not running reached every mask
    signature as a default.
    """

    assert pointcloud_ranging.POINTCLOUD_MAX_METERS == 10.0
    assert not hasattr(ranging_defaults, 'MAX_RANGE_M')
    assert not hasattr(depth_common, 'DEPTH_MAX_METERS_DEFAULT')


def test_pointcloud_owns_its_sufficiency_floor() -> None:
    """The 10-sample floor is two different quantities, not one shared policy.

    The mask rows count pixels a segmenter selected and a depth gate cleaned,
    off a shared prepared region, reachable through the ``min_valid_pixels``
    node parameter. This one counts raw cloud points inside a fixed fractional
    crop of a detection box, with no override. Equal values, unrelated
    decisions -- so raising one must not move the other.
    """

    assert pointcloud_ranging.POINTCLOUD_MIN_VALID_POINTS == 10
    source = inspect.getsource(pointcloud_ranging)
    assert not any(
        isinstance(node, ast.ImportFrom)
        and node.module == ranging_defaults.__name__
        and any(alias.name == 'MIN_VALID_SAMPLES' for alias in node.names)
        for node in ast.parse(source).body
    ), 'the pointcloud floor must not be imported from ranging_defaults again'


def test_the_mask_rows_still_share_one_floor() -> None:
    """Both depth rows guard the same prepared selection, so both read one floor.

    Split these and a raised floor makes one row miss while the other reports
    on the identical pixels -- the disagreement ``prepare_depth_region`` exists
    to prevent.
    """

    assert (projective_ranging.MIN_VALID_PIXELS_DEFAULT
            == euclidean_reconstruction.MIN_VALID_POINTS_DEFAULT
            == ranging_defaults.MIN_VALID_SAMPLES)


def test_shared_values_are_pinned_so_a_change_is_deliberate() -> None:
    """Pins the shipped values; it does not vouch for them.

    Only ``MIN_VALID_SAMPLES`` and the two margins have ever been examined, and
    ``NEAR_SURFACE_BAND_M`` is measured *wrong* -- 0.75 removes 88% of the
    error tail (``docs/history/projective_parameter_sensitivity.md``). This
    test exists so that moving one is a decision, not a drift.
    """

    assert ranging_defaults.MIN_VALID_SAMPLES == 10
    assert ranging_defaults.FRONT_PERCENTILE == 25.0
    assert ranging_defaults.INLIER_AHEAD_MARGIN_M == 0.10
    assert ranging_defaults.INLIER_BEHIND_MARGIN_M == 0.35
    assert ranging_defaults.NEAR_SURFACE_BAND_M == 0.35
