"""Shared defaults must be imported from their owner, not copied by value."""

from __future__ import annotations

import ast
import inspect

import pytest

from ridgeback_autonomy.perception.target_localization.core import ranging_defaults


@pytest.mark.parametrize('module_name,local_name,shared_name', [
    ('depth_common', 'DEPTH_MAX_METERS_DEFAULT', 'MAX_RANGE_M'),
    ('pointcloud_ranging', 'POINTCLOUD_MAX_METERS', 'MAX_RANGE_M'),
    ('pointcloud_ranging', 'POINTCLOUD_FRONT_PERCENTILE', 'FRONT_PERCENTILE'),
    ('pointcloud_ranging', 'POINTCLOUD_INLIER_AHEAD_MARGIN_M', 'INLIER_AHEAD_MARGIN_M'),
    ('pointcloud_ranging', 'POINTCLOUD_INLIER_BEHIND_MARGIN_M', 'INLIER_BEHIND_MARGIN_M'),
    ('pointcloud_ranging', 'POINTCLOUD_MIN_VALID_POINTS', 'MIN_VALID_SAMPLES'),
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


def test_shared_values_preserve_the_validated_tuning() -> None:
    assert ranging_defaults.MAX_RANGE_M == 10.0
    assert ranging_defaults.MIN_VALID_SAMPLES == 10
    assert ranging_defaults.FRONT_PERCENTILE == 25.0
    assert ranging_defaults.INLIER_AHEAD_MARGIN_M == 0.10
    assert ranging_defaults.INLIER_BEHIND_MARGIN_M == 0.35
    assert ranging_defaults.NEAR_SURFACE_BAND_M == 0.35
