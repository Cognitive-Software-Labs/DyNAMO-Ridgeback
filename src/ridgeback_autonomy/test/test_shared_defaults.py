"""Shared defaults must be imported from their owner, not copied by value."""

from __future__ import annotations

import ast
import inspect
import pathlib

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


def test_the_near_surface_band_is_two_sensors_not_one_policy() -> None:
    """A camera depth window and a lidar run-merge distance, equal by history.

    The projective band is depth kept either side of a camera anchor and is
    bounded by the occluder standoff; the polar one is how far a lidar bearing
    run's median may sit from the nearest run and still merge, bounded by leg
    spacing. The projective value is measured wrong (0.75 removes 88% of its
    error tail); applying that through a shared constant would widen the polar
    merge, admitting the parallax background that band exists to drop. Sharing
    made a two-sensor retune look like a one-line fix.
    """

    from ridgeback_autonomy.perception.target_localization.core import (
        isolation_2d, polar_profiling,
    )

    assert not hasattr(ranging_defaults, 'NEAR_SURFACE_BAND_M')
    assert isolation_2d.NEAREST_MODE_BAND_M_DEFAULT == 0.35
    assert polar_profiling.RANGE_BAND_M_DEFAULT == 0.35
    for module in (isolation_2d, polar_profiling):
        assert not any(
            isinstance(node, ast.ImportFrom)
            and node.module == ranging_defaults.__name__
            and any(alias.name == 'NEAR_SURFACE_BAND_M' for alias in node.names)
            for node in ast.parse(inspect.getsource(module)).body
        ), f'{module.__name__} must not import a shared near-surface band again'


def test_ranging_defaults_holds_only_multi_estimator_constants() -> None:
    """The module's entry rule, enforced rather than described.

    A constant earns a place here only when two or more estimators must move
    together. This pins the current tenancy both ways: a new constant arriving
    without an argument fails, and a tenant losing a reader (so it should move
    to that reader) fails too. Three have already been evicted for failing the
    rule -- MAX_RANGE_M, the pointcloud half of MIN_VALID_SAMPLES, and
    NEAR_SURFACE_BAND_M -- each of which read as shared policy while it sat
    here.
    """

    expected_readers = {
        'MIN_VALID_SAMPLES': {'projective_ranging', 'euclidean_reconstruction'},
        'FRONT_PERCENTILE': {'pointcloud_ranging', 'isolation_3d'},
        'INLIER_AHEAD_MARGIN_M': {'pointcloud_ranging', 'isolation_3d'},
        'INLIER_BEHIND_MARGIN_M': {'pointcloud_ranging', 'isolation_3d'},
    }

    declared = {
        target.id
        for node in ast.parse(inspect.getsource(ranging_defaults)).body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }
    assert declared == set(expected_readers), (
        'a constant entered or left ranging_defaults without its tenancy being '
        'argued; update expected_readers and the module docstring together')

    core = pathlib.Path(inspect.getfile(ranging_defaults)).parent
    actual_readers = {name: set() for name in expected_readers}
    for path in core.glob('*.py'):
        if path.stem == 'ranging_defaults':
            continue
        for node in ast.parse(path.read_text(encoding='utf-8')).body:
            if not isinstance(node, ast.ImportFrom):
                continue
            if node.module != ranging_defaults.__name__:
                continue
            for alias in node.names:
                if alias.name in actual_readers:
                    actual_readers[alias.name].add(path.stem)

    assert actual_readers == expected_readers
    for name, readers in actual_readers.items():
        assert len(readers) >= 2, (
            f'{name} has one reader left ({readers}); move it to that module '
            'rather than leaving a shared-looking constant behind')


def test_shared_values_are_pinned_so_a_change_is_deliberate() -> None:
    """Pins the shipped values; it does not vouch for them.

    Only ``MIN_VALID_SAMPLES`` and the two margins have ever been examined.
    This test exists so that moving one is a decision, not a drift.
    """

    assert ranging_defaults.MIN_VALID_SAMPLES == 10
    assert ranging_defaults.FRONT_PERCENTILE == 25.0
    assert ranging_defaults.INLIER_AHEAD_MARGIN_M == 0.10
    assert ranging_defaults.INLIER_BEHIND_MARGIN_M == 0.35
