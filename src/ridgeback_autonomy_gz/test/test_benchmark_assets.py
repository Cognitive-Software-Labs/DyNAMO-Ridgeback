"""The Gazebo benchmark asset catalog serves every packaged scenario."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ridgeback_autonomy.benchmarking.asset_catalog import (
    CATALOG_SCHEMA,
    DEFAULT_TARGET,
    load_asset_catalog,
    scene_object_models,
)
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios
from ridgeback_autonomy.benchmarking.scene_audit import load_sdf_model


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CATALOG = PACKAGE_ROOT / 'sim' / 'benchmark_assets.yaml'
SCENARIO_DIR = PACKAGE_ROOT.parent / 'ridgeback_autonomy' / 'config'
SCENARIO_FILES = ('benchmark_scenarios_examples.yaml', 'benchmark_scenarios_full.yaml')
PUBLIC_WORLD = 'target_distance_calibration'
# Formats the Gazebo server advertises through GetSimulatorFeatures.
GAZEBO_SPAWN_FORMATS = {'sdf', 'urdf'}


def _packaged_object_models() -> frozenset[str]:
    models = frozenset()
    for name in SCENARIO_FILES:
        models |= scene_object_models(load_scenarios(str(SCENARIO_DIR / name)))
    return models


def test_catalog_resolves_every_packaged_scenario() -> None:
    catalog = load_asset_catalog(str(CATALOG))
    object_models = _packaged_object_models()

    selection = catalog.select(PUBLIC_WORLD, DEFAULT_TARGET, object_models)

    assert catalog.schema == CATALOG_SCHEMA
    assert set(selection.objects) == object_models
    assert selection.spawn_formats <= GAZEBO_SPAWN_FORMATS


def test_catalog_lists_exactly_the_scenario_objects_and_their_models() -> None:
    catalog = load_asset_catalog(str(CATALOG))
    model_dirs = {
        path.parent.name for path in (PACKAGE_ROOT / 'sim' / 'models').glob('*/model.sdf')
    }

    assert set(catalog.objects) == set(_packaged_object_models())
    assert set(catalog.objects) | set(catalog.targets) == model_dirs


def test_public_world_maps_to_the_packaged_world_of_the_same_name() -> None:
    world = load_asset_catalog(str(CATALOG)).worlds[PUBLIC_WORLD]

    assert world.native_name == PUBLIC_WORLD
    assert Path(world.resource.uri) == PACKAGE_ROOT / 'sim' / 'worlds' / f'{PUBLIC_WORLD}.sdf'
    root = ET.parse(world.resource.uri).getroot()
    assert root.find('world').get('name') == world.native_name


def _floor_top(world_uri: str) -> float:
    model = next(
        m for m in ET.parse(world_uri).getroot().iter('model')
        if m.get('name') == 'calibration_floor')
    z = float(model.findtext('pose').split()[2])
    height = float(model.find('.//collision/geometry/box/size').text.split()[2])
    return z + height / 2.0


def test_spawnable_resources_are_static_models_standing_on_the_floor() -> None:
    catalog = load_asset_catalog(str(CATALOG))
    floor_top = _floor_top(catalog.worlds[PUBLIC_WORLD].resource.uri)

    for resource in (*catalog.targets.values(), *catalog.objects.values()):
        model = ET.parse(resource.uri).getroot().find('model')
        assert Path(resource.uri).parent.name == resource.logical_name
        assert model.findtext('static') == 'true', resource.logical_name
        # Origin at ground contact, spawned onto the floor's top surface.
        lowest = min(part.z_range[0] for part in load_sdf_model(resource.uri).parts)
        assert lowest == pytest.approx(0.0, abs=1e-4), resource.logical_name
        assert resource.spawn_z_m == pytest.approx(floor_top), resource.logical_name
