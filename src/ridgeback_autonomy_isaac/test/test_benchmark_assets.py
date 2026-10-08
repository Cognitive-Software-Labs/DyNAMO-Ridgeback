"""The Isaac benchmark asset catalog serves the public world and G1."""

from __future__ import annotations

from pathlib import Path

import pytest

from ridgeback_autonomy.benchmarking.asset_catalog import (
    AssetCatalogError,
    CATALOG_SCHEMA,
    DEFAULT_TARGET,
    load_asset_catalog,
    scene_object_models,
)
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios
import scene_provider
import worlds


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CATALOG = PACKAGE_ROOT / 'sim' / 'benchmark_assets.yaml'
SCENARIO_DIR = PACKAGE_ROOT.parent / 'ridgeback_autonomy' / 'config'
SCENARIO_FILES = ('benchmark_scenarios_examples.yaml', 'benchmark_scenarios_full.yaml')
PUBLIC_WORLD = 'target_distance_calibration'


def test_public_world_maps_to_the_stage_the_runner_loads() -> None:
    catalog = load_asset_catalog(str(CATALOG))
    world = catalog.worlds[PUBLIC_WORLD]

    assert catalog.schema == CATALOG_SCHEMA
    assert set(catalog.worlds) == {PUBLIC_WORLD}
    assert world.native_name == 'g1_distance_calibration'
    assert Path(world.resource.uri).resolve() == Path(
        worlds.resolve_world(world.native_name)).resolve()
    # The runner seats the robot from this registered floor.
    assert worlds.floor_z_for_world(world.native_name) == pytest.approx(0.05)


def test_target_is_the_ground_origin_g1_in_an_advertised_format() -> None:
    g1 = load_asset_catalog(str(CATALOG)).targets[DEFAULT_TARGET]
    wrapper = PACKAGE_ROOT / 'sim' / 'isaac' / 'usd' / 'models' / 'g1' / 'g1.usda'

    assert Path(g1.uri) == wrapper
    text = wrapper.read_text(encoding='utf-8')
    assert 'defaultPrim = "g1"' in text
    assert 'double3 xformOp:translate = (0, 0, 0.792)' in text
    assert g1.resource_format in scene_provider.SPAWN_FORMATS
    # Literal spawn height, as in Gazebo.
    assert g1.spawn_z_m == 0.0
    assert scene_provider.resolve_resource_path(g1.uri) == g1.uri


def test_scenes_placing_objects_are_refused_with_the_missing_names() -> None:
    catalog = load_asset_catalog(str(CATALOG))
    for name in SCENARIO_FILES:
        scenes = load_scenarios(str(SCENARIO_DIR / name))
        object_models = scene_object_models(scenes)
        assert object_models, name

        with pytest.raises(AssetCatalogError) as excinfo:
            catalog.select(PUBLIC_WORLD, DEFAULT_TARGET, object_models)

        for model in object_models:
            assert f'object "{model}"' in str(excinfo.value)
        for scene in scenes:
            if not scene.objects:
                catalog.select(PUBLIC_WORLD, DEFAULT_TARGET, [])
