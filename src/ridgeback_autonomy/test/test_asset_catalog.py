"""Benchmark asset-catalog schema, resolution, and failure reporting."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest
import yaml

from ridgeback_autonomy.benchmarking.asset_catalog import (
    CATALOG_SCHEMA,
    DEFAULT_TARGET,
    AssetCatalogError,
    load_asset_catalog,
    scene_object_models,
)
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios, parse_scenarios


CONFIG_DIR = Path(__file__).parents[1] / 'config'
PUBLIC_WORLD = 'target_distance_calibration'


def write_catalog(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / 'catalog' / 'benchmark_assets.yaml'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
    return path


def touch(tmp_path: Path, relative: str) -> Path:
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('resource', encoding='utf-8')
    return path


def valid_document() -> dict:
    return {
        'schema': CATALOG_SCHEMA,
        'worlds': {
            PUBLIC_WORLD: {
                'native_name': 'native_calibration',
                'resource': '../worlds/native_calibration.world',
                'format': 'native',
            },
        },
        'targets': {
            'g1': {'resource': '../models/g1/model.res', 'format': 'native', 'spawn_z_m': 0.01},
        },
        'objects': {
            'cabinet': {'resource': '../models/cabinet/model.res', 'format': 'NATIVE'},
            'table': {'resource': 'scheme://models/table', 'format': 'other'},
            'unused': {'resource': '../models/unused/model.res', 'format': 'native'},
        },
    }


@pytest.fixture
def catalog_path(tmp_path: Path) -> Path:
    for relative in ('worlds/native_calibration.world', 'models/g1/model.res',
                     'models/cabinet/model.res', 'models/unused/model.res'):
        touch(tmp_path, relative)
    return write_catalog(tmp_path, valid_document())


def scenes_with_objects(*models: str):
    return parse_scenarios({'scenes': [{
        'id': 'scene',
        'robots': [{'x': 2.0, 'y': 0.0}],
        'objects': [{'model': model, 'x': 1.0, 'y': 0.0} for model in models],
    }]})


def test_valid_catalog_resolves_the_selected_logical_assets(tmp_path, catalog_path) -> None:
    catalog = load_asset_catalog(str(catalog_path))
    scenes = scenes_with_objects('cabinet', 'table', 'cabinet')

    selection = catalog.select(PUBLIC_WORLD, DEFAULT_TARGET, scene_object_models(scenes))

    assert selection.world.native_name == 'native_calibration'
    assert selection.world.resource.uri == str(tmp_path / 'worlds/native_calibration.world')
    assert selection.target.uri == str(tmp_path / 'models/g1/model.res')
    assert selection.target.spawn_z_m == pytest.approx(0.01)
    assert set(selection.objects) == {'cabinet', 'table'}
    assert selection.objects['cabinet'].uri == str(tmp_path / 'models/cabinet/model.res')
    assert selection.objects['cabinet'].resource_format == 'native'
    assert selection.objects['cabinet'].spawn_z_m == 0.0
    # An explicit URI is the simulator's to interpret; the loader keeps it verbatim.
    assert selection.objects['table'].uri == 'scheme://models/table'
    assert selection.spawn_formats == {'native', 'other'}


def test_provenance_records_the_catalog_hash_and_only_the_selected_resources(
        catalog_path) -> None:
    catalog = load_asset_catalog(str(catalog_path))

    provenance = catalog.select(PUBLIC_WORLD, 'g1', ['cabinet']).provenance()

    assert provenance['schema'] == CATALOG_SCHEMA
    assert provenance['path'] == str(catalog_path)
    assert provenance['sha256'] == hashlib.sha256(catalog_path.read_bytes()).hexdigest()
    assert provenance['world']['public_name'] == PUBLIC_WORLD
    assert provenance['world']['native_name'] == 'native_calibration'
    assert provenance['world']['resource'] == '../worlds/native_calibration.world'
    assert provenance['target']['name'] == 'g1'
    assert provenance['target']['resource'] == '../models/g1/model.res'
    assert list(provenance['objects']) == ['cabinet']


def test_catalog_hash_follows_the_exact_file_bytes(tmp_path, catalog_path) -> None:
    before = load_asset_catalog(str(catalog_path)).sha256
    catalog_path.write_text(catalog_path.read_text(encoding='utf-8') + '# comment\n',
                            encoding='utf-8')

    assert load_asset_catalog(str(catalog_path)).sha256 != before


@pytest.mark.parametrize('schema', [None, 'ridgeback_benchmark_assets/2', 1])
def test_unknown_schema_is_rejected(tmp_path, catalog_path, schema) -> None:
    document = valid_document()
    if schema is None:
        del document['schema']
    else:
        document['schema'] = schema

    with pytest.raises(AssetCatalogError, match='unsupported schema'):
        load_asset_catalog(str(write_catalog(tmp_path, document)))


def test_unknown_world_is_rejected(catalog_path) -> None:
    catalog = load_asset_catalog(str(catalog_path))

    with pytest.raises(AssetCatalogError, match='world "g1_distance_calibration"'):
        catalog.select('g1_distance_calibration', 'g1', [])


def test_unknown_target_is_rejected(catalog_path) -> None:
    catalog = load_asset_catalog(str(catalog_path))

    with pytest.raises(AssetCatalogError, match='target "humanoid"'):
        catalog.select(PUBLIC_WORLD, 'humanoid', [])


def test_every_missing_object_is_reported_before_a_run_starts(catalog_path) -> None:
    catalog = load_asset_catalog(str(catalog_path))
    scenes = scenes_with_objects('cabinet', 'iv_pole', 'privacy_curtain')

    with pytest.raises(AssetCatalogError) as error:
        catalog.select('elsewhere', 'g1', scene_object_models(scenes))

    message = str(error.value)
    assert 'world "elsewhere"' in message
    assert 'object "iv_pole"' in message and 'object "privacy_curtain"' in message
    assert 'cabinet' not in message


def test_a_resource_missing_on_disk_is_rejected_at_load(tmp_path, catalog_path) -> None:
    os.remove(tmp_path / 'models/cabinet/model.res')

    with pytest.raises(AssetCatalogError, match='objects.cabinet: resource .* not found'):
        load_asset_catalog(str(catalog_path))


@pytest.mark.parametrize('section, name, field, value, match', [
    ('objects', 'cabinet', 'resource', '/abs/model.res', 'relative to the catalog'),
    ('objects', 'cabinet', 'resource', '', '"resource" must be a non-empty string'),
    ('objects', 'cabinet', 'format', None, '"format" must be a non-empty string'),
    ('objects', 'cabinet', 'spawn_z_m', True, 'finite number'),
    ('objects', 'cabinet', 'spawn_z_m', float('nan'), 'finite number'),
    ('objects', 'cabinet', 'prim_path', '/World/cabinet', 'unknown fields: prim_path'),
    ('targets', 'g1', 'uri', 'model://g1', 'unknown fields: uri'),
    ('worlds', PUBLIC_WORLD, 'native_name', '', '"native_name" must be a non-empty string'),
])
def test_malformed_entries_are_rejected(
        tmp_path, catalog_path, section, name, field, value, match) -> None:
    document = valid_document()
    if value is None:
        del document[section][name][field]
    else:
        document[section][name][field] = value

    with pytest.raises(AssetCatalogError, match=match):
        load_asset_catalog(str(write_catalog(tmp_path, document)))


def test_unknown_top_level_section_and_empty_worlds_are_rejected(tmp_path, catalog_path) -> None:
    document = valid_document()
    document['scenes'] = []
    with pytest.raises(AssetCatalogError, match='unknown fields: scenes'):
        load_asset_catalog(str(write_catalog(tmp_path, document)))

    document = valid_document()
    document['worlds'] = {}
    with pytest.raises(AssetCatalogError, match='at least one world'):
        load_asset_catalog(str(write_catalog(tmp_path, document)))


@pytest.mark.parametrize('section', ['worlds', 'targets', 'objects'])
@pytest.mark.parametrize('value', [None, [], False, 0, ''])
def test_falsey_non_mapping_catalog_sections_are_rejected(
        tmp_path, catalog_path, section, value) -> None:
    document = valid_document()
    document[section] = value

    with pytest.raises(AssetCatalogError, match=f'"{section}" must be a mapping'):
        load_asset_catalog(str(write_catalog(tmp_path, document)))


def test_invalid_yaml_is_a_catalog_error(tmp_path) -> None:
    path = tmp_path / 'broken.yaml'
    path.write_text('schema: [unclosed\n', encoding='utf-8')

    with pytest.raises(AssetCatalogError, match='not valid YAML'):
        load_asset_catalog(str(path))


def test_scene_object_models_cover_both_packaged_scenario_files() -> None:
    examples = scene_object_models(
        load_scenarios(str(CONFIG_DIR / 'benchmark_scenarios_examples.yaml')))
    full = scene_object_models(load_scenarios(str(CONFIG_DIR / 'benchmark_scenarios_full.yaml')))

    assert 'hospital_bed' in examples
    assert examples | full == {
        'cabinet', 'cabinet_tall', 'hospital_bed', 'hospital_bed_high',
        'iv_pole', 'privacy_curtain', 'table', 'table_low',
    }
