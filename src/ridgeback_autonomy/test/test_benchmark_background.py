"""Background layout, drop rule, generated drop lists, and their run-time checks."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import shutil
import sys

import pytest
import yaml

from ridgeback_autonomy.benchmarking import background as bg
from ridgeback_autonomy.benchmarking.asset_catalog import load_asset_catalog
from ridgeback_autonomy.benchmarking.scenarios import (
    ObjectSpec, RobotSpec, Scene, load_scenarios,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = REPO_ROOT / 'src' / 'ridgeback_autonomy' / 'config'
GZ_CATALOG = REPO_ROOT / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'benchmark_assets.yaml'
ISAAC_CATALOG = (
    REPO_ROOT / 'src' / 'ridgeback_autonomy_isaac' / 'sim' / 'benchmark_assets.yaml')
SCENARIO_FILES = sorted(CONFIG_DIR.glob('benchmark_scenarios_*.yaml'))

_spec = importlib.util.spec_from_file_location(
    'build_background', REPO_ROOT / 'tools' / 'benchmark' / 'build_background.py')
build_background = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = build_background
_spec.loader.exec_module(build_background)

SQUARE = bg.Footprint(-0.25, 0.25, -0.25, 0.25)


@pytest.fixture(scope='module')
def evaluation():
    return build_background.evaluate('lab')


def _scene(robots=((4.0, 0.0),), objects=()) -> Scene:
    return Scene(
        id='synthetic',
        robots=tuple(RobotSpec(x=x, y=y, yaw=math.pi) for x, y in robots),
        objects=tuple(ObjectSpec(model=model, x=x, y=y, yaw=0.0) for model, x, y in objects),
        repeats_override=None,
    )


def _reasons(x: float, y: float, scene: Scene, footprints=None) -> tuple[str, ...]:
    outline = bg.placed_outline(SQUARE, x, y, 0.0)
    return bg.drop_reasons(outline, scene, footprints or {'box': SQUARE})


# --- the drop rule ------------------------------------------------------------

def test_a_prop_beside_a_target_is_dropped_for_its_footprint() -> None:
    assert bg.TARGET_FOOTPRINT in _reasons(4.0, 0.85, _scene())
    assert bg.TARGET_FOOTPRINT not in _reasons(4.0, 1.5, _scene())


def test_a_prop_between_camera_and_target_is_dropped_for_the_sight_line() -> None:
    assert bg.TARGET_SIGHTLINE in _reasons(2.0, 0.5, _scene())


def test_the_keep_out_cone_ends_the_keep_out_distance_behind_the_target() -> None:
    target_range = 4.0 - bg.CAMERA_XY[0]
    near_edge_inside = target_range + bg.KEEP_OUT_BEHIND_M - 0.3
    near_edge_outside = target_range + bg.KEEP_OUT_BEHIND_M + 0.05

    assert bg.BEHIND_TARGET in _reasons(
        bg.CAMERA_XY[0] + near_edge_inside + 0.25, 0.0, _scene())
    assert _reasons(bg.CAMERA_XY[0] + near_edge_outside + 0.25, 0.0, _scene()) == ()


def test_a_prop_off_the_target_bearing_survives_behind_it() -> None:
    assert _reasons(6.0, 3.0, _scene()) == ()


def test_objects_keep_their_footprint_margin_and_sight_line() -> None:
    scene = _scene(robots=((3.0, 2.0),), objects=(('box', 6.0, -2.0),))

    assert bg.OBJECT_FOOTPRINT in _reasons(6.0, -2.7, scene)
    assert bg.OBJECT_SIGHTLINE in _reasons(3.0, -1.0, scene)
    assert _reasons(6.0, -3.5, scene) == ()


def test_the_keep_out_distance_clears_every_designed_behind_target_occluder() -> None:
    # The interfere_behind and trap_layered families stand an object on a
    # target's bearing behind it; the keep-out cone must reach 0.5 m past the
    # farthest of them so the background never crowds the designed layering.
    scenes = [scene for path in SCENARIO_FILES for scene in load_scenarios(str(path))
              if scene.id.startswith(('interfere_behind', 'trap_layered'))]
    behind = []
    for scene in scenes:
        for obj in scene.objects:
            for robot in scene.robots:
                gap = bg.camera_range((obj.x, obj.y)) - bg.camera_range((robot.x, robot.y))
                offset = abs(bg.camera_bearing((obj.x, obj.y))
                             - bg.camera_bearing((robot.x, robot.y)))
                if gap > 0.0 and offset < math.atan(
                        (bg.TARGET_RADIUS_M + bg.MARGIN_M) / bg.camera_range((robot.x, robot.y))):
                    behind.append(gap)

    assert behind
    assert bg.KEEP_OUT_BEHIND_M == pytest.approx(max(behind) + 0.5, abs=0.05)


# --- the packaged layout and its generated drop lists -------------------------

def test_the_packaged_drop_file_is_current(evaluation) -> None:
    layout, footprints, scenarios = evaluation
    _layout_path, drops_path = bg.background_files(str(CONFIG_DIR), 'lab')

    regenerated = build_background.render('lab', layout, footprints, scenarios)

    assert Path(drops_path).read_text(encoding='utf-8') == regenerated, (
        'regenerate with tools/benchmark/build_background.py')


def test_the_layout_stays_inside_the_room_without_touching_props(evaluation) -> None:
    layout, footprints, _scenarios = evaluation

    assert bg.layout_problems(layout, footprints) == []


def test_every_kept_prop_clears_its_scene_and_every_dropped_one_has_a_reason(evaluation) -> None:
    layout, footprints, scenarios = evaluation
    catalog = load_asset_catalog(str(GZ_CATALOG))
    object_footprints = {
        model: build_background.sdf_footprint(resource.uri)
        for model, resource in catalog.objects.items()}
    outlines = bg.prop_outlines(layout, footprints)

    for _sha, scenes, drops in scenarios.values():
        for scene in scenes:
            for prop in layout.props:
                reasons = bg.drop_reasons(outlines[prop.id], scene, object_footprints)
                assert bool(reasons) == (prop.id in drops[scene.id]), (scene.id, prop.id)


def test_every_packaged_scene_resolves_to_known_props() -> None:
    for path in SCENARIO_FILES:
        scenes = yaml.safe_load(path.read_text())['scenes']
        resolved = bg.resolve_scene_background(
            'lab', str(CONFIG_DIR), str(path), [scene['id'] for scene in scenes])
        ids = {prop.id for prop in resolved.layout.props}

        assert set(resolved.scene_drops) == {scene['id'] for scene in scenes}
        assert all(set(dropped) <= ids for dropped in resolved.scene_drops.values())


def test_both_simulator_catalogs_serve_every_layout_model_on_the_floor() -> None:
    layout = bg.load_background_layout(bg.background_files(str(CONFIG_DIR), 'lab')[0])
    for path, resource_format in ((GZ_CATALOG, 'sdf'), (ISAAC_CATALOG, 'usd')):
        catalog = load_asset_catalog(str(path))
        selection = catalog.select(layout.world, 'g1', (), layout.models)

        assert set(selection.background) == set(layout.models)
        assert set(catalog.background) == set(layout.models)
        assert {resource.resource_format for resource in selection.background.values()} == {
            resource_format}
        assert {resource.spawn_z_m for resource in selection.background.values()} == {0.05}
        # Background props are not scene objects, and do not leak into them.
        assert not set(catalog.objects) & set(catalog.background)
        assert 'background' in selection.provenance()


def test_a_run_without_background_keeps_its_catalog_provenance() -> None:
    selection = load_asset_catalog(str(GZ_CATALOG)).select(
        'target_distance_calibration', 'g1', {'table'})

    assert 'background' not in selection.provenance()


# --- run-time refusals --------------------------------------------------------

@pytest.fixture
def config_copy(tmp_path: Path) -> Path:
    config = tmp_path / 'config'
    (config / bg.DROPS_DIRECTORY).mkdir(parents=True)
    for name in ('benchmark_background_lab.yaml', 'benchmark_scenarios_full.yaml'):
        shutil.copyfile(CONFIG_DIR / name, config / name)
    shutil.copyfile(CONFIG_DIR / bg.DROPS_DIRECTORY / 'lab.yaml',
                    config / bg.DROPS_DIRECTORY / 'lab.yaml')
    return config


def _scene_ids(path: Path) -> list[str]:
    return [scene['id'] for scene in yaml.safe_load(path.read_text())['scenes']]


def test_an_edited_scenario_file_is_refused_until_regenerated(config_copy: Path) -> None:
    scenario = config_copy / 'benchmark_scenarios_full.yaml'
    scenario.write_text(scenario.read_text().replace('x: 5.02', 'x: 5.03', 1))

    with pytest.raises(bg.BackgroundError, match='regenerate'):
        bg.resolve_scene_background('lab', str(config_copy), str(scenario), _scene_ids(scenario))


def test_an_edited_layout_is_refused_until_regenerated(config_copy: Path) -> None:
    layout = config_copy / 'benchmark_background_lab.yaml'
    layout.write_text(layout.read_text().replace('x: 11.705', 'x: 11.70', 1))
    scenario = config_copy / 'benchmark_scenarios_full.yaml'

    with pytest.raises(bg.BackgroundError, match='different layout'):
        bg.resolve_scene_background('lab', str(config_copy), str(scenario), _scene_ids(scenario))


def test_a_scene_without_a_drop_list_is_refused(config_copy: Path) -> None:
    scenario = config_copy / 'benchmark_scenarios_full.yaml'

    with pytest.raises(bg.BackgroundError, match='no drop list for scene'):
        bg.resolve_scene_background(
            'lab', str(config_copy), str(scenario), [*_scene_ids(scenario), 'new_scene'])


def test_an_unknown_scenario_file_is_refused(config_copy: Path) -> None:
    custom = config_copy / 'my_scenes.yaml'
    shutil.copyfile(config_copy / 'benchmark_scenarios_full.yaml', custom)

    with pytest.raises(bg.BackgroundError, match='no drop lists for scenario file my_scenes'):
        bg.resolve_scene_background('lab', str(config_copy), str(custom), _scene_ids(custom))


def test_an_unknown_background_name_is_refused() -> None:
    with pytest.raises(bg.BackgroundError, match='lower-case identifier'):
        bg.background_files(str(CONFIG_DIR), '../lab')
    with pytest.raises(FileNotFoundError):
        bg.resolve_scene_background(
            'office', str(CONFIG_DIR), str(SCENARIO_FILES[0]), [])


def test_layout_parsing_rejects_duplicate_ids_and_unknown_fields() -> None:
    document = {
        'schema': bg.LAYOUT_SCHEMA,
        'world': 'w',
        'room': {'x_min': 0, 'x_max': 1, 'y_min': 0, 'y_max': 1},
        'props': [{'id': 'a', 'model': 'm', 'x': 0, 'y': 0, 'yaw': 0}] * 2,
    }
    with pytest.raises(bg.BackgroundError, match='duplicate prop id'):
        bg.parse_background_layout(document)

    document['props'] = [{'id': 'a', 'model': 'm', 'x': 0, 'y': 0, 'z': 1}]
    with pytest.raises(bg.BackgroundError, match='unknown fields: z'):
        bg.parse_background_layout(document)
