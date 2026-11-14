"""Background layout, drop rule, placement, generated data, and their run-time checks."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

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
G1_MODEL = REPO_ROOT / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'models' / 'g1' / 'model.sdf'
SCENARIO_FILES = sorted(CONFIG_DIR.glob('benchmark_scenarios_*.yaml'))
FULL = 'benchmark_scenarios_full.yaml'

_spec = importlib.util.spec_from_file_location(
    'build_background', REPO_ROOT / 'tools' / 'benchmark' / 'build_background.py')
build_background = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = build_background
_spec.loader.exec_module(build_background)

SQUARE = bg.Footprint(-0.25, 0.25, -0.25, 0.25)
ROBOT = build_background.robot_footprint()

# Sight-line and object drops of the full scenario file with the robot at the
# origin, as the rule before per-scene placement computed them. Placing a
# scene at dx = 0 must reproduce them exactly.
ORIGIN_SIGHTLINE_AND_OBJECT_DROPS = {
    'single_free_02': {'m_cart': ['target_sightline']},
    'multi_04': {'m_cart': ['target_sightline']},
    'interfere_bearing_01': {'m_cart': ['object_footprint']},
    'interfere_bearing_02': {
        'm_chair': ['object_footprint'], 'm_bin': ['object_footprint', 'object_sightline']},
    'far_mid_01': {'c_bin': ['target_sightline'], 'c_cart': ['target_sightline']},
    'far_mid_02': {
        'c_desk_a': ['target_sightline'], 'c_chair_a': ['target_sightline'],
        'c_bin': ['target_sightline']},
    'far_clamp_01': {
        'm_trolley': ['target_sightline'], 'm_chair': ['target_sightline'],
        'c_cart': ['target_sightline']},
    'far_clamp_02': {
        'c_desk_a': ['target_sightline'], 'c_desk_b': ['target_sightline'],
        'c_chair_a': ['target_sightline']},
    'far_beyond_01': {
        'm_board': ['target_sightline'], 'c_desk_a': ['target_sightline'],
        'c_desk_b': ['target_sightline'], 'c_chair_a': ['target_sightline']},
    'trap_layered_01': {'m_bin': ['object_footprint']},
}


@pytest.fixture(scope='module')
def evaluation():
    return build_background.evaluate('lab')


@pytest.fixture(scope='module')
def object_footprints():
    catalog = load_asset_catalog(str(GZ_CATALOG))
    return {model: build_background.sdf_footprint(resource.uri)
            for model, resource in catalog.objects.items()}


def _scene(robots=((4.0, 0.0),), objects=(), scene_id='synthetic') -> Scene:
    return Scene(
        id=scene_id,
        robots=tuple(RobotSpec(x=x, y=y, yaw=math.pi) for x, y in robots),
        objects=tuple(ObjectSpec(model=model, x=x, y=y, yaw=0.0) for model, x, y in objects),
        repeats_override=None,
    )


def _reasons(x: float, y: float, scene: Scene, dx: float = 0.0,
             footprint: bg.Footprint = SQUARE) -> tuple[str, ...]:
    frame = bg.scene_frame(scene, dx, ROBOT, {'box': SQUARE})
    return bg.drop_reasons(bg.placed_outline(footprint, x, y, 0.0), frame)


def _layout(props, room=bg.Room(-5.9, 11.9, -4.9, 4.9), gap=1.0) -> bg.BackgroundLayout:
    return bg.BackgroundLayout(
        path='synthetic.yaml', sha256='', world='w', room=room,
        placement=bg.Placement(backdrop_gap_m=gap, dx_step_m=0.05),
        props=tuple(bg.BackgroundProp(prop_id, model, x, y, 0.0)
                    for prop_id, model, x, y in props),
    )


# A wide panel standing across the +x axis: the backdrop of a target on it.
PANEL = bg.Footprint(-0.1, 0.1, -2.0, 2.0)
PROP_FOOTPRINTS = {'panel': PANEL, 'box': SQUARE}


def _place(layout, scene, gap=None) -> bg.PlacedScene:
    return bg.place_scene(layout, PROP_FOOTPRINTS, ROBOT, scene, {'box': SQUARE},
                          backdrop_gap_m=gap)


# --- the drop rule ------------------------------------------------------------

def test_target_footprint_fires_inside_the_prop_clearance_and_not_beyond() -> None:
    near = bg.TARGET_PROP_CLEARANCE_M - 0.01
    far = bg.TARGET_PROP_CLEARANCE_M + 0.01

    assert _reasons(4.0, 0.25 + near, _scene()) == (bg.TARGET_FOOTPRINT,)
    assert _reasons(4.0, 0.25 + far, _scene()) == ()


def test_a_prop_between_camera_and_target_is_dropped_for_the_sight_line() -> None:
    assert bg.TARGET_SIGHTLINE in _reasons(2.0, 0.5, _scene())


def test_a_target_with_nothing_behind_it_may_have_a_prop_right_behind_it() -> None:
    # 0.60 m from the spawn point to the prop's near face, on the bearing.
    assert _reasons(4.0 + 0.6 + 0.25, 0.0, _scene()) == ()


def test_a_designed_object_behind_a_target_keeps_its_cone_clear_up_to_the_object() -> None:
    scene = _scene(objects=(('box', 6.0, 0.6),))
    # Off the object's own sight line but inside the target's cone, nearer
    # than the object: dropped for the cone alone.
    assert _reasons(5.0, -0.45, scene) == (bg.BEHIND_TARGET_OBJECT,)
    # The same prop with no object behind the target stays.
    assert _reasons(5.0, -0.45, _scene()) == ()
    # Beyond the object the cone ends.
    assert _reasons(7.0, -0.55, scene) == ()


def test_objects_keep_their_footprint_margin_and_sight_line() -> None:
    scene = _scene(robots=((3.0, 2.0),), objects=(('box', 6.0, -2.0),))

    assert bg.OBJECT_FOOTPRINT in _reasons(6.0, -2.7, scene)
    assert bg.OBJECT_SIGHTLINE in _reasons(3.0, -1.0, scene)
    assert _reasons(6.0, -3.5, scene) == ()


def test_the_placed_robot_drops_a_prop_beside_it_and_the_camera_moves_with_it() -> None:
    scene = _scene()

    assert _reasons(2.0, 0.9, scene, dx=2.0) == (bg.ROBOT_FOOTPRINT,)
    assert _reasons(2.0, 0.9, scene, dx=0.0) == ()
    assert bg.camera_xy(2.0) == pytest.approx((2.0 + bg.CAMERA_MOUNT_XY[0], 0.0))
    assert bg.scene_frame(scene, 2.0, ROBOT, {}).camera == bg.camera_xy(2.0)


def test_shifting_a_scene_shifts_every_rule_with_it() -> None:
    scene = _scene(objects=(('box', 6.0, 0.6),))

    for x, y in ((2.0, 0.5), (5.0, -0.45), (6.0, 1.25), (4.0, 0.74)):
        assert _reasons(x + 1.5, y, scene, dx=1.5) == _reasons(x, y, scene, dx=0.0)


def test_at_the_origin_a_scene_keeps_the_former_sight_line_and_object_drops(
        evaluation, object_footprints) -> None:
    layout, footprints, _robot, scenarios = evaluation
    outlines = bg.prop_outlines(layout, footprints)
    kept_reasons = {bg.TARGET_SIGHTLINE, bg.OBJECT_FOOTPRINT, bg.OBJECT_SIGHTLINE}

    found = {}
    for scene in scenarios[FULL][1]:
        frame = bg.scene_frame(scene, 0.0, ROBOT, object_footprints)
        drops = {
            prop_id: sorted(set(reasons) & kept_reasons)
            for prop_id, reasons in bg.frame_drops(outlines, frame).items()}
        drops = {prop_id: reasons for prop_id, reasons in drops.items() if reasons}
        if drops:
            found[scene.id] = drops

    assert found == ORIGIN_SIGHTLINE_AND_OBJECT_DROPS


def test_designed_behind_target_objects_stay_clear_in_the_generated_data(
        evaluation, object_footprints) -> None:
    layout, footprints, _robot, scenarios = evaluation
    outlines = bg.prop_outlines(layout, footprints)
    _sha, scenes, placed = scenarios[FULL]
    designed = [scene for scene in scenes
                if scene.id.startswith('interfere_behind_') or scene.id == 'trap_layered_01']

    assert designed
    for scene in designed:
        placement = placed[scene.id].placement
        frame = bg.scene_frame(scene, placement.dx_m, ROBOT, object_footprints)
        assert frame.behind_cones, scene.id
        for prop_id, outline in outlines.items():
            if prop_id in placement.drops:
                continue
            for bearing, half_width, limit in frame.behind_cones:
                nearest = bg.nearest_in_cone(outline, frame.camera, bearing, half_width)
                assert nearest is None or nearest >= limit, (scene.id, prop_id)


def _stl_vertices(path: Path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if 84 + 50 * count == len(raw):
        for index in range(count):
            values = struct.unpack_from('<12f', raw, 84 + 50 * index)
            yield from (values[3:6], values[6:9], values[9:12])
        return
    for line in raw.decode().splitlines():
        fields = line.split()
        if fields and fields[0] == 'vertex':
            yield tuple(float(value) for value in fields[1:4])


def _g1_planar_points() -> list[tuple[float, float]]:
    """Every G1 visual mesh vertex in the model frame, projected on the floor."""

    points = []
    for visual in ET.parse(G1_MODEL).getroot().iter('visual'):
        x, y, _z, roll, pitch, yaw = (
            float(value) for value in (visual.findtext('pose') or '0 0 0 0 0 0').split())
        assert visual.find('geometry/mesh/scale') is None
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        uri = visual.findtext('geometry/mesh/uri')
        for vx, vy, vz in _stl_vertices(G1_MODEL.parent / uri):
            points.append((
                x + cy * cp * vx + (cy * sp * sr - sy * cr) * vy + (cy * sp * cr + sy * sr) * vz,
                y + sy * cp * vx + (sy * sp * sr + cy * cr) * vy + (sy * sp * cr - cy * sr) * vz,
            ))
    return points


@pytest.fixture(scope='module')
def g1_points():
    return _g1_planar_points()


def test_the_prop_clearance_keeps_props_off_the_g1_whatever_its_yaw(g1_points) -> None:
    reach = max(math.hypot(x, y) for x, y in g1_points)

    assert reach == pytest.approx(0.393, abs=0.002)
    assert bg.TARGET_PROP_CLEARANCE_M >= reach + bg.MIN_PROP_GAP_M


def test_no_kept_prop_touches_a_g1_in_the_generated_data(evaluation, g1_points) -> None:
    layout, footprints, _robot, scenarios = evaluation
    outlines = bg.prop_outlines(layout, footprints)
    g1 = bg.Footprint(min(x for x, _ in g1_points), max(x for x, _ in g1_points),
                      min(y for _, y in g1_points), max(y for _, y in g1_points))

    for _sha, scenes, placed in scenarios.values():
        for scene in scenes:
            placement = placed[scene.id].placement
            for target in scene.robots:
                body = bg.placed_outline(g1, target.x + placement.dx_m, target.y, target.yaw)
                for prop_id, outline in outlines.items():
                    if prop_id not in placement.drops:
                        assert bg.outline_distance(outline, body) >= bg.MIN_PROP_GAP_M, (
                            scene.id, prop_id)


def test_nearest_in_cone_is_exact_on_edges_between_corners() -> None:
    # A wall across the cone: its nearest point is mid-edge, on the bearing.
    wall = bg.placed_outline(PANEL, 5.0, 0.0, 0.0)

    assert bg.nearest_in_cone(wall, (0.0, 0.0), 0.0, 0.1) == pytest.approx(4.9)
    assert bg.nearest_in_cone(wall, (0.0, 0.0), 0.0, 0.1, beyond=6.0) is None
    # A box off to the side is outside a narrow cone.
    assert bg.nearest_in_cone(bg.placed_outline(SQUARE, 5.0, 2.0, 0.0),
                              (0.0, 0.0), 0.0, 0.1) is None


# --- placement ----------------------------------------------------------------

def test_the_feasible_interval_keeps_robot_targets_and_objects_off_the_walls() -> None:
    room = bg.Room(-5.9, 11.9, -4.9, 4.9)
    robot_back = min(x for x, _ in ROBOT)

    low, high = bg.feasible_dx(room, _scene(), ROBOT, {})
    assert low == pytest.approx(room.x_min + bg.MARGIN_M - robot_back)
    assert high == pytest.approx(room.x_max - (4.0 + bg.TARGET_RADIUS_M + bg.MARGIN_M))

    _low, high = bg.feasible_dx(room, _scene(objects=(('box', 9.0, 0.0),)), ROBOT,
                                {'box': SQUARE})
    assert high == pytest.approx(room.x_max - (9.25 + bg.MARGIN_M))

    with pytest.raises(bg.BackgroundError, match='leaves the room in y'):
        bg.feasible_dx(room, _scene(robots=((4.0, 4.5),)), ROBOT, {})


def test_the_placement_aims_for_the_backdrop_gap_on_the_grid() -> None:
    layout = _layout([('panel', 'panel', 8.0, 0.0)])
    scene = _scene(robots=((3.0, 0.0),))

    placed = _place(layout, scene)

    # gap = panel face 7.9 - target 3.0 - dx, so the aim of 1.0 is dx 3.9.
    assert placed.placement.dx_m == 3.9
    assert placed.placement.anchor == bg.Anchor(0, 'panel', pytest.approx(1.0))
    assert _place(layout, scene) == placed


def test_equal_gaps_prefer_fewer_drops_then_the_smaller_shift() -> None:
    scene = _scene(robots=((3.0, 0.0),))
    # 1.025 lies halfway between the gaps of dx 3.85 (1.05) and 3.9 (1.0).
    assert _place(_layout([('panel', 'panel', 8.0, 0.0)]), scene, 1.025).placement.dx_m == 3.85

    # A box just behind the robot's back at dx 3.85 but not at 3.9; the room
    # keeps the robot from backing past it, where the box would be a backdrop.
    layout = _layout([('panel', 'panel', 8.0, 0.0), ('box', 'box', 2.83, 0.0)],
                     room=bg.Room(1.5, 11.9, -4.9, 4.9))
    placed = _place(layout, scene, 1.025)
    assert placed.placement.dx_m == 3.9
    assert placed.placement.drops == ()


def test_a_prop_backdrop_wins_over_the_wall_and_the_wall_is_the_fallback() -> None:
    scene = _scene(robots=((3.0, 0.0),))
    off_bearing = _layout([('box', 'box', 8.0, 3.0)])

    placed = _place(off_bearing, scene)
    assert placed.placement.anchor.backdrop == bg.WALL
    # The east wall at 11.9: gap = 11.9 - 3.0 - dx.
    assert placed.placement.dx_m == 7.9
    assert placed.placement.anchor.gap_m == pytest.approx(1.0)

    # The panel can stand at most 1.6 m behind this target before the robot
    # meets the room's west wall; the east wall could give the aimed 3.0 m
    # exactly, once the target drops the panel. The panel still wins.
    narrow = _layout([('panel', 'panel', 8.0, 0.0)], room=bg.Room(-1.5, 11.9, -4.9, 4.9))
    placed = _place(narrow, _scene(robots=((7.0, 0.0),)), 3.0)
    assert placed.placement.anchor.backdrop == 'panel'
    assert placed.placement.dx_m == -0.7
    assert placed.placement.anchor.gap_m == pytest.approx(1.6)


def test_the_anchor_is_the_farthest_target_and_the_first_on_ties() -> None:
    assert bg.anchor_target(_scene(robots=((3.0, 0.0), (5.0, 1.0), (4.0, 0.0)))) == 1
    assert bg.anchor_target(_scene(robots=((4.0, 1.0), (4.0, -1.0)))) == 0


def test_every_generated_placement_is_on_the_grid_inside_the_room(evaluation) -> None:
    layout, _footprints, _robot, scenarios = evaluation
    step = layout.placement.dx_step_m
    room = layout.room

    for _sha, scenes, placed in scenarios.values():
        for scene in scenes:
            dx = placed[scene.id].placement.dx_m
            assert abs(dx / step - round(dx / step)) < 1e-9, scene.id
            assert float(build_background._number(dx)) == dx, scene.id
            for target in scene.robots:
                reach = bg.TARGET_RADIUS_M + bg.MARGIN_M
                assert room.x_min <= target.x + dx - reach, scene.id
                assert target.x + dx + reach <= room.x_max, scene.id


def test_far_beyond_01_fits_in_the_room(evaluation) -> None:
    layout, _footprints, _robot, scenarios = evaluation
    _sha, scenes, placed = scenarios[FULL]
    scene = next(scene for scene in scenes if scene.id == 'far_beyond_01')

    dx = placed[scene.id].placement.dx_m
    assert scene.robots[0].x + dx + bg.TARGET_RADIUS_M + bg.MARGIN_M <= layout.room.x_max


# --- the packaged layout and its generated data -------------------------------

def test_the_packaged_drop_file_is_current(evaluation) -> None:
    layout, footprints, robot, scenarios = evaluation
    _layout_path, drops_path = bg.background_files(str(CONFIG_DIR), 'lab')

    regenerated = build_background.render('lab', layout, footprints, robot, scenarios)

    assert Path(drops_path).read_text(encoding='utf-8') == regenerated, (
        'regenerate with tools/benchmark/build_background.py')


def test_the_layout_stays_inside_the_room_without_touching_props(evaluation) -> None:
    layout, footprints, _robot, _scenarios = evaluation

    assert bg.layout_problems(layout, footprints) == []


def test_every_kept_prop_clears_its_placed_scene_and_every_dropped_one_has_a_reason(
        evaluation, object_footprints) -> None:
    layout, footprints, _robot, _scenarios = evaluation
    outlines = bg.prop_outlines(layout, footprints)

    for path in SCENARIO_FILES:
        scenes = load_scenarios(str(path))
        resolved = bg.resolve_scene_background(
            'lab', str(CONFIG_DIR), str(path), [scene.id for scene in scenes])
        for scene in scenes:
            placement = resolved.placements[scene.id]
            frame = bg.scene_frame(scene, placement.dx_m, ROBOT, object_footprints)
            for prop in layout.props:
                reasons = bg.drop_reasons(outlines[prop.id], frame)
                assert bool(reasons) == (prop.id in placement.drops), (scene.id, prop.id)


def test_every_packaged_scene_resolves_to_known_props() -> None:
    for path in SCENARIO_FILES:
        scenes = yaml.safe_load(path.read_text())['scenes']
        resolved = bg.resolve_scene_background(
            'lab', str(CONFIG_DIR), str(path), [scene['id'] for scene in scenes])
        ids = {prop.id for prop in resolved.layout.props}

        assert set(resolved.placements) == {scene['id'] for scene in scenes}
        assert all(set(dropped) <= ids for dropped in resolved.scene_drops.values())
        assert {resolved.anchor(scene_id).backdrop for scene_id in resolved.placements} <= (
            ids | {bg.WALL})


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
    for name in ('benchmark_background_lab.yaml', FULL):
        shutil.copyfile(CONFIG_DIR / name, config / name)
    shutil.copyfile(CONFIG_DIR / bg.DROPS_DIRECTORY / 'lab.yaml',
                    config / bg.DROPS_DIRECTORY / 'lab.yaml')
    return config


def _scene_ids(path: Path) -> list[str]:
    return [scene['id'] for scene in yaml.safe_load(path.read_text())['scenes']]


def test_an_edited_scenario_file_is_refused_until_regenerated(config_copy: Path) -> None:
    scenario = config_copy / FULL
    scenario.write_text(scenario.read_text().replace('x: 5.02', 'x: 5.03', 1))

    with pytest.raises(bg.BackgroundError, match='regenerate'):
        bg.resolve_scene_background('lab', str(config_copy), str(scenario), _scene_ids(scenario))


def test_an_edited_layout_is_refused_until_regenerated(config_copy: Path) -> None:
    layout = config_copy / 'benchmark_background_lab.yaml'
    layout.write_text(layout.read_text().replace('x: 11.705', 'x: 11.70', 1))
    scenario = config_copy / FULL

    with pytest.raises(bg.BackgroundError, match='different layout'):
        bg.resolve_scene_background('lab', str(config_copy), str(scenario), _scene_ids(scenario))


def test_a_drop_file_from_the_robot_at_the_origin_is_refused(config_copy: Path) -> None:
    drops = config_copy / bg.DROPS_DIRECTORY / 'lab.yaml'
    drops.write_text(drops.read_text().replace(
        bg.DROPS_SCHEMA, 'ridgeback_benchmark_background_drops/1', 1))
    scenario = config_copy / FULL

    with pytest.raises(bg.BackgroundError, match='robot at the world origin.*Regenerate'):
        bg.resolve_scene_background('lab', str(config_copy), str(scenario), _scene_ids(scenario))


def test_a_layout_without_placement_is_refused() -> None:
    with pytest.raises(bg.BackgroundError, match='robot at the world origin'):
        bg.parse_background_layout({'schema': 'ridgeback_benchmark_background_layout/1'})
    with pytest.raises(bg.BackgroundError, match='placement'):
        bg.parse_background_layout({
            'schema': bg.LAYOUT_SCHEMA, 'world': 'w',
            'room': {'x_min': 0, 'x_max': 1, 'y_min': 0, 'y_max': 1},
            'props': [{'id': 'a', 'model': 'm', 'x': 0, 'y': 0}]})


def test_a_scene_without_a_drop_list_is_refused(config_copy: Path) -> None:
    scenario = config_copy / FULL

    with pytest.raises(bg.BackgroundError, match='no drop list for scene'):
        bg.resolve_scene_background(
            'lab', str(config_copy), str(scenario), [*_scene_ids(scenario), 'new_scene'])


def test_an_unknown_scenario_file_is_refused(config_copy: Path) -> None:
    custom = config_copy / 'my_scenes.yaml'
    shutil.copyfile(config_copy / FULL, custom)

    with pytest.raises(bg.BackgroundError, match='no drop lists for scenario file my_scenes'):
        bg.resolve_scene_background('lab', str(config_copy), str(custom), _scene_ids(custom))


def test_an_unknown_background_name_is_refused() -> None:
    with pytest.raises(bg.BackgroundError, match='lower-case identifier'):
        bg.background_files(str(CONFIG_DIR), '../lab')
    with pytest.raises(FileNotFoundError):
        bg.resolve_scene_background(
            'office', str(CONFIG_DIR), str(SCENARIO_FILES[0]), [])


def test_layout_parsing_rejects_duplicate_ids_unknown_fields_and_the_wall() -> None:
    document = {
        'schema': bg.LAYOUT_SCHEMA,
        'world': 'w',
        'room': {'x_min': 0, 'x_max': 1, 'y_min': 0, 'y_max': 1},
        'placement': {'backdrop_gap_m': 1.0, 'dx_step_m': 0.05},
        'props': [{'id': 'a', 'model': 'm', 'x': 0, 'y': 0, 'yaw': 0}] * 2,
    }
    with pytest.raises(bg.BackgroundError, match='duplicate prop id'):
        bg.parse_background_layout(document)

    document['props'] = [{'id': 'a', 'model': 'm', 'x': 0, 'y': 0, 'z': 1}]
    with pytest.raises(bg.BackgroundError, match='unknown fields: z'):
        bg.parse_background_layout(document)

    document['props'] = [{'id': bg.WALL, 'model': 'm', 'x': 0, 'y': 0}]
    with pytest.raises(bg.BackgroundError, match='reserved'):
        bg.parse_background_layout(document)
