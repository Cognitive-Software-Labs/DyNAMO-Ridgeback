"""F3 outfit assignment, recoloured assets and Gazebo skin generation, independent of Kit."""
from collections import Counter
import hashlib
from itertools import combinations
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from PIL import Image
import pytest

from ridgeback_leader_follow import crowd_appearance, f3_scene
from ridgeback_leader_follow.crowd_appearance import (
    LOOKALIKE_OF, MIN_TOP_DELTA_E, OUTFITS, assign_outfits, delta_e)
from ridgeback_leader_follow.crowd_runner import main
from ridgeback_leader_follow.crowd_scenarios import CROWD_SIZES, SCENARIOS, load_scenario

SIM = Path(__file__).resolve().parents[1] / 'sim'
ASSETS = SIM / 'assets/maya_f3'
COLLADA = '{http://www.collada.org/2005/11/COLLADASchema}'


def all_scenarios():
    return [load_scenario(name, size) for name in SCENARIOS
            for size in (CROWD_SIZES if name == 'crowd' else (4,))]


def test_outfits_are_named_uniquely_and_tops_are_clearly_distinct():
    assert len({o.name for o in OUTFITS}) == len(OUTFITS)
    for a, b in combinations(OUTFITS, 2):
        assert delta_e(a.top, b.top) >= MIN_TOP_DELTA_E, (a.name, b.name)


@pytest.mark.parametrize('scenario', all_scenarios(), ids=lambda s: f'{s.name}-{len(s.people)}')
def test_assignment_is_deterministic_and_only_lookalikes_match(scenario):
    outfits = assign_outfits(scenario)
    assert outfits == assign_outfits(scenario)
    assert list(outfits) == [p.name for p in scenario.people]
    leader = next(p.name for p in scenario.people if p.role == 'leader')
    assert outfits[leader] == OUTFITS[0]
    for a, b in combinations(scenario.people, 2):
        imitates = LOOKALIKE_OF.get(a.name) == b.name or LOOKALIKE_OF.get(b.name) == a.name
        assert (outfits[a.name] == outfits[b.name]) == imitates, (a.name, b.name)
    if scenario.name.startswith('lookalike'):
        assert outfits['lf_lookalike'] == outfits[leader]


def test_assignment_leaves_trajectories_untouched():
    for scenario in all_scenarios():
        before = repr(scenario)
        assign_outfits(scenario)
        assert repr(scenario) == before


def test_appearance_modules_never_publish_to_ros():
    for module in (crowd_appearance, f3_scene):
        source = Path(module.__file__).read_text()
        assert 'rclpy' not in source and 'create_publisher' not in source


def test_outfit_textures_match_receipt_and_palette():
    receipt = json.loads((ASSETS / 'outfits.json').read_text())
    for name, digest in receipt['sha256'].items():
        assert hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() == digest
    for name, digest in receipt['source_sha256'].items():
        assert hashlib.sha256((SIM / 'assets/maya' / name).read_bytes()).hexdigest() == digest
    for outfit in OUTFITS:
        files = f3_scene.texture_files(outfit, SIM / 'assets/maya', ASSETS)
        assert set(files) == set(f3_scene.SOURCE_IMAGES)
        assert all(path.is_file() for path in files.values())
        for garment in f3_scene.GARMENTS:
            pixels = np.asarray(Image.open(files[garment]).convert('RGB'), dtype=float) / 255
            # Recolouring preserves mean linear albedo; compare in sRGB with tolerance.
            assert delta_e(pixels.reshape(-1, 3).mean(0), getattr(outfit, garment)) < 6


def test_collada_skin_is_receipted_single_rooted_and_masked():
    receipt = json.loads((ASSETS / 'collada.json').read_text())
    skin = ASSETS / 'walk.dae'
    assert hashlib.sha256(skin.read_bytes()).hexdigest() == receipt['sha256']['walk.dae']
    assert receipt['max_deviation_from_isaac_cache_m'] < 1e-3
    assert receipt['joined_skin_max_deviation_m'] < 1e-3
    tree = ET.parse(skin)
    controllers = list(tree.iter(f'{COLLADA}instance_controller'))
    assert len(controllers) == 1
    assert len(controllers[0].findall(f'{COLLADA}skeleton')) == 1
    images = {e.text for e in tree.iter(f'{COLLADA}init_from') if e.text and e.text.endswith('.png')}
    assert images == {f'{name}.png' for name in f3_scene.SOURCE_IMAGES}
    masked = [e for e in tree.iter(f'{COLLADA}transparent') if e.get('opaque') == 'A_ONE']
    assert len(masked) == 3


def test_gazebo_skins_are_unique_per_outfit_and_shared_by_lookalikes(tmp_path):
    crossing = load_scenario('crossing')
    skins = f3_scene.gazebo_actor_skins(crossing, tmp_path / 'crossing')
    assert len(set(skins.values())) == 2
    for path in set(skins.values()):
        outfit = path.stem
        text = path.read_text()
        for canonical in f3_scene.SOURCE_IMAGES:
            unique = path.parent / f'{outfit}_{canonical}.png'
            assert f'<init_from>{unique.name}</init_from>' in text
            assert unique.resolve().is_file()
    twins = f3_scene.gazebo_actor_skins(load_scenario('lookalike_parallel'), tmp_path / 'twins')
    assert twins['lf_leader'] == twins['lf_lookalike']
    crowd = f3_scene.gazebo_actor_skins(load_scenario('crowd', 8), tmp_path / 'crowd')
    assert max(Counter(crowd.values()).values()) == 1


def test_gazebo_world_uses_f3_skins_and_gait_only_when_given(tmp_path):
    from ridgeback_leader_follow.crowd_geometry import gazebo_world
    scenario = load_scenario('crossing')
    baseline = ET.fromstring(gazebo_world(scenario))
    assert baseline.find('.//plugin/gait_speed') is None
    assert all(e.text.endswith('cesium_man/walk.dae') for e in baseline.iter('filename')
               if e.text.endswith('.dae'))
    skins = f3_scene.gazebo_actor_skins(scenario, tmp_path)
    world = ET.fromstring(gazebo_world(scenario, skins, f3_scene.gait_speed()))
    assert float(world.find('.//plugin/gait_speed').text) == pytest.approx(f3_scene.gait_speed())
    for actor in world.iter('actor'):
        assert actor.find('skin/filename').text == str(skins[actor.get('name')])


def test_f3_profile_is_selectable_but_not_with_lab_demo_walks():
    with pytest.raises(SystemExit):
        main(['--backend', 'gz', '--scene-profile', 'f3', '--demo-cycles', '1'])
