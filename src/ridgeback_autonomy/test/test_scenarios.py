from __future__ import annotations

import os

import pytest

from ridgeback_autonomy.benchmarking.scenarios import (
    DEFAULT_ROBOT_YAW_RAD,
    ObjectSpec,
    RobotSpec,
    Scene,
    load_scenarios,
    parse_scenarios,
)


CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def test_parse_minimal_scene_applies_default_yaw() -> None:
    scenes = parse_scenarios({'scenes': [{'id': 's', 'robots': [{'x': 2.5, 'y': 0.0}]}]})

    assert scenes == (
        Scene(
            id='s',
            robots=(RobotSpec(x=2.5, y=0.0, yaw=DEFAULT_ROBOT_YAW_RAD),),
            objects=(),
            repeats_override=None,
        ),
    )


def test_defaults_block_overrides_robot_yaw() -> None:
    scenes = parse_scenarios({
        'defaults': {'robot_yaw_rad': 1.0},
        'scenes': [{'id': 's', 'robots': [{'x': 1.0, 'y': 2.0}]}],
    })

    assert scenes[0].robots[0].yaw == 1.0


def test_object_parsing_and_default_yaw() -> None:
    scenes = parse_scenarios({
        'scenes': [{
            'id': 'bed',
            'robots': [{'x': 3.5, 'y': 0.0}],
            'objects': [{'model': 'hospital_bed', 'x': 2.2, 'y': 0.1}],
        }],
    })

    assert scenes[0].objects == (ObjectSpec(model='hospital_bed', x=2.2, y=0.1, yaw=0.0),)


def test_repeats_override_honored() -> None:
    scenes = parse_scenarios({
        'scenes': [{'id': 's', 'robots': [{'x': 1.0, 'y': 0.0}], 'repeats_override': 3}],
    })

    assert scenes[0].repeats_override == 3


def test_empty_robots_rejected() -> None:
    with pytest.raises(ValueError, match='robots'):
        parse_scenarios({'scenes': [{'id': 's', 'robots': []}]})


def test_missing_id_rejected() -> None:
    with pytest.raises(ValueError, match='id'):
        parse_scenarios({'scenes': [{'robots': [{'x': 1.0, 'y': 0.0}]}]})


def test_bad_object_model_rejected() -> None:
    with pytest.raises(ValueError, match='model'):
        parse_scenarios({
            'scenes': [{'id': 's', 'robots': [{'x': 1.0, 'y': 0.0}],
                        'objects': [{'model': '', 'x': 1.0, 'y': 0.0}]}],
        })


def test_non_numeric_coordinate_rejected() -> None:
    with pytest.raises(ValueError, match='robot.x'):
        parse_scenarios({'scenes': [{'id': 's', 'robots': [{'x': 'near', 'y': 0.0}]}]})


def test_bool_coordinate_rejected() -> None:
    # bool is an int subclass; it must not slip through as 1.0.
    with pytest.raises(ValueError, match='robot.y'):
        parse_scenarios({'scenes': [{'id': 's', 'robots': [{'x': 1.0, 'y': True}]}]})


def test_invalid_repeats_override_rejected() -> None:
    with pytest.raises(ValueError, match='repeats_override'):
        parse_scenarios({
            'scenes': [{'id': 's', 'robots': [{'x': 1.0, 'y': 0.0}], 'repeats_override': 0}],
        })


def test_duplicate_scene_id_rejected() -> None:
    with pytest.raises(ValueError, match='duplicate'):
        parse_scenarios({
            'scenes': [
                {'id': 'dup', 'robots': [{'x': 1.0, 'y': 0.0}]},
                {'id': 'dup', 'robots': [{'x': 2.0, 'y': 0.0}]},
            ],
        })


def test_empty_scenes_rejected() -> None:
    with pytest.raises(ValueError, match='scenes'):
        parse_scenarios({'scenes': []})


def test_shipped_grid_scenario_reproduces_15_single_robot_scenes() -> None:
    scenes = load_scenarios(os.path.join(CONFIG_DIR, 'benchmark_scenarios.yaml'))

    assert len(scenes) == 15
    assert all(len(scene.robots) == 1 and not scene.objects for scene in scenes)
    # The 5x3 forward/lateral grid is covered exactly once.
    poses = {(scene.robots[0].x, scene.robots[0].y) for scene in scenes}
    assert poses == {
        (f, l) for f in (1.5, 2.5, 3.5, 4.5, 5.5) for l in (-0.75, 0.0, 0.75)
    }


def test_shipped_examples_scenario_parses() -> None:
    scenes = load_scenarios(os.path.join(CONFIG_DIR, 'benchmark_scenarios_examples.yaml'))

    by_id = {scene.id: scene for scene in scenes}
    assert 'inter_robot_occlusion_near_far' in by_id
    assert len(by_id['inter_robot_occlusion_near_far'].robots) == 2
    assert by_id['bed_occluder_single'].objects[0].model == 'hospital_bed'
