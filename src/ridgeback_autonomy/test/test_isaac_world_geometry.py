from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _load_worlds_module():
    path = (_repo_root() / 'src' / 'ridgeback_autonomy_isaac' / 'sim' /
            'isaac' / 'worlds.py')
    spec = importlib.util.spec_from_file_location('isaac_worlds', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


worlds = _load_worlds_module()


@pytest.mark.parametrize(
    ('world', 'floor_z'),
    [
        ('mock_hospital', 0.05),
        ('empty', 0.05),
        ('g1_distance_calibration', 0.05),
        ('warehouse', 0.0),
        ('warehouse_full', 0.0),
        ('office', 0.0),
        ('hospital', 0.0),
    ],
)
def test_registered_world_heights_share_one_floor_source(world, floor_z):
    assert worlds.floor_z_for_world(world) == pytest.approx(floor_z)
    assert worlds.spawn_z_for_world(world) == pytest.approx(floor_z + 0.02617)
    assert worlds.lidar_plane_z_for_world(world) == pytest.approx(
        floor_z + 0.02617 + 0.2264)


def test_known_world_file_uses_its_registered_floor(tmp_path):
    world_path = tmp_path / 'mock_hospital.usda'
    world_path.touch()

    assert worlds.floor_z_for_world(world_path) == pytest.approx(0.05)


def test_unknown_world_requires_an_explicit_floor(tmp_path):
    world_path = tmp_path / 'custom.usda'
    world_path.touch()

    with pytest.raises(ValueError, match='floor height'):
        worlds.floor_z_for_world(world_path)

    assert worlds.spawn_z_for_world(world_path, floor_z=1.25) == pytest.approx(
        1.27617)
    assert worlds.lidar_plane_z_for_world(
        world_path, floor_z=1.25) == pytest.approx(1.50257)


def test_runtime_and_map_tools_do_not_pin_old_geometry_defaults():
    repo = _repo_root()
    runner = (repo / 'src' / 'ridgeback_autonomy_isaac' / 'sim' / 'isaac' /
              'isaac_runner.py').read_text(encoding='utf-8')
    occupancy = (repo / 'tools' / 'isaac' / 'gt_occupancy.py').read_text(
        encoding='utf-8')

    assert 'default=0.076' not in runner
    assert 'LIDAR_PLANE_Z = 0.3024' not in occupancy
