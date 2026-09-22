from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
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
        ('initial_test_world', 0.05),
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
    world_path = tmp_path / 'initial_test_world.usda'
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


def test_initial_test_world_map_matches_analytical_sdf_slice():
    repo = _repo_root()
    tools = repo / 'tools' / 'isaac'
    sys.path.insert(0, str(tools))
    try:
        from sdf2usd import parse_world
        from gt_occupancy import build_grid, flood_free
    finally:
        sys.path.remove(str(tools))

    sdf = (repo / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'worlds' /
           'initial_test_world.sdf')
    plane_z = worlds.lidar_plane_z_for_world(sdf)
    source_grid, include_mask, origin = build_grid(
        parse_world(sdf), plane_z=plane_z)
    occupied = source_grid == 100
    free = flood_free(occupied, origin, (0.0, 0.0), 0.05)
    unknown = ~(occupied | free) | include_mask

    map_path = (repo / 'src' / 'ridgeback_autonomy' / 'sim' /
                'ground_truth_maps' / 'initial_test_world.npz')
    with np.load(map_path) as committed:
        assert np.array_equal(committed['grid'], source_grid)
        assert np.array_equal(committed['ignore'], unknown)
        assert committed['origin'] == pytest.approx(origin)
        assert float(committed['resolution']) == pytest.approx(0.05)
        assert float(committed['plane_z']) == pytest.approx(plane_z)
