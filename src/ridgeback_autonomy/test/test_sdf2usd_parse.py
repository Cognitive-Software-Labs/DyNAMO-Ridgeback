"""Parse-layer tests for tools/isaac/sdf2usd.py (no pxr required).

The emit/check layers run under isaac_venv (pxr); the converter's --check
mode covers them end-to-end. Here we pin the dependency-free parse layer:
the SDF subset extraction and the pose/AABB math the check mode trusts.
"""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SDF2USD = _REPO_ROOT / "tools" / "isaac" / "sdf2usd.py"
_WORLDS = _REPO_ROOT / "src" / "ridgeback_autonomy" / "sim" / "worlds"

spec = importlib.util.spec_from_file_location("sdf2usd", _SDF2USD)
sdf2usd = importlib.util.module_from_spec(spec)
sys.modules["sdf2usd"] = sdf2usd  # dataclasses resolve hints via sys.modules
spec.loader.exec_module(sdf2usd)


@pytest.fixture(scope="module")
def hospital():
    return sdf2usd.parse_world(_WORLDS / "mock_hospital.sdf")


def test_world_inventory(hospital):
    assert hospital.name == "mock_hospital"
    assert len(hospital.models) == 32
    assert len(hospital.includes) == 1
    assert len(hospital.lights) == 2

    geoms = [g for m in hospital.models for g in m.geoms]
    assert len(geoms) == 100
    assert sum(g.kind == "box" for g in geoms) == 98
    assert sum(g.kind == "sphere" for g in geoms) == 2
    assert all(m.static for m in hospital.models)


def test_named_wall_exact(hospital):
    wall = next(m for m in hospital.models if m.name == "wall_north")
    assert wall.pose == (5.0, 8.0, 1.4, 0.0, 0.0, 0.0)
    col = next(g for g in wall.geoms if g.is_collision)
    assert col.size == (22.0, 0.2, 2.8)
    vis = next(g for g in wall.geoms if not g.is_collision)
    assert vis.material.diffuse == (0.98, 0.98, 0.99, 1.0)


def test_per_geom_pose_and_yawed_model(hospital):
    table = next(m for m in hospital.models if m.name == "exam_table_n1")
    assert table.pose[5] == 0.35  # yawed model
    top = next(g for g in table.geoms if g.name == "top")
    assert top.pose == (0.0, 0.0, 0.25, 0.0, 0.0, 0.0)
    assert top.size == (1.35, 0.82, 0.10)


def test_transparent_material(hospital):
    details = next(m for m in hospital.models if m.name == "hospital_details")
    curtain = next(g for g in details.geoms if g.name == "privacy_curtain_s2")
    assert curtain.material.diffuse[3] == 0.55


def test_g1_include(hospital):
    inc = hospital.includes[0]
    assert inc.uri == "model://g1"
    assert inc.name == "default_g1_far_right_room"
    # z = floor top (0.05): G1 stands ON the slab, not sunk to its ankles
    assert inc.pose == (13.2, 4.65, 0.05, 0.0, 0.0, 3.14159)


def test_lights(hospital):
    sun = next(li for li in hospital.lights if li.name == "sun")
    assert sun.kind == "directional"
    assert sun.direction == (-0.25, 0.15, -1.0)
    sphere_light = next(li for li in hospital.lights if li.name == "sphere_light")
    assert sphere_light.kind == "point"
    assert sphere_light.attenuation_range == 12.0


def test_rpy_to_quat_yaw_only():
    w, x, y, z = sdf2usd.rpy_to_quat(0.0, 0.0, math.pi / 2)
    assert abs(w - math.cos(math.pi / 4)) < 1e-12
    assert abs(z - math.sin(math.pi / 4)) < 1e-12
    assert x == 0.0 and y == 0.0


def test_world_aabb_of_axis_aligned_wall(hospital):
    wall = next(m for m in hospital.models if m.name == "wall_north")
    lo, hi = sdf2usd.model_world_aabb(wall)
    assert lo == pytest.approx((5 - 11, 8 - 0.1, 1.4 - 1.4))
    assert hi == pytest.approx((5 + 11, 8 + 0.1, 1.4 + 1.4))


def test_world_aabb_respects_yaw():
    model = sdf2usd.Model(
        name="m", static=True, pose=(0, 0, 0, 0, 0, math.pi / 4),
        geoms=[sdf2usd.Geom(name="g", kind="box", size=(2.0, 2.0, 2.0),
                            pose=(0.0,) * 6, is_collision=False, material=None)],
    )
    lo, hi = sdf2usd.model_world_aabb(model)
    half_diag = math.sqrt(2)
    assert hi[0] == pytest.approx(half_diag)
    assert hi[1] == pytest.approx(half_diag)
    assert hi[2] == pytest.approx(1.0)


def test_calibration_world_parses():
    world = sdf2usd.parse_world(_WORLDS / "g1_distance_calibration.sdf")
    assert len(world.models) == 5
    assert all(m.static for m in world.models)
