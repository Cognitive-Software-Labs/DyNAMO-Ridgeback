"""Generated Isaac clutter assets remain reproducible and SDF-equivalent."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    HAS_PXR = True
except ImportError:
    HAS_PXR = False


pytestmark = pytest.mark.skipif(not HAS_PXR, reason='pxr is unavailable')


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GZ_MODELS = REPO_ROOT / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'models'
USD_MODELS = PACKAGE_ROOT / 'sim' / 'isaac' / 'usd' / 'models'
OBJECTS = (
    'cabinet',
    'cabinet_tall',
    'hospital_bed',
    'hospital_bed_high',
    'iv_pole',
    'privacy_curtain',
    'table',
    'table_low',
)

spec = importlib.util.spec_from_file_location(
    'sdf2usd_generated_assets', REPO_ROOT / 'tools' / 'isaac' / 'sdf2usd.py')
sdf2usd = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = sdf2usd
spec.loader.exec_module(sdf2usd)


def _paths(name: str) -> tuple[Path, Path]:
    return GZ_MODELS / name / 'model.sdf', USD_MODELS / name / f'{name}.usda'


@pytest.mark.parametrize('name', OBJECTS)
def test_generated_usd_bounds_match_sdf_within_one_millimetre(name: str) -> None:
    sdf_path, usd_path = _paths(name)
    world = sdf2usd.parse_sdf(sdf_path)

    assert sdf2usd.check_usd(world, usd_path, tol=1e-3) == []

    stage = Usd.Stage.Open(str(usd_path))
    assert not stage.GetDefaultPrim().GetChild('physicsScene')
    assert not stage.GetDefaultPrim().GetChild('ambient_dome')
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.guide],
        False,
        True,
    )
    bounds = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    assert bounds.GetMin()[2] == pytest.approx(0.0, abs=1e-3)


@pytest.mark.parametrize('name', OBJECTS)
def test_regeneration_is_byte_stable(name: str, tmp_path: Path) -> None:
    sdf_path, committed_path = _paths(name)
    regenerated = tmp_path / f'{name}.usda'

    sdf2usd.emit_usd(sdf2usd.parse_sdf(sdf_path), regenerated, {})

    assert regenerated.read_bytes() == committed_path.read_bytes()


def test_visuals_have_materials_and_collisions_are_invisible() -> None:
    _, usd_path = _paths('table')
    stage = Usd.Stage.Open(str(usd_path))
    prims = list(Usd.PrimRange(stage.GetDefaultPrim()))
    visuals = [prim for prim in prims if prim.GetName().startswith('visual_')]
    collisions = [prim for prim in prims if prim.GetName().startswith('collision_')]

    assert visuals and collisions
    assert all(UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
               for prim in visuals)
    assert all(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in collisions)
    assert all(UsdGeom.Imageable(prim).GetVisibilityAttr().Get()
               == UsdGeom.Tokens.invisible for prim in collisions)
    assert all(UsdGeom.Imageable(prim).GetPurposeAttr().Get()
               == UsdGeom.Tokens.guide for prim in collisions)


def test_iv_pole_uses_usd_cylinders() -> None:
    _, usd_path = _paths('iv_pole')
    stage = Usd.Stage.Open(str(usd_path))
    cylinders = [prim for prim in Usd.PrimRange(stage.GetDefaultPrim())
                 if prim.IsA(UsdGeom.Cylinder)]

    assert len(cylinders) == 4
