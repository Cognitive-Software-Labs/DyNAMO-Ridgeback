"""Stage operations behind Isaac scene control, against the packaged G1.

Needs pxr, which the Isaac interpreter provides without starting Kit; skipped
where USD is not importable. (A module-level ``importorskip`` would skip the
whole session under the ROS launch_testing pytest plugin.)
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from scene_provider import (
    InvalidResource,
    SceneEntities,
    SceneRequestError,
    SpawnPlan,
    yaw_quaternion,
)
from usd_scene import UsdScene

try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = UsdGeom = UsdPhysics = None

pytestmark = pytest.mark.skipif(
    Usd is None, reason='needs pxr; run with the Isaac interpreter')


G1 = str(Path(__file__).resolve().parents[1] / 'sim' / 'isaac' / 'usd' / 'models' / 'g1'
         / 'g1.usda')
G1_ORIGIN_LIFT_M = 0.792
G1_BODIES = 51
G1_JOINTS = 51
ENTITY = '/scene_entities/bench_g1'


@pytest.fixture
def stage():
    return Usd.Stage.CreateInMemory()


def yaw_of(orientation) -> float:
    x, y, z, w = orientation
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def test_g1_spawns_static_at_the_requested_pose(stage) -> None:
    scene = UsdScene(stage)

    scene.spawn(ENTITY, G1, (3.0, -1.0, 0.0), yaw_quaternion(2.5))

    position, orientation = scene.pose(ENTITY)
    assert position == pytest.approx((3.0, -1.0, 0.0), abs=1e-9)
    assert yaw_of(orientation) == pytest.approx(2.5, abs=1e-9)
    # The wrapper's own lift survives, so the feet stay at the requested z.
    model = stage.GetPrimAtPath(f'{ENTITY}/model')
    lift = UsdGeom.Xformable(model).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    assert lift[2] == pytest.approx(G1_ORIGIN_LIFT_M)

    bodies = [p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert len(bodies) == G1_BODIES
    assert all(not UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get()
               for p in bodies)
    joints = [p for p in Usd.PrimRange(model, Usd.PrimAllPrimsPredicate)
              if p.IsA(UsdPhysics.Joint)]
    assert len(joints) == G1_JOINTS
    assert not any(p.IsActive() for p in joints)
    roots = [p for p in Usd.PrimRange(model, Usd.PrimAllPrimsPredicate)
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert roots
    for root in roots:
        # Read the composed list: without Kit's PhysX plugin the schema is
        # unregistered and GetAppliedSchemas() hides it.
        assert 'PhysxArticulationAPI' in root.GetMetadata('apiSchemas').ApplyOperations([])
        assert root.GetAttribute('physxArticulation:articulationEnabled').Get() is False


def test_delete_removes_the_whole_entity(stage) -> None:
    scene = UsdScene(stage)
    scene.spawn(ENTITY, G1, (1.0, 0.0, 0.0), yaw_quaternion(0.0))

    assert scene.delete(ENTITY)

    assert not stage.GetPrimAtPath(ENTITY).IsValid()
    assert not stage.GetPrimAtPath(f'{ENTITY}/model').IsValid()
    assert scene.delete(ENTITY)


def test_unloadable_resources_leave_nothing_behind(stage, tmp_path) -> None:
    garbage = tmp_path / 'garbage.usda'
    garbage.write_text('this is not usd\n')
    no_default = tmp_path / 'no_default.usda'
    no_default.write_text('#usda 1.0\ndef Xform "thing"\n{\n}\n')
    scene = UsdScene(stage)

    for resource in (garbage, no_default, tmp_path / 'missing.usda'):
        with pytest.raises(InvalidResource):
            scene.spawn(ENTITY, str(resource), (0.0, 0.0, 0.0), yaw_quaternion(0.0))
        assert not stage.GetPrimAtPath(ENTITY).IsValid()


def test_registry_and_stage_stay_in_step(stage, tmp_path) -> None:
    no_default = tmp_path / 'no_default.usda'
    no_default.write_text('#usda 1.0\ndef Xform "thing"\n{\n}\n')
    entities = SceneEntities(UsdScene(stage), 'r100_0001/robot')

    name = entities.spawn(SpawnPlan('bench_g1', False, G1, (2.0, 0.5, 0.0),
                                    yaw_quaternion(1.0)))
    with pytest.raises(SceneRequestError):
        entities.spawn(SpawnPlan('bench_bad', False, str(no_default), (0.0, 0.0, 0.0),
                                 yaw_quaternion(0.0)))

    scope = stage.GetPrimAtPath('/scene_entities')
    assert [child.GetName() for child in scope.GetChildren()] == [name]
    entities.delete(name)
    assert scope.GetChildren() == []
    assert entities.names() == ['r100_0001/robot']
