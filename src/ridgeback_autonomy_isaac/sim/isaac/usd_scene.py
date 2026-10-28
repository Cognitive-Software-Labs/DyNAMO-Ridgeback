"""USD operations behind benchmark scene control (main loop only).

Each spawned entity is an Xform at the requested world pose with one child,
``model``, that references the resource's default prim. The child keeps the
resource's own root transform intact -- ``g1.usda`` relies on its root
translate to put the origin at ground contact -- exactly as worlds place G1.

Spawned entities are static set dressing: their rigid bodies and any
articulation are disabled, as the world-authored G1 is, so a vendored
articulation stands where it is placed instead of collapsing under gravity,
while its colliders and render geometry stay visible to physics and sensors.
Their joints are deactivated, since nothing they connect can move.

Needs only pxr, so it also runs in a plain interpreter with USD available.
"""
from __future__ import annotations

from scene_provider import InvalidResource, SPAWNED_ENTITY_SCOPE


def freeze_dynamics(prim) -> int:
    """Disable articulations and rigid bodies under ``prim`` and deactivate
    its joints; return the number of bodies made static.

    PhysX refuses a joint between two static bodies with an error per joint
    (51 per G1), and still parses a merely disabled joint: G1's world-anchored
    root joint then warns that its frames disagree once the entity is moved.
    Inactive joint prims are not parsed at all.
    """
    from pxr import Sdf, Usd, UsdPhysics

    frozen = 0
    joints = []
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdPhysics.Joint):
            joints.append(p)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            # the composed list, not GetAppliedSchemas(): without Kit's PhysX
            # plugin the schema is unregistered and would be hidden
            schemas = p.GetMetadata('apiSchemas')
            if schemas is None or 'PhysxArticulationAPI' not in schemas.ApplyOperations([]):
                p.AddAppliedSchema('PhysxArticulationAPI')
            p.CreateAttribute('physxArticulation:articulationEnabled',
                              Sdf.ValueTypeNames.Bool).Set(False)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            frozen += 1
    for joint in joints:
        joint.SetActive(False)
    return frozen


class UsdScene:
    """Spawns, removes, and reads static entities on one stage."""

    def __init__(self, stage, scope: str = SPAWNED_ENTITY_SCOPE) -> None:
        self._stage = stage
        self._scope = scope

    def spawn(self, prim_path, resource_path, position, orientation) -> None:
        from pxr import Gf, Sdf, Tf, UsdGeom

        try:
            layer = Sdf.Layer.FindOrOpen(resource_path)
        except Tf.ErrorException as exc:
            raise InvalidResource(
                f'cannot open "{resource_path}" as a USD layer: {exc}') from exc
        if layer is None:
            raise InvalidResource(f'cannot open "{resource_path}" as a USD layer')
        if not layer.defaultPrim or not layer.GetPrimAtPath(
                Sdf.Path.absoluteRootPath.AppendChild(layer.defaultPrim)):
            raise InvalidResource(f'"{resource_path}" has no default prim')
        if self._stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f'prim {prim_path} already exists')

        if not self._stage.GetPrimAtPath(self._scope).IsValid():
            UsdGeom.Scope.Define(self._stage, self._scope)
        try:
            entity = UsdGeom.Xform.Define(self._stage, prim_path)
            entity.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(*position))
            x, y, z, w = orientation
            entity.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(w, x, y, z))
            model = self._stage.DefinePrim(f'{prim_path}/model', 'Xform')
            model.GetReferences().AddReference(resource_path)
            if not any(spec.layer == layer for spec in model.GetPrimStack()):
                raise InvalidResource(f'"{resource_path}" did not compose')
            freeze_dynamics(model)
        except BaseException:
            self._stage.RemovePrim(prim_path)
            raise

    def delete(self, prim_path) -> bool:
        if self._stage.GetPrimAtPath(prim_path).IsValid():
            self._stage.RemovePrim(prim_path)
        return not self._stage.GetPrimAtPath(prim_path).IsValid()

    def pose(self, prim_path):
        from pxr import Usd, UsdGeom

        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f'prim {prim_path} is missing from the stage')
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        t = matrix.ExtractTranslation()
        q = matrix.ExtractRotationQuat()
        i = q.GetImaginary()
        return (t[0], t[1], t[2]), (i[0], i[1], i[2], q.GetReal())
