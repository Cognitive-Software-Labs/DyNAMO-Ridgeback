#!/usr/bin/env python3
"""Open the robot in the FULL Isaac Sim UI, optionally comparing colliders.

`isaac_runner.py` boots `isaacsim.exp.base.python.kit` -- a minimal experience
with most UI extensions absent, which is why the mesh/collider inspection menus
are missing when you look at a runner window. This boots
`isaacsim.exp.full.kit` instead, so the Physics debug visualisation, mesh
inspection and property panels are all there.

    # just look at the robot, full UI
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/inspect_robot.py

    # chassis collider: convexHull vs convexDecomposition, side by side
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/inspect_robot.py --compare-colliders

In compare mode the LEFT copy (y=0) is convexHull and the RIGHT copy
(y=+separation) is convexDecomposition, both drawn from the same chassis
collision mesh. Collider visualisation is switched on at startup; if you lose
it, it is Physics > Debug > Toggle Collision Visualization in the menu.

convexHull wraps the whole shell in one convex volume, so it fills the
underside cavity and squares off nothing -- cheap, but it cannot represent a
concave hull. convexDecomposition splits it into several convex pieces that
follow the real form, at the cost of more contact work per step. For a
platform that only ever pushes into flat walls the hull is usually enough;
the point of this view is to see how much of the real shape the hull throws
away before deciding.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ROBOT = (REPO / "src/ridgeback_autonomy/sim/isaac/usd/robots/ridgeback_r100"
              / "ridgeback_r100.usda")
CHASSIS = (REPO / "src/ridgeback_autonomy/sim/isaac/usd/robots/ridgeback_r100"
                / "payloads/meshes/ridgeback_chassis_clearpath.usd")


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot", type=Path, default=ROBOT)
    ap.add_argument("--chassis", type=Path, default=CHASSIS)
    ap.add_argument("--compare-colliders", action="store_true",
                    help="two chassis copies: convexHull vs convexDecomposition")
    ap.add_argument("--separation", type=float, default=1.30)
    ap.add_argument("--experience", default="isaacsim.exp.full.kit",
                    help="kit experience file name under isaacsim/apps/")
    return ap.parse_args()


args = parse_args()

from isaacsim import SimulationApp  # noqa: E402

import isaacsim  # noqa: E402
exp = Path(isaacsim.__file__).parent / "apps" / args.experience
if not exp.exists():
    print(f"no such experience: {exp}", file=sys.stderr)
    sys.exit(2)

app = SimulationApp({"headless": False}, experience=str(exp))

import carb  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

settings = carb.settings.get_settings()
# Draw collision geometry from the start; without this the comparison is
# invisible because colliders are guide-purpose and never render.
settings.set("/persistent/physics/visualizationDisplayColliders", True)
settings.set("/persistent/physics/visualizationSimulationOutput", True)
settings.set("/physics/visualizationDisplayColliders", True)

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(900.0)
UsdLux.DistantLight.Define(stage, "/World/key").CreateIntensityAttr(2500.0)


def place(path: str, asset: Path, y: float):
    """Reference an asset under a plain offset parent.

    The referenced layers author their own xformOp:translate and AddXformOp
    refuses to stack a second one, so the offset goes on a parent.
    """
    holder = UsdGeom.Xform.Define(stage, path)
    holder.AddTranslateOp().Set(Gf.Vec3d(0.0, y, 0.0))
    inner = UsdGeom.Xform.Define(stage, path + "/asset")
    inner.GetPrim().GetReferences().AddReference(str(asset))
    return inner.GetPrim()


def set_approximation(root: Usd.Prim, approx: str) -> int:
    n = 0
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr().Set(approx)
            n += 1
    return n


if args.compare_colliders:
    left = place("/World/convexHull", args.chassis, 0.0)
    right = place("/World/convexDecomposition", args.chassis, args.separation)
    a = set_approximation(left, "convexHull")
    b = set_approximation(right, "convexDecomposition")
    print(f"convexHull: {a} collider(s) | convexDecomposition: {b} collider(s)",
          flush=True)
    print("LEFT (y=0) = convexHull   RIGHT (y=+%.2f) = convexDecomposition"
          % args.separation, flush=True)
else:
    place("/World/robot", args.robot, 0.0)

for _ in range(120):
    app.update()

print("\nINSPECTOR READY", flush=True)
try:
    while app.is_running():
        app.update()
except KeyboardInterrupt:
    pass
app.close()
