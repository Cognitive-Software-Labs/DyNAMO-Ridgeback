#!/usr/bin/env python3
"""Extract the Clearpath Ridgeback chassis from NVIDIA's stock Isaac asset.

Our robot USD comes from a URDF import whose body is a coarse shell -- open
gaps under the deck, no rear end panel, flat untextured materials. NVIDIA ships
an authored Ridgeback in the Isaac asset catalog (BSD-3-Clause, Clearpath
Robotics) whose hull matches ours to within a few mm but is a closed, textured,
properly chamfered body. This pulls the chassis geometry out of it so the robot
USD can reference it in place of the imported shell.

    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/extract_vendor_chassis.py --out <dir>

What is kept, and why the rest is dropped:

* base_link/visuals   -- body, end covers (incl. the rear panel the import
                         never had), side covers, lights, rockers, top deck.
* base_link/collisions/mesh_0  chassis convexHull, and mesh_5 the deck plate.

* WHEELS (visuals mesh_3/4/6/7, collision cylinders mesh_1..4) are dropped:
  the vendor drives its base as ONE rigid body with three dummy planar joints
  and static wheels, while our rig keeps real articulated wheel links that
  spin. Taking theirs would double every wheel.
* The UR5 arm, its base plate (mesh_15 / collision mesh_6), the dummy joint
  chain, physicsScene, FlatGrid and Render prims all go -- we have our own
  drive_rig and articulation root, and a second one would fight it.

Internal prim paths are preserved so material bindings keep resolving; the
whole /ridgeback prim is copied and then pruned, rather than re-parenting the
pieces into a fresh root and breaking every binding.
"""
from __future__ import annotations

import argparse
import shutil
import sys
import urllib.request
from pathlib import Path

ASSET_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
             "Assets/Isaac/6.0/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd")
LICENSE_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
               "Assets/Isaac/6.0/Isaac/Robots/Clearpath/LICENSE")

# base_link/visuals children to drop (wheels + arm mount plate)
DROP_VISUALS = {"mesh_3", "mesh_4", "mesh_6", "mesh_7", "mesh_15"}
# base_link/collisions children to drop (wheel cylinders + arm base)
DROP_COLLISIONS = {"mesh_1", "mesh_2", "mesh_3", "mesh_4", "mesh_6"}
# top-level prims under /ridgeback to drop
DROP_ROOT_CHILDREN = {"world", "dummy_base_x", "dummy_base_y"}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path,
                    default=Path("src/ridgeback_autonomy/sim/isaac/usd/robots/"
                                 "ridgeback_r100/payloads/meshes"),
                    help="directory to write ridgeback_chassis_clearpath.usd into")
    ap.add_argument("--name", default="ridgeback_chassis_clearpath")
    ap.add_argument("--src", type=Path, default=None,
                    help="local ridgeback_ur5.usd (downloaded if omitted)")
    ap.add_argument("--cache", type=Path, default=Path("/tmp/ridgeback_ur5.usd"))
    return ap.parse_args()


def fetch(args) -> Path:
    if args.src:
        return args.src
    if not args.cache.exists():
        print(f"downloading {ASSET_URL}", flush=True)
        urllib.request.urlretrieve(ASSET_URL, args.cache)
    return args.cache


def main() -> int:
    args = parse_args()
    src = fetch(args)

    from pxr import Usd, UsdGeom, UsdPhysics

    args.out.mkdir(parents=True, exist_ok=True)
    dst = args.out / f"{args.name}.usd"

    # Flatten first: the source composes references/payloads, and we want a
    # single self-contained layer in the repo rather than a boot-time fetch.
    stage = Usd.Stage.Open(str(src))
    stage.Flatten().Export(str(dst))
    stage = Usd.Stage.Open(str(dst))

    removed = []

    def drop(path: str):
        if stage.RemovePrim(path):
            removed.append(path)

    for name in sorted(DROP_VISUALS):
        drop(f"/ridgeback/base_link/visuals/{name}")
    for name in sorted(DROP_COLLISIONS):
        drop(f"/ridgeback/base_link/collisions/{name}")
    for name in sorted(DROP_ROOT_CHILDREN):
        drop(f"/ridgeback/{name}")
    for prim in list(stage.GetPseudoRoot().GetChildren()):
        if prim.GetName() in ("physicsScene", "FlatGrid", "Render"):
            drop(str(prim.GetPath()))
    for prim in list(stage.GetPrimAtPath("/ridgeback").GetChildren()):
        if prim.GetName().startswith("ur_arm"):
            drop(str(prim.GetPath()))

    # Strip physics from what remains. The chassis becomes plain geometry that
    # our own chassis_link rigid body owns; a nested rigid body or a second
    # articulation root would fight ours at load.
    body_schemas = ("PhysicsRigidBodyAPI", "PhysicsArticulationRootAPI",
                    "PhysxArticulationAPI", "PhysicsMassAPI")
    joint_types = ("PhysicsFixedJoint", "PhysicsPrismaticJoint",
                   "PhysicsRevoluteJoint")
    stripped = 0
    joints = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        applied = set(prim.GetAppliedSchemas())
        for schema in body_schemas:
            if schema in applied:
                prim.RemoveAppliedSchema(schema)
                stripped += 1
        if prim.GetTypeName() in joint_types:
            joints.append(str(prim.GetPath()))
    for path in joints:            # after traversal: RemovePrim during it is unsafe
        drop(path)

    root = stage.GetPrimAtPath("/ridgeback")
    stage.SetDefaultPrim(root)
    stage.GetRootLayer().Save()

    print(f"removed {len(removed)} prims, stripped {stripped} physics schemas")
    for p in removed:
        print(f"  - {p}")

    # Ship the upstream licence beside the asset: BSD-3-Clause requires the
    # notice travel with redistributions.
    lic = args.out / f"{args.name}.LICENSE"
    try:
        urllib.request.urlretrieve(LICENSE_URL, lic)
        print(f"wrote {lic}")
    except Exception as exc:
        print(f"WARNING: could not fetch upstream LICENSE ({exc})", file=sys.stderr)
        return 1

    kept = []
    for grp in ("visuals", "collisions"):
        par = stage.GetPrimAtPath(f"/ridgeback/base_link/{grp}")
        kept.append(f"{grp}: " + ", ".join(c.GetName() for c in par.GetChildren()))
    print(f"wrote {dst} ({dst.stat().st_size/1e6:.1f} MB)")
    for k in kept:
        print(f"  {k}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
