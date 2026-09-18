#!/usr/bin/env python3
"""Read-only inventory of a composed USD (including instance proxies).

Use on the original NVIDIA asset, the production asset and candidate stage.
Unresolved dependencies are reported; missing assets never prove missing physics.
"""

import argparse, hashlib, json
from pathlib import Path
from pxr import Usd, UsdPhysics, UsdUtils


def inventory(path):
    path = Path(path).resolve()
    stage = Usd.Stage.Open(str(path))
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
    result = {
        "source": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "unresolved_dependencies": [str(p) for p in unresolved],
        "layer_hashes": {
            l.realPath: hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest()
            for l in layers
            if l.realPath and Path(l.realPath).exists()
        },
        "joints": [],
        "rigid_bodies": [],
        "collisions": [],
    }
    for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        attrs = {
            a.GetName(): str(a.Get())
            for a in p.GetAttributes()
            if a.GetName().startswith(("physics:", "physx", "isaac:"))
            and a.HasAuthoredValueOpinion()
        }
        item = {"path": str(p.GetPath()), "type": p.GetTypeName(), "attributes": attrs}
        if p.IsA(UsdPhysics.Joint):
            item["body0"] = [
                str(t) for t in UsdPhysics.Joint(p).GetBody0Rel().GetTargets()
            ]
            item["body1"] = [
                str(t) for t in UsdPhysics.Joint(p).GetBody1Rel().GetTargets()
            ]
            result["joints"].append(item)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            result["rigid_bodies"].append(item)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            result["collisions"].append(item)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("asset")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.write_text(json.dumps(inventory(a.asset), indent=2))
