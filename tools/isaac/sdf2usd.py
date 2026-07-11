#!/usr/bin/env python3
"""Convert this repo's Gazebo SDF worlds to USD stages for Isaac Sim.

Handles exactly the SDF subset the in-repo worlds use (box/sphere geometry,
static models, per-visual/collision poses, PreviewSurface-able materials,
directional/point lights, ``model://`` includes) and refuses anything else,
so a silent partial conversion can never masquerade as a ported world.
The SDF files remain the source of truth; regenerate after editing them.

Parse layer is dependency-free (unit-tested via colcon); the emit/check
layers need pxr and run under isaac_venv:

    isaac_venv/bin/python3 tools/isaac/sdf2usd.py \
        src/ridgeback_autonomy/sim/worlds/mock_hospital.sdf \
        src/ridgeback_autonomy/sim/isaac/usd/worlds/mock_hospital.usda \
        --model-ref g1=../models/g1/g1.usda
    ... --check   # re-open output, assert per-model world AABBs vs SDF

Geometry-to-lidar semantics match Gazebo: visuals are render geometry (RTX
lidar and cameras see them), collisions become invisible PhysX colliders.
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path


# --------------------------------------------------------------------------
# Parse layer (no pxr)
# --------------------------------------------------------------------------

@dataclass
class Material:
    diffuse: tuple = (0.5, 0.5, 0.5, 1.0)
    emissive: tuple = (0.0, 0.0, 0.0, 1.0)
    specular: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass
class Geom:
    """One <visual> or <collision> entry."""
    name: str
    kind: str                    # "box" | "sphere"
    size: tuple                  # box: (sx,sy,sz); sphere: (r,)
    pose: tuple                  # (x,y,z,roll,pitch,yaw) relative to link
    is_collision: bool
    material: Material | None


@dataclass
class Model:
    name: str
    static: bool
    pose: tuple
    geoms: list[Geom] = field(default_factory=list)


@dataclass
class Include:
    name: str
    uri: str                     # e.g. "model://g1"
    pose: tuple


@dataclass
class Light:
    name: str
    kind: str                    # "directional" | "point"
    pose: tuple
    diffuse: tuple
    direction: tuple | None
    attenuation_range: float | None


@dataclass
class World:
    name: str
    gravity: tuple
    ambient: tuple
    models: list[Model]
    includes: list[Include]
    lights: list[Light]


def _floats(text, expect=None, pad=None):
    vals = tuple(float(v) for v in text.split())
    if pad is not None and len(vals) < pad:
        vals = vals + (0.0,) * (pad - len(vals))
    if expect is not None and len(vals) != expect:
        raise ValueError(f"expected {expect} floats, got {text!r}")
    return vals


def _pose(el) -> tuple:
    p = el.find("pose")
    return _floats(p.text, expect=6) if p is not None else (0.0,) * 6


def _material(el) -> Material | None:
    m = el.find("material")
    if m is None:
        return None

    def color(tag, default):
        c = m.find(tag)
        return _floats(c.text, pad=4)[:4] if c is not None else default

    diffuse = color("diffuse", None) or color("ambient", (0.5, 0.5, 0.5, 1.0))
    return Material(
        diffuse=diffuse,
        emissive=color("emissive", (0.0, 0.0, 0.0, 1.0)),
        specular=color("specular", (0.0, 0.0, 0.0, 1.0)),
    )


def _geom(el, is_collision: bool) -> Geom:
    g = el.find("geometry")
    box, sphere = g.find("box"), g.find("sphere")
    if box is not None:
        kind, size = "box", _floats(box.find("size").text, expect=3)
    elif sphere is not None:
        kind, size = "sphere", (float(sphere.find("radius").text),)
    else:
        unsupported = [c.tag for c in g]
        raise ValueError(f"unsupported geometry {unsupported} in {el.get('name')!r}")
    return Geom(
        name=el.get("name"),
        kind=kind,
        size=size,
        pose=_pose(el),
        is_collision=is_collision,
        material=None if is_collision else _material(el),
    )


def parse_world(sdf_path) -> World:
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{sdf_path}: no <world>")

    models, includes, lights = [], [], []
    for m in world.findall("model"):
        static = (m.findtext("static", "false").strip().lower() == "true")
        model = Model(name=m.get("name"), static=static, pose=_pose(m))
        for link in m.findall("link"):
            link_pose = _pose(link)
            if link_pose != (0.0,) * 6:
                raise ValueError(f"{model.name}: non-zero link pose unsupported")
            for v in link.findall("visual"):
                model.geoms.append(_geom(v, is_collision=False))
            for c in link.findall("collision"):
                model.geoms.append(_geom(c, is_collision=True))
        models.append(model)

    for inc in world.findall("include"):
        includes.append(Include(
            name=inc.findtext("name", "include"),
            uri=inc.findtext("uri", "").strip(),
            pose=_pose(inc),
        ))

    for li in world.findall("light"):
        direction = li.find("direction")
        att = li.find("attenuation/range")
        lights.append(Light(
            name=li.get("name"),
            kind=li.get("type"),
            pose=_pose(li),
            diffuse=_floats(li.findtext("diffuse", "1 1 1 1"), pad=4)[:4],
            direction=_floats(direction.text, expect=3) if direction is not None else None,
            attenuation_range=float(att.text) if att is not None else None,
        ))

    scene_ambient = _floats(world.findtext("scene/ambient", "0.4 0.4 0.4 1"), pad=4)[:4]
    gravity = _floats(world.findtext("gravity", "0 0 -9.81"), expect=3)
    return World(
        name=world.get("name"),
        gravity=gravity,
        ambient=scene_ambient,
        models=models,
        includes=includes,
        lights=lights,
    )


# ---- pose math (shared by emit + check) ------------------------------------

def rpy_to_quat(roll, pitch, yaw):
    """SDF rpy -> quaternion (w, x, y, z); R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rot_mat(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _apply(pose, point):
    q = rpy_to_quat(*pose[3:])
    r = _rot_mat(q)
    x, y, z = point
    return tuple(pose[i] + r[i][0] * x + r[i][1] * y + r[i][2] * z for i in range(3))


def model_world_aabb(model: Model, include_visual=True, include_collision=True):
    """World-space AABB over the model's geoms (corners of each geom's OBB)."""
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for g in model.geoms:
        if g.is_collision and not include_collision:
            continue
        if not g.is_collision and not include_visual:
            continue
        if g.kind == "box":
            hx, hy, hz = (s / 2 for s in g.size)
            corners = [(sx * hx, sy * hy, sz * hz)
                       for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        else:
            r = g.size[0]
            corners = [(sx * r, sy * r, sz * r)
                       for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        for c in corners:
            p = _apply(model.pose, _apply(g.pose, c))
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
    return tuple(lo), tuple(hi)


# --------------------------------------------------------------------------
# Emit layer (pxr)
# --------------------------------------------------------------------------

def _sanitize(name):
    out = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in name)
    return out if not out[:1].isdigit() else "_" + out


def _freeze_include_dynamics(prim) -> None:
    """Included models are set dressing: kill their dynamics, keep their
    colliders (static obstacles the robot cannot drive through) and their
    visuals (RTX lidar and cameras raytrace render geometry). The vendored
    G1 is a live 51-body articulation with 29 unactuated dofs — at play it
    would flop under gravity and jitter against the floor otherwise."""
    # PhysxSchema python module only exists inside Kit — author the physx
    # attribute + apiSchemas entry by hand so this stays runnable under
    # plain-pxr isaac_venv (same constraint as the rest of this file).
    from pxr import Sdf, Usd, UsdPhysics

    frozen = 0
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            schemas = p.GetMetadata("apiSchemas") or Sdf.TokenListOp()
            items = list(schemas.GetAddedOrExplicitItems())
            if "PhysxArticulationAPI" not in items:
                schemas.appendedItems = list(schemas.appendedItems) + [
                    "PhysxArticulationAPI"]
                p.SetMetadata("apiSchemas", schemas)
            p.CreateAttribute("physxArticulation:articulationEnabled",
                              Sdf.ValueTypeNames.Bool).Set(False)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            frozen += 1
    print(f"  include {prim.GetName()}: dynamics frozen on {frozen} bodies",
          flush=True)


def emit_usd(world: World, out_path: Path, model_refs: dict[str, str]):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, f"/{_sanitize(world.name)}")
    stage.SetDefaultPrim(root.GetPrim())
    root_path = root.GetPath()

    scene = UsdPhysics.Scene.Define(stage, root_path.AppendChild("physicsScene"))
    g = Gf.Vec3f(*world.gravity)
    mag = g.GetLength()
    scene.CreateGravityDirectionAttr(g / mag if mag else Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(mag)

    looks = stage.DefinePrim(root_path.AppendChild("Looks"), "Scope")
    material_cache = {}

    def material_for(mat: Material):
        key = (mat.diffuse, mat.emissive, mat.specular)
        if key in material_cache:
            return material_cache[key]
        name = f"mat_{len(material_cache):03d}"
        m = UsdShade.Material.Define(stage, looks.GetPath().AppendChild(name))
        shader = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("shader"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*mat.diffuse[:3]))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*mat.emissive[:3]))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
            1.0 - max(mat.specular[:3]))
        if mat.diffuse[3] < 1.0:
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
                float(mat.diffuse[3]))
        m.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface")
        material_cache[key] = m
        return m

    def set_pose(xformable, pose):
        xformable.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
        w, x, y, z = rpy_to_quat(*pose[3:])
        xformable.AddOrientOp().Set(Gf.Quatf(w, x, y, z))

    def define_geom(parent_path, geom: Geom):
        gpath = parent_path.AppendChild(_sanitize(geom.name))
        if geom.kind == "box":
            prim = UsdGeom.Cube.Define(stage, gpath)
            prim.CreateSizeAttr(1.0)
            set_pose(prim, geom.pose)
            prim.AddScaleOp().Set(Gf.Vec3f(*geom.size))
        else:
            prim = UsdGeom.Sphere.Define(stage, gpath)
            prim.CreateRadiusAttr(float(geom.size[0]))
            set_pose(prim, geom.pose)
        if geom.is_collision:
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            UsdGeom.Imageable(prim).CreateVisibilityAttr(
                UsdGeom.Tokens.invisible)
            UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
        elif geom.material is not None:
            UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(
                material_for(geom.material))
        return prim

    for model in world.models:
        if not model.static:
            raise ValueError(f"non-static model {model.name!r} unsupported")
        mx = UsdGeom.Xform.Define(stage, root_path.AppendChild(_sanitize(model.name)))
        set_pose(mx, model.pose)
        for geom in model.geoms:
            define_geom(mx.GetPath(), geom)

    for inc in world.includes:
        model_name = inc.uri.removeprefix("model://")
        if model_name not in model_refs:
            raise ValueError(
                f"include {inc.uri!r}: pass --model-ref {model_name}=<usd path>")
        ix = UsdGeom.Xform.Define(stage, root_path.AppendChild(_sanitize(inc.name)))
        set_pose(ix, inc.pose)
        # Reference on a CHILD prim, never on the posed prim itself: a
        # reference merges the target prim into the referencing prim, so
        # our xformOp:translate would mask the target's own ops (the G1
        # wrapper's 0.792 pelvis lift silently vanished this way and the
        # figure stood waist-deep in the floor).
        inner = stage.DefinePrim(ix.GetPath().AppendChild("model"), "Xform")
        inner.GetReferences().AddReference(model_refs[model_name])
        _freeze_include_dynamics(inner)

    for light in world.lights:
        lpath = root_path.AppendChild(_sanitize(light.name))
        if light.kind == "directional":
            lp = UsdLux.DistantLight.Define(stage, lpath)
            lp.CreateIntensityAttr(1000.0)
            if light.direction is not None:
                # DistantLight emits along local -Z; aim it down <direction>
                d = Gf.Vec3d(*light.direction).GetNormalized()
                rot = Gf.Rotation(Gf.Vec3d(0, 0, -1), d)
                q = rot.GetQuat()
                UsdGeom.Xformable(lp).AddOrientOp().Set(Gf.Quatf(
                    q.GetReal(), *(float(v) for v in q.GetImaginary())))
        elif light.kind == "point":
            lp = UsdLux.SphereLight.Define(stage, lpath)
            lp.CreateRadiusAttr(0.05)
            lp.CreateIntensityAttr(30000.0)
            if light.attenuation_range:
                lp.CreateSpecularAttr(1.0)
            UsdGeom.Xformable(lp).AddTranslateOp().Set(Gf.Vec3d(*light.pose[:3]))
        else:
            raise ValueError(f"unsupported light type {light.kind!r}")
        lp.CreateColorAttr(Gf.Vec3f(*light.diffuse[:3]))

    # Match gz scene ambient so cameras see comparable base illumination
    dome = UsdLux.DomeLight.Define(stage, root_path.AppendChild("ambient_dome"))
    dome.CreateIntensityAttr(300.0)
    dome.CreateColorAttr(Gf.Vec3f(*world.ambient[:3]))

    stage.GetRootLayer().Save()


# --------------------------------------------------------------------------
# Check layer (pxr): converted USD vs parsed SDF, per-model world AABBs
# --------------------------------------------------------------------------

def check_usd(world: World, usd_path: Path, tol=1e-5) -> list[str]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    # positional (Boost binding): time, purposes, useExtentsHint, then
    # ignoreVisibility=True because collision prims are authored invisible
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.guide],
        False,
        True,
    )
    root = stage.GetDefaultPrim()
    errors = []

    expected_children = {(_sanitize(m.name)) for m in world.models}
    expected_children |= {(_sanitize(i.name)) for i in world.includes}
    for model in world.models:
        prim = root.GetChild(_sanitize(model.name))
        if not prim:
            errors.append(f"missing model prim: {model.name}")
            continue
        want_lo, want_hi = model_world_aabb(model)
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        got_lo, got_hi = tuple(box.GetMin()), tuple(box.GetMax())
        for want, got, side in ((want_lo, got_lo, "min"), (want_hi, got_hi, "max")):
            if any(abs(w - g) > tol for w, g in zip(want, got)):
                errors.append(
                    f"{model.name} AABB {side}: sdf={tuple(round(v, 6) for v in want)} "
                    f"usd={tuple(round(v, 6) for v in got)}")

    # Includes: the referenced model's ground-contact convention says its
    # feet sit at local z=0, so the composed subtree's lowest point must
    # land at the include's pose z (this is what regresses if a reference
    # ever masks the wrapper's own xform ops again).
    for inc in world.includes:
        prim = root.GetChild(_sanitize(inc.name))
        if not prim:
            errors.append(f"missing include prim: {inc.name}")
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        feet_z = box.GetMin()[2]
        if abs(feet_z - inc.pose[2]) > 0.02:
            errors.append(
                f"{inc.name}: subtree bottom z={feet_z:.4f}, expected "
                f"pose z={inc.pose[2]:.4f} (wrapper xform masked?)")

    got_children = {c.GetName() for c in root.GetChildren()}
    missing = expected_children - got_children
    if missing:
        errors.append(f"missing prims: {sorted(missing)}")
    return errors


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("sdf", type=Path)
    ap.add_argument("usd", type=Path)
    ap.add_argument("--model-ref", action="append", default=[],
                    metavar="NAME=USD_PATH",
                    help="USD file to reference for <include model://NAME>")
    ap.add_argument("--check", action="store_true",
                    help="only verify an existing output against the SDF")
    args = ap.parse_args()

    world = parse_world(args.sdf)
    refs = dict(kv.split("=", 1) for kv in args.model_ref)

    if not args.check:
        args.usd.parent.mkdir(parents=True, exist_ok=True)
        if args.usd.exists():
            args.usd.unlink()
        emit_usd(world, args.usd, refs)
        n_geoms = sum(len(m.geoms) for m in world.models)
        print(f"wrote {args.usd}: {len(world.models)} models / {n_geoms} geoms, "
              f"{len(world.includes)} includes, {len(world.lights)} lights")

    errors = check_usd(world, args.usd)
    if errors:
        print("CHECK FAILED:", *errors, sep="\n  - ")
        return 1
    print(f"check ok: {len(world.models)} model AABBs match within tolerance")
    return 0


if __name__ == "__main__":
    sys.exit(main())
