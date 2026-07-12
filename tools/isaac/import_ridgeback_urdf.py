#!/usr/bin/env python3
"""Regenerate the Ridgeback R100 USD from the Clearpath description.

Pipeline (run when clearpath/robot.yaml changes, not per-launch):

1. ``generate_description`` (clearpath_generator_common) renders
   clearpath/robot.urdf.xacro from clearpath/robot.yaml (repo-local
   setup_path — no ~/clearpath mirror needed)
2. ``xacro`` flattens it to a plain URDF
3. Isaac's URDF importer (isaacsim.asset.importer.urdf, 6.0 class API)
   converts it to USD, keeping every fixed-joint frame as a prim
   (merge_fixed_joints=False) so the sensor frames the topic contract
   names (lidar2d_{0,1}_laser, camera_0_link) exist for sensor mounting
4. A kinematic-holonomic drive rig is appended: a world-anchored
   prismatic-X -> prismatic-Y -> revolute-Z chain into base_link, each
   joint with a pure velocity drive (stiffness 0). The runner converts
   TwistStamped body velocities into these three joint targets; PhysX
   still resolves base collisions. Wheels stay undriven visuals.

Run under isaac_venv with the workspace sourced:

    source install/setup.bash
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        tools/isaac/import_ridgeback_urdf.py

Output: src/ridgeback_autonomy/sim/isaac/usd/robots/ridgeback_r100.usd
(committed artifact; regeneration is deliberate, reviewed in git).
"""
import argparse
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SETUP_PATH = REPO / "clearpath"
# The 6.0 importer treats usd_path as a package DIRECTORY and writes
# <usd_path>/<stem>/<stem>.usda (+ payloads/, Textures/). We flatten one
# level so the committed layout is robots/ridgeback_r100/ridgeback_r100.usda.
OUT_DIR = REPO / "src/ridgeback_autonomy/sim/isaac/usd/robots"
OUT_USD = OUT_DIR / "ridgeback_r100.usd"          # importer staging dir
ENTRY_USD = OUT_DIR / "ridgeback_r100" / "ridgeback_r100.usda"

RIG = {
    "px": dict(kind="prismatic", axis="X"),
    "py": dict(kind="prismatic", axis="Y"),
    "rz": dict(kind="revolute", axis="Z"),
}
DRIVE_DAMPING = 1e5      # pure velocity drive: stiffness 0, high damping
DUMMY_MASS = 1.0         # kg; carriers between the virtual joints


def sh(*cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def generate_flat_urdf(workdir: Path) -> Path:
    sh("ros2", "run", "clearpath_generator_common", "generate_description",
       "-s", f"{SETUP_PATH}/")
    flat = workdir / "ridgeback_r100.urdf"
    with flat.open("w") as f:
        subprocess.run(["xacro", str(SETUP_PATH / "robot.urdf.xacro")],
                       check=True, stdout=f)
    _sanitize_urdf(flat)
    return flat


def _sanitize_urdf(urdf_path: Path) -> None:
    """Make the Clearpath URDF digestible for the 6.0 importer.

    - Give every material-less <visual> a named material: the importer's
      usdex material pass crashes on the unnamed default it fabricates
      (NameCache rejects None) — seen with camera_0_link.
    - Drop all <gazebo> extension elements: gz-only, and their namespaced
      attributes (e.g. {http://gazebosim.org/schema}expressed_in) become
      invalid USD attribute names in the importer's undefined-element
      passthrough.
    """
    import xml.etree.ElementTree as ET
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    named = 0
    for link in root.iter("link"):
        for visual in link.findall("visual"):
            if visual.find("material") is None:
                ET.SubElement(visual, "material",
                              {"name": "clearpath_dark_grey"})
                named += 1

    dropped = 0
    for parent in root.iter():
        for gz in list(parent.findall("gazebo")):
            parent.remove(gz)
            dropped += 1

    tree.write(urdf_path, xml_declaration=True, encoding="unicode")
    print(f"urdf sanitized: {named} anonymous material(s) named, "
          f"{dropped} <gazebo> element(s) dropped", flush=True)


def clearpath_package_paths() -> list:
    """package:// roots the URDF references, for the importer's resolver."""
    from ament_index_python.packages import get_package_share_directory
    pkgs = ("clearpath_platform_description", "clearpath_sensors_description",
            "clearpath_mounts_description", "clearpath_manipulators_description")
    paths = []
    for pkg in pkgs:
        try:
            paths.append({pkg: get_package_share_directory(pkg)})
        except Exception:
            pass
    return paths


def import_urdf_to_usd(urdf_path: Path) -> None:
    # NOTE: app.close() fast-shuts the process — nothing after it runs.
    # All work (import, rig, cleanup, verdict print) happens before close.
    import traceback

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    ok = False
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        # 6.0.1 converter bug: materials harvested from OBJ/DAE meshes can
        # carry name=None (faces without a material — several Clearpath
        # .dae meshes do this) and usdex NameCache.getPrimNames rejects
        # None. Name them before the safe-name pass.
        from urdf_usd_converter._impl import material_cache as _mc
        _orig_store_safe_names = _mc.MaterialCache.store_safe_names

        def _store_safe_names_naming_anon(self, data):
            for i, md in enumerate(data.material_data_list):
                if md.name is None:
                    md.name = f"anon_mesh_material_{i}"
            return _orig_store_safe_names(self, data)

        _mc.MaterialCache.store_safe_names = _store_safe_names_naming_anon

        OUT_USD.parent.mkdir(parents=True, exist_ok=True)
        importer = URDFImporter()
        importer.config = URDFImporterConfig(
            urdf_path=str(urdf_path),
            usd_path=str(OUT_USD),
            merge_fixed_joints=False,        # keep sensor frame prims
            fix_base=False,                  # rig anchors it instead
            collision_type="Convex Hull",
            ros_package_paths=clearpath_package_paths(),
        )
        out = importer.import_urdf()
        print(f"imported -> {out}", flush=True)

        add_planar_rig(Path(out))
        add_sensor_prims(Path(out))
        attach_visual_meshes(app, Path(out), urdf_path)

        # flatten <staging .usd dir>/ridgeback_r100/... -> robots/ridgeback_r100/
        import shutil
        pkg_dir = Path(out).parent                 # <staging>/ridgeback_r100
        final_dir = ENTRY_USD.parent
        if final_dir.exists():
            shutil.rmtree(final_dir)
        pkg_dir.rename(final_dir)
        shutil.rmtree(OUT_USD, ignore_errors=True)

        urdf_path.unlink(missing_ok=True)
        ok = True
        print(f"IMPORT OK -> {ENTRY_USD}", flush=True)
    except Exception:
        traceback.print_exc()
        print("IMPORT FAILED", flush=True)
    app.close()
    if not ok:
        sys.exit(1)


def add_planar_rig(usd_path: Path) -> None:
    """Append world -> px -> py -> rz -> base_link velocity-drive chain."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    default_prim = stage.GetDefaultPrim()
    root_path = default_prim.GetPath()

    # Anchor the rig to the robot's root RIGID BODY. base_link itself is a
    # massless URDF dummy that the importer leaves as a plain Xform — a
    # joint targeting it gets dropped by PhysX (and the whole chain after
    # it, which is how we end up with dofs == ['py']). chassis_link is the
    # first real body.
    base_link = None
    for prim in Usd.PrimRange(default_prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in (
                "base_link", "chassis_link"):
            base_link = prim
            break
    if base_link is None:
        raise RuntimeError(f"no base_link/chassis_link rigid body under {root_path}")
    print(f"rig anchor body: {base_link.GetPath()}", flush=True)

    # Strip the importer's articulation root; a floating-base articulation
    # excludes world-anchored joints (they'd become maximal-coordinate
    # joints, not dofs). The rig instead forms a FIXED-BASE articulation:
    # PhysX convention puts ArticulationRootAPI on the world-attached
    # joint (px below), which pulls px/py/rz into the articulation as
    # dofs alongside the wheels.
    # RemoveAPI on the composed stage cannot always defeat an apiSchemas
    # entry authored inside a payload layer — edit every sublayer where
    # the schema is actually authored.
    from pxr import Sdf
    for layer in stage.GetUsedLayers():
        if layer.anonymous:
            continue

        # NewtonArticulationRootAPI matters too: at stage attach Isaac's
        # multi-backend layer materializes a PhysX articulation root from
        # it, resurrecting the root we removed.
        ROOT_TOKENS = {"PhysicsArticulationRootAPI", "NewtonArticulationRootAPI"}

        def _strip(prim_spec, path):
            schemas = prim_spec.GetInfo("apiSchemas") if prim_spec.HasInfo("apiSchemas") else None
            if not schemas:
                return
            drop = set(ROOT_TOKENS)
            # Wheels are purely visual in the kinematic rig: the planar
            # chain has no z dof, so a wheel collider that ever meets a
            # floor cannot depenetrate vertically and PhysX resolves the
            # overlap sideways — the robot skates away (the P3 "spawn
            # fling", ~3.5e5 contact impulses at rest). Strip their
            # colliders so no world's floor height can start that fight.
            if "_wheel_link/cylinder" in path.pathString:
                drop.add("PhysicsCollisionAPI")
            items = schemas.GetAddedOrExplicitItems()
            if drop & set(items):
                lo = Sdf.TokenListOp()
                lo.explicitItems = [s for s in items if s not in drop]
                prim_spec.SetInfo("apiSchemas", lo)

        layer.Traverse("/", lambda path: _strip(layer.GetPrimAtPath(path), path)
                       if path.IsPrimPath() and layer.GetPrimAtPath(path) else None)
        layer.Save()
    for prim in Usd.PrimRange(default_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    rig_scope = UsdGeom.Scope.Define(stage, root_path.AppendChild("drive_rig"))

    def dummy_body(name):
        x = UsdGeom.Xform.Define(stage, rig_scope.GetPath().AppendChild(name))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(x.GetPrim())
        mass.CreateMassAttr(DUMMY_MASS)
        return x.GetPrim()

    anchor = dummy_body("anchor")
    dummy_x = dummy_body("carrier_x")
    dummy_y = dummy_body("carrier_y")

    def joint(name, kind, axis, body0, body1):
        jpath = rig_scope.GetPath().AppendChild(name)
        if kind == "prismatic":
            j = UsdPhysics.PrismaticJoint.Define(stage, jpath)
            drive_kind = "linear"
        else:
            j = UsdPhysics.RevoluteJoint.Define(stage, jpath)
            drive_kind = "angular"
        j.CreateAxisAttr(axis)
        if body0 is not None:                 # None = anchored to the world
            j.CreateBody0Rel().SetTargets([body0.GetPath()])
        j.CreateBody1Rel().SetTargets([body1.GetPath()])
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive_kind)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(DRIVE_DAMPING)
        drive.CreateTargetVelocityAttr(0.0)
        return j

    # PhysX consumes the root-API joint as the FIXED attachment (not a
    # dof) — give it a dedicated world->anchor fixed joint so px/py/rz
    # all remain real dofs.
    world_fix = UsdPhysics.FixedJoint.Define(
        stage, rig_scope.GetPath().AppendChild("world_fix"))
    world_fix.CreateBody1Rel().SetTargets([anchor.GetPath()])
    UsdPhysics.ArticulationRootAPI.Apply(world_fix.GetPrim())
    # Default TGS velocity-iteration count under-converges the coupled
    # px/py/rz velocity-drive solve: mixed translate+rotate commands
    # tracked at only ~60-70% (single axes exact). 32/16 makes combined
    # commands track exactly (probed on 6.0.1).
    from pxr import PhysxSchema
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(world_fix.GetPrim())
    art_api.CreateSolverPositionIterationCountAttr(32)
    art_api.CreateSolverVelocityIterationCountAttr(16)

    joint("px", "prismatic", "X", anchor, dummy_x)
    joint("py", "prismatic", "Y", dummy_x, dummy_y)
    rz = joint("rz", "revolute", "Z", dummy_y, base_link)

    # Joint frames: carriers sit at the world origin; the anchor body rests
    # at its imported pose. Author localPos0 on rz so the joint is satisfied
    # at rest instead of PhysX snapping the chassis to the origin.
    from pxr import Gf, UsdGeom as _UsdGeom
    xf = _UsdGeom.Xformable(base_link)
    world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rest = world.ExtractTranslation()
    rz.CreateLocalPos0Attr(Gf.Vec3f(rest))
    rz.CreateLocalPos1Attr(Gf.Vec3f(0.0))

    # The 6.0.1 importer silently drops mesh <collision> elements (the
    # chassis body-collision.stl becomes an empty Xform, purpose=guide,
    # no collider) — without it the robot has no body collision at all
    # (only the riser/camera boxes came through). Author an AABB box
    # collider from the same STL the URDF references.
    _author_chassis_collider(stage, base_link)

    # Wheels must spin freely (visual only) — strip importer-added drives.
    freed = 0
    for prim in Usd.PrimRange(default_prim):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        if "wheel" not in prim.GetName().lower():
            continue
        for kind in ("angular", "linear"):
            if prim.HasAPI(UsdPhysics.DriveAPI, kind):
                drive = UsdPhysics.DriveAPI(prim, kind)
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(0.0)
                drive.CreateTargetVelocityAttr(0.0)
                freed += 1

    stage.GetRootLayer().Save()
    print(f"planar drive rig appended ({freed} wheel drives freed) -> {usd_path}",
          flush=True)


def attach_visual_meshes(app, usd_path: Path, urdf_path: Path) -> None:
    """The 6.0.1 importer silently drops ALL visual mesh geometry (it
    parses the DAEs — its material pass harvests them — then writes empty
    Xforms named after the mesh file stems; zero Mesh prims, no mesh
    files, for this URDF). Convert every URDF visual mesh with Kit's
    asset converter into payloads/meshes/ and reference each under its
    matching (correctly-transformed, material-bound) empty Xform.
    """
    import xml.etree.ElementTree as ET

    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.kit.asset_converter")
    import omni.kit.asset_converter as asset_converter
    from ament_index_python.packages import get_package_share_directory
    from pxr import Usd, UsdGeom

    def resolve(uri: str) -> Path:
        if uri.startswith("package://"):
            pkg, _, rel = uri.removeprefix("package://").partition("/")
            return Path(get_package_share_directory(pkg)) / rel
        if uri.startswith("file://"):
            return Path(uri.removeprefix("file://"))
        if uri.startswith("file:"):
            return Path(uri.removeprefix("file:"))
        return Path(uri)

    # link name -> [(mesh abs path, scale-or-None)]
    visuals: dict[str, list] = {}
    root = ET.parse(urdf_path).getroot()
    for link in root.iter("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            scale = mesh.get("scale")
            visuals.setdefault(link.get("name"), []).append(
                (resolve(mesh.get("filename")), scale))

    mesh_dir = usd_path.parent / "payloads/meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    converted: dict[Path, Path] = {}

    def convert(src: Path) -> Path:
        if src in converted:
            return converted[src]
        dst = mesh_dir / (src.stem + ".usd")
        print(f"  converting {src.name} ...", flush=True)
        ctx = asset_converter.AssetConverterContext()
        # meshes only — DAE "scenes" (d435.dae) carry cameras/lights at
        # multi-meter offsets that would balloon the robot's bounds
        ctx.ignore_camera = True
        ctx.ignore_light = True
        ctx.ignore_animations = True
        task = asset_converter.get_instance().create_converter_task(
            str(src), str(dst), None, ctx)
        # the converter is asyncio-driven: polling is_finished() without
        # awaiting never schedules it (hung a full regen). Drive the
        # canonical coroutine, with a hard per-mesh timeout.
        import asyncio
        loop = asyncio.get_event_loop()
        try:
            ok = loop.run_until_complete(
                asyncio.wait_for(task.wait_until_finished(), timeout=120))
        except asyncio.TimeoutError:
            raise RuntimeError(f"asset convert timed out for {src}")
        if not ok or not dst.exists():
            raise RuntimeError(f"asset convert failed for {src}: "
                               f"{task.get_error_message()}")
        # The converter normalizes into its own centimeter stage: sources
        # with explicit units (DAE <unit meter="X">) get their vertices
        # scaled by X/0.01 (d435.dae in meters came out x100); unitless
        # STLs pass through untouched. Compensate on the slot, and keep
        # the file metadata honest for standalone viewing.
        from pxr import Usd, UsdGeom
        mstage = Usd.Stage.Open(str(dst))
        written_mpu = UsdGeom.GetStageMetersPerUnit(mstage)
        slot_scale = 1.0
        if src.suffix.lower() == ".dae":
            import re as _re
            m = _re.search(rb'<unit[^>]*meter="([0-9.eE+-]+)"',
                           src.read_bytes()[:4096])
            src_unit = float(m.group(1)) if m else 1.0
            slot_scale = written_mpu / src_unit
        UsdGeom.SetStageMetersPerUnit(mstage, 1.0)
        mstage.GetRootLayer().Save()
        converted[src] = (dst, slot_scale)
        return converted[src]

    stage = Usd.Stage.Open(str(usd_path))
    by_name = {}
    for prim in Usd.PrimRange(stage.GetDefaultPrim()):
        by_name.setdefault(prim.GetName(), []).append(prim)

    attached = 0
    for link_name, meshes in visuals.items():
        link_prims = [p for p in by_name.get(link_name, [])
                      if "/Geometry/" in p.GetPath().pathString]
        if not link_prims:
            print(f"  WARNING: no Geometry prim for link {link_name}",
                  flush=True)
            continue
        link_prim = link_prims[0]
        for src, scale in meshes:
            from pxr import Tf
            safe = Tf.MakeValidIdentifier(src.stem)
            # importer names the empty visual Xform after the stem (its
            # own sanitizer may differ from ours — try both); fall back
            # to creating one at identity
            slot = link_prim.GetChild(src.stem) or link_prim.GetChild(safe)
            if not slot:
                slot = stage.DefinePrim(
                    link_prim.GetPath().AppendChild(safe), "Xform")
            dst, unit_scale = convert(src)
            # reference on a CHILD prim, never on the slot: direct arcs
            # beat ancestral ones, so a reference on the slot itself lets
            # the mesh file's identity xform ops override the importer's
            # visual-origin ops that live in the payload layer (the axle
            # rendered unrotated this way — same masking class as the G1
            # include bug)
            geom = stage.DefinePrim(slot.GetPath().AppendChild("geom"),
                                    "Xform")
            geom.GetReferences().AddReference(
                f"./payloads/meshes/{dst.name}")
            sx = sy = sz = unit_scale
            if scale:
                usx, usy, usz = (float(v) for v in scale.split())
                sx, sy, sz = sx * usx, sy * usy, sz * usz
            if (sx, sy, sz) != (1.0, 1.0, 1.0):
                from pxr import Gf, UsdGeom as _ug
                attr = geom.GetAttribute("xformOp:scale")
                if attr:      # referenced /World already carries the op
                    attr.Set(Gf.Vec3f(sx, sy, sz))
                else:
                    _ug.Xformable(geom).AddScaleOp().Set(
                        Gf.Vec3f(sx, sy, sz))
            attached += 1

    stage.GetRootLayer().Save()
    # verify the geometry actually composes through the references
    check = Usd.Stage.Open(str(usd_path))
    n_mesh = sum(1 for p in Usd.PrimRange(
        check.GetDefaultPrim(), Usd.TraverseInstanceProxies())
        if p.IsA(UsdGeom.Mesh))
    print(f"visual meshes attached: {attached} slots, "
          f"{len(converted)} files converted, {n_mesh} Mesh prims compose",
          flush=True)
    if n_mesh == 0:
        raise RuntimeError("mesh attach produced no composed Mesh prims")


def add_sensor_prims(usd_path: Path) -> None:
    """Author the GPU sensor prims INTO the robot package so the committed
    USD is the complete digital twin (open it in the Isaac GUI and the
    sensors are there). Specs stay in their committed sources —
    sim/isaac/ust10lx_2d.json and config/camera_config.json — and are
    baked here at regen time; sensors.py only binds render products and
    ROS publishers to these prims at runtime.
    """
    import json
    import math

    from isaacsim.core.utils.extensions import enable_extension
    # registers OmniLidar + OmniSensorGenericLidarCoreAPI
    enable_extension("omni.usd.schema.omni_sensors")
    from pxr import Gf, Usd, UsdGeom, Vt

    sim_dir = REPO / "src/ridgeback_autonomy/sim/isaac"
    lidar_spec = json.loads((sim_dir / "ust10lx_2d.json").read_text())["attributes"]
    cam_cfg = json.loads(
        (REPO / "src/ridgeback_autonomy/config/camera_config.json")
        .read_text())["camera"]

    stage = Usd.Stage.Open(str(usd_path))

    def find(name):
        for prim in Usd.PrimRange(stage.GetDefaultPrim()):
            if prim.GetName() == name:
                return prim
        raise RuntimeError(f"prim {name} missing from imported robot")

    # Two prims per lidar: the generic rotary model only fires a 180-deg
    # drum transit per tick from startAzimuthOffsetDeg (the valid-window
    # subset of it), regardless of tickRate or emitter patterns —
    # measured against the analytic world grid, not documented. Offset 0
    # covers azimuths [-135, 0], offset -135 covers [0, +135];
    # ros_io.LidarScanAssembler merges both clouds into the 270-deg scan.
    for i in (0, 1):
        laser = find(f"lidar2d_{i}_laser")
        for prim_name, az_offset in (("rtx_lidar", 0.0),
                                     ("rtx_lidar_l", -135.0)):
            lidar = stage.DefinePrim(
                laser.GetPath().AppendChild(prim_name), "OmniLidar")
            if not lidar.ApplyAPI("OmniSensorGenericLidarCoreAPI"):
                raise RuntimeError(
                    "OmniSensorGenericLidarCoreAPI not registered")
            for name, value in lidar_spec.items():
                attr = lidar.GetAttribute(name)
                if not attr:
                    raise RuntimeError(
                        f"{lidar.GetPath()}: no attribute {name}")
                if isinstance(value, list):
                    if all(isinstance(v, int) for v in value):
                        value = Vt.UIntArray(value) if min(value) >= 0 \
                            else Vt.IntArray(value)
                    else:
                        value = Vt.FloatArray([float(v) for v in value])
                attr.Set(value)
            lidar.GetAttribute(
                "omni:sensor:Core:startAzimuthOffsetDeg").Set(az_offset)

    link = find("camera_0_link")
    cam = UsdGeom.Camera.Define(
        stage, link.GetPath().AppendChild("d455_color"))
    xf = UsdGeom.Xformable(cam.GetPrim())
    # color optical pose (matches the camera_optical_tf static publish);
    # quaternion turns USD's -Z-forward/+Y-up into the ROS optical frame
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.015, 0.0))
    xf.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, -0.5))
    width, height = int(cam_cfg["width"]), int(cam_cfg["height"])
    fx, fy = float(cam_cfg["fx"]), float(cam_cfg["fy"])
    focal = 24.0
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(width * focal / fx)
    cam.CreateVerticalApertureAttr(height * focal / fy)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))

    stage.GetRootLayer().Save()
    hfov = math.degrees(2 * math.atan(width / (2 * fx)))
    print(f"sensor prims baked: 2x UST-10LX + D455 camera "
          f"({width}x{height}, hfov {hfov:.1f} deg) -> {usd_path}", flush=True)


def _stl_aabb(path: Path):
    """Min/max corners of an STL (binary or ASCII), model units."""
    import re
    import struct

    data = path.read_bytes()
    if len(data) >= 84:
        (n_tris,) = struct.unpack_from("<I", data, 80)
        if 84 + 50 * n_tris == len(data):          # well-formed binary STL
            lo = [float("inf")] * 3
            hi = [float("-inf")] * 3
            for i in range(n_tris):
                base = 84 + 50 * i + 12            # skip the normal
                for v in range(3):
                    x, y, z = struct.unpack_from("<3f", data, base + 12 * v)
                    for k, val in enumerate((x, y, z)):
                        lo[k] = min(lo[k], val)
                        hi[k] = max(hi[k], val)
            return lo, hi
    floats = re.findall(
        rb"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", data)
    if not floats:
        raise RuntimeError(f"cannot parse STL {path}")
    verts = [tuple(float(c) for c in f) for f in floats]
    return ([min(v[k] for v in verts) for k in range(3)],
            [max(v[k] for v in verts) for k in range(3)])


def _author_chassis_collider(stage, chassis_prim) -> None:
    from ament_index_python.packages import get_package_share_directory
    from pxr import Gf, UsdGeom, UsdPhysics

    stl = Path(get_package_share_directory(
        "clearpath_platform_description")) / "meshes/r100/body-collision.stl"
    lo, hi = _stl_aabb(stl)
    size = [hi[k] - lo[k] for k in range(3)]
    center = [(hi[k] + lo[k]) / 2.0 for k in range(3)]
    if not (0.3 < size[0] < 2.0 and 0.3 < size[1] < 2.0):
        print(f"WARNING: chassis collider AABB looks off: size={size}",
              flush=True)

    cube = UsdGeom.Cube.Define(
        stage, chassis_prim.GetPath().AppendChild("chassis_collision"))
    cube.CreateSizeAttr(1.0)
    cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*center))
    UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    print(f"chassis collider: AABB of {stl.name} size="
          f"({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) center="
          f"({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--skip-generate", action="store_true",
                    help="reuse existing clearpath/robot.urdf.xacro")
    args = ap.parse_args()

    workdir = OUT_USD.parent
    workdir.mkdir(parents=True, exist_ok=True)

    if args.skip_generate and (SETUP_PATH / "robot.urdf.xacro").exists():
        flat = workdir / "ridgeback_r100.urdf"
        with flat.open("w") as f:
            subprocess.run(["xacro", str(SETUP_PATH / "robot.urdf.xacro")],
                           check=True, stdout=f)
        _sanitize_urdf(flat)         # importer chokes on raw output either way
    else:
        flat = generate_flat_urdf(workdir)

    import_urdf_to_usd(flat)
    flat.unlink(missing_ok=True)
    print("done")


if __name__ == "__main__":
    sys.exit(main())
