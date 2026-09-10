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


def import_urdf_to_usd(urdf_path: Path, graft: bool = True) -> None:
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
        import tempfile
        pkg_dir = Path(out).parent                 # <staging>/ridgeback_r100
        final_dir = ENTRY_USD.parent

        # The rmtree below wipes the whole committed robot directory, which is
        # where the vendored Clearpath chassis and its licence live. They are
        # not importer output and re-fetching them costs a 7.5 MB download, so
        # carry them across.
        carried = {}
        vendor_dir = final_dir / "payloads" / "meshes"
        stash = Path(tempfile.mkdtemp(prefix="ridgeback_vendor_"))
        for name in ("ridgeback_chassis_clearpath.usd",
                     "ridgeback_chassis_clearpath.LICENSE"):
            src = vendor_dir / name
            if src.exists():
                shutil.copy2(src, stash / name)
                carried[name] = stash / name

        if final_dir.exists():
            shutil.rmtree(final_dir)
        pkg_dir.rename(final_dir)
        shutil.rmtree(OUT_USD, ignore_errors=True)

        for name, src in carried.items():
            (final_dir / "payloads" / "meshes").mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, final_dir / "payloads" / "meshes" / name)
        shutil.rmtree(stash, ignore_errors=True)
        if carried:
            print(f"carried {len(carried)} vendored file(s) across the rebuild",
                  flush=True)

        # Must happen here, not after import_urdf_to_usd returns: kit runs with
        # --/app/fastShutdown=True, so app.close() below hard-exits the process
        # and anything queued after this call never runs.
        if graft:
            graft_vendor_chassis(ENTRY_USD)

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


# Imported meshes the vendor chassis supersedes. Matched against what each
# "geom" prim references rather than against prim paths: attach_visual_meshes
# nests geoms differently per link (body/geom, but side_cover/geom under
# *_side_cover_link), and a hardcoded path list silently misses the odd ones --
# which is how the first pass left both side covers and both end covers
# rendering on top of the vendor's.
VENDOR_REPLACED_MESHES = {
    "body.usd", "side-cover.usd", "end-cover.usd", "lights.usd",
    "axle.usd", "rocker.usd", "top.usd",
}


# Camera mast, measured off the robot 2026-09-10. A 37.5 mm square aluminium
# extrusion on a plate bolted to the top deck. It is not in the Clearpath
# description at all -- the URDF leaves the camera floating in mid-air -- so it
# is authored here rather than coming through the import.
#
# The D435 is bracketed to the mast's FRONT FACE, not sitting on top: the
# camera's back face lands at 0.2590 against a mast front face of 0.2143, i.e.
# a ~45 mm standoff. So the extrusion runs past the camera and ends 50 mm above
# its top (1.045), rather than terminating at the camera's underside.
MAST_SIZE = 0.0375          # square section, m
MAST_X = 0.1955             # 70 of 98 on the tape, as a fraction of the hull
MAST_Z0 = 0.2800            # top deck upper face, above base_link
MAST_Z1 = 1.0950            # camera top 1.045 + 50 mm of extrusion above it

# The bracket carrying the camera off the mast's front face. Its span is
# derived from the live camera mesh rather than hardcoded, so it still fits
# when the RealSense model changes (the D455 body is deeper and much wider
# than the D435 mesh the import currently pulls in).
STANDOFF_SECTION = 0.030                   # square, m

# Mesh path fragments identifying the sensor bodies to re-colour. Matches any
# RealSense variant: the model name is part of the mesh filename, so pinning
# one spelling silently stops painting when the model is swapped.
SENSOR_MESH_TOKENS = ("/hokuyo_ust/", "/d435/", "/d435i/", "/d455/",
                      "/realsense/")


def _ensure_material(stage, path, diffuse, metallic, roughness):
    """Author a UsdPreviewSurface material (idempotent)."""
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*diffuse))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def _paint_sensors(stage, root_path: str, dark) -> int:
    """Give the lidar and camera meshes a sensible colour.

    The STL/DAE converter binds a flat white `DefaultMaterial` to the Hokuyo
    geometry, so both scanners render as white blocks against the vendor
    chassis's proper materials. Those bindings are *direct* on the mesh, so an
    inherited binding on the parent would lose -- rebind each mesh.
    """
    from pxr import Usd, UsdGeom

    painted = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(root_path) or not prim.IsA(UsdGeom.Mesh):
            continue
        if any(tok in path for tok in SENSOR_MESH_TOKENS):
            _bind(prim, dark)
            painted += 1
    return painted


def _camera_mesh_bounds(stage):
    """World-space bounds of the RealSense body, or None if it is absent."""
    import numpy as np
    from pxr import Usd, UsdGeom

    tc = Usd.TimeCode.Default()
    lo = np.array([np.inf] * 3)
    hi = np.array([-np.inf] * 3)
    found = False
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not any(t in path for t in ("/d435/", "/d435i/", "/d455/",
                                       "/realsense/")):
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts:
            continue
        found = True
        a = np.array([[q[0], q[1], q[2]] for q in pts], dtype=np.float64)
        m = np.array(UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(tc),
                     dtype=np.float64).reshape(4, 4)
        w = (np.c_[a, np.ones(len(a))] @ m)[:, :3]
        lo = np.minimum(lo, w.min(0))
        hi = np.maximum(hi, w.max(0))
    return (lo, hi) if found else None


def _author_camera_mast(stage, chassis_prim) -> None:
    """Author the mast, the camera standoff bracket, and their materials.

    Purely cosmetic-plus-collision: no ROS frame hangs off either, so nothing
    in the TF tree changes. Both sit entirely above the 2D lidar plane
    (0.2264), starting at the deck top 0.280, so neither can occlude a scanner
    -- worth re-checking in the empty world after any change to these numbers.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    height = MAST_Z1 - MAST_Z0
    if height <= 0:
        print(f"WARNING: mast height {height:.3f} <= 0, skipped", flush=True)
        return

    mats = "/tn__r1000001_bC/Materials"
    alu = _ensure_material(stage, f"{mats}/mast_aluminium",
                           (0.74, 0.75, 0.77), 0.85, 0.32)
    black = _ensure_material(stage, f"{mats}/bracket_black",
                             (0.045, 0.045, 0.05), 0.0, 0.55)
    sensor_grey = _ensure_material(stage, f"{mats}/sensor_dark_grey",
                                   (0.14, 0.145, 0.16), 0.25, 0.45)

    def box(name, centre, size, material, collide=True):
        cube = UsdGeom.Cube.Define(
            stage, chassis_prim.GetPath().AppendChild(name))
        cube.CreateSizeAttr(1.0)
        UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*centre))
        UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(*size))
        if collide:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        _bind(cube.GetPrim(), material)
        return cube

    box("camera_mast", (MAST_X, 0.0, MAST_Z0 + height / 2.0),
        (MAST_SIZE, MAST_SIZE, height), alu)
    print(f"camera mast: {MAST_SIZE*1000:.1f} mm square, {height:.3f} m tall, "
          f"x={MAST_X:+.4f}, spans z {MAST_Z0:.3f}..{MAST_Z1:.3f} "
          f"(lidar plane 0.2264 is below it), light-grey aluminium",
          flush=True)

    # Standoff: mast front face out to the camera's back face, at the camera's
    # mid-height. Derived from the live mesh so it still fits if the RealSense
    # model changes.
    cam = _camera_mesh_bounds(stage)
    if cam is None:
        print("WARNING: no RealSense mesh found — standoff skipped", flush=True)
    else:
        lo, hi = cam
        x0 = MAST_X + MAST_SIZE / 2.0
        x1 = float(lo[0])
        span = x1 - x0
        if span <= 0.001:
            print(f"WARNING: camera back face {x1:.4f} is not clear of the "
                  f"mast front {x0:.4f} — standoff skipped", flush=True)
        else:
            zc = float(0.5 * (lo[2] + hi[2]))
            box("camera_standoff", (x0 + span / 2.0, 0.0, zc),
                (span, STANDOFF_SECTION, STANDOFF_SECTION), black)
            print(f"camera standoff: {span*1000:.1f} mm bracket, black, "
                  f"x {x0:.4f}..{x1:.4f} at z {zc:.4f}", flush=True)

    painted = _paint_sensors(stage, str(chassis_prim.GetPath().GetParentPath()),
                             sensor_grey)
    print(f"sensor meshes re-coloured: {painted} "
          f"(lidars + RealSense; the converter left them flat white)",
          flush=True)


def graft_vendor_chassis(usd_path: Path) -> None:
    """Swap the imported chassis shell for Clearpath's authored one.

    The 6.0.1 importer yields a coarse body -- open gaps under the deck, no
    rear end panel, flat materials. tools/isaac/extract_vendor_chassis.py
    pulls the authored chassis out of NVIDIA's stock Ridgeback asset; this
    references it and hides what it replaces.

    Wheels stay ours (articulated, they spin; the vendor's are static and were
    dropped on extract). The AABB Cube collider from _author_chassis_collider
    is deactivated in favour of the vendor convexHull, which is tighter at the
    chamfered corners where the box was square.
    """
    from pxr import Usd, UsdGeom

    chassis_usd = (usd_path.parent / "payloads" / "meshes"
                   / "ridgeback_chassis_clearpath.usd")
    if not chassis_usd.exists():
        print(f"WARNING: {chassis_usd.name} missing — keeping the imported "
              f"shell. Run tools/isaac/extract_vendor_chassis.py to fetch it.",
              flush=True)
        return

    stage = Usd.Stage.Open(str(usd_path))
    stage.SetEditTarget(stage.GetRootLayer())
    chassis = None
    for prim in stage.Traverse():
        if prim.GetName() == "chassis_link":
            chassis = prim
            break
    if chassis is None:
        print("WARNING: no chassis_link — vendor graft skipped", flush=True)
        return

    hidden = []
    for prim in Usd.PrimRange(chassis):
        refs = prim.GetMetadata("references")
        if not refs:
            continue
        assets = [Path(r.assetPath).name
                  for r in getattr(refs, "prependedItems", []) or []]
        assets += [Path(r.assetPath).name
                   for r in getattr(refs, "addedItems", []) or []]
        if not any(a in VENDOR_REPLACED_MESHES for a in assets):
            continue
        # Hide the geom's PARENT: the geom itself carries the reference, but
        # the parent is the link-level prim the rest of the tree addresses.
        target = prim.GetParent() if prim.GetName() == "geom" else prim
        UsdGeom.Imageable(target).CreateVisibilityAttr().Set(
            UsdGeom.Tokens.invisible)
        hidden.append(str(target.GetPath()).split("chassis_link/")[-1])

    # riser_link/box is a *rendered* Cube (only box_1, its collider, is
    # guide-purpose). The vendor deck is the real top surface now, so the box
    # would z-fight it.
    box = stage.GetPrimAtPath(chassis.GetPath().AppendChild("riser_link")
                              .AppendChild("box"))
    if box:
        UsdGeom.Imageable(box).CreateVisibilityAttr().Set(
            UsdGeom.Tokens.invisible)
        hidden.append("riser_link/box")

    cube = stage.GetPrimAtPath(
        chassis.GetPath().AppendChild("chassis_collision"))
    if cube:
        cube.SetActive(False)

    vendor = stage.DefinePrim(
        chassis.GetPath().AppendChild("vendor_chassis"), "Xform")
    vendor.GetReferences().AddReference(
        f"./payloads/meshes/{chassis_usd.name}")

    _author_camera_mast(stage, chassis)

    stage.GetRootLayer().Save()
    print(f"vendor chassis grafted ({len(hidden)} imported prims hidden, "
          f"AABB collider deactivated) -> {usd_path}", flush=True)
    for h in sorted(hidden):
        print(f"    hidden: {h}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--skip-generate", action="store_true",
                    help="reuse existing clearpath/robot.urdf.xacro")
    ap.add_argument("--no-vendor-chassis", action="store_true",
                    help="keep the imported shell instead of grafting "
                         "Clearpath's authored chassis over it")
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

    import_urdf_to_usd(flat, graft=not args.no_vendor_chassis)
    flat.unlink(missing_ok=True)
    print("done")


if __name__ == "__main__":
    sys.exit(main())
