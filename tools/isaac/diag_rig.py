#!/usr/bin/env python3
"""One-boot standalone diagnostic for the P3 planar-rig motion bug.

Loads a selected world + the Ridgeback package EXACTLY like isaac_runner.py
(same stage order, derived spawn_z translate + world_fix localPos0 authoring,
same articulation bring-up), then steps ~300 frames with NO commands and
prints the physical evidence the fling/slide fix must be based on:

  A. authored joint frames on the drive_rig (world_fix/px/py/rz) as
     composed on the live stage
  B. pre-play authored world transforms of chassis/axle/rockers/wheels,
     wheel-cylinder radius, predicted wheel-bottom z vs the registered floor,
     and both lidar planes vs their derived world height
  C. per-60-frame: rig dof positions/velocities (px/py/rz) vs chassis
     world pose from the physics tensor backend vs the USD-composed pose
     (do they agree?), wheel center/bottom world z
  D. PhysX contact pairs involving the wheels (impulse + normal), via
     the contact-report subscription if reachable
  E. post-stop drift: pose delta over the final 60 frames

Run (same env as the runner):

    source install/setup.bash
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 tools/isaac/diag_rig.py

--battery instead runs the P3 motion acceptance battery in the same
boot: stillness at spawn, set_planar_pose teleport, forward / strafe /
spin / combined tracking, command-timeout stop, and stop drift < 2 cm.
Prints one PASS/FAIL line per phase and a final verdict.
"""
import argparse
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SIM_DIR = REPO / "src/ridgeback_autonomy_isaac/sim/isaac"
sys.path.insert(0, str(SIM_DIR))

FRAMES = 300
PRINT_EVERY = 60
HEIGHT_TOLERANCE = 0.0005

WHEELS = {
    "FL": "Geometry/base_link/chassis_link/axle_link/front_rocker_link/front_left_wheel_link",
    "FR": "Geometry/base_link/chassis_link/axle_link/front_rocker_link/front_right_wheel_link",
    "RL": "Geometry/base_link/chassis_link/axle_link/rear_rocker_link/rear_left_wheel_link",
    "RR": "Geometry/base_link/chassis_link/axle_link/rear_rocker_link/rear_right_wheel_link",
}
CHASSIS = "Geometry/base_link/chassis_link"


def yaw_of_wxyz(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="initial_test_world",
                    help="registered Isaac world name")
    ap.add_argument("--usd-only", action="store_true",
                    help="verify authored wheel/lidar heights without starting "
                         "SimulationApp (works on hosts without an RTX device)")
    ap.add_argument("--battery", action="store_true",
                    help="run the P3 motion acceptance battery instead of "
                         "the passive 300-frame diagnostic")
    ap.add_argument("--spin-transforms", action="store_true",
                    help="spin in place and log whether the LIDAR prim's "
                         "world transform keeps up with the chassis body's "
                         "(port-history.md detached-lidar desync test)")
    ap.add_argument("--wz", type=float, default=0.4,
                    help="yaw rate for --spin-transforms (rad/s)")
    ap.add_argument("--frames", type=int, default=240,
                    help="frames to spin for --spin-transforms")
    args = ap.parse_args()

    if args.usd_only:
        sys.exit(run_usd_geometry(args.world))

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    code = 1
    try:
        code = run(app, world=args.world, battery=args.battery,
                   spin_transforms=args)
    except Exception:
        import traceback
        traceback.print_exc()
        print("DIAG FAILED", flush=True)
    # verdict printed inside run() BEFORE close — Kit swallows late stdout
    app.close()
    sys.exit(code)


def run_usd_geometry(world: str) -> int:
    """Check authored robot heights using USD composition only."""
    from pxr import Usd, UsdGeom

    from worlds import (
        floor_z_for_world,
        lidar_plane_z_for_world,
        spawn_z_for_world,
    )

    floor_z = floor_z_for_world(world)
    spawn_z = spawn_z_for_world(world)
    lidar_plane_z = lidar_plane_z_for_world(world)
    robot_usd = SIM_DIR / "usd/robots/ridgeback_r100/ridgeback_r100.usda"
    stage = Usd.Stage.Open(str(robot_usd))
    root = stage.GetDefaultPrim()
    if not root:
        raise RuntimeError(f"robot USD has no default prim: {robot_usd}")
    tc = Usd.TimeCode.Default()

    def named(name):
        matches = [p for p in Usd.PrimRange(root) if p.GetName() == name]
        if len(matches) != 1:
            raise RuntimeError(f"expected one {name}, found {len(matches)}")
        return matches[0]

    def relative_z(prim):
        return float(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            tc).ExtractTranslation()[2])

    print(f"world={world} floor_z={floor_z:.5f} spawn_z={spawn_z:.5f} "
          f"lidar_plane_z={lidar_plane_z:.5f}")
    ok = True
    cylinder = stage.GetPrimAtPath(
        f"{root.GetPath()}/{WHEELS['FL']}/cylinder")
    if not cylinder:
        raise RuntimeError("front-left wheel cylinder missing")
    radius = float(cylinder.GetAttribute("radius").Get())
    wheel_bottom_z = spawn_z + relative_z(cylinder.GetParent()) - radius
    wheel_error = wheel_bottom_z - floor_z
    passed = abs(wheel_error) <= HEIGHT_TOLERANCE
    ok &= passed
    print(f"wheel_bottom_z={wheel_bottom_z:.5f} expected={floor_z:.5f} "
          f"error={wheel_error:+.5f} [{'PASS' if passed else 'FAIL'}]")

    for name in ("lidar2d_0_laser", "lidar2d_1_laser"):
        sensor_z = spawn_z + relative_z(named(name))
        error = sensor_z - lidar_plane_z
        passed = abs(error) <= HEIGHT_TOLERANCE
        ok &= passed
        print(f"{name}_z={sensor_z:.5f} expected={lidar_plane_z:.5f} "
              f"error={error:+.5f} [{'PASS' if passed else 'FAIL'}]")

    print(f"USD GEOMETRY {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def run(app, world: str = "initial_test_world", battery: bool = False,
        spin_transforms=None) -> int:
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    from robot_rig import RidgebackRig
    from worlds import (
        floor_z_for_world,
        get_assets_root,
        lidar_plane_z_for_world,
        resolve_world,
        spawn_z_for_world,
    )

    world_path = resolve_world(world, get_assets_root())
    floor_z = floor_z_for_world(world)
    spawn_z = spawn_z_for_world(world)
    lidar_plane_z = lidar_plane_z_for_world(world)
    print(f"loading world: {world_path}", flush=True)
    print(f"expected heights: floor_z={floor_z:.5f} spawn_z={spawn_z:.5f} "
          f"lidar_plane_z={lidar_plane_z:.5f}", flush=True)
    ctx = omni.usd.get_context()
    ctx.open_stage(world_path)
    stage = ctx.get_stage()

    # runner parity: bridge AFTER open_stage
    enable_extension("isaacsim.ros2.bridge")

    robot_usd = str(SIM_DIR / "usd/robots/ridgeback_r100/ridgeback_r100.usda")
    robot_prim_path = "/ridgeback"
    robot_prim = stage.DefinePrim(robot_prim_path, "Xform")
    # Match the runner: a bare local path is URL-joined against a remote stock
    # world's root layer and silently fails to compose the robot.
    robot_prim.GetReferences().AddReference(Path(robot_usd).as_uri())
    UsdGeom.XformCommonAPI(robot_prim).SetTranslate(Gf.Vec3d(0.0, 0.0, spawn_z))
    wf = stage.GetPrimAtPath(f"{robot_prim_path}/drive_rig/world_fix")
    if not wf:
        raise RuntimeError("drive_rig/world_fix missing")
    UsdPhysics.FixedJoint(wf).CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, spawn_z))

    # ---- A: authored joint frames on the composed stage -------------------
    print("\n=== A. drive_rig joint frames (composed stage) ===", flush=True)
    for j in ("world_fix", "px", "py", "rz"):
        prim = stage.GetPrimAtPath(f"{robot_prim_path}/drive_rig/{j}")
        rec = {}
        for a in ("physics:localPos0", "physics:localPos1",
                  "physics:localRot0", "physics:localRot1"):
            attr = prim.GetAttribute(a)
            rec[a.split(":")[1]] = attr.Get() if attr and attr.HasAuthoredValue() \
                else ("(default)" if attr else "(no attr)")
        b0 = prim.GetRelationship("physics:body0").GetTargets()
        b1 = prim.GetRelationship("physics:body1").GetTargets()
        print(f"  {j}: body0={[str(t) for t in b0] or 'WORLD'} "
              f"body1={[str(t) for t in b1]}", flush=True)
        print(f"      {rec}", flush=True)
    for name, rel in WHEELS.items():
        jname = {"FL": "front_left", "FR": "front_right",
                 "RL": "rear_left", "RR": "rear_right"}[name]
        jp = stage.GetPrimAtPath(
            f"{robot_prim_path}/Physics/{jname}_wheel_joint")
        if jp:
            lp0 = jp.GetAttribute("physics:localPos0").Get()
            ax = jp.GetAttribute("physics:axis").Get()
            print(f"  {jname}_wheel_joint: axis={ax} localPos0={lp0}", flush=True)

    # ---- B: pre-play authored world transforms ----------------------------
    print("\n=== B. pre-play authored world transforms ===", flush=True)
    tc = Usd.TimeCode.Default()

    def world_of(rel_path):
        p = stage.GetPrimAtPath(f"{robot_prim_path}/{rel_path}")
        if not p:
            return None
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(tc)
        return m.ExtractTranslation()

    for label, rel in [("base_link", "Geometry/base_link"),
                       ("chassis_link", CHASSIS),
                       ("axle_link", CHASSIS + "/axle_link")] + \
                      [(f"wheel_{k}", v) for k, v in WHEELS.items()]:
        t = world_of(rel)
        print(f"  {label}: world_t=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})"
              if t is not None else f"  {label}: MISSING", flush=True)
    radius = None
    cyl = stage.GetPrimAtPath(
        f"{robot_prim_path}/{WHEELS['FL']}/cylinder")
    if cyl:
        radius = cyl.GetAttribute("radius").Get()
    wz = world_of(WHEELS["FL"])
    geometry_ok = True
    if radius is not None and wz is not None:
        wheel_bottom_z = wz[2] - radius
        wheel_error = wheel_bottom_z - floor_z
        geometry_ok &= abs(wheel_error) <= HEIGHT_TOLERANCE
        print(f"  wheel radius={radius}  predicted wheel bottom z="
              f"{wheel_bottom_z:.5f}  floor top={floor_z:.5f}  "
              f"error={wheel_error:+.5f} "
              f"[{'PASS' if abs(wheel_error) <= HEIGHT_TOLERANCE else 'FAIL'}]",
              flush=True)
    else:
        geometry_ok = False
        print("  wheel height [FAIL: wheel cylinder missing]", flush=True)

    laser_prims = {}
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetName() in ("lidar2d_0_laser", "lidar2d_1_laser"):
            laser_prims[prim.GetName()] = prim
    for name in ("lidar2d_0_laser", "lidar2d_1_laser"):
        prim = laser_prims.get(name)
        if prim is None:
            geometry_ok = False
            print(f"  {name} [FAIL: prim missing]", flush=True)
            continue
        laser_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            tc).ExtractTranslation()
        sensor_error = laser_world[2] - lidar_plane_z
        passed = abs(sensor_error) <= HEIGHT_TOLERANCE
        geometry_ok &= passed
        print(f"  {name}: world z={laser_world[2]:.5f} "
              f"expected={lidar_plane_z:.5f} error={sensor_error:+.5f} "
              f"[{'PASS' if passed else 'FAIL'}]", flush=True)

    # ---- contact reporting on wheels + chassis -----------------------------
    for rel in list(WHEELS.values()) + [CHASSIS]:
        p = stage.GetPrimAtPath(f"{robot_prim_path}/{rel}")
        if p and p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0)
            # sleeping bodies emit no contact reports
            PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateSleepThresholdAttr().Set(0)

    contacts = {}          # pair -> [count, max_impulse, sample_normal]

    def on_contact(*cb_args):
        try:
            headers, cdata = cb_args[0], cb_args[1]
            from pxr import PhysicsSchemaTools
            for h in headers:
                a0 = getattr(h, "actor0", None) or getattr(h, "collider0", 0)
                a1 = getattr(h, "actor1", None) or getattr(h, "collider1", 0)
                p0 = str(PhysicsSchemaTools.intToSdfPath(a0))
                p1 = str(PhysicsSchemaTools.intToSdfPath(a1))
                if "wheel" not in p0 and "wheel" not in p1:
                    continue
                pair = tuple(sorted((p0, p1)))
                rec = contacts.setdefault(pair, [0, 0.0, None])
                rec[0] += 1
                try:
                    off = h.contact_data_offset
                    num = h.num_contact_data
                    for d in cdata[off:off + num]:
                        imp = d.impulse
                        mag = math.sqrt(imp.x**2 + imp.y**2 + imp.z**2)
                        if mag >= rec[1]:
                            rec[1] = mag
                            n = d.normal
                            rec[2] = (round(n.x, 3), round(n.y, 3), round(n.z, 3))
                except Exception:
                    pass
        except Exception as e:
            contacts.setdefault(("CALLBACK_ERROR", str(e)), [1, 0.0, None])

    sub = None
    try:
        import omni.physx
        iface = omni.physx.get_physx_simulation_interface()
        for fn_name in ("subscribe_physics_contact_report_events",
                        "subscribe_contact_report_events"):
            fn = getattr(iface, fn_name, None)
            if fn:
                sub = fn(on_contact)
                print(f"contact report: subscribed via {fn_name}", flush=True)
                break
        if sub is None:
            print("contact report: no subscribe API found", flush=True)
    except Exception as e:
        print(f"contact report unavailable: {e}", flush=True)

    # ---- articulation bring-up (runner parity) -----------------------------
    roots = [p.GetPath().pathString for p in Usd.PrimRange(robot_prim)
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    rig_roots = [r for r in roots if "/drive_rig/" in r]
    art_root_path = rig_roots[0] if rig_roots else roots[0]
    print(f"\narticulation roots={roots} using={art_root_path}", flush=True)

    scene_prim = next(p for p in stage.Traverse()
                      if p.GetTypeName() == "PhysicsScene")
    PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr(120.0)

    rig = RidgebackRig(art_root_path, odom_noise=0.0)

    timeline = omni.timeline.get_timeline_interface()
    # runner parity: worlds author no timeCodes -> zero-length looping
    # play range pins get_current_time() at 0
    timeline.set_end_time(1.0e9)
    timeline.set_looping(False)
    timeline.play()
    for _ in range(240):
        app.update()
        if rig.ready():
            break
    else:
        print("DIAG FAILED: articulation never initialized", flush=True)
        return 1
    rig.initialize()
    print(f"dof_names={list(rig._art.dof_names)}", flush=True)
    print(f"dofs pre-reset: {['%.4f' % v for v in rig._rig_positions()]}",
          flush=True)
    rig.set_planar_pose(0.0, 0.0, 0.0)
    print(f"dofs post-reset: {['%.4f' % v for v in rig._rig_positions()]}",
          flush=True)

    # tensor-backend world poses for chassis + wheels
    rp = {}
    try:
        from isaacsim.core.experimental.prims import RigidPrim
        for k, v in list(WHEELS.items()) + [("chassis", CHASSIS)]:
            rp[k] = RigidPrim(f"{robot_prim_path}/{v}")
    except Exception as e:
        print(f"RigidPrim views unavailable: {e}", flush=True)

    def tensor_pose(k):
        try:
            pos, quat = rp[k].get_world_poses()
            p = pos.numpy()[0]
            q = quat.numpy()[0]          # wxyz
            return p, yaw_of_wxyz(q[0], q[1], q[2], q[3])
        except Exception:
            return None, None

    def usd_pose():
        p = stage.GetPrimAtPath(f"{robot_prim_path}/{CHASSIS}")
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(tc)
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        im = q.GetImaginary()
        return t, yaw_of_wxyz(q.GetReal(), im[0], im[1], im[2])

    if spin_transforms is not None and spin_transforms.spin_transforms:
        return run_spin_transforms(
            app, timeline, rig, stage, robot_prim_path, tc,
            tensor_pose, spin_transforms.wz, spin_transforms.frames)

    if battery:
        return run_battery(app, timeline, rig, contacts)

    # ---- C/D: 300 frames, no commands --------------------------------------
    print(f"\n=== C. stepping {FRAMES} frames, no commands ===", flush=True)
    last_sim_time = timeline.get_current_time()
    marks = {}
    for f in range(1, FRAMES + 1):
        sim_time = timeline.get_current_time()
        frame_dt = max(sim_time - last_sim_time, 0.0)
        last_sim_time = sim_time
        rig.step(frame_dt if frame_dt > 0 else 1.0 / 60.0, now=sim_time)
        app.update()

        if f % PRINT_EVERY == 0 or f == 1:
            dofs = rig._rig_positions()
            dvel = rig._rig_velocities()
            cpos, cyaw = tensor_pose("chassis")
            upos, uyaw = usd_pose()
            marks[f] = (dofs, cpos, cyaw)
            print(f"\n-- frame {f} (sim t={sim_time:.3f}) --", flush=True)
            print(f"  dofs px/py/rz: ({dofs[0]:+.4f}, {dofs[1]:+.4f}, "
                  f"{dofs[2]:+.4f})  vel: ({dvel[0]:+.4f}, {dvel[1]:+.4f}, "
                  f"{dvel[2]:+.4f})", flush=True)
            if cpos is not None:
                print(f"  chassis tensor: ({cpos[0]:+.4f}, {cpos[1]:+.4f}, "
                      f"{cpos[2]:+.4f}) yaw {cyaw:+.4f}", flush=True)
            print(f"  chassis USD:    ({upos[0]:+.4f}, {upos[1]:+.4f}, "
                  f"{upos[2]:+.4f}) yaw {uyaw:+.4f}", flush=True)
            for k in WHEELS:
                wpos, _ = tensor_pose(k)
                if wpos is not None and radius is not None:
                    print(f"  wheel {k}: center z={wpos[2]:+.4f} "
                          f"bottom z={wpos[2] - radius:+.4f} "
                          f"(floor {floor_z}, pen {floor_z - (wpos[2] - radius):+.4f})",
                          flush=True)
            if contacts:
                print("  contacts since last mark:", flush=True)
                for pair, (cnt, imp, nrm) in sorted(contacts.items()):
                    s0 = pair[0].split("/")[-1]
                    s1 = pair[1].split("/")[-1] if len(pair) > 1 else "?"
                    print(f"    {s0} <-> {s1}: n={cnt} max|J|={imp:.3f} "
                          f"normal={nrm}", flush=True)
                contacts.clear()
            else:
                print("  contacts since last mark: none", flush=True)

    # ---- E: verdict data ----------------------------------------------------
    print("\n=== E. summary ===", flush=True)
    d0, c0, y0 = marks[PRINT_EVERY * (FRAMES // PRINT_EVERY - 1)]
    d1, c1, y1 = marks[FRAMES]
    print(f"  drift last {PRINT_EVERY} frames: dofs "
          f"d=({d1[0]-d0[0]:+.4f}, {d1[1]-d0[1]:+.4f}, {d1[2]-d0[2]:+.4f})",
          flush=True)
    if c0 is not None and c1 is not None:
        print(f"  chassis tensor drift: ({c1[0]-c0[0]:+.4f}, "
              f"{c1[1]-c0[1]:+.4f}) yaw {y1-y0:+.4f}", flush=True)
        print(f"  dof-vs-world mismatch at end: "
              f"dx={c1[0]-d1[0]:+.4f} dy={c1[1]-d1[1]:+.4f} "
              f"dyaw={y1-d1[2]:+.4f}", flush=True)
    print(f"\nGEOMETRY {'PASS' if geometry_ok else 'FAIL'}", flush=True)
    print("DIAG DONE", flush=True)
    return 0 if geometry_ok else 1


LIDAR_LASER_NAME = "lidar2d_0_laser"   # located by name: its parent chain
                                       # moved when the reparent fix landed
# diamond-notch vertex nearest the front lidar, in base_link (sliced from the
# committed robot USD at the scan plane by tools/isaac/self_occlusion_check.py)
NOTCH_VERTEX_BASE = (0.3432, -0.0002, 0.2264)


def run_spin_transforms(app, timeline, rig, stage, robot_prim_path, tc,
                        tensor_pose, wz, frames) -> int:
    """Does the LIDAR prim's world transform keep up with the chassis body?

    The desync test recorded in `archive/engineering/port-history.md`. The phantom return
    band appears only while the robot moves, but the chassis and the lidars
    are one rigid body, so geometry alone cannot explain it. If the sensor's
    pose and the geometry it rays against are taken from different instants,
    a transient relative yaw appears -- and the notch tangency amplifies a few
    degrees of it into a 0.4-0.5 m false return.

    `lidar2d_0_laser` and `chassis_link` are both children of `base_link`, so
    their relative yaw is authored as ZERO and must stay zero at every instant
    under any rigid motion. Any nonzero value here is the bug.
    """
    from pxr import UsdGeom

    def usd_yaw_prim(p):
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(tc)
        q = m.ExtractRotationQuat()
        im = q.GetImaginary()
        return (m.ExtractTranslation(),
                yaw_of_wxyz(q.GetReal(), im[0], im[1], im[2]))

    def usd_yaw(rel):
        p = stage.GetPrimAtPath(f"{robot_prim_path}/{rel}")
        if not p:
            raise RuntimeError(f"prim not found: {robot_prim_path}/{rel}")
        return usd_yaw_prim(p)

    from pxr import Usd
    laser = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
        if prim.GetName() == LIDAR_LASER_NAME:
            laser = prim
            break
    if laser is None:
        raise RuntimeError(f"{LIDAR_LASER_NAME} not found under "
                           f"{robot_prim_path}")
    print(f"lidar prim: {laser.GetPath()}", flush=True)

    print(f"\n=== SPIN TRANSFORM DESYNC TEST (wz={wz} rad/s, "
          f"{frames} frames) ===", flush=True)
    print("lidar2d_0_laser and chassis_link are both under base_link, so"
          " their\nrelative yaw is authored 0 and must stay 0. Nonzero =="
          " desync.\n", flush=True)
    print(f"{'frame':>6s} {'chassis_tensor':>15s} {'chassis_usd':>12s} "
          f"{'lidar_usd':>10s} {'lidar-chassis':>14s} {'notch_r_from_lidar':>19s}",
          flush=True)

    worst = 0.0
    worst_frame = -1
    last = timeline.get_current_time()
    for f in range(1, frames + 1):
        t = timeline.get_current_time()
        dt = max(t - last, 0.0)
        last = t
        rig.set_cmd(0.0, 0.0, wz, now=t)
        rig.step(dt if dt > 0 else 1.0 / 60.0, now=t)
        app.update()

        _, ct = tensor_pose("chassis")
        cpos, cy = usd_yaw(CHASSIS)
        lpos, ly = usd_yaw_prim(laser)
        rel = math.degrees(ly - cy)
        rel = (rel + 180.0) % 360.0 - 180.0
        if abs(rel) > abs(worst):
            worst, worst_frame = rel, f
        # where the notch vertex sits relative to the lidar, live
        nx = cpos[0] + NOTCH_VERTEX_BASE[0] * math.cos(cy) \
            - NOTCH_VERTEX_BASE[1] * math.sin(cy)
        ny = cpos[1] + NOTCH_VERTEX_BASE[0] * math.sin(cy) \
            + NOTCH_VERTEX_BASE[1] * math.cos(cy)
        notch_r = math.hypot(nx - lpos[0], ny - lpos[1])
        if f % 20 == 0 or f <= 3:
            ct_s = f"{math.degrees(ct):+.4f}" if ct is not None else "n/a"
            print(f"{f:6d} {ct_s:>15s} {math.degrees(cy):+12.4f} "
                  f"{math.degrees(ly):+10.4f} {rel:+14.6f} "
                  f"{notch_r:19.5f}", flush=True)

    print(f"\nworst |lidar - chassis| yaw = {abs(worst):.6f} deg "
          f"(frame {worst_frame})", flush=True)
    # 0.4 rad/s needs ~3.6-5 deg of relative yaw to fake a 0.40-0.55 m return
    # off the notch edge, which passes 34.6 mm from the emitter.
    if abs(worst) < 0.01:
        print("VERDICT: transforms stay locked — the sensor turns with the "
              "chassis.\n         This is the expected PASS once the lidars "
              "are parented to chassis_link\n         (see archive/engineering/port-history.md). "
              "notch_r above should also hold constant; if it\n         "
              "sweeps, the sensor is detached from the body again.",
              flush=True)
    elif abs(worst) < 3.0:
        print("VERDICT: measurable desync, but too small on its own to "
              "explain 0.40-0.55 m\n         returns (needs ~3.6-5 deg). "
              "Contributing factor at most.", flush=True)
    else:
        print("VERDICT: the USD-stage transform of the lidar prim does NOT "
              "follow the chassis.", flush=True)
        print("         Large enough to explain the band, and notch_r above "
              "sweeps straight\n         through the observed 0.40-0.55 m "
              "range.", flush=True)
        print("         CAVEAT before acting: this proves the USD-STAGE read "
              "is stale, not\n         that the RTX sensor uses that stale "
              "transform — it may take its pose\n         from Fabric/USDRT, "
              "which physics does update. SLAM working at all is\n         "
              "evidence the emitted scan does rotate correctly. Discriminate "
              "first:\n         spin while checking whether DISTANT world "
              "returns stay consistent with\n         the robot's yaw. If "
              "they do, the sensor pose is fine and it is the\n         "
              "chassis geometry that is mis-transformed in the RTX scene — "
              "the inverse\n         desync, same root area.", flush=True)
    return 0


def _step_frames(app, timeline, rig, n, cmd=None):
    """Step n render frames, refreshing cmd every frame like teleop would."""
    last = timeline.get_current_time()
    for _ in range(n):
        t = timeline.get_current_time()
        dt = max(t - last, 0.0)
        last = t
        if cmd is not None:
            rig.set_cmd(*cmd, now=t)
        rig.step(dt if dt > 0 else 1.0 / 60.0, now=t)
        app.update()


def _body_vel(rig):
    vx_w, vy_w, wz = rig._rig_velocities()
    yaw = rig._rig_positions()[2]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx_w + s * vy_w, -s * vx_w + c * vy_w, wz)


def run_battery(app, timeline, rig, contacts) -> int:
    print("\n=== P3 MOTION BATTERY ===", flush=True)
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)

    # 1. stillness at spawn --------------------------------------------------
    t0 = timeline.get_current_time()
    contacts.clear()
    _step_frames(app, timeline, rig, 120)
    d = rig._rig_positions()
    check("still at origin",
          abs(d[0]) < 5e-3 and abs(d[1]) < 5e-3 and abs(d[2]) < 1e-2,
          f"dofs=({d[0]:+.4f}, {d[1]:+.4f}, {d[2]:+.4f})")
    n_events = sum(v[0] for k, v in contacts.items())
    check("no wheel contacts", n_events == 0,
          f"{n_events} events {list(contacts)[:2] if contacts else ''}")
    t1 = timeline.get_current_time()
    check("sim time advances", t1 - t0 > 0.5,
          f"dt={t1 - t0:.3f}s over 120 frames")

    # 2. set_planar_pose teleport -------------------------------------------
    rig.set_planar_pose(1.0, -0.5, 0.5)
    _step_frames(app, timeline, rig, 10)
    d = rig._rig_positions()
    check("teleport (1.0, -0.5, 0.5)",
          abs(d[0] - 1.0) < 2e-2 and abs(d[1] + 0.5) < 2e-2
          and abs(d[2] - 0.5) < 2e-2,
          f"dofs=({d[0]:+.4f}, {d[1]:+.4f}, {d[2]:+.4f})")

    # 3. axis tracking: fwd / strafe / spin ----------------------------------
    for name, cmd, axis, want in [
        ("forward 0.3 m/s", (0.3, 0.0, 0.0), 0, 0.3),
        ("strafe 0.3 m/s", (0.0, 0.3, 0.0), 1, 0.3),
        ("spin 0.8 rad/s", (0.0, 0.0, 0.8), 2, 0.8),
    ]:
        rig.set_planar_pose(0.0, 0.0, 0.0)
        _step_frames(app, timeline, rig, 10)
        _step_frames(app, timeline, rig, 180, cmd=cmd)
        bv = _body_vel(rig)
        leak = max(abs(bv[i]) for i in range(3) if i != axis)
        check(name,
              abs(bv[axis] - want) < 0.1 * want + 0.02 and leak < 0.05,
              f"body vel=({bv[0]:+.3f}, {bv[1]:+.3f}, {bv[2]:+.3f}) "
              f"leak={leak:.3f}")

    # 4. holonomic combined --------------------------------------------------
    rig.set_planar_pose(0.0, 0.0, 0.0)
    _step_frames(app, timeline, rig, 10)
    cmd = (0.2, -0.1, 0.4)
    _step_frames(app, timeline, rig, 180, cmd=cmd)
    bv = _body_vel(rig)
    ok = all(abs(bv[i] - cmd[i]) < 0.15 * abs(cmd[i]) + 0.02 for i in range(3))
    check("combined (0.2, -0.1, 0.4)", ok,
          f"body vel=({bv[0]:+.3f}, {bv[1]:+.3f}, {bv[2]:+.3f})")

    # 5. explicit stop drift -------------------------------------------------
    rig.set_planar_pose(0.0, 0.0, 0.0)
    _step_frames(app, timeline, rig, 10)
    _step_frames(app, timeline, rig, 120, cmd=(0.3, 0.0, 0.0))
    _step_frames(app, timeline, rig, 60, cmd=(0.0, 0.0, 0.0))  # 1 s to settle
    a = rig._rig_positions()
    _step_frames(app, timeline, rig, 120)
    b = rig._rig_positions()
    drift = math.hypot(b[0] - a[0], b[1] - a[1])
    check("stop drift < 2 cm", drift < 0.02,
          f"drift={drift * 100:.2f} cm over 2 s after settle")

    # 6. cmd timeout stop (no zero cmd ever sent) ----------------------------
    rig.set_planar_pose(0.0, 0.0, 0.0)
    _step_frames(app, timeline, rig, 10)
    _step_frames(app, timeline, rig, 120, cmd=(0.3, 0.0, 0.0))
    _step_frames(app, timeline, rig, 180)          # no cmd: timeout at 0.5 s
    bv = _body_vel(rig)
    check("cmd timeout stops robot", abs(bv[0]) < 0.01 and abs(bv[2]) < 0.01,
          f"body vel=({bv[0]:+.3f}, {bv[1]:+.3f}, {bv[2]:+.3f})")

    verdict = all(results)
    print(f"\nBATTERY {'PASS' if verdict else 'FAIL'} "
          f"({sum(results)}/{len(results)})", flush=True)
    print("DIAG DONE", flush=True)
    return 0 if verdict else 1


if __name__ == "__main__":
    main()
