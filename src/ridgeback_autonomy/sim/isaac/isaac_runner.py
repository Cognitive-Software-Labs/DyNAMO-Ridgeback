#!/usr/bin/env python3
"""Isaac Sim 6.0 runner for the Ridgeback exploration stack.

Standalone SimulationApp process replacing the gz server: loads a world
USD (worlds.py resolution), references the committed Ridgeback package,
drives it kinematic-holonomically from cmd_vel (robot_rig.py), and
publishes /clock, odometry+TF, and ground-truth pose (ros_io.py). GPU
sensors attach in sensors.py (P4).

Run under isaac_venv with the workspace sourced (bridge then uses system
rclpy/CycloneDDS):

    source install/setup.bash
    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
        src/ridgeback_autonomy/sim/isaac/isaac_runner.py \
        --world mock_hospital --headless true

Timing: physics at --physics-hz (default 120). --rtf 1.0 keeps sim time
at wall speed (interactive default); --rtf 0 runs unthrottled for
benchmarks — legal because the whole stack runs on sim time.
"""
import argparse
import math
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="mock_hospital")
    ap.add_argument("--namespace", default="r100_0001")
    ap.add_argument("--headless", default="true",
                    choices=["true", "false"])
    ap.add_argument("--livestream", default="false", choices=["true", "false"],
                    help="WebRTC livestream (implies headless render window)")
    ap.add_argument("--physics-hz", type=float, default=120.0)
    ap.add_argument("--sim-mode", default="realtime",
                    choices=["realtime", "deterministic"],
                    help="realtime: wall-throttled (--rtf), render-coupled "
                         "timing, realistic noise — deployment fidelity + "
                         "real-time-deadline testing. deterministic: fixed "
                         "sim-dt per frame (omni.kit.loop manual mode), "
                         "contention-immune byte-identical sensor data — "
                         "reproducible A/B benchmarks (rtf:=0 to run flat out, "
                         "rtf:=1 headless:=false to watch).")
    ap.add_argument("--sensor-hz", type=float, default=40.0,
                    help="lidar sweep rate; in deterministic mode this is the "
                         "fixed sim-dt (1/sensor_hz) advanced per frame, so "
                         "scan skew = wz/sensor_hz (matches the real 40Hz "
                         "un-deskewed sweep). Keep physics_hz an integer "
                         "multiple.")
    ap.add_argument("--rtf", type=float, default=1.0,
                    help="real-time-factor throttle; 0 = unthrottled. Honored "
                         "in both modes (rtf:=0 for fast deterministic A/B, "
                         "rtf:=1 to watch a run at real-time).")
    ap.add_argument("--odom-noise", type=float, default=None,
                    help="odometry drift scale; 0 = perfect odom. Default by "
                         "mode: 0 (deterministic), 1.0 (realtime).")
    ap.add_argument("--camera", default="true", choices=["true", "false"],
                    help="attach D455 camera render + publishers; false = "
                         "lidar-only (saves GPU/RTF for SLAM/nav benchmarks)")
    ap.add_argument("--robot-usd", default=None,
                    help="override the committed robot package entry USD")
    ap.add_argument("--odom-tf", default="false", choices=["true", "false"],
                    help="publish odom->base_link TF from the runner. Off by "
                         "default: the EKF include owns that TF, like the "
                         "real platform. Enable for standalone runs")
    ap.add_argument("--animate-g1", default="false", choices=["true", "false"],
                    help="give world-authored G1 figures a gentle kinematic "
                         "idle (sway + drift). Demo sugar only — keep off "
                         "for benchmarks; the calibration targets must not "
                         "move (real pedestrian animation is a planned "
                         "post-port addon)")
    ap.add_argument("--spawn", default="0,0,0",
                    help="robot spawn x,y,yaw in the world frame")
    ap.add_argument("--spawn-z", type=float, default=0.076,
                    help="base_link height. Wheels carry no colliders (the "
                         "z-less rig would fight any floor contact), so this "
                         "is visual + sensor-height truth: wheel bottoms sit "
                         "at spawn_z + axle 0.05 - radius 0.0759; default "
                         "puts them on mock_hospital's floor top (z=0.05) "
                         "with 1 mm slack")
    return ap.parse_args()


def main():
    args = parse_args()
    if args.odom_noise is None:
        args.odom_noise = 0.0 if args.sim_mode == "deterministic" else 1.0
    if args.sim_mode == "deterministic" and args.physics_hz % args.sensor_hz != 0:
        print(f"WARN: --physics-hz {args.physics_hz} is not an integer multiple "
              f"of --sensor-hz {args.sensor_hz}; physics substeps per frame "
              f"become non-integer and determinism is weakened.", flush=True)
    headless = args.headless == "true"

    from isaacsim import SimulationApp
    app_cfg = {"headless": headless}
    if not headless:
        # Windowed on a software-X display (e.g. VNC): a vsync-locked
        # present loop can stall this manually-driven update loop (the
        # full isaacsim app runs Kit's own main loop and is immune).
        # Decouple presentation from our stepping.
        app_cfg["extra_args"] = [
            "--/app/vsync=false",
            "--/app/runLoops/present/rateLimitEnabled=false",
        ]
    app = SimulationApp(app_cfg)

    exit_code = 1
    try:
        exit_code = run(app, args)
    except Exception:
        import traceback
        traceback.print_exc()
        print("RUNNER FAILED", flush=True)
    app.close()
    sys.exit(exit_code)


def run(app, args) -> int:
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    from robot_rig import RidgebackRig
    from ros_io import RosIO
    from worlds import get_assets_root, resolve_world

    # Stock envs (warehouse/office/hospital) resolve to <assets_root>/Isaac/...
    # and stream from S3/Nucleus; repo-local worlds ignore it. Only reachable
    # now that we are inside the SimulationApp process.
    assets_root = get_assets_root()
    world_path = resolve_world(args.world, assets_root)
    print(f"loading world: {world_path}", flush=True)
    ctx = omni.usd.get_context()
    ctx.open_stage(world_path)
    stage = ctx.get_stage()

    # ORDER MATTERS (6.0.1): the ros2 bridge crashes in omni.graph.core
    # if a stage is opened after the extension is enabled — enable it only
    # once the world stage is in place.
    enable_extension("isaacsim.ros2.bridge")
    if args.livestream == "true":
        enable_extension("omni.kit.livestream.webrtc")

    # --- robot ------------------------------------------------------------
    robot_usd = args.robot_usd or str(
        Path(__file__).resolve().parent / "usd/robots/ridgeback_r100/ridgeback_r100.usda")
    x0, y0, yaw0 = (float(v) for v in args.spawn.split(","))

    robot_prim_path = "/ridgeback"
    robot_prim = stage.DefinePrim(robot_prim_path, "Xform")
    # When the world's root layer is remote (a stock env streaming from S3),
    # a bare local path in AddReference gets URL-joined against that remote
    # anchor and 404s (the /ridgeback subtree never composes). Force an
    # absolute file:// URI so the asset resolver keeps our robot local
    # regardless of where the world layer lives. (P7)
    robot_ref = robot_usd if "://" in robot_usd else Path(robot_usd).as_uri()
    robot_prim.GetReferences().AddReference(robot_ref)
    # place the base at its ride height (wheels are collider-free visuals) —
    # AND anchor the rig's world fixed-joint at the same height: its
    # unauthored localPos0 defaults to the world origin, which would yank
    # the chain back to z=0 and grind the wheels into the floor.
    UsdGeom.XformCommonAPI(robot_prim).SetTranslate(
        Gf.Vec3d(0.0, 0.0, args.spawn_z))
    wf = stage.GetPrimAtPath(f"{robot_prim_path}/drive_rig/world_fix")
    if not wf:
        raise RuntimeError("drive_rig/world_fix joint missing from robot USD")
    UsdPhysics.FixedJoint(wf).CreateLocalPos0Attr(
        Gf.Vec3f(0.0, 0.0, args.spawn_z))
    # spawn pose goes into the rig's joint offsets after initialize();
    # the reference itself stays at the world origin so px/py/rz remain
    # the single source of planar pose truth.

    # the articulation root lives inside the referenced package (name is
    # sanitized from the URDF robot name) — find it instead of hardcoding
    from pxr import UsdPhysics
    roots = [p.GetPath().pathString for p in Usd.PrimRange(robot_prim)
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if not roots:
        raise RuntimeError(f"no ArticulationRootAPI under {robot_prim_path}")
    # the rig's world-anchored joint is the intended (fixed-base) root
    rig_roots = [r for r in roots if "/drive_rig/" in r]
    art_root_path = rig_roots[0] if rig_roots else roots[0]
    if len(roots) > 1:
        print(f"WARNING: multiple articulation roots {roots}, using "
              f"{art_root_path}", flush=True)
    print(f"articulation root: {art_root_path}", flush=True)

    # --- physics / timing ---------------------------------------------------
    # No SimulationContext: the deprecated isaacsim.core.api variant
    # segfaults in _init_stage->render on 6.0.1 headless. Timeline +
    # app.update() (smoke-test-proven) drive stepping; physics rate is set
    # on the world's PhysicsScene prim.
    from pxr import PhysxSchema
    scene_prim = None
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            scene_prim = prim
            break
    if scene_prim is None:
        # Stock Isaac envs may ship without an authored PhysicsScene; the rig
        # needs one to carry TimeStepsPerSecond. Author a default. (P7)
        scene_prim = UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()
        print("world has no PhysicsScene — created /physicsScene", flush=True)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene.CreateTimeStepsPerSecondAttr(float(args.physics_hz))

    from sensors import attach_camera, attach_lidars
    attach_lidars(stage, robot_prim_path, args.namespace)
    if args.camera == "true":
        attach_camera(stage, robot_prim_path, args.namespace)
    else:
        print("camera disabled (--camera false): lidar-only run", flush=True)

    ros = RosIO(args.namespace, odom_tf=args.odom_tf == "true")
    rig = RidgebackRig(art_root_path, odom_noise=args.odom_noise)

    timeline = omni.timeline.get_timeline_interface()
    sim_dt = 1.0 / args.sensor_hz
    if args.sim_mode == "deterministic":
        # Fixed-step: make app.update() advance sim-time by exactly sim_dt
        # regardless of render/wall duration. The omni.kit.loop manual runner
        # lives BELOW the isaacsim.core.api SimulationContext that segfaults
        # headless in 6.0.1, so we drive it directly. This decouples every
        # sim-time-stamped payload from box load: a scan's inherent skew
        # becomes wz*sim_dt (the real 40Hz sweep) instead of wz*render_dt.
        # Verified byte-fixed dt by tools/isaac/smoke_test.py. name='main'
        # fixes ONLY the sim loop — 'rendering_0' and the GUI 'present' loop
        # stay wall-tracked (verified: get_manual_mode False), so a throttled
        # deterministic run (rtf:=1 headless:=false) is watchable windowed.
        # Kept deterministic-only so realtime stays render-coupled by design.
        from omni.kit.loop import _loop as omni_loop
        _loop = omni_loop.acquire_loop_interface()
        _loop.set_manual_step_size(sim_dt)
        _loop.set_manual_mode(True, name="main")
        timeline.set_time_codes_per_second(float(args.sensor_hz))
        _thr = "unthrottled" if args.rtf == 0 else f"throttled rtf={args.rtf}"
        print(f"deterministic mode: fixed sim-dt {sim_dt * 1e3:.2f} ms "
              f"({args.sensor_hz} Hz), {_thr}", flush=True)
    # Converted worlds author no timeCodes, so Kit's play range is
    # zero-length and looping pins get_current_time() at ~0 forever —
    # /clock never advances and the cmd_vel timeout can never fire.
    # Force an effectively infinite, non-looping range.
    timeline.set_end_time(1.0e9)
    timeline.set_looping(False)
    timeline.play()
    # the experimental Articulation attaches to the tensor backend on a
    # physics-ready event — pump frames until it reports initialized
    for _ in range(240):
        app.update()
        if rig.ready():
            break
    else:
        raise RuntimeError("articulation tensor backend never initialized")
    rig.initialize()
    # Always reset: physics settling during backend attach can translate
    # the chain (constraint snap); this puts the rig dofs exactly at the
    # requested spawn pose with zero velocity.
    rig.set_planar_pose(x0, y0, yaw0)

    # optional demo idle for world-authored G1 figures (children of the
    # world's default prim whose name mentions g1 — the robot lives at
    # /ridgeback, outside that subtree)
    g1_anim = []
    if args.animate_g1 == "true":
        for prim in stage.GetDefaultPrim().GetChildren():
            if "g1" not in prim.GetName().lower():
                continue
            tr = prim.GetAttribute("xformOp:translate")
            orq = prim.GetAttribute("xformOp:orient")
            if not (tr and orq and tr.HasAuthoredValue()):
                continue
            t0 = tr.Get()
            q0 = orq.Get()
            yaw0 = 2.0 * math.atan2(q0.GetImaginary()[2], q0.GetReal())
            g1_anim.append((tr, orq, t0, yaw0))
        print(f"animating {len(g1_anim)} g1 figure(s)", flush=True)

    print("RUNNER READY", flush=True)

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    frames = 0
    wall_start = time.monotonic()
    last_sim_time = timeline.get_current_time()
    last_body_twist = (0.0, 0.0, 0.0)
    import random as _random
    imu_rng = _random.Random(1)
    imu_sigma_gyro = 0.005 * args.odom_noise      # rad/s
    imu_sigma_accel = 0.05 * args.odom_noise      # m/s^2
    while app.is_running() and not stop["flag"]:
        sim_time = timeline.get_current_time()
        if args.sim_mode == "deterministic":
            # manual mode advanced the timeline by exactly sim_dt last frame
            frame_dt = sim_dt
        else:
            # realtime: bound a stalled frame's dt so a long render can't
            # inject a huge rig velocity step / IMU dt. (Does NOT bound the
            # lidar skew — the RTX render already baked it; realtime skew is
            # only well-behaved while the box sustains rtf~1.)
            measured = max(sim_time - last_sim_time, 0.0)
            frame_dt = min(measured, 2.0 * sim_dt) if measured > 0.0 else sim_dt
        last_sim_time = sim_time

        if ros.take_reset():
            # in-session benchmark reset (P7): return the robot to spawn and
            # re-zero odom so probe --repeat N can start a fresh run without
            # relaunching the sim. GT/odom re-seed inside set_planar_pose.
            rig.set_planar_pose(x0, y0, yaw0)
            last_body_twist = (0.0, 0.0, 0.0)
            print(f"sim reset: robot -> spawn ({x0},{y0},{yaw0})", flush=True)

        cmd = ros.take_cmd()
        if cmd is not None:
            rig.set_cmd(*cmd, now=sim_time)
        rig.step(frame_dt, now=sim_time)

        # kinematic idle: slow figure-of-motion drift + heading sway
        for tr, orq, t0, yaw0 in g1_anim:
            from pxr import Gf
            dx = 0.35 * math.sin(0.35 * sim_time)
            dy = 0.20 * math.sin(0.22 * sim_time + 0.7)
            yaw = yaw0 + 0.45 * math.sin(0.35 * sim_time + 1.2)
            tr.Set(Gf.Vec3d(t0[0] + dx, t0[1] + dy, t0[2]))
            orq.Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))

        app.update()          # one render frame + its physics substeps
        frames += 1

        sim_time = timeline.get_current_time()
        ros.publish_clock(sim_time)
        odom_state, body_twist = rig.update_odom()
        ros.publish_odom(sim_time, odom_state, body_twist)
        dt = frame_dt
        ros.publish_imu(
            sim_time,
            body_twist[2] + imu_rng.gauss(0.0, imu_sigma_gyro),
            (body_twist[0] - last_body_twist[0]) / dt
            + imu_rng.gauss(0.0, imu_sigma_accel),
            (body_twist[1] - last_body_twist[1]) / dt
            + imu_rng.gauss(0.0, imu_sigma_accel),
        )
        last_body_twist = body_twist
        ros.publish_ground_truth(sim_time, *rig.ground_truth())
        ros.spin_once()

        if args.rtf > 0:
            target_wall = wall_start + sim_time / args.rtf
            lag = target_wall - time.monotonic()
            if lag > 0:
                time.sleep(lag)

    sim_elapsed = timeline.get_current_time()
    achieved = sim_elapsed / max(time.monotonic() - wall_start, 1e-9)
    print(f"RUNNER EXIT after {frames} frames, sim {sim_elapsed:.1f}s, "
          f"achieved RTF {achieved:.2f}", flush=True)
    ros.shutdown()
    return 0


if __name__ == "__main__":
    main()
