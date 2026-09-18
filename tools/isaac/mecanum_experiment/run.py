#!/usr/bin/env python3
"""Isolated wheel-contact experiment. Run with Isaac Python, sourced workspace.

--output must be a fresh directory under artifacts/. Each invocation is one
independent boot; summaries distinguish test failures from harness failures.
"""

from __future__ import annotations
import argparse, hashlib, json, math, os, subprocess, sys, time, traceback
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
PRODUCTION = (
    ROOT
    / "src/ridgeback_autonomy_isaac/sim/isaac/usd/robots/ridgeback_r100/ridgeback_r100.usda"
)
sys.path.insert(0, str(ROOT / "tools/isaac"))
from model import Description, WHEELS, build


def cases():
    result = [dict(name="settle", command=[0, 0, 0], duration=15, kind="settle")]
    for axis, label in [(0, "forward"), (1, "strafe")]:
        for speed in [0.1, 0.2]:
            for sign in [-1, 1]:
                cmd = [0, 0, 0]
                cmd[axis] = sign * speed
                result.append(
                    dict(
                        name=f"{label}_{sign*speed:+.1f}",
                        command=cmd,
                        duration=5,
                        kind="linear",
                    )
                )
    for sign in [-1, 1]:
        result.append(
            dict(
                name=f"rotate_{sign:+}",
                command=[0, 0, sign * 0.3],
                duration=5,
                kind="rotate",
            )
        )
    result.extend(
        [
            dict(name="diagonal", command=[0.1, 0.1, 0], duration=5, kind="diagnostic"),
            dict(name="timeout", command=[0.2, 0, 0], duration=5, kind="timeout"),
        ]
    )
    for label, yaw, cmd in [
        ("front", 0, [0.1, 0, 0]),
        ("lateral", math.pi / 2, [0, -0.1, 0]),
        ("angled", math.pi / 4, [0.1 / math.sqrt(2), -0.1 / math.sqrt(2), 0]),
    ]:
        result.append(
            dict(name="wall_" + label, command=cmd, duration=6, kind="wall", yaw=yaw)
        )
    result.append(
        dict(
            name="low_obstacle",
            command=[0.1, 0, 0],
            duration=7,
            kind="diagnostic",
            obstacle=True,
        )
    )
    return result


def evaluate(case, rows, contacts):
    """Numerical gates operate only on recorded observations, never command pose."""
    a = np.array([r["position"] for r in rows])
    v = np.array([r["body_velocity"] for r in rows])
    yaw = np.unwrap([r["yaw"] for r in rows])
    t = np.array([r["time"] for r in rows])
    kind = case["kind"]
    checks = {
        "finite": bool(
            np.isfinite(a).all() and np.isfinite(v).all() and np.isfinite(yaw).all()
        ),
        "sensor_attachment": max(r["sensor_translation_error"] for r in rows) <= 0.001
        and max(r["sensor_rotation_error"] for r in rows) <= math.radians(0.1),
    }
    metrics = {}
    if kind == "settle":
        tail = t >= 5
        delta = a[tail] - a[tail][0]
        metrics.update(
            hold_drift=float(np.max(np.linalg.norm(delta[:, :2], axis=1))),
            hold_yaw=float(np.ptp(yaw[tail])),
            penetration=float(
                max(r["floor_penetration"] for r in rows if r["time"] >= 5)
            ),
        )
        checks.update(
            settled_by_5s=bool(
                max(
                    np.linalg.norm(r["linear_velocity"])
                    for r in rows
                    if 4 <= r["time"] <= 5
                )
                < 0.01
            ),
            hold_translation=metrics["hold_drift"] <= 0.005,
            hold_yaw=metrics["hold_yaw"] <= math.radians(0.5),
            penetration=metrics["penetration"] <= 0.002,
        )
    elif kind == "linear":
        target = np.array(case["command"][:2])
        axis = int(np.argmax(np.abs(target)))
        steady = v[t >= 2]
        metrics.update(
            speed_error=float(
                abs(np.mean(steady[:, axis]) - target[axis]) / abs(target[axis])
            ),
            cross_speed=float(np.max(np.abs(steady[:, 1 - axis]))),
        )
        checks.update(
            speed=metrics["speed_error"] <= 0.1,
            cross_speed=metrics["cross_speed"] <= 0.02,
        )
    elif kind == "rotate":
        metrics.update(
            yaw_speed_error=float(
                abs(np.mean(v[t >= 2, 2]) - case["command"][2])
                / abs(case["command"][2])
            ),
            translation=float(np.max(np.linalg.norm(a[:, :2] - a[0, :2], axis=1))),
        )
        checks.update(
            yaw_speed=metrics["yaw_speed_error"] <= 0.1,
            centre_drift=metrics["translation"] <= 0.02,
        )
    elif kind == "timeout":
        metrics["max_speed_after_stop"] = float(
            np.max(np.linalg.norm(v[t >= 3.5, :2], axis=1))
        )
        checks.update(
            target_timeout=all(
                np.max(np.abs(r["targets"])) < 1e-8 for r in rows if r["time"] >= 2.5
            ),
            stop=metrics["max_speed_after_stop"] < 0.01,
        )
    elif kind == "wall":
        hold = [r for r in rows if 4 <= r["time"] <= 6]
        recovery = [r for r in rows if r["time"] >= 7.5]
        metrics.update(
            contact_error=float(abs(np.median([r["wall_gap"] for r in hold]))),
            penetration=float(max(0, -min(r["wall_gap"] for r in rows))),
            recovery_clearance=float(min(r["wall_gap"] for r in recovery)),
        )
        checks.update(
            contact=bool(contacts),
            stop_location=metrics["contact_error"] <= 0.01,
            penetration=metrics["penetration"] <= 0.005,
            recovery=metrics["recovery_clearance"] >= 0.08,
        )
    return {
        "checks": checks,
        "metrics": metrics,
        "passed": all(checks.values()),
        "required": kind != "diagnostic",
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--model", choices=["physical", "baseline"], default="physical")
    p.add_argument("--physics-hz", type=int, choices=[120, 240], default=120)
    p.add_argument(
        "--only", choices=[c["name"] for c in cases()], help="one case for debugging"
    )
    args = p.parse_args()
    out = args.output.resolve()
    out.relative_to(ROOT / "artifacts")
    out.mkdir(parents=True, exist_ok=False)
    (out / "source.patch").write_bytes(
        subprocess.check_output(["git", "diff", "HEAD"], cwd=ROOT)
    )
    # Untracked implementation inputs are preserved as well as the tracked diff.
    import shutil

    shutil.copytree(
        Path(__file__).parent,
        out / "implementation",
        ignore=shutil.ignore_patterns("__pycache__"),
    )
    (out / "revision.txt").write_text(
        subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True)
    )
    (out / "git-status.txt").write_text(
        subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True)
    )
    setup = out / "setup"
    setup.mkdir()
    shutil.copy2(ROOT / "clearpath/robot.yaml", setup / "robot.yaml")
    subprocess.run(
        [
            "ros2",
            "run",
            "clearpath_generator_common",
            "generate_description",
            "-s",
            str(setup) + "/",
        ],
        check=True,
    )
    urdf = out / "robot.urdf"
    with urdf.open("w") as f:
        subprocess.run(
            ["xacro", str(setup / "robot.urdf.xacro"), "is_sim:=true"],
            stdout=f,
            check=True,
        )
    (out / "run.json").write_text(
        json.dumps(
            {
                "model": args.model,
                "physics_hz": args.physics_hz,
                "cases": cases(),
                "gates": {
                    "sensor_translation_m": 0.001,
                    "sensor_rotation_rad": math.radians(0.1),
                },
                "environment": {
                    "isaac_python": str(Path(sys.executable).resolve()),
                    "isaac_version": __import__(
                        "importlib.metadata", fromlist=["version"]
                    ).version("isaacsim"),
                },
            },
            indent=2,
        )
    )
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "disable_viewport_updates": True})
    code = 1
    try:
        result = run(app, args, out, urdf)
        (out / "summary.json").write_text(json.dumps(result, indent=2))
        code = 0 if result["passed"] else 2
    except Exception:
        (out / "error.txt").write_text(traceback.format_exc())
        traceback.print_exc()
        (out / "summary.json").write_text(
            json.dumps({"status": "HARNESS_ERROR", "passed": False})
        )
    print("EXPERIMENT_EXIT", code, flush=True)
    # app.close may terminate directly; all evidence is already flushed.
    app.close(exit_code=code)
    return code


def run(app, args, out, urdf):
    import omni.usd, omni.timeline, omni.physx
    import warp as wp
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, PhysicsSchemaTools
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.robot_motion.controllers")
    import isaacsim.robot_motion.experimental.motion_generation as mg
    from isaacsim.robot_motion.controllers import HolonomicController
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from scipy.spatial.transform import Rotation

    candidate = out / "candidate.usda"
    metadata = build(urdf, candidate, wall=True, obstacle=True)
    (out / "geometry.json").write_text(json.dumps(metadata, indent=2))
    hashpaths = [
        urdf,
        candidate,
        ROOT / "clearpath/robot.yaml",
        *map(Path, metadata["source_meshes"]),
    ]
    if args.model == "baseline":
        hashpaths.extend(PRODUCTION.parent.rglob("*"))
    (out / "hashes.json").write_text(
        json.dumps(
            {
                str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in hashpaths
                if p.is_file()
            },
            indent=2,
        )
    )
    ctx = omni.usd.get_context()
    ctx.open_stage(str(candidate))
    for _ in range(5):
        app.update()
    stage = ctx.get_stage()
    robotpath = "/World/robot"
    chassispath = robotpath + "/chassis"
    if args.model == "baseline":
        stage.RemovePrim(robotpath)
        robot = stage.DefinePrim(robotpath, "Xform")
        robot.GetReferences().AddReference(str(PRODUCTION))
        z = 0.02617
        UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0, 0, z))
        UsdPhysics.FixedJoint(
            stage.GetPrimAtPath(robotpath + "/drive_rig/world_fix")
        ).CreateLocalPos0Attr(Gf.Vec3f(0, 0, z))
        chassispath = robotpath + "/Geometry/base_link/chassis_link"
    PhysxSchema.PhysxSceneAPI(
        stage.GetPrimAtPath("/World/physics")
    ).CreateTimeStepsPerSecondAttr(float(args.physics_hz))
    # Contact callback covers wheel/floor as well as obstacle contacts.
    if args.model == "baseline":
        PhysxSchema.PhysxContactReportAPI.Apply(
            stage.GetPrimAtPath(chassispath)
        ).CreateThresholdAttr(0.0)
    contact_log = []
    current = [None, 0.0]

    def on_contact(headers, data):
        for h in headers:
            paths = [
                str(PhysicsSchemaTools.intToSdfPath(getattr(h, k)))
                for k in ["actor0", "actor1", "collider0", "collider1"]
            ]
            if not any("/World/robot" in x for x in paths):
                continue
            contact_log.append(
                {
                    "case": current[0],
                    "time": current[1],
                    "paths": paths,
                    "contacts": [
                        {
                            "position": list(c.position),
                            "normal": list(c.normal),
                            "impulse": list(c.impulse),
                            "separation": float(c.separation),
                        }
                        for c in data[
                            h.contact_data_offset : h.contact_data_offset
                            + h.num_contact_data
                        ]
                    ],
                }
            )

    subscription = (
        omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
            on_contact
        )
    )
    timeline = omni.timeline.get_timeline_interface()
    from omni.kit.loop import _loop

    loop = _loop.acquire_loop_interface()
    dt = 1 / args.physics_hz
    loop.set_manual_step_size(dt)
    loop.set_manual_mode(True, name="main")
    timeline.set_time_codes_per_second(float(args.physics_hz))
    timeline.set_end_time(1e9)
    stage.Flatten().Export(str(out / "effective-stage.usda"))
    roots = [
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    art = Articulation(roots[0])
    chassis = RigidPrim(chassispath)
    timeline.play()
    for _ in range(240):
        app.update()
        if art.is_physics_tensor_entity_initialized():
            break
    if not art.is_physics_tensor_entity_initialized():
        raise RuntimeError("articulation backend unavailable")
    names = list(art.dof_names)
    wheel_names = [w + "_wheel_joint" for w in WHEELS]
    indices = [names.index(w) for w in wheel_names]
    controller = HolonomicController(
        robot_joint_space=names,
        wheel_joint_names=wheel_names,
        wheel_radius=[metadata["radius"]] * 4,
        wheel_positions=metadata["wheel_positions"],
        wheel_orientations=[[1, 0, 0, 0]] * 4,
        mecanum_angles=[45, 135, 135, 45],
        wheel_axis=[0, 1, 0],
        rotation_direction=[0, 0, 1],
        device="cpu",
    )
    target_cache = {}

    def wheel_targets(cmd):
        key = tuple(cmd)
        if key in target_cache:
            return target_cache[key]
        state = mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=["control_point"],
                linear_velocities=(
                    ["control_point"],
                    wp.array([[cmd[0], cmd[1], 0]], dtype=wp.float32, device="cpu"),
                ),
                angular_velocities=(
                    ["control_point"],
                    wp.array([[0, 0, cmd[2]]], dtype=wp.float32, device="cpu"),
                ),
            )
        )
        result = controller.forward(mg.RobotState(), state, 0)
        mapped = dict(
            zip(result.joints.velocity_names, result.joints.velocities.numpy())
        )
        target_cache[key] = np.array([mapped[n] for n in wheel_names])
        return target_cache[key]

    jac = np.column_stack([wheel_targets(v) for v in np.eye(3)])
    inverse = np.linalg.pinv(jac)
    (out / "controller.json").write_text(
        json.dumps(
            {
                "wheel_names": wheel_names,
                "twist_to_wheel": jac.tolist(),
                "all_dofs": names,
            },
            indent=2,
        )
    )
    points = np.array(metadata["collision_points"])
    if args.model == "baseline":
        sys.path.insert(0, str(ROOT / "src/ridgeback_autonomy_isaac/sim/isaac"))
        from robot_rig import RidgebackRig

        rig = RidgebackRig(roots[0], odom_noise=0.0)
        rig.initialize()
        from validate_chassis_contacts import _points_for_prim

        pts = []
        for prim in Usd.PrimRange(
            stage.GetPrimAtPath(robotpath), Usd.TraverseInstanceProxies()
        ):
            if (
                prim.HasAPI(UsdPhysics.CollisionAPI)
                and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                != False
            ):
                pts.extend(_points_for_prim(prim, UsdGeom, Gf, Usd.TimeCode.Default()))
        points = np.column_stack([np.array(pts), np.zeros(len(pts))])
    sensor_names = ["camera_0_link", "lidar2d_0_link", "lidar2d_1_link"]
    sensor_prims = []
    for name in sensor_names:
        matches = [
            p
            for p in stage.Traverse()
            if p.GetName() == name and str(p.GetPath()).startswith(robotpath + "/")
        ]
        if len(matches) != 1:
            raise RuntimeError(f"expected one sensor frame {name}, got {len(matches)}")
        sensor_prims.append(matches[0])

    def relative_sensor():
        inv = (
            UsdGeom.Xformable(stage.GetPrimAtPath(chassispath))
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            .GetInverse()
        )
        return np.array(
            [
                np.array(
                    UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    )
                    * inv
                )
                for p in sensor_prims
            ]
        )

    reference_sensor = relative_sensor()
    results = {}
    stream = (out / "samples.jsonl").open("w")
    contacts_stream = (out / "contacts.jsonl").open("w")
    wallclock = time.monotonic()
    simtotal = 0
    for case in cases():
        if args.only and args.only != case["name"]:
            continue
        yaw = case.get("yaw", 0.0)
        rz = Rotation.from_euler("z", yaw)
        support = float(np.max((points @ rz.as_matrix().T)[:, 0]))
        x = 1 - support - 0.2 if case["kind"] == "wall" else 0.0
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath("/World/wall")
        ).GetCollisionEnabledAttr().Set(case["kind"] == "wall")
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath("/World/obstacle")
        ).GetCollisionEnabledAttr().Set(case.get("obstacle", False))
        if args.model == "physical":
            q = rz.as_quat()
            art.set_world_poses(
                positions=np.array([[x, 0, metadata["spawn_z"]]]),
                orientations=np.array([[q[3], *q[:3]]]),
            )
            art.set_velocities(
                linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3))
            )
            art.set_dof_positions(np.zeros((1, len(names))))
            art.set_dof_velocities(np.zeros((1, len(names))))
            art.set_dof_velocity_targets(
                np.zeros((1, len(indices))), dof_indices=indices
            )
        else:
            rig.set_planar_pose(x, 0, yaw)
        current[0] = None
        if case["kind"] != "settle":
            for _ in range(5 * args.physics_hz):
                app.update()
                simtotal += dt
        rows = []
        contact_log.clear()
        odom = np.zeros(3)
        current[0] = case["name"]
        duration = case["duration"] + (2 if case["kind"] == "wall" else 0)
        for step in range(round(duration * args.physics_hz) + 1):
            t = step * dt
            current[1] = t
            cmd = np.array(case["command"], dtype=float)
            if case["kind"] == "timeout" and t >= 2.5 - 1e-9:
                cmd[:] = 0
            if case["kind"] == "wall" and t > 6:
                cmd = -cmd
            targets = wheel_targets(cmd)
            if args.model == "physical":
                art.set_dof_velocity_targets(
                    targets.reshape(1, -1), dof_indices=indices
                )
            else:
                if case["kind"] != "timeout" or t < 2:
                    rig.set_cmd(*cmd, float(timeline.get_current_time()))
                rig.step(dt, float(timeline.get_current_time()))
            app.update()
            simtotal += dt
            pos, quat = chassis.get_world_poses()
            pos = pos.numpy()[0]
            q = quat.numpy()[0]
            rotation = Rotation.from_quat([*q[1:], q[0]])
            linear, angular = chassis.get_velocities()
            bv = rotation.inv().apply(linear.numpy()[0])
            av = rotation.inv().apply(angular.numpy()[0])
            angles = rotation.as_euler("xyz")
            wvel = art.get_dof_velocities().numpy()[0, indices]
            twist = inverse @ wvel
            odom[:2] += (
                np.array(
                    [
                        [np.cos(odom[2]), -np.sin(odom[2])],
                        [np.sin(odom[2]), np.cos(odom[2])],
                    ]
                )
                @ twist[:2]
                * dt
            )
            odom[2] += twist[2] * dt
            # Chassis support is exact for its rigid colliders. Wheel lowest
            # point uses the declared bounding radius, a conservative bound.
            worldpoints = points @ rotation.as_matrix().T + pos
            row = {
                "case": case["name"],
                "time": t,
                "position": pos.tolist(),
                "quaternion": q.tolist(),
                "yaw": float(angles[2]),
                "roll": float(angles[0]),
                "pitch": float(angles[1]),
                "body_velocity": [float(bv[0]), float(bv[1]), float(av[2])],
                "targets": targets.tolist(),
                "wheel_velocities": wvel.tolist(),
                "wheel_odometry": odom.tolist(),
                "wall_gap": float(1 - np.max(worldpoints[:, 0])),
                "lowest_z": float(np.min(worldpoints[:, 2])),
                "sensor_error": float(
                    np.max(np.abs(relative_sensor() - reference_sensor))
                ),
            }
            sensor = relative_sensor()
            row["sensor_translation_error"] = float(
                np.max(
                    np.linalg.norm(
                        sensor[:, 3, :3] - reference_sensor[:, 3, :3], axis=1
                    )
                )
            )
            row["sensor_rotation_error"] = float(
                max(
                    Rotation.from_matrix(x[:3, :3].T @ y[:3, :3]).magnitude()
                    for x, y in zip(sensor, reference_sensor)
                )
            )
            row["linear_velocity"] = linear.numpy()[0].tolist()
            row["floor_penetration"] = max(
                [0.0]
                + [
                    -d["separation"]
                    for c in contact_log[-64:]
                    if abs(c["time"] - t) < dt / 2 and "/World/floor" in c["paths"]
                    for d in c["contacts"]
                ]
            )
            if step % max(1, args.physics_hz // 30) == 0:
                row["joint_positions"] = art.get_dof_positions().numpy()[0].tolist()
                stream.write(json.dumps(row) + "\n")
                rows.append(row)
            if not np.isfinite(pos).all() or np.linalg.norm(pos) > 20:
                raise RuntimeError(f'candidate unstable: {case["name"]}, pose {pos}')
        relevant = [
            c for c in contact_log if any("/World/wall" in p for p in c["paths"])
        ]
        result = evaluate(case, rows, relevant)
        result["metrics"].update(
            sensor_translation_max=max(r["sensor_translation_error"] for r in rows),
            sensor_rotation_max=max(r["sensor_rotation_error"] for r in rows),
            wheel_tracking_rms=float(
                np.sqrt(
                    np.mean(
                        [
                            (np.array(r["targets"]) - r["wheel_velocities"]) ** 2
                            for r in rows
                        ]
                    )
                )
            ),
            max_roll=max(abs(r["roll"]) for r in rows),
            max_pitch=max(abs(r["pitch"]) for r in rows),
            obstacle_contact_events=sum(
                any("/World/obstacle" in p for p in c["paths"]) for c in contact_log
            ),
        )
        if args.model == "baseline" and case["kind"] == "settle":
            result.update(
                passed=False,
                note="Fixed-height baseline cannot qualify gravity settling.",
            )
        results[case["name"]] = result
        for c in contact_log:
            contacts_stream.write(json.dumps(c) + "\n")
        stream.flush()
        contacts_stream.flush()
        (out / "partial-results.json").write_text(json.dumps(results, indent=2))
        print(case["name"], result, flush=True)
    stream.close()
    contacts_stream.close()
    timeline.stop()
    elapsed = time.monotonic() - wallclock
    return {
        "status": (
            "PASS"
            if all(r["passed"] for r in results.values() if r["required"])
            else "FAIL"
        ),
        "passed": all(r["passed"] for r in results.values() if r["required"]),
        "cases": results,
        "sim_seconds": simtotal,
        "wall_seconds": elapsed,
        "rtf": simtotal / elapsed,
        "model": args.model,
        "physics_hz": args.physics_hz,
        "complete_matrix": args.only is None,
    }


if __name__ == "__main__":
    sys.exit(main())
