#!/usr/bin/env python3
"""Reproduce deleted G1 links surviving in Isaac Sim's Fabric scene.

Run with the Isaac interpreter. Source the ROS workspace for --through-provider.
--measure-latency times a scope query with the lab props and samples Fabric
immediately after each USD delete and for six SimulationApp updates.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
from statistics import median, quantiles
import sys
import threading
from time import perf_counter
import types


ROOT = Path(__file__).resolve().parents[2]
ISAAC_PACKAGE = ROOT / 'src/ridgeback_autonomy_isaac'
ISAAC = ISAAC_PACKAGE / 'sim/isaac'
USD = ISAAC / 'usd'
G1 = USD / 'models/g1/g1.usda'
BED = USD / 'models/hospital_bed/hospital_bed.usda'
WORLD = USD / 'worlds/g1_distance_calibration.usda'
SCOPE = '/scene_entities'
WIDTH, HEIGHT, FOCAL_PX = 320, 240, 192.0
SPOT = (8.36, 0.31, 0.05)
BED_POSITION = (7.25, -0.12, 0.05)


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['unique', 'reuse', 'deactivate', 'nofreeze'],
                        default='unique')
    parser.add_argument('--cycles', type=int, default=1000)
    parser.add_argument('--present-frames', type=int, default=3)
    parser.add_argument('--gap-frames', type=int, default=4)
    parser.add_argument('--observe-updates', type=int, default=0,
                        help='minimum post-delete updates before checking depth and Fabric')
    parser.add_argument('--with-object', action='store_true')
    parser.add_argument('--play', type=int, choices=[0, 1], default=1)
    parser.add_argument('--renderer', default='RealTimePathTracing')
    parser.add_argument('--max-minutes', type=float, default=25.0)
    parser.add_argument('--out', type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--measure-latency', action='store_true')
    modes.add_argument('--through-provider', action='store_true')
    modes.add_argument('--inject-persistent', action='store_true',
                       help='inject a Fabric-only prim and exercise the ROS cleanup failure')
    args, kit_args = parser.parse_known_args()
    if args.cycles < 1 or args.present_frames < 1 or args.gap_frames < 2:
        parser.error('need positive cycles and present frames, and at least two gap frames')
    sys.argv[1:] = kit_args
    return args


def report(log, **record):
    log.write(json.dumps(record) + '\n')
    if 'event' in record:
        print('REPRO', json.dumps(record), flush=True)


def fabric_paths(stage, prefix):
    """Read all Fabric paths, including orphans with no parent in Fabric."""
    paths = stage.SelectPrims(device='cpu', want_paths=True).GetPaths()
    prefix = prefix.rstrip('/')
    return [path for path in map(str, paths) if path == prefix or path.startswith(prefix + '/')]


def measure_latency(args, scene, fabric, step, log):
    import yaml

    config = ROOT / 'src/ridgeback_autonomy/config/benchmark_background_lab.yaml'
    layout = yaml.safe_load(config.read_text())
    catalog = ISAAC_PACKAGE / 'sim/benchmark_assets.yaml'
    assets = yaml.safe_load(catalog.read_text())['background']
    for prop in layout['props']:
        asset = assets[prop['model']]
        scene.spawn(f"{SCOPE}/bench_repro_bg_{prop['id']}", str(catalog.parent / asset['resource']),
                    (prop['x'], prop['y'], asset['spawn_z_m']), yaw_quaternion(prop['yaw']))
    step(10)
    timings = []
    for _ in range(100):
        start = perf_counter()
        paths = fabric_paths(fabric, SCOPE)
        timings.append((perf_counter() - start) * 1e3)
    report(log, event='scope_query', props=len(layout['props']), prims=len(paths),
           median_ms=median(timings), p95_ms=quantiles(timings, n=100)[94])

    first_clean = Counter()
    nonempty = [0] * 7
    for index in range(args.cycles):
        path = f'{SCOPE}/bench_repro_{index:05d}_target_0'
        scene.spawn(path, str(G1), SPOT, yaw_quaternion(3.62))
        step(args.present_frames)
        if not fabric_paths(fabric, path):
            raise RuntimeError(f'{path} never appeared in Fabric')
        if not scene.delete(path):
            raise RuntimeError(f'USD delete failed for {path}')
        counts = [len(fabric_paths(fabric, path))]
        for _ in range(6):
            step()
            counts.append(len(fabric_paths(fabric, path)))
        clean = 7 if counts[-1] else max(
            (offset + 1 for offset, count in enumerate(counts) if count), default=0)
        first_clean[clean] += 1
        for offset, count in enumerate(counts):
            nonempty[offset] += bool(count)
        report(log, cycle=index, counts=counts, first_clean=clean)
    report(log, event='latency', cycles=args.cycles, nonempty_by_offset=nonempty,
           first_clean=dict(first_clean), persistent_at_offset_6=first_clean[7])


def run_cycles(args, fabric, stage, step, spawn, delete, depth, log, audit_window):
    import numpy as np
    from pxr import UsdGeom

    step(60)
    baseline = depth()
    if args.play and (baseline is None or not np.isfinite(baseline).any()):
        raise RuntimeError('depth annotator did not produce a usable baseline')
    if baseline is not None:
        np.save(args.out / 'baseline.npy', baseline)

    def foreground_count(image):
        if not args.play:
            return 0
        if image is None:
            raise RuntimeError('missing depth image during cycle')
        nearer = np.isfinite(image) & np.isfinite(baseline) & (image < baseline - 0.03)
        return int(nearer.sum())

    started = perf_counter()
    cycle_seconds = 0.0
    ghosts = 0
    completed = 0
    for index in range(args.cycles):
        if perf_counter() - started > args.max_minutes * 60:
            break
        start = perf_counter()
        suffix = 'fixed' if args.variant == 'reuse' else f'{index:05d}'
        target = f'bench_repro_{suffix}_target_0'
        bed = f'bench_repro_{index:05d}_obj_0'
        if args.with_object:
            spawn(bed, BED, BED_POSITION, yaw_quaternion(0.61))
            step()
        spawn(target, G1, SPOT, yaw_quaternion(3.62))
        step(args.present_frames)
        present_px = foreground_count(depth())
        if args.variant == 'deactivate':
            stage.GetPrimAtPath(f'{SCOPE}/{target}').SetActive(False)
            step()
        delete(target)
        step()
        if args.with_object:
            delete(bed)
        step(args.gap_frames - 1)
        cycle_seconds += perf_counter() - start

        # Capture the independent result after the final audit and a render.
        observation = max(args.observe_updates, audit_window + 1)
        step(max(0, observation - (args.gap_frames - 1)))
        image = depth()
        ghost_px = foreground_count(image)
        mesh_paths = fabric.GetPrimsWithTypeName('Mesh')
        meshes = [path for path in map(str, mesh_paths) if path.startswith(SCOPE + '/')]
        # Independently check meshes, and check all other Fabric prim types as well.
        remaining = [path for path in fabric_paths(fabric, SCOPE) if path != SCOPE]
        if ghost_px or meshes or remaining:
            ghosts += 1
            if image is not None:
                np.save(args.out / f'ghost_{index:05d}_depth.npy', image)
            report(log, event='leftover', cycle=index, ghost_px=ghost_px, meshes=meshes,
                   prims=remaining)
        report(log, cycle=index, present_px=present_px, ghost_px=ghost_px, meshes=len(meshes),
               prims=len(remaining))
        completed += 1
    report(log, event='done', cycles=completed, ghosts=ghosts,
           cycle_seconds=round(cycle_seconds, 3), seconds=round(perf_counter() - started, 3),
           depth_checked=bool(args.play), variant=args.variant)
    return 0 if completed == args.cycles and ghosts == 0 else 1


def check_persistent(scene, fabric, app, timeline, log):
    import rclpy
    from ridgeback_autonomy.benchmarking.scene_control import (
        SceneControlError,
        SceneControlFailure,
        create_scene_control_client,
    )
    from scene_provider import RobotState, SceneControlService, SceneEntities, SimFrame

    name = 'bench_x_target_0'
    path = f'{SCOPE}/{name}/model/orphan'
    fabric.DefinePrim(path, 'Xform')
    if path not in fabric_paths(fabric, SCOPE):
        raise RuntimeError('Fabric injection was not visible to the probe')
    scene.remove_fabric_prims = lambda _paths: []

    rclpy.init(args=[])
    service = SceneControlService(SceneEntities(scene, 'r100_0001/robot'))
    node = rclpy.create_node('fabric_residue_repro')
    client = create_scene_control_client(
        node, service.service_namespace, observe_clock=False,
        delete_timeout_sec=1.0, retry_sleep_sec=0.05)
    result = {}

    def run_client():
        try:
            client.wait_for_services(5.0)
            result['entities'] = list(client.find_benchmark_entities())
            try:
                client.delete_entity(name)
            except SceneControlError as exc:
                result['delete_failure'] = exc.failure.name
        except Exception as exc:
            result['error'] = str(exc)

    thread = threading.Thread(target=run_client, daemon=True)
    thread.start()
    try:
        deadline = perf_counter() + 15.0
        while thread.is_alive() and perf_counter() < deadline:
            app.update()
            service.process_pending(
                SimFrame(timeline.get_current_time(), RobotState(0.0, 0.0, 0.0, 0.0)))
        if thread.is_alive():
            raise RuntimeError('client did not finish')
        report(log, event='persistent_check', **result)
        return int(result.get('entities') != [name] or
                   result.get('delete_failure') != SceneControlFailure.CLEANUP_INCOMPLETE.name)
    finally:
        service.shutdown()
        node.destroy_node()
        rclpy.shutdown()



def main():
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(ISAAC))
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
    from isaacsim import SimulationApp

    app = SimulationApp({'headless': True, 'renderer': args.renderer, 'anti_aliasing': 0,
                         'fast_shutdown': False})
    import carb
    import numpy as np
    import omni.timeline
    import omni.usd
    import usdrt
    from pxr import Gf, UsdGeom, UsdPhysics

    use_ros = args.through_provider or args.inject_persistent
    if not use_ros:
        # UsdScene needs only the scope and error type for the standalone probe.
        stub = types.ModuleType('scene_provider')
        stub.InvalidResource = type('InvalidResource', (Exception,), {})
        stub.SPAWNED_ENTITY_SCOPE = SCOPE
        sys.modules['scene_provider'] = stub
    import usd_scene

    if not carb.settings.get_settings().get('/app/useFabricSceneDelegate'):
        raise RuntimeError('the reproducer requires the Fabric Scene Delegate')
    ctx = omni.usd.get_context()
    if not ctx.open_stage(str(WORLD)):
        raise RuntimeError(f'could not open {WORLD}')
    for _ in range(30):
        app.update()
    stage = ctx.get_stage()
    if not any(prim.GetTypeName() == 'PhysicsScene' for prim in stage.Traverse()):
        UsdPhysics.Scene.Define(stage, '/physicsScene')
    if args.variant == 'nofreeze':
        usd_scene.freeze_dynamics = lambda _prim: 0

    timeline = omni.timeline.get_timeline_interface()
    timeline.set_end_time(1.0e9)
    timeline.set_looping(False)
    if args.play:
        timeline.play()
    fabric = usdrt.Usd.Stage.Attach(ctx.get_stage_id())
    scene = usd_scene.UsdScene(stage, stage_id=ctx.get_stage_id() if use_ros else None)
    if args.inject_persistent:
        with (args.out / 'cycles.jsonl').open('w', buffering=1) as log:
            return check_persistent(scene, fabric, app, timeline, log)

    update = app.update

    audit_window = 0
    if args.through_provider:
        from scene_provider import FABRIC_GRACE_UPDATES, FABRIC_AUDIT_UPDATES
        from scene_provider import FABRIC_REPAIR_ATTEMPTS, SceneEntities, SpawnPlan

        entities = SceneEntities(scene, 'r100_0001/robot')

        def update():
            app.update()
            entities.audit(timeline.get_current_time())

        def spawn(name, resource, position, orientation):
            entities.spawn(SpawnPlan(name, False, str(resource), position, orientation))

        delete = entities.delete
        audit_window = FABRIC_GRACE_UPDATES + FABRIC_AUDIT_UPDATES + FABRIC_REPAIR_ATTEMPTS
    else:
        def spawn(name, resource, position, orientation):
            scene.spawn(f'{SCOPE}/{name}', str(resource), position, orientation)

        def delete(name):
            if not scene.delete(f'{SCOPE}/{name}'):
                raise RuntimeError(f'USD delete failed for {name}')

    def step(count=1):
        for _ in range(count):
            update()

    with (args.out / 'cycles.jsonl').open('w', buffering=1) as log:
        if args.measure_latency:
            measure_latency(args, scene, fabric, step, log)
            return 0

        import omni.replicator.core as rep

        camera = UsdGeom.Camera.Define(stage, '/repro_cam')
        camera.CreateFocalLengthAttr(FOCAL_PX * 0.02)
        camera.CreateHorizontalApertureAttr(WIDTH * 0.02)
        camera.CreateVerticalApertureAttr(HEIGHT * 0.02)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(4.20, 0, 1.19),
                                      Gf.Vec3d(*SPOT[:2], 0.9), Gf.Vec3d(0, 0, 1))
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(view.GetInverse())
        product = rep.create.render_product('/repro_cam', (WIDTH, HEIGHT))
        annotator = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
        annotator.attach(product)

        def depth():
            data = annotator.get_data()
            if isinstance(data, dict):
                data = data.get('data')
            image = np.asarray(data, dtype=np.float32).squeeze()
            return image.copy() if image.shape == (HEIGHT, WIDTH) else None

        return run_cycles(args, fabric, stage, step, spawn, delete, depth, log, audit_window)


if __name__ == '__main__':
    status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # Kit 6.1.0-rc.26 corrupted its heap during SimulationApp.close in the probe.
    os._exit(status)
