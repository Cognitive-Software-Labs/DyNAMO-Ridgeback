#!/usr/bin/env python3
"""Audit benchmark scenes against the current camera and LiDAR geometry.

Installed as ``scene_audit``. Reads a scenario YAML, the Gazebo asset catalog
and the robot description, and writes ``audit.json`` (plus optional per-scene
PNG previews and a check of the YAML's certification comments) to
``artifacts/benchmarks/scene_audit/<stamp>/`` unless ``--out`` says otherwise.
See :mod:`ridgeback_autonomy.benchmarking.scene_audit` for every definition.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

from ridgeback_autonomy.benchmarking import scene_audit as sa
from ridgeback_autonomy.benchmarking import scene_certifications as certs
from ridgeback_autonomy.benchmarking.asset_catalog import (
    DEFAULT_TARGET,
    load_asset_catalog,
    scene_object_models,
)
from ridgeback_autonomy.benchmarking.paths import (
    BENCHMARK_ROOT_RELATIVE,
    anchored_path,
    write_json_atomic,
)
from ridgeback_autonomy.benchmarking.process_utils import git_provenance
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios


DEFAULT_WORLD = 'target_distance_calibration'
CAMERA_FRAME = 'camera_0_color_optical_frame'
LASER_FRAME = 'lidar2d_0_laser'
SELF_CHECK_NEAR_M = 1e-3


def _workspace_root() -> str:
    from ridgeback_common.paths import workspace_root

    return str(workspace_root(__file__))


def _sha256(path: str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _resolve_uri(uri: str) -> Path | None:
    if uri.startswith('file://'):
        return Path(uri[len('file://'):])
    if uri.startswith('package://'):
        package, _, rest = uri[len('package://'):].partition('/')
        try:
            from ament_index_python.packages import get_package_share_directory
            return Path(get_package_share_directory(package)) / rest
        except Exception:  # noqa: BLE001 - lookup failure just skips the mesh
            return None
    return Path(uri)


def sensor_robot_triangles(urdf: ET.Element, segments: int):
    """Visual geometry of every robot link in base_link, plus what was skipped."""

    from ridgeback_autonomy import robot_description as rd

    chunks, skipped, links = [], [], {}
    for link in urdf.findall('link'):
        name = link.get('name')
        visuals = link.findall('visual')
        if not visuals:
            continue
        try:
            link_pose = rd.transform_to(urdf, name)
        except LookupError:
            skipped.append(f'{name}: not attached to base_link')
            continue
        for visual in visuals:
            geometry = visual.find('geometry')
            pose = link_pose @ rd.pose_matrix(*rd.origin(visual))
            tris = None
            if geometry.find('box') is not None:
                size = geometry.find('box').get('size').split()
                tris = sa.box_triangles([float(v) for v in size])
            elif geometry.find('cylinder') is not None:
                cyl = geometry.find('cylinder')
                tris = sa.cylinder_triangles(float(cyl.get('radius')), float(cyl.get('length')),
                                             segments)
            elif geometry.find('mesh') is not None:
                mesh = geometry.find('mesh')
                path = _resolve_uri(mesh.get('filename'))
                if path is None or not path.is_file():
                    skipped.append(f'{name}: mesh {mesh.get("filename")} unresolved')
                    continue
                tris = sa.read_stl(path)
                if mesh.get('scale'):
                    tris = tris * np.array([float(v) for v in mesh.get('scale').split()])
            if tris is None:
                skipped.append(f'{name}: unsupported geometry')
                continue
            tris = sa.transform_triangles(tris, pose)
            chunks.append(tris)
            links.setdefault(name, []).append(tris)
    links = {name: np.concatenate(parts) for name, parts in links.items()}
    return (np.concatenate(chunks) if chunks else np.zeros((0, 3, 3))), links, skipped


def current_sensor_geometry(
    robot_yaml: str,
    profiles: list[str],
    *,
    base_link_world_z: float,
    pixel_center_offset: float,
    segments: int,
):
    """Sensor geometry from the expanded simulation description."""

    from ridgeback_autonomy import robot_description as rd

    cameras, lasers, near = [], [], []
    urdf = None
    for profile in profiles:
        urdf = rd.expand_robot_yaml(robot_yaml, is_sim=True, profile_name=profile)
        cameras.append(rd.transform_to(urdf, CAMERA_FRAME))
        lasers.append(rd.transform_to(urdf, LASER_FRAME))
        sensor = rd.gazebo_sensor(urdf, 'rgbd_camera')
        near.append(float(sensor.find('camera/clip/near').text))
    if any(not np.allclose(c, cameras[0], atol=1e-12) for c in cameras) or \
            any(not np.allclose(laser, lasers[0], atol=1e-12) for laser in lasers):
        raise RuntimeError('camera or laser pose depends on the camera profile')
    body, links, skipped = sensor_robot_triangles(urdf, segments)
    footprint = None
    low = body.reshape(-1, 3)
    low = low[low[:, 2] <= 0.3]
    if len(low):
        footprint = (float(low[:, 0].min()), float(low[:, 0].max()),
                     float(low[:, 1].min()), float(low[:, 1].max()))
    geometry = sa.SensorGeometry(
        name='current',
        base_link_world_z=base_link_world_z,
        camera_in_base=cameras[0],
        laser_in_base=lasers[0],
        intrinsics=tuple(sa.profile_intrinsics(p) for p in profiles),
        laser=sa.contract_laser_spec(),
        near_clip_m=near[0],
        pixel_center_offset=pixel_center_offset,
        footprint_xy=footprint,
        provenance={
            'source': f'expanded simulation description of {robot_yaml}',
            'camera_frame': CAMERA_FRAME,
            'laser_frame': LASER_FRAME,
            'near_clip_source': 'rgbd_camera <clip><near> in the expanded description',
            'laser_scan_source': 'ridgeback_common.lidar_contract (UST-10LX raw contract)',
            'footprint_source': 'xy bounds of robot visual geometry below 0.3 m in base_link',
        },
    )
    return geometry, body, links, skipped


def _self_occlusion(geometry: sa.SensorGeometry, links: dict, skipped) -> dict:
    """Pixels each robot link covers; the camera's own housing is excluded.

    The colour optical origin sits inside the D455 housing mesh, whose front
    face the real lens looks through, so that one link is not an occluder.
    """

    housing = 'camera_0_link'
    per_link = {}
    for name, tris in sorted(links.items()):
        if name == housing:
            continue
        per_link[name] = {
            'triangles': int(len(tris)),
            'pixels_near_1mm': sa.render_self(tris, geometry, near=SELF_CHECK_NEAR_M),
            'pixels_at_camera_near_clip': sa.render_self(tris, geometry,
                                                         near=geometry.near_clip_m),
        }
    covering = sorted(name for name, entry in per_link.items()
                      if any(entry['pixels_near_1mm'].values()))
    clean = not covering
    return {
        'excluded_links': {housing: 'camera housing around the optical origin'},
        'links_skipped': skipped,
        'per_link': per_link,
        'covering_links': covering,
        'omitted_from_scenes': clean,
        'justification': (
            'no robot link except the camera housing covers any pixel even with a 1 mm near '
            'plane, so the sensor robot is omitted from the scene triangles'
            if clean else f'links {covering} cover pixels: the robot must be added to scenes'),
    }


def _preview(path: Path, scene, result: dict, render: sa.ProfileRender, lidar: dict,
             geometry: sa.SensorGeometry) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    entities = render.entities
    code = render.code
    ent = np.where(code >= 0, code >> 8, -1)
    palette_t = ['#e4572e', '#29335c', '#f3a712', '#a8c686']
    palette_o = ['#669bbc', '#8d6a9f', '#5b8e7d', '#c08497']
    colors = ['#ffffff']
    for e in entities:
        if e.kind == sa.KIND_WORLD:
            colors.append('#d9d9d9' if 'floor' in e.name else '#eeeeee')
        elif e.kind == sa.KIND_TARGET:
            colors.append(palette_t[e.index % len(palette_t)])
        else:
            colors.append(palette_o[e.index % len(palette_o)])
    fig, (ax_cam, ax_top) = plt.subplots(1, 2, figsize=(13, 5.2),
                                         gridspec_kw={'width_ratios': [1.25, 1]})
    ax_cam.imshow(ent + 1, cmap=ListedColormap(colors), vmin=0, vmax=len(colors) - 1,
                  interpolation='nearest')
    intr = render.intrinsics
    for t in render.targets:
        if t['bbox']:
            u0, v0, u1, v1 = t['bbox']
            ax_cam.add_patch(plt.Rectangle((u0 - 0.5, v0 - 0.5), u1 - u0 + 1, v1 - v0 + 1,
                                           fill=False, ec='k', lw=0.8))
            label = (f"T{t['index']} vis {t['visible_fraction_floor_kept']}"
                     f" in {t['in_frame_fraction']} c {t['center_ratio']}")
            ax_cam.text(u0, v0 - 4, label, fontsize=7)
        if t['center_px']:
            ax_cam.plot(*t['center_px'], marker='+', color='k', ms=8)
    ax_cam.set_xlim(-0.5, intr.width - 0.5)
    ax_cam.set_ylim(intr.height - 0.5, -0.5)
    ax_cam.set_title(f'{scene.id}  camera {intr.name}', fontsize=9)
    ax_cam.set_xticks([])
    ax_cam.set_yticks([])

    # Top-down: robot footprint, frustum, LiDAR hits, slices of every entity.
    cam = geometry.camera_in_world
    hfov = math.atan(intr.width / 2 / intr.fx)
    for sign in (-1, 1):
        ang = sign * hfov
        ax_top.plot([cam[0, 3], cam[0, 3] + 11 * math.cos(ang)],
                    [cam[1, 3], cam[1, 3] + 11 * math.sin(ang)], color='#999', lw=0.6, ls='--')
    if geometry.footprint_xy:
        x0, x1, y0, y1 = geometry.footprint_xy
        ax_top.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fc='#bbb', ec='k', lw=0.6))
    segs = lidar['_segments']
    seg_ent = lidar['_segment_entity']
    for eid, e in enumerate(entities):
        if e.kind == sa.KIND_WORLD:
            continue
        s = segs[seg_ent == eid]
        color = colors[eid + 1]
        for a, b in s:
            ax_top.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=1.2)
        tri = lidar['geometry'].triangles[lidar['geometry'].entity_rows[eid]]
        pts = tri.reshape(-1, 3)[:, :2][:: max(1, len(tri) // 600)]
        ax_top.scatter(pts[:, 0], pts[:, 1], s=0.3, color=color, alpha=0.4)
        ax_top.text(e.pose[0], e.pose[1] + 0.35, f'{e.kind[0].upper()}{e.index}', fontsize=7)
    origin = geometry.laser_in_world[:3, 3]
    hit = np.isfinite(lidar['_ranges']) & (lidar['_owner_entity'] >= 0)
    owners = lidar['_owner_entity'][hit]
    keep = np.array([entities[o].kind != sa.KIND_WORLD for o in owners], dtype=bool)
    r = lidar['_ranges'][hit][keep]
    a = lidar['_angles'][hit][keep]
    ax_top.scatter(origin[0] + r * np.cos(a), origin[1] + r * np.sin(a), s=2, color='k')
    ax_top.set_aspect('equal')
    ax_top.set_xlim(-1, 11.5)
    ax_top.set_ylim(-4.5, 4.5)
    ax_top.grid(alpha=0.3)
    ax_top.set_title(f'top-down, laser plane z={lidar["plane_world_z_m"]} m', fontsize=9)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=90)
    plt.close(fig)


def _write_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--scenario', required=True, help='Scenario YAML to audit')
    parser.add_argument('--profile', action='append', dest='profiles',
                        help='Camera profile (repeatable; default: every shared profile)')
    parser.add_argument('--out', help='Output directory (default: '
                        f'{BENCHMARK_ROOT_RELATIVE}/scene_audit/<stamp>)')
    parser.add_argument('--images', action='store_true', help='Write per-scene PNG previews')
    parser.add_argument('--gallery', help='Also write the HTML scene gallery here (implies --images)')
    parser.add_argument('--scene', action='append', dest='scenes', help='Only these scene ids')
    parser.add_argument('--legacy-geometry', nargs='?', const='pixel',
                        choices=sorted(sa.LEGACY_PRESETS),
                        help='Historical geometry preset instead of the description')
    parser.add_argument('--base-link-z', type=float, default=sa.GAZEBO_BASE_LINK_WORLD_Z,
                        help='World z of the sensor base_link (default: Gazebo settled height)')
    parser.add_argument('--pixel-center-offset', type=float, default=0.5,
                        help='Sample offset in px; 0.5 matches Gazebo and Isaac renders')
    parser.add_argument('--cylinder-segments', type=int, default=sa.DEFAULT_CYLINDER_SEGMENTS)
    parser.add_argument('--asset-catalog', help='Asset catalog (default: Gazebo catalog)')
    parser.add_argument('--world', default=DEFAULT_WORLD)
    parser.add_argument('--robot-yaml', help='Robot declaration (default: clearpath/robot.yaml)')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    args.images = args.images or bool(args.gallery)
    started = time.monotonic()
    root = _workspace_root()
    scenario = anchored_path(root, args.scenario)
    catalog_path = anchored_path(root, args.asset_catalog or os.path.join(
        'src', 'ridgeback_autonomy_gz', 'sim', 'benchmark_assets.yaml'))
    robot_yaml = anchored_path(root, args.robot_yaml or os.path.join('clearpath', 'robot.yaml'))
    scenes = load_scenarios(scenario)
    if args.scenes:
        wanted = set(args.scenes)
        scenes = tuple(s for s in scenes if s.id in wanted)
        missing = wanted - {s.id for s in scenes}
        if missing:
            print(f'unknown scene ids: {sorted(missing)}', file=sys.stderr)
            return 2

    catalog = load_asset_catalog(catalog_path)
    selection = catalog.select(args.world, DEFAULT_TARGET, scene_object_models(scenes))
    segments = args.cylinder_segments
    target = sa.load_sdf_model(selection.target.uri, cylinder_segments=segments)
    objects = {name: sa.load_sdf_model(res.uri, cylinder_segments=segments)
               for name, res in selection.objects.items()}
    world = sa.load_world_models(selection.world.resource.uri, cylinder_segments=segments)
    builder = sa.SceneBuilder(
        target=target, objects=objects, world=world,
        target_spawn_z=selection.target.spawn_z_m,
        object_spawn_z={n: r.spawn_z_m for n, r in selection.objects.items()})

    self_check = None
    if args.legacy_geometry:
        sensors = sa.legacy_sensor_geometry(
            args.legacy_geometry, base_link_world_z=args.base_link_z)
    else:
        from ridgeback_common.camera_profiles import CAMERA_PROFILE_CHOICES

        profiles = args.profiles or list(CAMERA_PROFILE_CHOICES)
        sensors, body, links, skipped = current_sensor_geometry(
            robot_yaml, profiles, base_link_world_z=args.base_link_z,
            pixel_center_offset=args.pixel_center_offset, segments=segments)
        self_check = _self_occlusion(sensors, links, skipped)
        if not self_check['omitted_from_scenes']:
            print('warning: sensor robot body is inside the camera frustum', file=sys.stderr)

    stamp = time.strftime('%Y%m%d_%H%M%S')
    suffix = f'_legacy-{args.legacy_geometry}' if args.legacy_geometry else ''
    out = Path(anchored_path(root, args.out) if args.out else os.path.join(
        root, *BENCHMARK_ROOT_RELATIVE.parts, 'scene_audit', stamp + suffix))
    out.mkdir(parents=True, exist_ok=True)

    auditor = sa.SceneAuditor(builder, sensors)
    results = {}
    timings = {}
    for scene in scenes:
        t0 = time.monotonic()
        result, renders, lidar = auditor.audit_scene(scene)
        timings[scene.id] = round(time.monotonic() - t0, 3)
        results[scene.id] = result
        if args.images:
            for name, render in renders.items():
                _preview(out / 'images' / name / f'{scene.id}.png', scene, result, render,
                         lidar, sensors)
        print(f'{scene.id}: {timings[scene.id]:.2f} s', flush=True)

    claims = certs.parse_claims(Path(scenario).read_text(encoding='utf-8'))
    certification = {}
    for intr in sensors.intrinsics:
        rows = certs.check_claims(claims, results, intr.name)
        banner = certs.check_in_box_banner(results, intr.name)
        _write_rows(out / f'certifications_{intr.name}.csv', rows)
        _write_rows(out / f'in_box_banner_{intr.name}.csv', banner)
        certification[intr.name] = {'claims': rows, 'in_box_banner': banner}

    document = {
        'schema': sa.SCHEMA,
        'created': time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'git': git_provenance(root),
        'scenario': {'path': scenario, 'sha256': _sha256(scenario), 'scenes': len(scenes)},
        'asset_catalog': selection.provenance(),
        'sensor_geometry': sensors.as_dict(),
        'base_link_world_z': {
            'used_m': args.base_link_z,
            'gazebo_m': sa.GAZEBO_BASE_LINK_WORLD_Z,
            'isaac_m': sa.ISAAC_BASE_LINK_WORLD_Z,
        },
        'self_occlusion': self_check,
        'triangles': {
            'target': {target.name: target.triangle_count},
            'objects': {n: m.triangle_count for n, m in objects.items()},
            'world': {m.name: m.triangle_count for m in world},
            'cylinder_segments': segments,
        },
        'model_notes': {m.name: list(m.notes) for m in (target, *objects.values())},
        'torso_centre_model_z_m': auditor.torso_z,
        'conventions': {
            'pixel_sampling': 'pixel (i, j) samples image coordinate (i + offset, j + offset); '
                              'offset = pixel_center_offset_px',
            'vis_claims_compare': certs.PRIMARY_VIS,
            'definitions': 'see ridgeback_autonomy.benchmarking.scene_audit module docstring',
        },
        'assumptions': [
            'Gazebo visual geometry for targets, objects and world (gpu_lidar and the camera '
            'both render visuals)',
            'sensor robot at the world origin, zero yaw, base_link at base_link_world_z',
            f'targets at catalog spawn_z {selection.target.spawn_z_m} m; objects at '
            f'{sorted({r.spawn_z_m for r in selection.objects.values()})} m',
            'Isaac uses a different G1 asset (Unitree 29-DoF Dex1 USD); this audit models '
            'the Gazebo G1',
        ],
        'runtime_s': round(time.monotonic() - started, 2),
        'scene_runtime_s': timings,
        'scenes': list(results.values()),
        'certification_check': certification,
    }
    write_json_atomic(out / 'audit.json', document)
    print(f'wrote {out / "audit.json"} in {document["runtime_s"]} s')
    if args.gallery:
        from ridgeback_autonomy.benchmarking.scene_gallery import render_gallery, repeats_by_scene
        rendered = [i['name'] for i in document['sensor_geometry']['intrinsics']]
        profile = certs.EDGE_BAND_PROFILE if certs.EDGE_BAND_PROFILE in rendered else rendered[0]
        gallery = Path(anchored_path(root, args.gallery))
        gallery.write_text(render_gallery(
            audit=document, scenario_text=Path(scenario).read_text(encoding='utf-8'),
            repeats=repeats_by_scene(scenes), image_dir=out / 'images' / profile,
            profile=profile, scenario_name=Path(scenario).name), encoding='utf-8')
        print(f'wrote {gallery}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
