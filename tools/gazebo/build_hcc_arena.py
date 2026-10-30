#!/usr/bin/env python3
"""Build the RoboCup HCC arena Gazebo world from the arena CAD export (GLB).

The CAD doorways are narrower than the Ridgeback stack accepts, so this edits
the export instead of hand-modelling a copy:

* every hinged doorway whose clear width (latch-side frame to the open leaf)
  is below ``--door-width`` gets its latch jamb moved outwards along the wall,
  and its leaf lengthened by the same amount;
* every sliding panel is slid until its gap reaches ``--door-width``;
* exterior openings (leaves outside the building) are closed with a wall box,
  so exploration stays inside the arena;
* the ceiling is dropped so the GUI and the sun see into the rooms.

Outputs, all in the Gazebo world frame (robot spawn at the origin):

    <out>/robocup_hcc_arena.sdf
    <out>/meshes/robocup_hcc_arena.glb            visual (glTF, Y-up)
    <out>/meshes/robocup_hcc_arena_collision.stl  walls, frames, leaves,
                                                  panels, furniture boxes

Gazebo's glTF loader keeps the file Y-up, so the SDF rolls the visual by
+90 deg; the collision STL is written Z-up directly.

    python3 tools/gazebo/build_hcc_arena.py \\
        ~/Downloads/2026_09_01_RoboCup_HCC_Arena_V1.glb \\
        src/ridgeback_autonomy_gz/sim/worlds

Pure python + numpy. The script is deterministic for a given input file.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np

WORLD = 'robocup_hcc_arena'
CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16,
      5125: np.uint32, 5126: np.float32}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
# glTF Y-up -> Z-up: rotate +90 deg about X, (x, y, z) -> (x, -z, y).
YUP2ZUP = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
STRUCTURE = ('Walls_2', 'Frames', 'WallsTop')
NOT_FURNITURE = STRUCTURE + ('Floor', 'Ceiling')
JAMB_TOL = 0.005          # vertex snapping tolerance on jamb coordinates [m]
FRAME_DEPTH = 0.045       # frame inner face sits 0.04 m inside the jamb


class Glb:
    """A GLB file with editable POSITION data and node transforms."""

    def __init__(self, path: Path):
        raw = path.read_bytes()
        self.source_sha256 = hashlib.sha256(raw).hexdigest()
        off, chunks = 12, []
        while off < len(raw):
            length, _ = struct.unpack_from('<I4s', raw, off)
            chunks.append(raw[off + 8:off + 8 + length])
            off += 8 + length
        self.json = json.loads(chunks[0])
        self.bin = bytearray(chunks[1])
        self.parent = {c: i for i, n in enumerate(self.json['nodes'])
                       for c in n.get('children', [])}
        self.by_name = {n.get('name'): i for i, n in enumerate(self.json['nodes'])}

    # -- transforms ---------------------------------------------------------
    @staticmethod
    def _local(node: dict) -> np.ndarray:
        if 'matrix' in node:
            return np.array(node['matrix'], float).reshape(4, 4).T
        x, y, z, w = node.get('rotation', [0, 0, 0, 1])
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        m = np.eye(4)
        m[:3, :3] = rot @ np.diag(node.get('scale', [1, 1, 1]))
        m[:3, 3] = node.get('translation', [0, 0, 0])
        return m

    def world(self, i: int) -> np.ndarray:
        """Node -> Z-up CAD frame."""
        m = self._local(self.json['nodes'][i])
        while i in self.parent:
            i = self.parent[i]
            m = self._local(self.json['nodes'][i]) @ m
        return YUP2ZUP @ m

    # -- vertex data --------------------------------------------------------
    def _view(self, acc_i: int):
        acc = self.json['accessors'][acc_i]
        view = self.json['bufferViews'][acc['bufferView']]
        dt = np.dtype(CT[acc['componentType']])
        n = NC[acc['type']]
        stride = view.get('byteStride', 0) or dt.itemsize * n
        base = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
        return acc, dt, n, stride, base

    def read(self, acc_i: int) -> np.ndarray:
        acc, dt, n, stride, base = self._view(acc_i)
        return np.stack([np.frombuffer(self.bin, dt, n, base + k * stride)
                         for k in range(acc['count'])]).astype(float)

    def write_positions(self, acc_i: int, pos: np.ndarray) -> None:
        acc, dt, n, stride, base = self._view(acc_i)
        for k, row in enumerate(pos.astype(np.float32)):
            self.bin[base + k * stride:base + k * stride + 12] = row.tobytes()
        acc['min'] = pos.min(0).astype(np.float32).tolist()
        acc['max'] = pos.max(0).astype(np.float32).tolist()

    def primitives(self, i: int):
        node = self.json['nodes'][i]
        if 'mesh' not in node:
            return []
        return self.json['meshes'][node['mesh']]['primitives']

    def node_vertices(self, i: int, frame: np.ndarray) -> list[tuple[int, np.ndarray]]:
        """[(POSITION accessor, Nx3 points in ``frame`` coordinates)] for a node."""
        m = frame @ self.world(i)
        out = []
        for prim in self.primitives(i):
            acc = prim['attributes']['POSITION']
            p = self.read(acc)
            out.append((acc, (m @ np.c_[p, np.ones(len(p))].T).T[:, :3]))
        return out

    def set_node_vertices(self, i: int, frame: np.ndarray, acc: int, pts: np.ndarray) -> None:
        inv = np.linalg.inv(frame @ self.world(i))
        self.write_positions(acc, (inv @ np.c_[pts, np.ones(len(pts))].T).T[:, :3])

    def triangles(self, i: int, frame: np.ndarray, prim_filter=None) -> np.ndarray:
        m = frame @ self.world(i)
        tris = []
        for prim in self.primitives(i):
            if prim_filter and not prim_filter(prim):
                continue
            p = self.read(prim['attributes']['POSITION'])
            pw = (m @ np.c_[p, np.ones(len(p))].T).T[:, :3]
            idx = (self.read(prim['indices']).ravel().astype(int)
                   if 'indices' in prim else np.arange(len(p)))
            tris.append(pw[idx].reshape(-1, 3, 3))
        return np.concatenate(tris) if tris else np.zeros((0, 3, 3))

    def mesh_nodes(self):
        """Mesh nodes reachable from the default scene."""
        stack = list(self.json['scenes'][self.json.get('scene', 0)]['nodes'])
        while stack:
            i = stack.pop()
            if 'mesh' in self.json['nodes'][i]:
                yield i
            stack.extend(self.json['nodes'][i].get('children', []))

    def save(self, path: Path) -> None:
        self.json['buffers'][0]['byteLength'] = len(self.bin)
        js = json.dumps(self.json, separators=(',', ':')).encode()
        js += b' ' * (-len(js) % 4)
        bn = bytes(self.bin) + b'\0' * (-len(self.bin) % 4)
        total = 12 + 8 + len(js) + 8 + len(bn)
        path.write_bytes(struct.pack('<4sII', b'glTF', 2, total)
                         + struct.pack('<I4s', len(js), b'JSON') + js
                         + struct.pack('<I4s', len(bn), b'BIN\0') + bn)


def _solid(glb: Glb, prim: dict) -> bool:
    """Door leaf and panel bodies; handles stay visual-only."""
    return glb.json['materials'][prim.get('material', 0)]['name'].startswith(('paint', 'AeroSilver'))


def _jamb_pairs(values: np.ndarray) -> list[tuple[float, float]]:
    """Group sorted frame coordinates into (jamb, frame inner face) pairs."""
    v = np.unique(np.round(values, 3))
    groups = np.split(v, np.where(np.diff(v) > 0.06)[0] + 1)
    return [(g.min(), g.max()) for g in groups if len(g)]


def _slide_panel(glb: Glb, i: int, name: str, frame: np.ndarray, frames_v: np.ndarray,
                 lo: np.ndarray, hi: np.ndarray, target: float) -> dict:
    """Slide a panel along its wall until its gap reaches ``target``."""
    b = int(np.argmax((hi - lo)[:2]))
    a = 1 - b
    across = (lo[a] + hi[a]) / 2
    band = frames_v[(np.abs(frames_v[:, a] - across) < 0.25)
                    & (frames_v[:, b] > lo[b] - 2.5) & (frames_v[:, b] < hi[b] + 2.5)]
    pairs = _jamb_pairs(band[:, b])
    panel = (lo[b], hi[b])
    # The opening is the jamb-to-jamb span the panel overlaps most.
    overlap = [min(q[1], panel[1]) - max(p[0], panel[0]) for p, q in zip(pairs, pairs[1:])]
    k = int(np.argmax(overlap))
    inner = (pairs[k][1], pairs[k + 1][0])
    clear, side = max((panel[0] - inner[0], -1), (inner[1] - panel[1], +1))
    delta = max(0.0, target - clear)
    if delta:
        shift = np.zeros(3)
        shift[b] = -side * delta
        node = glb.json['nodes'][i]
        parent = glb.world(glb.parent[i]) if i in glb.parent else YUP2ZUP
        local = np.linalg.solve((frame @ parent)[:3, :3], shift)
        node['translation'] = (np.array(node.get('translation', [0, 0, 0])) + local).tolist()
    gap = (inner[0], panel[0] + delta) if side < 0 else (panel[1] - delta, inner[1])
    return {'name': name, 'kind': 'sliding', 'axis': 'xy'[b],
            'clear_before': round(clear, 3), 'clear_after': round(clear + delta, 3),
            'gap': [round(g, 3) for g in gap], 'wall': [round(band[:, a].min(), 3), round(band[:, a].max(), 3)]}


def widen_doors(glb: Glb, frame: np.ndarray, target: float, building) -> tuple[list, list]:
    """Edit hinged and sliding doorways in place; return (report, exterior openings)."""
    frames_v = np.concatenate([p for _, p in glb.node_vertices(glb.by_name['Frames'], frame)])
    structure = {name: glb.node_vertices(glb.by_name[name], frame) for name in STRUCTURE}
    report, exterior = [], []

    for name, i in sorted(glb.by_name.items()):
        if not name or not name.startswith(('Door_', 'SlidingDoor_')):
            continue
        verts = glb.node_vertices(i, frame)
        # Extents come from the leaf or panel body: clear width is measured to
        # the door face, handles excluded.
        body = np.concatenate([p for prim, (_, p) in zip(glb.primitives(i), verts) if _solid(glb, prim)])
        lo, hi = body.min(0), body.max(0)
        if name.startswith('SlidingDoor_'):
            report.append(_slide_panel(glb, i, name, frame, frames_v, lo, hi, target))
            continue

        # An open leaf points away from its wall; the hinge end touches it.
        a = int(np.argmax((hi - lo)[:2]))
        b = 1 - a
        bc = (lo[b] + hi[b]) / 2
        ends = [lo[a], hi[a]]
        hinge = ends[int(np.argmin([np.min(np.abs(frames_v[:, a] - e) + np.abs(frames_v[:, b] - bc))
                                    for e in ends]))]
        band = frames_v[(np.abs(frames_v[:, a] - hinge) < 0.16) & (np.abs(frames_v[:, b] - bc) < 2.5)]
        wall = (band[:, a].min(), band[:, a].max())
        pairs = _jamb_pairs(band[:, b])
        hinge_pair = min(pairs, key=lambda p: abs((p[0] + p[1]) / 2 - bc))
        # The leaf sits just inside its hinge jamb, on the opening side.
        s = 1.0 if bc > (hinge_pair[0] + hinge_pair[1]) / 2 else -1.0
        ahead = [p for p in pairs if (p[0] - hinge_pair[0]) * s > 0.1]
        if not ahead:
            raise SystemExit(f'{name}: no latch jamb found')
        latch_pair = min(ahead, key=lambda p: abs(p[0] - hinge_pair[0]))
        latch_outer = latch_pair[1] if s > 0 else latch_pair[0]
        latch_inner = latch_pair[0] if s > 0 else latch_pair[1]

        outside = (hi[a] > building[1][a] + 0.05) or (lo[a] < building[0][a] - 0.05)
        if outside:
            opening = (min(hinge_pair[0], latch_pair[0]), max(hinge_pair[1], latch_pair[1]))
            if all(abs(e['opening'][0] - opening[0]) > 1e-3 for e in exterior):
                exterior.append({'axis': 'xy'[b], 'opening': opening, 'wall': wall})
            continue

        leaf_face = hi[b] if s > 0 else lo[b]
        clear = abs(latch_inner - leaf_face)
        delta = max(0.0, target - clear)
        if delta:
            moved = 0
            for name_s, items in structure.items():
                for k, (acc, p) in enumerate(items):
                    in_wall = (p[:, a] > wall[0] - 0.03) & (p[:, a] < wall[1] + 0.03)
                    along = p[:, b] * s
                    jamb = in_wall & (along >= latch_outer * s - FRAME_DEPTH) & (along <= latch_outer * s + JAMB_TOL)
                    beyond = in_wall & (along > latch_outer * s + JAMB_TOL)
                    if beyond.any() and (along[beyond].min() - latch_outer * s) < delta + 0.05:
                        raise SystemExit(f'{name}: wall beyond the latch jamb is shorter than {delta:.3f} m')
                    if jamb.any():
                        p = p.copy()
                        p[jamb, b] += s * delta
                        glb.set_node_vertices(glb.by_name[name_s], frame, acc, p)
                        items[k] = (acc, p)
                        moved += int(jamb.sum())
            if moved == 0:
                raise SystemExit(f'{name}: no jamb vertices found at {latch_outer:.3f}')
            # Lengthen the leaf from its hinge end; the handle rides along.
            far = 1.0 if abs(hi[a] - hinge) > abs(lo[a] - hinge) else -1.0
            for acc, p in verts:
                tip = np.abs(p[:, a] - hinge) > 0.3
                p = p.copy()
                p[tip, a] += far * delta
                glb.set_node_vertices(i, frame, acc, p)
        report.append({'name': name, 'kind': 'hinged', 'axis': 'xy'[b],
                       'clear_before': round(clear, 3), 'clear_after': round(clear + delta, 3),
                       'gap': sorted([round(leaf_face, 3), round(latch_inner + s * delta, 3)]),
                       'wall': [round(w, 3) for w in wall]})
    return report, exterior


def box_triangles(corners: np.ndarray) -> np.ndarray:
    """12 triangles of a box given its 8 corners indexed by (x, y, z) bits."""
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for q in faces:
        tris += [corners[[q[0], q[1], q[2]]], corners[[q[0], q[2], q[3]]]]
    return np.array(tris)


def furniture_boxes(glb: Glb, frame: np.ndarray, min_height: float = 0.03) -> list[tuple[str, np.ndarray]]:
    """One oriented box per furniture mesh node, from its local-frame bounds."""
    boxes = []
    for i in glb.mesh_nodes():
        name = glb.json['nodes'][i].get('name', '')
        if name in NOT_FURNITURE or name.startswith(('Door_', 'SlidingDoor_')):
            continue
        local = np.concatenate([glb.read(p['attributes']['POSITION']) for p in glb.primitives(i)])
        lo, hi = local.min(0), local.max(0)
        corners = np.array([[(lo, hi)[(k >> 2) & 1][0], (lo, hi)[(k >> 1) & 1][1], (lo, hi)[k & 1][2]]
                            for k in range(8)])
        corners = (frame @ glb.world(i) @ np.c_[corners, np.ones(8)].T).T[:, :3]
        if np.ptp(corners[:, 2]) < min_height:
            continue  # floor decals
        boxes.append((name, corners))
    return boxes


def write_stl(path: Path, tris: np.ndarray) -> None:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, tris
    path.write_bytes(b'robocup_hcc_arena collision'.ljust(80, b' ')
                     + struct.pack('<I', len(tris)) + rec.tobytes())


SDF = """<?xml version="1.0"?>
<!-- Generated by tools/gazebo/build_hcc_arena.py from the RoboCup HCC arena
     CAD export (sha256 {sha}). Do not edit by hand; rerun the script.
     Doorways widened to {width:.2f} m clear; exterior openings closed. -->
<sdf version="1.9">
  <world name="{world}">

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::Imu" filename="libgz-sim-imu-system.so"/>
    <plugin name="gz::sim::systems::NavSat" filename="libgz-sim-navsat-system.so"/>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.72 0.72 0.72 1</ambient>
      <background>0.96 0.97 0.99 1</background>
      <shadows>true</shadows>
    </scene>

    <physics name="default_physics" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.25 0.15 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>40 40</size></plane>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="arena">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <pose>{vx:.4f} {vy:.4f} 0 1.5707963267948966 0 0</pose>
          <geometry>
            <mesh><uri>meshes/{world}.glb</uri></mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh><uri>meshes/{world}_collision.stl</uri></mesh>
          </geometry>
        </collision>
      </link>
    </model>
{closures}
  </world>
</sdf>
"""

CLOSURE = """
    <model name="closed_exit_{k}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>0.55 0.57 0.6 1</ambient>
            <diffuse>0.62 0.64 0.68 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('source', type=Path, help='arena CAD export (.glb)')
    ap.add_argument('out', type=Path, help='worlds directory to write into')
    ap.add_argument('--door-width', type=float, default=1.05,
                    help='minimum clear doorway width at every height [m]')
    ap.add_argument('--spawn', type=float, nargs=2, default=(9.05, 6.5), metavar=('X', 'Y'),
                    help='robot spawn, metres from the floor slab corner (corridor centre)')
    args = ap.parse_args()

    glb = Glb(args.source)
    floor = glb.triangles(glb.by_name['Floor'], np.eye(4)).reshape(-1, 3)
    # World frame: Z-up, metres, spawn point at the origin.
    origin = np.array([*(floor.min(0)[:2] + args.spawn), 0.0])
    frame = np.eye(4)
    frame[:3, 3] = -origin
    walls = glb.triangles(glb.by_name['Walls_2'], frame).reshape(-1, 3)
    building = (walls.min(0)[:2], walls.max(0)[:2])

    report, exterior = widen_doors(glb, frame, args.door_width, building)

    ceiling = glb.json['nodes'][glb.by_name['Ceiling']]
    ceiling.pop('mesh', None)

    (args.out / 'meshes').mkdir(parents=True, exist_ok=True)
    glb.save(args.out / 'meshes' / f'{WORLD}.glb')

    tris = [glb.triangles(glb.by_name[n], frame) for n in STRUCTURE if n != 'WallsTop']
    tris += [glb.triangles(i, frame, lambda prim: _solid(glb, prim)) for n, i in glb.by_name.items()
             if n and n.startswith(('Door_', 'SlidingDoor_'))]
    boxes = furniture_boxes(glb, frame)
    tris += [box_triangles(c) for _, c in boxes]
    closures = []
    for k, ex in enumerate(exterior):
        a = 1 if ex['axis'] == 'x' else 0
        lo, hi = np.zeros(3), np.zeros(3)
        lo[1 - a], hi[1 - a] = ex['opening']
        lo[a], hi[a] = ex['wall']
        hi[2] = 2.26
        corners = np.array([[(lo, hi)[(m >> 2) & 1][0], (lo, hi)[(m >> 1) & 1][1], (lo, hi)[m & 1][2]]
                            for m in range(8)])
        tris.append(box_triangles(corners))
        c, size = (lo + hi) / 2, hi - lo
        closures.append(CLOSURE.format(k=k, x=c[0], y=c[1], z=c[2], sx=size[0], sy=size[1], sz=size[2]))
    write_stl(args.out / 'meshes' / f'{WORLD}_collision.stl', np.concatenate(tris))

    visual = frame @ YUP2ZUP  # visual pose = frame translation, roll +90 deg
    (args.out / f'{WORLD}.sdf').write_text(SDF.format(
        sha=glb.source_sha256, width=args.door_width, world=WORLD,
        vx=visual[0, 3], vy=visual[1, 3], closures=''.join(closures)))

    for r in report:
        print(f"{r['name']:15s} {r['kind']:7s} wall {r['axis']}-axis  clear {r['clear_before']:.3f} -> "
              f"{r['clear_after']:.3f} m  gap {r['gap']}")
    for ex in exterior:
        print(f"closed exterior opening along {ex['axis']}: {np.round(ex['opening'], 3).tolist()}")
    print(f'furniture boxes: {len(boxes)}; collision triangles: {sum(len(t) for t in tris)}')
    print(f'world origin = CAD point {np.round(origin, 3).tolist()} (Z-up)')


if __name__ == '__main__':
    main()
