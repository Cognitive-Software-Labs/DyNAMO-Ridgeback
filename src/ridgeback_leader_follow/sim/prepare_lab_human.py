"""Blender 4.2.9: blender -b --python prepare_lab_human.py -- <maya-directory>.

Bake the CC0 Innerscene/MakeHuman pedestrian into evaluated world-space frames.
Keep separate triangle material assignments, UV transforms and alpha cutouts.
"""
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

import bpy
from mathutils import Matrix
import numpy as np


def main():
    output = Path(sys.argv[sys.argv.index('--') + 1]).resolve()
    source = (output / 'source.glb').read_bytes()
    size = struct.unpack_from('<I', source, 12)[0]
    gltf = json.loads(source[20:20 + size])
    binary = source[28 + size:]
    materials = gltf['materials']
    for i, image in enumerate(gltf['images']):
        view = gltf['bufferViews'][image['bufferView']]
        start = view.get('byteOffset', 0)
        suffix = '.png' if image['mimeType'] == 'image/png' else '.jpg'
        image['file'] = f'image_{i}{suffix}'
        (output / image['file']).write_bytes(binary[start:start + view['byteLength']])
    for material in materials:
        tex = material['pbrMetallicRoughness'].get('baseColorTexture')
        if tex is not None:
            material['image'] = gltf['images'][gltf['textures'][tex['index']]['source']]['file']
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(output / 'source.glb'))
    scene = bpy.context.scene
    scene.render.fps = 24
    objects = [o for o in scene.objects if o.type == 'MESH' and len(o.material_slots)]
    material_ids = {m['name']: i for i, m in enumerate(materials)}
    frames, normal_frames, faces, uvs, groups = [], [], [], [], []
    # Source faces -Y after glTF import; the path convention faces +X.
    rotation = Matrix.Rotation(math.pi / 2, 4, 'Z')
    for frame in range(48):
        scene.frame_set(frame)
        positions, normals, offset = [], [], 0
        for obj in objects:
            evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
            mesh = evaluated.to_mesh()
            mesh.calc_loop_triangles()
            transform = rotation @ evaluated.matrix_world
            positions.extend([transform @ v.co for v in mesh.vertices])
            normal_transform = transform.to_3x3().inverted().transposed()
            normals.extend([(normal_transform @ v.normal).normalized() for v in mesh.vertices])
            if frame == 0:
                for tri in mesh.loop_triangles:
                    material_id = material_ids[obj.material_slots[tri.material_index].material.name]
                    material = materials[material_id]
                    tex = material['pbrMetallicRoughness'].get('baseColorTexture', {})
                    uv_transform = tex.get('extensions', {}).get('KHR_texture_transform', {})
                    sx, sy = uv_transform.get('scale', [1, 1])
                    ox, oy = uv_transform.get('offset', [0, 0])
                    faces.append([v + offset for v in tri.vertices])
                    groups.append(material_id)
                    for loop in tri.loops:
                        u, v = mesh.uv_layers.active.data[loop].uv if mesh.uv_layers.active else (0., 0.)
                        uvs.append((u * sx + ox, 1 - ((1 - v) * sy + oy)))
            offset += len(mesh.vertices)
            evaluated.to_mesh_clear()
        frames.append(np.asarray(positions, dtype=np.float32))
        normal_frames.append(np.asarray(normals, dtype=np.float32))
    points = np.stack(frames)
    floor = points[:, :, 2].min()
    scale = 1.72 / (points[:, :, 2].max() - floor)
    points[:, :, 2] -= floor
    points *= scale
    np.savez_compressed(output / 'walk.npz', points=points, normals=np.stack(normal_frames), faces=np.asarray(faces, np.int32),
                        uv=np.asarray(uvs, np.float32), materials=np.asarray(groups, np.int32),
                        fps=24., cycle_seconds=2., gait_speed_mps=1.3 * scale)
    (output / 'materials.json').write_text(json.dumps(materials, indent=2) + '\n')
    receipt = {'blender': bpy.app.version_string, 'height_m': 1.72, 'fps': 24,
               'frames': 48, 'cycle_seconds': 2., 'vertices': points.shape[1],
               'faces': len(faces), 'gait_speed_mps': float(1.3 * scale),
               'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in output.iterdir() if p.is_file() and p.name != 'conversion.json'}}
    (output / 'conversion.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('Baked lab pedestrian:', receipt)


if __name__ == '__main__':
    main()
