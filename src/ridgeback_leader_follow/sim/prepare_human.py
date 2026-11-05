"""Run with Blender 4.2.9: blender -b --python prepare_human.py -- <asset-dir>.

Convert the pinned CC-BY-4.0 Cesium Man GLB to a Gazebo skin/animation and
an identical evaluated vertex cache for Isaac. Blender is an authoring tool,
not a runtime dependency. See assets/cesium_man/NOTICE.md for attribution.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector
import numpy as np


def main():
    output = Path(sys.argv[sys.argv.index('--') + 1]).resolve()
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(output / 'CesiumMan.glb'))
    scene = bpy.context.scene
    scene.render.fps = 24
    scene.frame_start, scene.frame_end = 1, 48
    mesh_obj = bpy.data.objects['Cesium_Man']
    root = bpy.data.objects['Z_UP']
    scene.frame_set(1)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated = mesh_obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    positions = np.array([evaluated.matrix_world @ v.co for v in mesh.vertices])
    evaluated.to_mesh_clear()
    # Cesium Man's Blender world is Z-up and faces +X. Put its feet on z=0
    # and scale its height to 1.75 m; retain the original animation and texture.
    scale = 1.75 / np.ptp(positions[:, 2])
    normalization = Matrix.Scale(scale, 4)
    normalization.translation.z = -float(positions[:, 2].min()) * scale
    root.matrix_world = normalization @ root.matrix_world
    frames = []
    for frame in range(1, 49):
        scene.frame_set(frame)
        evaluated = mesh_obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = evaluated.to_mesh()
        mesh.calc_loop_triangles()
        frames.append(np.array([evaluated.matrix_world @ v.co for v in mesh.vertices],
                               dtype=np.float32))
        if frame == 1:
            faces = np.array([tri.vertices for tri in mesh.loop_triangles], dtype=np.int32)
            uv = np.array([mesh.uv_layers.active.data[i].uv
                           for tri in mesh.loop_triangles for i in tri.loops], dtype=np.float32)
        evaluated.to_mesh_clear()
    # Blender evaluates a cyclic animation at 24 Hz. The 48-frame cache has
    # the same 2-second period as Collada's exported action.
    np.savez_compressed(output / 'walk.npz', points=np.stack(frames), faces=faces, uv=uv,
                        fps=24., cycle_seconds=2.)
    bpy.ops.object.select_all(action='DESELECT')
    for obj in (root, bpy.data.objects['Armature'], mesh_obj):
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_obj
    bpy.ops.wm.collada_export(
        filepath=str(output / 'walk.dae'), selected=True, include_children=True,
        include_armatures=True, include_animations=True, include_all_actions=False,
        sampling_rate=1, triangulate=True, use_texture_copies=True,
        export_global_up_selection='Z', apply_global_orientation=False)
    for image in bpy.data.images:
        if image.packed_file:
            image.filepath_raw = str(output / 'texture.png')
            image.file_format = 'PNG'
            image.save()
    # Collada references Blender's exported image name; the USD cache uses
    # the same PNG bytes rather than a second artwork source.
    from xml.etree import ElementTree as ET
    tree = ET.parse(output / 'walk.dae')
    namespace = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    names = [p.text for p in tree.findall('.//c:library_images/c:image/c:init_from', namespace)]
    texture = output / names[0]
    if texture.name != 'texture.png':
        (output / 'texture.png').write_bytes(texture.read_bytes())
    sources = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
               for p in output.iterdir() if p.is_file() and p.suffix in ('.glb', '.dae', '.npz', '.png')}
    (output / 'conversion.json').write_text(json.dumps({
        'blender': bpy.app.version_string, 'height_m': 1.75,
        'cycle_seconds': 2., 'fps': 24, 'frames': 48, 'sha256': sources,
        'source_revision': (output / 'asset_revision.txt').read_text().strip(),
    }, indent=2) + '\n')
    print('Baked shared pedestrian:', len(frames), frames[0].shape, names)


main()
