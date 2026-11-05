"""Blender 4.2.9: blender -b --python prepare_f3_gazebo_human.py -- <maya-dir> <maya_f3-dir>.

Export the CC0 Maya pedestrian as a Gazebo Collada skin and walk with the
same +X facing, 1.72 m height and floor seating as the Isaac vertex cache.
Textures are referenced by canonical names; the F3 scene links each name to
the shared Maya texture or the outfit's recoloured texture at run time.
"""
import hashlib
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import bpy
from mathutils import Matrix
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ridgeback_leader_follow.f3_scene import CANONICAL  # noqa: E402

FRAMES = 48
# Source materials with alphaMode MASK. Blender's exporter drops their alpha;
# gz-common reads <transparent opaque="A_ONE"> with a texture as an alpha cutout.
ALPHA_MASKED = ('eyebrow', 'eyelashes', 'ponytail')


def skinned_meshes(scene):
    # Same selection and order as prepare_lab_human.py, so vertices line up.
    return [o for o in scene.objects if o.type == 'MESH' and len(o.material_slots)]


def evaluated_points(scene, objects, frame):
    scene.frame_set(frame)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    points = []
    for obj in objects:
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        points.extend([evaluated.matrix_world @ v.co for v in mesh.vertices])
        evaluated.to_mesh_clear()
    return np.asarray(points, dtype=np.float64)


def single_root(scene, rig):
    """Parent extra root bones (the IK feet) under their calves, keeping the walk.

    gz-common merges several skeleton roots but does not animate the extra ones,
    which leaves the shoes at the rest pose. Armature-space poses are recorded
    per frame and re-keyed as local transforms after re-parenting.
    """
    roots = [b for b in rig.data.bones if b.parent is None]
    if len(roots) == 1:
        return []
    main = max(roots, key=lambda b: len(b.children_recursive)).name
    moved = [b.name for b in roots if b.name != main]
    poses = []
    for frame in range(FRAMES + 1):
        scene.frame_set(frame)
        poses.append({pb.name: pb.matrix.copy() for pb in rig.pose.bones})
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='EDIT')
    edit = rig.data.edit_bones
    for name in moved:
        parent = name.replace('foot', 'calf')
        edit[name].parent = edit[parent if parent in edit else main]
        edit[name].use_connect = False
    bpy.ops.object.mode_set(mode='OBJECT')
    action = rig.animation_data.action
    for curve in list(action.fcurves):
        action.fcurves.remove(curve)
    for bone in rig.pose.bones:
        if bone.bone.inherit_scale != 'FULL' or not bone.bone.use_inherit_rotation:
            raise RuntimeError(f'Unsupported inheritance on {bone.name}')
        bone.rotation_mode = 'QUATERNION'
    for frame, pose in enumerate(poses):
        for bone in rig.pose.bones:
            rest = bone.bone.matrix_local
            if bone.parent is None:
                basis = rest.inverted() @ pose[bone.name]
            else:
                relative = bone.parent.bone.matrix_local.inverted() @ rest
                basis = relative.inverted() @ pose[bone.parent.name].inverted() @ pose[bone.name]
            bone.matrix_basis = basis
            for path in ('location', 'rotation_quaternion', 'scale'):
                bone.keyframe_insert(path, frame=frame)
    return moved


def main():
    arguments = sys.argv[sys.argv.index('--') + 1:]
    source, output = (Path(a).resolve() for a in arguments[:2])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(source / 'source.glb'))
    scene = bpy.context.scene
    scene.render.fps = 24
    rig = next(o for o in scene.objects if o.type == 'ARMATURE')
    root = rig.parent
    for obj in list(scene.objects):
        if obj not in (root, rig) and obj.parent is not rig:
            bpy.data.objects.remove(obj)  # unbound helper geometry, e.g. Icosphere
    objects = skinned_meshes(scene)
    # Rotate to +X, then seat and scale exactly as the Isaac bake does.
    rotation = Matrix.Rotation(math.pi / 2, 4, 'Z')
    root.matrix_world = rotation @ root.matrix_world
    heights = np.concatenate([evaluated_points(scene, objects, f)[:, 2] for f in range(FRAMES)])
    floor, top = float(heights.min()), float(heights.max())
    scale = 1.72 / (top - floor)
    normalization = Matrix.Translation((0, 0, -floor * scale)) @ Matrix.Scale(scale, 4)
    root.matrix_world = normalization @ root.matrix_world
    cache = np.load(source / 'walk.npz')['points']
    deviation = max(float(np.abs(evaluated_points(scene, objects, f) - cache[f]).max())
                    for f in range(0, FRAMES, 6))
    if deviation > 1e-3:
        raise RuntimeError(f'Collada pose differs from the Isaac cache by {deviation} m')
    # Gazebo leaves skins whose vertices all have one bone weight (shoes, teeth,
    # brows) in the rest pose. One joined skin animates every vertex; the world
    # geometry is unchanged: each vertex carries its Isaac-cache index through the join.
    offset = 0
    for obj in objects:
        count = len(obj.data.vertices)
        attribute = obj.data.attributes.new('isaac_cache_index', 'INT', 'POINT')
        attribute.data.foreach_set('value', list(range(offset, offset + count)))
        offset += count
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = max(objects, key=lambda o: len(o.data.vertices))
    bpy.ops.object.join()
    objects = skinned_meshes(scene)
    reparented = single_root(scene, rig)
    order =np.zeros(len(objects[0].data.vertices), dtype=np.int64)
    objects[0].data.attributes['isaac_cache_index'].data.foreach_get('value', order)
    objects[0].data.attributes.remove(objects[0].data.attributes['isaac_cache_index'])
    joined_deviation = max(float(np.abs(evaluated_points(scene, objects, f) - cache[f][order]).max())
                           for f in range(0, FRAMES, 6))
    if len(objects) != 1 or joined_deviation > 1e-3:
        raise RuntimeError(f'Joined skin differs from the Isaac cache by {joined_deviation} m')
    for material in bpy.data.materials:
        name = CANONICAL.get(material.name)
        if name is None or not material.use_nodes:
            continue
        for node in material.node_tree.nodes:
            if node.type == 'TEX_IMAGE' and node.image is not None and any(
                    link.to_socket.name == 'Base Color' for link in node.outputs['Color'].links):
                node.image.name = name
                node.image.filepath_raw = f'//{name}.png'
                node.image.file_format = 'PNG'
    scene.frame_start, scene.frame_end = 0, FRAMES
    bpy.ops.object.select_all(action='DESELECT')
    for obj in (root, rig, *objects):
        obj.select_set(True)
    bpy.context.view_layer.objects.active = rig
    staging = output / '.collada_staging'
    staging.mkdir(exist_ok=True)
    bpy.ops.wm.collada_export(
        filepath=str(staging / 'walk.dae'), selected=True, include_children=True,
        include_armatures=True, include_animations=True, include_all_actions=False,
        sampling_rate=1, triangulate=True, use_texture_copies=False,
        export_global_up_selection='Z', apply_global_orientation=False)
    # Textures are linked per outfit at run time; keep only bare canonical names.
    namespace = 'http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('', namespace)
    tree = ET.parse(staging / 'walk.dae')
    referenced = []
    for init in tree.iter(f'{{{namespace}}}init_from'):
        if init.text and init.text.endswith('.png'):
            init.text = Path(init.text).name
            referenced.append(init.text)
    masked = []
    for lambert in tree.iter(f'{{{namespace}}}lambert'):
        texture = lambert.find(f'{{{namespace}}}diffuse/{{{namespace}}}texture')
        if texture is None or texture.get('texture').removesuffix('-sampler') not in ALPHA_MASKED:
            continue
        transparent = ET.Element(f'{{{namespace}}}transparent', opaque='A_ONE')
        ET.SubElement(transparent, f'{{{namespace}}}texture', texture=texture.get('texture'),
                      texcoord=texture.get('texcoord'))
        children = list(lambert)
        lambert.insert(children.index(lambert.find(f'{{{namespace}}}diffuse')) + 1, transparent)
        masked.append(texture.get('texture').removesuffix('-sampler'))
    if sorted(masked) != sorted(ALPHA_MASKED):
        raise RuntimeError(f'Alpha-masked materials not found: {masked}')
    for controller in tree.iter(f'{{{namespace}}}instance_controller'):
        if len(controller.findall(f'{{{namespace}}}skeleton')) != 1:
            raise RuntimeError('Gazebo skin must have exactly one skeleton root')
    tree.write(output / 'walk.dae', xml_declaration=True, encoding='utf-8')
    for path in staging.iterdir():
        path.unlink()
    staging.rmdir()
    missing = {f'{n}.png' for n in CANONICAL.values()} - set(referenced)
    if missing:
        raise RuntimeError(f'Collada export lost textures: {sorted(missing)}')
    receipt = {'blender': bpy.app.version_string, 'height_m': 1.72, 'fps': 24,
               'animation_frames': FRAMES, 'cycle_seconds': FRAMES / 24,
               'max_deviation_from_isaac_cache_m': deviation,
               'joined_skin_max_deviation_m': joined_deviation,
               'reparented_root_bones': reparented,
               'textures': sorted(set(referenced)), 'alpha_masked': sorted(masked),
               'sha256': {'walk.dae': hashlib.sha256((output / 'walk.dae').read_bytes()).hexdigest()}}
    (output / 'collada.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('Exported F3 Gazebo pedestrian:', receipt)


if __name__ == '__main__':
    main()
