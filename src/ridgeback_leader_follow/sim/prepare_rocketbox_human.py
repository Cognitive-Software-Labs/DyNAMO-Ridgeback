"""Blender 4.2.9: blender -b --python prepare_rocketbox_human.py -- <cache> <name> <output>.

Bake one Microsoft Rocketbox (MIT) avatar with the publisher's neutral walk and
idle clips into the shared realistic-profile assets:

- walk.npz / idle.npz: evaluated world-space vertex caches for Isaac and scoring;
- walk.dae / idle.dae: one skinned Collada skeleton per clip for Gazebo actors.

The avatar keeps its natural height. It faces +X, its lowest sole over both
clips rests on z=0 and its mean walking pelvis is centred on the origin. The
walk's root translation is removed so the scripted path supplies it; its
recorded speed becomes `gait_speed_mps`, so feet do not slide when a path is
walked at that speed. The idle loop is the clip's first eight seconds with the
final second blended back to its first pose.
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
from ridgeback_leader_follow.rocketbox_cast import IDLE, ROSTER, WALK  # noqa: E402

FPS = 30
IDLE_SECONDS, IDLE_BLEND_SECONDS, IDLE_CACHE_FPS = 8, 1, 10
NAMESPACE = 'http://www.collada.org/2005/11/COLLADASchema'


def import_clip(path):
    """Import a publisher clip and keep its own skeleton for retargeting."""
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=str(path))
    added = [o for o in bpy.data.objects if o not in before]
    rig = next(o for o in added if o.type == 'ARMATURE')
    for obj in added:
        if obj is not rig:
            bpy.data.objects.remove(obj)
    rig.animation_data.action.use_fake_user = True
    return rig, rig.animation_data.action


def leg_length(rig):
    scale = rig.matrix_world.to_scale()[0]
    return sum(rig.data.bones[f'Bip01 L {n}'].length for n in ('Thigh', 'Calf')) * scale


def retarget(scene, clip, avatar, frames):
    """Avatar poses with the clip's world bone orientations.

    Rocketbox clips and avatars share the Biped bone axes but not their rest
    poses or proportions, so local rotations cannot be copied. Each avatar bone
    takes the clip bone's world orientation; positions follow from the avatar's
    own bone offsets, and the pelvis path is scaled by the leg-length ratio.
    Returns (avatar object matrix, armature-space pose per bone) per frame.
    """
    world = avatar.matrix_world.copy()
    scale = world.to_scale()[0]
    unit = lambda m: Matrix.LocRotScale(m.to_translation(), m.to_quaternion(), None)
    rest = {b.name: unit(world @ b.matrix_local) for b in avatar.data.bones}
    order = [b for b in avatar.data.bones if b.parent is None]
    for bone in order:
        order.extend(bone.children)
    ratio = leg_length(avatar) / leg_length(clip)
    poses = []
    for frame in frames:
        scene.frame_set(frame)
        source = {b.name: unit(clip.matrix_world @ b.matrix) for b in clip.pose.bones}
        target = {}
        for bone in order:
            name = bone.name
            if bone.parent is None:
                position = source[name].to_translation() * ratio
            else:
                offset = (rest[bone.parent.name].inverted() @ rest[name]).to_translation()
                position = target[bone.parent.name] @ offset
            if name in source:
                rotation = source[name].to_quaternion()
            else:
                rotation = target[bone.parent.name].to_quaternion() @ (
                    rest[bone.parent.name].inverted() @ rest[name]).to_quaternion()
            target[name] = Matrix.LocRotScale(position, rotation, None)
        inverse = world.inverted()
        poses.append((world, {n: inverse @ m @ Matrix.Scale(scale, 4) for n, m in target.items()}))
    return poses, ratio


def key_bones(rig, base, samples, name):
    """Re-key object-level motion into bones so Gazebo, which animates joints only, sees it."""
    action = bpy.data.actions.new(name)
    action.use_fake_user = True
    rig.animation_data.action = action
    inverse = base.inverted()
    for frame, (world, poses) in enumerate(samples, start=1):
        target = {n: inverse @ world @ m for n, m in poses.items()}
        for bone in rig.pose.bones:
            rest = bone.bone.matrix_local
            if bone.parent is None:
                basis = rest.inverted() @ target[bone.name]
            else:
                relative = bone.parent.bone.matrix_local.inverted() @ rest
                basis = relative.inverted() @ target[bone.parent.name].inverted() @ target[bone.name]
            bone.matrix_basis = basis
            for path in ('location', 'rotation_quaternion', 'scale'):
                bone.keyframe_insert(path, frame=frame)
    return action


def blend_loop(samples, blend):
    """Fade the last `blend` samples into the first pose so the loop has no seam."""
    first_world, first = samples[0]
    out = list(samples[:len(samples) - blend])
    for i, (world, poses) in enumerate(samples[len(samples) - blend:], start=1):
        weight = i / blend
        mixed = {}
        for name, matrix in poses.items():
            a_loc, a_rot, a_scale = matrix.decompose()
            b_loc, b_rot, b_scale = first[name].decompose()
            mixed[name] = Matrix.LocRotScale(a_loc.lerp(b_loc, weight), a_rot.slerp(b_rot, weight),
                                             a_scale.lerp(b_scale, weight))
        a_loc, a_rot, a_scale = world.decompose()
        b_loc, b_rot, b_scale = first_world.decompose()
        out.append((Matrix.LocRotScale(a_loc.lerp(b_loc, weight), a_rot.slerp(b_rot, weight),
                                       a_scale.lerp(b_scale, weight)), mixed))
    return out


def evaluated(scene, mesh_object, frame, normals=False):
    scene.frame_set(frame)
    obj = mesh_object.evaluated_get(bpy.context.evaluated_depsgraph_get())
    data = obj.to_mesh()
    matrix = obj.matrix_world
    points = np.array([matrix @ v.co for v in data.vertices], np.float32)
    result = points
    if normals:
        rotation = matrix.to_3x3().inverted().transposed()
        result = (points, np.array([(rotation @ v.normal).normalized() for v in data.vertices], np.float32))
    obj.to_mesh_clear()
    return result


def export_collada(scene, rig, mesh_object, action, frames, path, opacity):
    rig.animation_data.action = action
    scene.frame_start, scene.frame_end = 1, frames
    bpy.ops.object.select_all(action='DESELECT')
    rig.select_set(True)
    mesh_object.select_set(True)
    bpy.context.view_layer.objects.active = rig
    bpy.ops.wm.collada_export(
        filepath=str(path), selected=True, include_children=True, include_armatures=True,
        include_animations=True, include_all_actions=False, sampling_rate=1, triangulate=True,
        use_texture_copies=False, export_global_up_selection='Z', apply_global_orientation=False)
    ET.register_namespace('', NAMESPACE)
    tree = ET.parse(path)
    q = lambda tag: f'{{{NAMESPACE}}}{tag}'
    # Exporter image IDs ("Map__1") repeat across avatars; Gazebo shares imported
    # resources between meshes, so name each image after its unique texture file.
    aliases = {}
    for image in tree.iter(q('image')):
        init = image.find(q('init_from'))
        init.text = Path(init.text).name
        aliases[image.get('id')] = Path(init.text).stem
    def rename(value):
        for old in sorted(aliases, key=len, reverse=True):
            if value == old or value.startswith(old + '-'):
                return aliases[old] + value[len(old):]
        return value
    for item in tree.iter():
        for key, value in list(item.attrib.items()):
            item.set(key, rename(value))
        if item.tag in (q('init_from'), q('source')) and item.text:
            item.text = rename(item.text)
    masked = 0
    for effect in tree.iter(q('effect')):
        lambert = effect.find(f'.//{q("lambert")}')
        texture = None if lambert is None else lambert.find(f'{q("diffuse")}/{q("texture")}')
        if texture is None or opacity is None or effect.get('id') != opacity + '-effect':
            continue
        transparent = ET.Element(q('transparent'), opaque='A_ONE')
        ET.SubElement(transparent, q('texture'), texture=texture.get('texture'), texcoord=texture.get('texcoord'))
        lambert.insert(list(lambert).index(lambert.find(q('diffuse'))) + 1, transparent)
        masked += 1
    if masked != (opacity is not None):
        raise RuntimeError(f'Expected {int(opacity is not None)} alpha-masked hair/lash material in {path.name}, found {masked}')
    for controller in tree.iter(q('instance_controller')):
        if len(controller.findall(q('skeleton'))) != 1:
            raise RuntimeError('Gazebo skin must have exactly one skeleton root')
    tree.write(path, xml_declaration=True, encoding='utf-8')


def main():
    cache, name, output = sys.argv[sys.argv.index('--') + 1:]
    cache, output = Path(cache).resolve(), Path(output).resolve()
    avatar, sex, description = ROSTER[name]
    assets = cache / 'Assets'
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.fbx(filepath=str(assets / 'Avatars' / avatar / 'Export' / (avatar.split('/')[-1] + '.fbx')))
    scene = bpy.context.scene
    scene.render.fps = FPS
    rig = next(o for o in scene.objects if o.type == 'ARMATURE')
    for obj in [o for o in scene.objects if o.type not in ('ARMATURE', 'MESH')]:
        bpy.data.objects.remove(obj)
    meshes = [o for o in scene.objects if o.type == 'MESH']
    if len(meshes) != 1:
        raise RuntimeError(f'Expected one skinned avatar mesh, found {[o.name for o in meshes]}')
    mesh_object = meshes[0]
    # A few source vertices (e.g. jewellery) carry no skin weights and would stay
    # behind once object-level motion moves into the bones; copy the nearest
    # weighted vertex's weights so they follow the body in both renderers.
    from mathutils.kdtree import KDTree
    vertices = mesh_object.data.vertices
    weighted = [v for v in vertices if any(g.weight > 0 for g in v.groups)]
    unweighted = [v for v in vertices if not any(g.weight > 0 for g in v.groups)]
    if unweighted:
        tree = KDTree(len(weighted))
        for i, v in enumerate(weighted):
            tree.insert(v.co, i)
        tree.balance()
        for v in unweighted:
            _, nearest, _ = tree.find(v.co)
            for group in weighted[nearest].groups:
                mesh_object.vertex_groups[group.group].add([v.index], group.weight, 'REPLACE')
    rebound_vertices = len(unweighted)
    # Gazebo's Collada skinning mishandles a scaled/rotated armature node (the
    # FBX rig is centimetres at 0.01 scale), so bake the object transforms into
    # the rest pose: the exported skeleton then hangs from an identity node.
    rig.animation_data_clear()  # the avatar file's own bind-pose take
    rest_before = evaluated(scene, mesh_object, 1)
    world = mesh_object.matrix_world.copy()
    mesh_object.parent = None
    mesh_object.matrix_world = world
    for obj in (mesh_object, rig):
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    mesh_object.parent = rig
    mesh_object.matrix_parent_inverse = Matrix.Identity(4)
    applied_shift = float(np.abs(evaluated(scene, mesh_object, 1) - rest_before).max())
    if applied_shift > 1e-4 or rig.matrix_world != Matrix.Identity(4):
        raise RuntimeError(f'Applying the rig transform moved the rest mesh by {applied_shift} m')
    roots = [b.name for b in rig.data.bones if b.parent is None]
    if len(roots) != 1:
        raise RuntimeError(f'Expected one skeleton root, found {roots}')
    for bone in rig.pose.bones:
        if bone.bone.inherit_scale != 'FULL' or not bone.bone.use_inherit_rotation:
            raise RuntimeError(f'Unsupported inheritance on {bone.name}')
        bone.rotation_mode = 'QUATERNION'

    # Point materials at the converted textures and describe them for Isaac.
    specs, opacity = [], None
    for slot in mesh_object.material_slots:
        material = slot.material
        spec = {'name': material.name, 'doubleSided': True,
                'pbrMetallicRoughness': {'metallicFactor': 0., 'roughnessFactor': .72}}
        for node in material.node_tree.nodes:
            if node.type != 'TEX_IMAGE' or node.image is None:
                continue
            stem = Path(node.image.filepath).stem
            if stem.endswith('_color'):
                file = output / (stem + ('.png' if '_opacity_' in stem else '.jpg'))
                spec['image'] = file.name
            elif stem.endswith('_normal'):
                file = output / (stem + '.jpg')
                spec['normal_image'] = file.name
            else:
                continue
            if not file.exists():
                raise RuntimeError(f'Converted texture missing: {file}')
            node.image.filepath = str(file)
            node.image.reload()
        if '_opacity' in material.name:
            spec['alphaMode'] = 'MASK'
            opacity = material.name
        if 'image' not in spec:
            raise RuntimeError(f'No colour texture for {material.name}')
        specs.append(spec)

    # Walk: drop the clip root's net translation over the cycle; keep bob and sway.
    walk_rig, walk = import_clip(assets / 'Animations' / (WALK[sex] + '.max.fbx'))
    start, end = (int(round(v)) for v in walk.frame_range)
    walk_frames = end - start
    stride = {}
    for curve in walk.fcurves:
        if curve.data_path == 'location' and curve.array_index in (0, 1):
            first, last = curve.evaluate(start), curve.evaluate(end)
            stride[curve.array_index] = last - first
            for point in curve.keyframe_points:
                fraction = (point.co[0] - start) / walk_frames
                for handle in ('co', 'handle_left', 'handle_right'):
                    getattr(point, handle)[1] -= (last - first) * fraction
    heading = math.atan2(stride.get(1, 0.), stride.get(0, 0.))
    walk_samples, ratio = retarget(scene, walk_rig, rig, range(start, end))
    stride_m = math.hypot(stride.get(0, 0.), stride.get(1, 0.)) * ratio
    idle_rig, idle = import_clip(assets / 'Animations' / (IDLE[sex] + '.max.fbx'))
    idle_start = int(round(idle.frame_range[0]))
    idle_samples, _ = retarget(scene, idle_rig, rig, range(idle_start, idle_start + IDLE_SECONDS * FPS))
    idle_samples = blend_loop(idle_samples, IDLE_BLEND_SECONDS * FPS)
    for clip in (walk_rig, idle_rig):
        bpy.data.objects.remove(clip)

    # Key provisional clips, measure facing/floor/centre, then key the final
    # normalized poses; the rig object keeps only the facing turn (no scale).
    identity = Matrix.Identity(4)
    rig.animation_data_create()
    walk_samples.append(walk_samples[0])
    idle_samples.append(idle_samples[0])
    walk_action = key_bones(rig, identity, walk_samples, f'{name}_walk_provisional')
    idle_action = key_bones(rig, identity, idle_samples, f'{name}_idle_provisional')
    turn = Matrix.Rotation(-heading, 4, 'Z')
    lowest, pelvis = [], []
    for action, count in ((walk_action, walk_frames), (idle_action, len(idle_samples) - 1)):
        rig.animation_data.action = action
        for frame in range(1, count + 1):
            points = evaluated(scene, mesh_object, frame)
            lowest.append(float((points @ np.array(turn.to_3x3()).T)[:, 2].min()))
            if action is walk_action:
                pelvis.append(np.array((turn @ rig.pose.bones[roots[0]].head)[:2]))
    centre = np.mean(pelvis, axis=0)
    normalization = Matrix.Translation((-centre[0], -centre[1], -min(lowest))) @ turn
    for action in (walk_action, idle_action):
        bpy.data.actions.remove(action)
    final = lambda samples: [(world, {n: normalization @ m for n, m in poses.items()}) for world, poses in samples]
    walk_samples, idle_samples = final(walk_samples), final(idle_samples)
    # Gazebo keeps the skeleton root joint's rest orientation for actors, so the
    # facing turn lives on the (unscaled) armature node, as in the earlier skins.
    rig.matrix_world = turn
    walk_action = key_bones(rig, turn, walk_samples, f'{name}_walk')
    idle_action = key_bones(rig, turn, idle_samples, f'{name}_idle')
    idle_samples = idle_samples[:-1]
    deviation = 0.
    rig.animation_data.action = walk_action
    for frame in range(1, walk_frames + 1, 6):
        scene.frame_set(frame)
        for bone in rig.pose.bones:
            expected = walk_samples[frame - 1][1][bone.name].to_translation()
            deviation = max(deviation, ((rig.matrix_world @ bone.matrix).to_translation() - expected).length)

    def cache(action, count, step):
        rig.animation_data.action = action
        frames = [evaluated(scene, mesh_object, f, normals=True) for f in range(1, count + 1, step)]
        return np.stack([f[0] for f in frames]), np.stack([f[1] for f in frames])
    walk_points, walk_normals = cache(walk_action, walk_frames, 1)
    idle_points, idle_normals = cache(idle_action, len(idle_samples), FPS // IDLE_CACHE_FPS)
    loop_gap = float(np.abs(idle_points[0] - evaluated(scene, mesh_object, len(idle_samples) + 1)).max())
    if deviation > 2e-3 or loop_gap > 2e-3:
        raise RuntimeError(f'Keyed skeleton differs from the retarget by {deviation} m; idle loop gap {loop_gap} m')
    rig.animation_data.action = walk_action
    if max(abs(a - b) for r, t in zip(rig.matrix_world, turn) for a, b in zip(r, t)) > 1e-6:
        raise RuntimeError('Rig object must be the unscaled facing turn for Gazebo skinning')

    data = mesh_object.data
    data.calc_loop_triangles()
    faces, uvs, groups = [], [], []
    for tri in data.loop_triangles:
        faces.append(list(tri.vertices))
        groups.append(tri.material_index)
        uvs.extend(tuple(data.uv_layers.active.data[loop].uv) for loop in tri.loops)
    cycle = walk_frames / FPS
    speed = stride_m / cycle
    height = float(idle_points[0][:, 2].max())
    common = dict(faces=np.asarray(faces, np.int32), uv=np.asarray(uvs, np.float32),
                  materials=np.asarray(groups, np.int32))
    np.savez_compressed(output / 'walk.npz', points=walk_points, normals=walk_normals, fps=float(FPS),
                        cycle_seconds=cycle, gait_speed_mps=speed, **common)
    np.savez_compressed(output / 'idle.npz', points=idle_points, normals=idle_normals, fps=float(IDLE_CACHE_FPS),
                        cycle_seconds=float(IDLE_SECONDS), **common)
    (output / 'materials.json').write_text(json.dumps(specs, indent=2) + '\n')
    export_collada(scene, rig, mesh_object, walk_action, walk_frames + 1, output / 'walk.dae', opacity)
    export_collada(scene, rig, mesh_object, idle_action, len(idle_samples) + 1, output / 'idle.dae', opacity)
    receipt = {'source_avatar': avatar, 'description': description, 'animation_set': sex,
               'walk_clip': WALK[sex], 'idle_clip': IDLE[sex], 'blender': bpy.app.version_string,
               'fps': FPS, 'walk_frames': walk_frames, 'cycle_seconds': cycle,
               'stride_m': stride_m, 'gait_speed_mps': speed, 'height_m': height,
               'idle_seconds': IDLE_SECONDS, 'idle_blend_seconds': IDLE_BLEND_SECONDS,
               'idle_cache_fps': IDLE_CACHE_FPS, 'vertices': int(walk_points.shape[1]), 'triangles': len(faces),
               'max_rekey_deviation_m': deviation, 'leg_length_ratio': ratio, 'rebound_unweighted_vertices': rebound_vertices, 'idle_loop_gap_m': loop_gap,
               'sha256': {p: hashlib.sha256((output / p).read_bytes()).hexdigest()
                          for p in ('walk.npz', 'idle.npz', 'walk.dae', 'idle.dae', 'materials.json')}}
    (output / 'conversion.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('Baked Rocketbox pedestrian', name, receipt, flush=True)


if __name__ == '__main__':
    main()
