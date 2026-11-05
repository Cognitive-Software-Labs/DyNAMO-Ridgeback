"""Blender 4.2.9 asset conversion: -- <asset-directory> [human]."""
import hashlib,json,sys
from pathlib import Path
import bpy
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))


def main():
    args=sys.argv[sys.argv.index('--')+1:];directory=Path(args[0]).resolve()
    if len(args)>1 and args[1]=='human':
        import prepare_lab_human as bake
        sys.argv=[sys.argv[0],'--',str(directory)];bake.main()
        bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
        bpy.ops.outliner.orphans_purge(do_recursive=True)
        import prepare_f3_gazebo_human as export
        specs=json.loads((directory/'materials.json').read_text())
        export.CANONICAL={m['name']:Path(m['image']).stem for m in specs if 'image' in m}
        export.ALPHA_MASKED=tuple(sorted({export.CANONICAL[m['name']] for m in specs if m.get('alphaMode')=='MASK' and m['name'] in export.CANONICAL}))
        sys.argv=[sys.argv[0],'--',str(directory),str(directory)];export.main()
        return
    source=next(directory.glob('*.gltf'));gltf=json.loads(source.read_text())
    bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(source))
    meshes=[o for o in bpy.context.scene.objects if o.type=='MESH']
    specs=gltf['materials']
    for i,spec in enumerate(specs):
        spec.setdefault('name',f'Material_{i}')
        for field,name in [('baseColorTexture','image')]:
            tex=spec.get('pbrMetallicRoughness',{}).get(field)
            if tex is not None:spec[name]=gltf['images'][gltf['textures'][tex['index']]['source']]['uri']
        if 'normalTexture' in spec:spec['normal_image']=gltf['images'][gltf['textures'][spec['normalTexture']['index']]['source']]['uri']
    ids={spec['name']:i for i,spec in enumerate(specs)}
    points=[];normals=[];faces=[];uvs=[];groups=[];offset=0
    for obj in meshes:
        mesh=obj.data;mesh.calc_loop_triangles();matrix=obj.matrix_world
        normal_matrix=matrix.to_3x3().inverted().transposed()
        points.extend(matrix@v.co for v in mesh.vertices)
        normals.extend((normal_matrix@v.normal).normalized() for v in mesh.vertices)
        for tri in mesh.loop_triangles:
            faces.append([v+offset for v in tri.vertices]);groups.append(ids[obj.material_slots[tri.material_index].material.name])
            for loop in tri.loops:
                u,v=mesh.uv_layers.active.data[loop].uv if mesh.uv_layers.active else (0.,0.)
                uvs.append((u,v))
        offset+=len(mesh.vertices)
    points=np.asarray(points,np.float32)
    minimum,maximum=points.min(axis=0),points.max(axis=0)
    center=np.array([(minimum[0]+maximum[0])/2,(minimum[1]+maximum[1])/2,minimum[2]])
    points-=center
    for obj in meshes:obj.location-=__import__('mathutils').Vector(center)
    np.savez_compressed(directory/'mesh.npz',points=points,normals=np.asarray(normals,np.float32),faces=np.asarray(faces,np.int32),uv=np.asarray(uvs,np.float32),materials=np.asarray(groups,np.int32))
    (directory/'materials.json').write_text(json.dumps(specs,indent=2)+'\n')
    bpy.ops.wm.collada_export(filepath=str(directory/'model.dae'),include_animations=False,triangulate=True,use_texture_copies=True,export_global_up_selection='Z',apply_global_orientation=False)
    receipt={'source':source.name,'blender':bpy.app.version_string,'floor_center_translation':center.tolist(),'dimensions_m':(maximum-minimum).tolist(),'vertices':len(points),'triangles':len(faces),'sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [directory/'mesh.npz',directory/'model.dae',directory/'materials.json']}}
    (directory/'conversion.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('Prepared static asset',directory.name,receipt,flush=True)


if __name__=='__main__':main()
