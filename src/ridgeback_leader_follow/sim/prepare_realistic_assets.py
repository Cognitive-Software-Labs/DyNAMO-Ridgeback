"""Blender 4.2.9 asset conversion: -- <asset-directory> [human]."""
import hashlib,json,sys
from pathlib import Path
import bpy
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))


def remove_loose_edges(directory):
    """Drop wire-only edges: Gazebo draws Collada <lines> as unlit white strokes."""
    from xml.etree import ElementTree as ET
    namespace='http://www.collada.org/2005/11/COLLADASchema';ns={'c':namespace}
    ET.register_namespace('',namespace);tree=ET.parse(directory/'model.dae');removed=0
    for mesh in tree.findall('.//c:mesh',ns):
        for primitive in mesh.findall('c:lines',ns)+mesh.findall('c:linestrips',ns):
            removed+=int(primitive.get('count',0));mesh.remove(primitive)
    tree.write(directory/'model.dae',encoding='utf-8',xml_declaration=True)
    return removed


def export_opacity(directory):
    """Keep the publisher's separate grayscale mask in Collada without repacking pixels."""
    from xml.etree import ElementTree as ET
    masks=directory/'opacity_maps.json'
    if not masks.exists():return
    path=json.loads(masks.read_text()).get('all')
    if path is None:return
    namespace='http://www.collada.org/2005/11/COLLADASchema';ns={'c':namespace}
    ET.register_namespace('',namespace);tree=ET.parse(directory/'model.dae');root=tree.getroot()
    images=root.find('c:library_images',ns)
    if images is None:images=ET.SubElement(root,'{'+namespace+'}library_images')
    image=ET.SubElement(images,'{'+namespace+'}image',id='opacity_source')
    ET.SubElement(image,'{'+namespace+'}init_from').text=path
    for effect in root.findall('c:library_effects/c:effect',ns):
        profile=effect.find('c:profile_COMMON',ns)
        for tag,sid in [('surface','opacity-surface'),('sampler2D','opacity-sampler')]:
            param=ET.SubElement(profile,'{'+namespace+'}newparam',sid=sid)
            node=ET.SubElement(param,'{'+namespace+'}'+tag,**({'type':'2D'} if tag=='surface' else {}))
            ET.SubElement(node,'{'+namespace+'}'+('init_from' if tag=='surface' else 'source')).text='opacity_source' if tag=='surface' else 'opacity-surface'
        shader=profile.find('c:technique/c:lambert',ns)
        if shader is None:shader=profile.find('c:technique/c:phong',ns)
        if shader is None:continue
        for old in shader.findall('c:transparent',ns):shader.remove(old)
        transparent=ET.SubElement(shader,'{'+namespace+'}transparent',opaque='RGB_ONE')
        ET.SubElement(transparent,'{'+namespace+'}texture',texture='opacity-sampler',texcoord='UVMap')
        extra=ET.SubElement(effect,'{'+namespace+'}extra');technique=ET.SubElement(extra,'{'+namespace+'}technique',profile='GOOGLEEARTH')
        ET.SubElement(technique,'{'+namespace+'}double_sided').text='1'
    tree.write(directory/'model.dae',encoding='utf-8',xml_declaration=True)


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
    source=next(p for p in directory.glob('*.gltf') if not p.name.startswith('.'));gltf=json.loads(source.read_text())
    for i,spec in enumerate(gltf['materials']):spec['name']=spec.get('name','Material')+'_material_'+str(i)
    temporary=directory/'.conversion-source.gltf';temporary.write_text(json.dumps(gltf))
    bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
    try:bpy.ops.import_scene.gltf(filepath=str(temporary))
    finally:temporary.unlink(missing_ok=True)
    meshes=[o for o in bpy.context.scene.objects if o.type=='MESH']
    source_triangles=sum(len(obj.data.polygons) for obj in meshes)
    if source_triangles>250000:
        for obj in meshes:
            bpy.context.view_layer.objects.active=obj
            modifier=obj.modifiers.new('SimulationLOD','DECIMATE');modifier.ratio=180000/source_triangles
            modifier.use_collapse_triangulate=True
            bpy.ops.object.modifier_apply(modifier=modifier.name)
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
    for obj in meshes:
        matrix=obj.matrix_world.copy();matrix.translation-=__import__('mathutils').Vector(center)
        obj.matrix_world=matrix
    bpy.context.view_layer.update()
    exported=np.asarray([obj.matrix_world@v.co for obj in meshes for v in obj.data.vertices],np.float32)
    deviation=float(np.max(np.abs(exported-points)))
    if deviation>1e-4:raise RuntimeError(f'Static Collada pose differs from the mesh cache by {deviation} m')
    np.savez_compressed(directory/'mesh.npz',points=points,normals=np.asarray(normals,np.float32),faces=np.asarray(faces,np.int32),uv=np.asarray(uvs,np.float32),materials=np.asarray(groups,np.int32))
    (directory/'materials.json').write_text(json.dumps(specs,indent=2)+'\n')
    bpy.ops.wm.collada_export(filepath=str(directory/'model.dae'),include_animations=False,triangulate=True,use_texture_copies=True,export_global_up_selection='Z',apply_global_orientation=False)
    export_opacity(directory)
    loose_edges=remove_loose_edges(directory)
    receipt={'source':source.name,'max_export_pose_deviation_m':deviation,'source_polygon_count':source_triangles,'triangle_budget':180000 if source_triangles>250000 else None,'blender':bpy.app.version_string,'floor_center_translation':center.tolist(),'dimensions_m':(maximum-minimum).tolist(),'vertices':len(points),'triangles':len(faces),'removed_loose_edges':loose_edges,'sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [directory/'mesh.npz',directory/'model.dae',directory/'materials.json']}}
    (directory/'conversion.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('Prepared static asset',directory.name,receipt,flush=True)


if __name__=='__main__':main()
