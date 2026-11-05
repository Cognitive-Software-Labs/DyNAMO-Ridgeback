"""Textured worlds and a varied pedestrian cast, shared by both simulators."""
from dataclasses import dataclass
import json
from pathlib import Path
import re
from xml.etree import ElementTree as ET

import numpy as np
from .realistic_layout import ENVIRONMENTS,PALETTE,layout


@dataclass(frozen=True)
class Pedestrian:
    model: str
    scale: float
    clothing: tuple


CAST=(Pedestrian('sam',1.05,(.06,.18,.48)),Pedestrian('harper',.96,(.42,.09,.08)),
      Pedestrian('maya',1.,(.6,.7,1.)),Pedestrian('jordan',1.03,(.15,.18,.2)),
      Pedestrian('sam',.98,(.18,.36,.14)),Pedestrian('harper',1.02,(.42,.3,.1)),
      Pedestrian('jordan',.97,(.12,.24,.34)),Pedestrian('maya',1.04,(.7,1.,.65)),
      Pedestrian('sam',1.,(.6,.22,.08)),Pedestrian('harper',1.,(.25,.13,.3)))


def asset_directory():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('ridgeback_leader_follow'))/'sim/assets/realistic'


def human_directory(model):
    return asset_directory().parent/'maya' if model=='maya' else asset_directory()/'humans'/model


def assign_people(scenario):
    assigned={};index=0
    for person in scenario.people:
        if person.name=='lf_lookalike':
            assigned[person.name]=assigned['lf_leader']
        else:
            assigned[person.name]=CAST[index%len(CAST)];index+=1
    return assigned


def load_scenario(name,crowd_size=4):
    """Keep six scenario families; the realistic alone route also includes turns."""
    from dataclasses import replace
    from .crowd_scenarios import load_scenario as baseline,Waypoint
    scenario=baseline(name,crowd_size)
    if name=='leader_alone':
        leader=replace(scenario.people[0],waypoints=tuple(Waypoint(*p) for p in ((0,2,0),(6,4,0),(12,6,1.5),(16,8,1.5),(20,9,0))))
        scenario=replace(scenario,people=(leader,))
    return scenario


def manifest_entry(scenario,environment):
    from dataclasses import asdict
    return {'environment':environment,'geometry':'Textured architectural scene, CC0 Poly Haven props and Innerscene pedestrians',
            'appearance':{name:asdict(spec) for name,spec in assign_people(scenario).items()},
            'qualification':'new visual profile; original F1/F2/F3 results do not qualify this population'}


def _clothing(name):
    return any(word in name.lower() for word in ('original top','original bottom','business fabric','plum satin','tailored trousers'))


def box_mesh_data(size,repeat=True):
    """UVs run horizontally around walls and in metres on textured surfaces."""
    vertices=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],np.float32)*np.array(size)/2
    quads=((0,3,2,1),(4,5,6,7),(0,1,5,4),(2,3,7,6),(3,0,4,7),(1,2,6,5))
    points=[];uv=[];normals=[];faces=[]
    for quad in quads:
        a,b,c,d=vertices[list(quad)];normal=np.cross(b-a,c-a);normal/=np.linalg.norm(normal)
        w,h=(np.linalg.norm(b-a)/2,np.linalg.norm(d-a)/2) if repeat else (1.,1.)
        for triangle,coords in [((a,b,c),((0,0),(w,0),(w,h))),((a,c,d),((0,0),(w,h),(0,h)))]:
            offset=len(points);points.extend(triangle);normals.extend([normal]*3);faces.append([offset,offset+1,offset+2]);uv.extend(coords)
    return {'faces':np.array(faces),'normals':np.array(normals),'uv':np.array(uv)},np.array(points)


def write_fixture_mesh(box,path):
    """Small textured Collada mesh with the same UVs as the Isaac fixture."""
    data,points=box_mesh_data(box.size,repeat=not box.material.startswith('sign_'))
    root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'unit',name='meter',meter='1');ET.SubElement(asset,'up_axis').text='Z_UP'
    texture=asset_directory()/'surfaces'/box.material/'Diffuse.jpg'
    effect=ET.SubElement(ET.SubElement(root,'library_effects'),'effect',id='finish-effect');profile=ET.SubElement(effect,'profile_COMMON')
    if texture.exists():
        image=ET.SubElement(ET.SubElement(root,'library_images'),'image',id='albedo');ET.SubElement(image,'init_from').text=texture.as_uri()
        param=ET.SubElement(profile,'newparam',sid='albedo-surface');surface=ET.SubElement(param,'surface',type='2D');ET.SubElement(surface,'init_from').text='albedo'
        param=ET.SubElement(profile,'newparam',sid='albedo-sampler');ET.SubElement(ET.SubElement(param,'sampler2D'),'source').text='albedo-surface'
    diffuse=ET.SubElement(ET.SubElement(ET.SubElement(profile,'technique',sid='common'),'lambert'),'diffuse')
    if texture.exists():ET.SubElement(diffuse,'texture',texture='albedo-sampler',texcoord='UVMap')
    else:ET.SubElement(diffuse,'color').text=' '.join(map(str,(*PALETTE[box.material][0],1.)))
    mat=ET.SubElement(ET.SubElement(root,'library_materials'),'material',id='finish');ET.SubElement(mat,'instance_effect',url='#finish-effect')
    geometry=ET.SubElement(ET.SubElement(root,'library_geometries'),'geometry',id='shape');mesh=ET.SubElement(geometry,'mesh')
    for name,values,params in [('positions',points,('X','Y','Z')),('normals',data['normals'],('X','Y','Z')),('uv',data['uv'],('S','T'))]:
        source=ET.SubElement(mesh,'source',id=name);ET.SubElement(source,'float_array',id=name+'-array',count=str(values.size)).text=' '.join(f'{v:.7g}' for v in values.flat)
        accessor=ET.SubElement(ET.SubElement(source,'technique_common'),'accessor',source='#'+name+'-array',count=str(len(values)),stride=str(len(params)))
        for field in params:ET.SubElement(accessor,'param',name=field,type='float')
    vertices=ET.SubElement(mesh,'vertices',id='vertices');ET.SubElement(vertices,'input',semantic='POSITION',source='#positions')
    triangles=ET.SubElement(mesh,'triangles',material='finish',count=str(len(data['faces'])))
    for offset,(semantic,source) in enumerate([('VERTEX','#vertices'),('NORMAL','#normals'),('TEXCOORD','#uv')]):
        attrib={'semantic':semantic,'source':source,'offset':str(offset)}
        if semantic=='TEXCOORD':attrib['set']='0'
        ET.SubElement(triangles,'input',**attrib)
    ET.SubElement(triangles,'p').text=' '.join(str(i) for face in data['faces'] for i in face for _ in range(3))
    node=ET.SubElement(ET.SubElement(ET.SubElement(root,'library_visual_scenes'),'visual_scene',id='Scene'),'node',id='Fixture')
    instance=ET.SubElement(node,'instance_geometry',url='#shape');binding=ET.SubElement(ET.SubElement(ET.SubElement(instance,'bind_material'),'technique_common'),'instance_material',symbol='finish',target='#finish')
    ET.SubElement(binding,'bind_vertex_input',semantic='UVMap',input_semantic='TEXCOORD',input_set='0')
    ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#Scene')
    path.parent.mkdir(parents=True,exist_ok=True);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return path


def clothing_color(name,color):
    # Casual trousers keep their neutral fabric rather than matching every shirt.
    return (.055,.065,.09) if 'original bottom' in name.lower().replace('_',' ') else color


def create_scene(stage,scenario,environment='office'):
    from pxr import Gf,Sdf,UsdGeom,UsdLux,UsdPhysics,UsdShade
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    root=asset_directory();materials={};meshes={}
    def material(name,color=(.7,.7,.7),rough=.65,metal=0.,image=None,normal=None,masked=False,tint=None):
        key=(name,tuple(color),image,normal,masked,tint)
        if key in materials:return materials[key]
        path='/Realistic/Materials/m'+str(len(materials));mat=UsdShade.Material.Define(stage,path)
        shader=UsdShade.Shader.Define(stage,path+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough)
        shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
        if name=='glass':shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.28)
        uv=UsdShade.Shader.Define(stage,path+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2')
        uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
        for field,file,color_space in [('Color',image,'sRGB'),('Normal',normal,'raw')]:
            if file is None:continue
            tex=UsdShade.Shader.Define(stage,path+'/'+field);tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(file)))
            tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set(color_space)
            tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
            tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
            if field=='Color':
                if tint is not None:tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*tint,1.))
                shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
                if masked:
                    shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'a')
                    shader.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(.5)
            else:
                tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2,2,2,1))
                tex.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1,-1,-1,0))
                shader.CreateInput('normal',Sdf.ValueTypeNames.Normal3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');materials[key]=mat;return mat
    def surface(name):
        color,rough,metal=PALETTE[name];directory=root/'surfaces'/name
        return material(name,color,rough,metal,directory/'Diffuse.jpg' if directory.exists() else None,directory/'nor_gl.jpg' if (directory/'nor_gl.jpg').exists() else None)
    def mesh(path,data,points):
        obj=UsdGeom.Mesh.Define(stage,path);obj.CreatePointsAttr(points)
        obj.CreateFaceVertexCountsAttr([3]*len(data['faces']));obj.CreateFaceVertexIndicesAttr(data['faces'].reshape(-1))
        obj.CreateNormalsAttr(data['normals'][0] if data['normals'].ndim==3 else data['normals']);obj.SetNormalsInterpolation('vertex')
        obj.CreateSubdivisionSchemeAttr('none');obj.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(obj).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying').Set(data['uv'])
        return obj
    def bind_asset(obj,data,directory,tint=None):
        specs=json.loads((directory/'materials.json').read_text())
        for i,spec in enumerate(specs):
            pbr=spec.get('pbrMetallicRoughness',{});color=pbr.get('baseColorFactor',[.7,.7,.7,1])[:3]
            clothing=tint if tint is not None and _clothing(spec.get('name','')) else None
            if clothing is not None and 'image' not in spec:color=clothing_color(spec.get('name',''),clothing)
            mat=material(str(directory)+str(i),color,pbr.get('roughnessFactor',.65),pbr.get('metallicFactor',0.),directory/spec['image'] if 'image' in spec else None,directory/spec['normal_image'] if 'normal_image' in spec else None,spec.get('alphaMode')=='MASK',clothing)
            faces=np.flatnonzero(data['materials']==i).tolist()
            if not faces:continue
            subset=UsdGeom.Subset.CreateGeomSubset(obj,'m'+str(i),'face',faces,'materialBind')
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
        UsdGeom.Subset.SetFamilyType(obj,'materialBind','partition')
    boxes,props=layout(environment)
    for box in boxes:
        # Explicit UVs in metres on every face; built-in cubes have no UV map.
        data,points=box_mesh_data(box.size,repeat=not box.material.startswith('sign_'))
        obj=mesh('/Realistic/Fixtures/'+box.name,data,points);obj.AddTranslateOp().Set(Gf.Vec3d(*box.position))
        if box.yaw:obj.AddRotateZOp().Set(box.yaw)
        mat=surface(box.material)
        if box.name.startswith('LightHousing'):
            mat=material('luminaire',(.95,.92,.84),.4)
            UsdShade.Shader.Get(stage,str(mat.GetPath())+'/Surface').CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(3.,2.85,2.55))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
        if box.collision:UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    for prop in props:
        directory=root/'props'/prop.asset
        if prop.asset not in meshes:
            with np.load(directory/'mesh.npz') as source:meshes[prop.asset]={k:source[k] for k in source.files}
        data=meshes[prop.asset];obj=mesh('/Realistic/Props/'+prop.name,data,data['points'])
        obj.AddTranslateOp().Set(Gf.Vec3d(*prop.position));obj.AddRotateZOp().Set(prop.yaw);obj.AddScaleOp().Set(Gf.Vec3f(prop.scale))
        bind_asset(obj,data,directory)
        if prop.asset=='tree_small_02':
            center=json.loads((directory/'conversion.json').read_text())['floor_center_translation']
            collider=UsdGeom.Cylinder.Define(stage,str(obj.GetPath())+'/TrunkCollider')
            collider.CreateRadiusAttr(.12);collider.CreateHeightAttr(2.)
            collider.AddTranslateOp().Set(Gf.Vec3d(-center[0],-center[1],1.))
            collider.MakeInvisible();UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
        else:UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    dome=UsdLux.DomeLight.Define(stage,'/Realistic/Lighting/Daylight');dome.CreateIntensityAttr(750. if environment!='courtyard' else 900.)
    dome.CreateTextureFileAttr(Sdf.AssetPath(str(root/'daylight.hdr')))
    dome.CreateTextureFormatAttr('latlong')
    dome.GetPrim().CreateAttribute('visibleInPrimaryRay',Sdf.ValueTypeNames.Bool).Set(True)
    sun=UsdLux.DistantLight.Define(stage,'/Realistic/Lighting/Sun');sun.AddRotateXYZOp().Set(Gf.Vec3f(315,25,25));sun.CreateIntensityAttr(1300. if environment=='courtyard' else 600.);sun.CreateAngleAttr(1.)
    if environment!='courtyard':
        for i,x in enumerate((-2,3,8,13)):
            light=UsdLux.RectLight.Define(stage,f'/Realistic/Lighting/Panel{i}');light.AddTranslateOp().Set(Gf.Vec3d(x,0,3.7 if environment=='office' else 4.7))
            light.CreateWidthAttr(2.2);light.CreateHeightAttr(.6);light.CreateIntensityAttr(18000. if environment=='office' else 15000.);light.CreateColorAttr(Gf.Vec3f(1.,.94,.85))
    if environment=='office':
        for i,x in enumerate((0,6,12)):
            light=UsdLux.RectLight.Define(stage,f'/Realistic/Lighting/WindowFill{i}')
            light.AddTranslateOp().Set(Gf.Vec3d(x,-5.8,2.8));light.AddRotateXYZOp().Set(Gf.Vec3f(70,0,0))
            light.CreateWidthAttr(3.);light.CreateHeightAttr(2.);light.CreateIntensityAttr(4000.)
            light.CreateColorAttr(Gf.Vec3f(.88,.94,1.))
    roots={}
    for person in scenario.people:
        spec=assign_people(scenario)[person.name];directory=human_directory(spec.model)
        if spec.model not in meshes:
            with np.load(directory/'walk.npz') as source:meshes[spec.model]={k:source[k] for k in source.files}
        data=dict(meshes[spec.model]);data['points']=data['points']*spec.scale;data['gait_speed_mps']=float(data['gait_speed_mps'])*spec.scale
        node=UsdGeom.Xform.Define(stage,'/People/'+person.name);translate=node.AddTranslateOp();rotate=node.AddRotateZOp()
        obj=mesh(str(node.GetPath())+'/human',data,data['points'][0]);bind_asset(obj,data,directory,spec.clothing if spec.model!="maya" else None)
        roots[person.name]=(node,translate,rotate,obj,data)
    return roots


def gazebo_world(scenario,output,environment):
    """Same layout and cast, native Gazebo actor animation and textured meshes."""
    from .crowd_geometry import gazebo_world as baseline
    root=asset_directory();skins={};cast=assign_people(scenario)
    for person in scenario.people:
        spec=cast[person.name];directory=human_directory(spec.model)
        if spec.model=='maya':
            # Existing Maya skin/texture export; this branch uses its original outfit.
            original=root.parent/'maya_f3/walk.dae'
            text=original.read_text()
            from .f3_scene import SOURCE_IMAGES
            for name,image in SOURCE_IMAGES.items():
                path=human_directory("maya")/image
                text=text.replace(f'<init_from>{name}.png</init_from>',f'<init_from>{path.as_uri()}</init_from>')
        else:
            text=(directory/'walk.dae').read_text()
            for file in directory.glob('image_*.png'):text=text.replace(f'<init_from>{file.name}</init_from>',f'<init_from>{file.as_uri()}</init_from>')
            ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};ET.register_namespace('',ns['c']);tree=ET.fromstring(text)
            for effect in tree.findall('.//c:effect',ns):
                if any(word in effect.get('id','').lower() for word in ('original_top','original_bottom','business_fabric')):
                    color=effect.find('.//c:diffuse/c:color',ns)
                    if color is not None:color.text=' '.join(map(str,(*clothing_color(effect.get('id',''),spec.clothing),1)))
            text=ET.tostring(tree,encoding='unicode')
        path=output/(person.name+'.dae');path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text);skins[person.name]=path
    tree=ET.fromstring(baseline(scenario,skins,1.345));world=tree.find('world')
    for model in list(world.findall('model')):world.remove(model)
    for light in list(world.findall('light')):world.remove(light)
    sun=ET.SubElement(world,'light',name='daylight',type='directional');ET.SubElement(sun,'direction').text='-0.3 0.4 -1';ET.SubElement(sun,'diffuse').text='0.95 0.94 0.9 1'
    scene=ET.SubElement(world,'scene');ET.SubElement(scene,'ambient').text='.45 .45 .45 1';ET.SubElement(scene,'background').text='.58 .72 .87 1';ET.SubElement(scene,'shadows').text='true'
    for actor in world.findall('actor'):
        spec=cast[actor.get('name')]
        for element in (actor.find('skin'),actor.find('animation')):ET.SubElement(element,'scale').text=str(spec.scale)
    controller=next(p for p in world.findall('plugin') if p.get('name')=='leader_follow::CrowdMotion')
    for config in controller.findall('person'):
        spec=cast[config.findtext('name')]
        with np.load(human_directory(spec.model)/'walk.npz') as data:speed=float(data['gait_speed_mps'])*spec.scale
        ET.SubElement(config,'gait_speed').text=str(speed)
    def model(name,position,yaw=0.):
        obj=ET.SubElement(world,'model',name=name);ET.SubElement(obj,'static').text='true'
        ET.SubElement(obj,'pose').text=' '.join(map(str,(*position,0,0,np.deg2rad(yaw))))
        return ET.SubElement(obj,'link',name='body')
    boxes,props=layout(environment)
    for box in boxes:
        link=model(box.name,box.position,box.yaw)
        for tag in ('visual','collision') if box.collision else ('visual',):
            item=ET.SubElement(link,tag,name=tag);geometry=ET.SubElement(item,'geometry')
            if tag=='visual':
                file=write_fixture_mesh(box,output.parent/'fixtures'/(box.name+'.dae'))
                ET.SubElement(ET.SubElement(geometry,'mesh'),'uri').text=file.as_uri()
                if box.material=='glass':ET.SubElement(item,'transparency').text='.72'
            else:ET.SubElement(ET.SubElement(geometry,'box'),'size').text=' '.join(map(str,box.size))
            if tag=='visual':
                color,rough,metal=PALETTE[box.material];mat=ET.SubElement(item,'material');ET.SubElement(mat,'diffuse').text=' '.join(map(str,(*color,1.)))
                texture=root/'surfaces'/box.material
                if texture.exists():
                    pbr=ET.SubElement(ET.SubElement(mat,'pbr'),'metal');ET.SubElement(pbr,'albedo_map').text=(texture/'Diffuse.jpg').as_uri();
                    if (texture/'Rough.jpg').exists():ET.SubElement(pbr,'roughness_map').text=(texture/'Rough.jpg').as_uri()
                    if (texture/'nor_gl.jpg').exists():ET.SubElement(pbr,'normal_map').text=(texture/'nor_gl.jpg').as_uri()
    for prop in props:
        link=model(prop.name,prop.position,prop.yaw)
        for tag in ('visual','collision'):
            item=ET.SubElement(link,tag,name=tag);geometry=ET.SubElement(item,'geometry')
            if tag=='collision' and prop.asset=='tree_small_02':
                center=json.loads((root/'props'/prop.asset/'conversion.json').read_text())['floor_center_translation']
                ET.SubElement(item,'pose').text=f'{-center[0]*prop.scale} {-center[1]*prop.scale} {prop.scale} 0 0 0'
                cylinder=ET.SubElement(geometry,'cylinder');ET.SubElement(cylinder,'radius').text=str(.12*prop.scale);ET.SubElement(cylinder,'length').text=str(2*prop.scale)
            else:
                mesh=ET.SubElement(geometry,'mesh');ET.SubElement(mesh,'uri').text=(root/'props'/prop.asset/'model.dae').as_uri();ET.SubElement(mesh,'scale').text=' '.join([str(prop.scale)]*3)
    ET.indent(tree);return ET.tostring(tree,encoding='unicode')
