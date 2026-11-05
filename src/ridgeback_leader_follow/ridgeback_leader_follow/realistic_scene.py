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
        return material(name,color,rough,metal,directory/'Diffuse.jpg' if directory.exists() else None,directory/'nor_gl.jpg' if directory.exists() else None)
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
            if clothing is not None and 'image' not in spec:color=clothing
            mat=material(str(directory)+str(i),color,pbr.get('roughnessFactor',.65),pbr.get('metallicFactor',0.),directory/spec['image'] if 'image' in spec else None,directory/spec['normal_image'] if 'normal_image' in spec else None,spec.get('alphaMode')=='MASK',clothing)
            faces=np.flatnonzero(data['materials']==i).tolist()
            if not faces:continue
            subset=UsdGeom.Subset.CreateGeomSubset(obj,'m'+str(i),'face',faces,'materialBind')
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
        UsdGeom.Subset.SetFamilyType(obj,'materialBind','partition')
    boxes,props=layout(environment)
    for box in boxes:
        # Explicit UVs in metres on every face; built-in cubes have no UV map.
        sx,sy,sz=box.size
        vertices=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],np.float32)*np.array(box.size)/2
        quads=((0,3,2,1),(4,5,6,7),(0,1,5,4),(3,7,6,2),(0,4,7,3),(1,2,6,5))
        points=[];uv=[];normals=[];faces=[]
        for quad in quads:
            a,b,c,d=vertices[list(quad)];normal=np.cross(b-a,c-a);normal/=np.linalg.norm(normal)
            w,h=np.linalg.norm(b-a)/2,np.linalg.norm(d-a)/2
            for triangle,coords in [((a,b,c),((0,0),(w,0),(w,h))),((a,c,d),((0,0),(w,h),(0,h)))]:
                offset=len(points);points.extend(triangle);normals.extend([normal]*3);faces.append([offset,offset+1,offset+2]);uv.extend(coords)
        data={'faces':np.array(faces),'normals':np.array(normals),'uv':np.array(uv)}
        obj=mesh('/Realistic/Fixtures/'+box.name,data,np.array(points));obj.AddTranslateOp().Set(Gf.Vec3d(*box.position))
        if box.yaw:obj.AddRotateZOp().Set(box.yaw)
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(surface(box.material))
        if box.collision:UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    for prop in props:
        directory=root/'props'/prop.asset
        if prop.asset not in meshes:
            with np.load(directory/'mesh.npz') as source:meshes[prop.asset]={k:source[k] for k in source.files}
        data=meshes[prop.asset];obj=mesh('/Realistic/Props/'+prop.name,data,data['points'])
        obj.AddTranslateOp().Set(Gf.Vec3d(*prop.position));obj.AddRotateZOp().Set(prop.yaw);obj.AddScaleOp().Set(Gf.Vec3f(prop.scale))
        bind_asset(obj,data,directory);UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    dome=UsdLux.DomeLight.Define(stage,'/Realistic/Lighting/Daylight');dome.CreateIntensityAttr(450. if environment!='courtyard' else 800.)
    dome.CreateTextureFileAttr(Sdf.AssetPath(str(root/'daylight.hdr')))
    sun=UsdLux.DistantLight.Define(stage,'/Realistic/Lighting/Sun');sun.AddRotateXYZOp().Set(Gf.Vec3f(315,25,25));sun.CreateIntensityAttr(1300. if environment=='courtyard' else 600.);sun.CreateAngleAttr(1.)
    if environment!='courtyard':
        for i,x in enumerate((-2,3,8,13)):
            light=UsdLux.RectLight.Define(stage,f'/Realistic/Lighting/Panel{i}');light.AddTranslateOp().Set(Gf.Vec3d(x,0,3.7 if environment=='office' else 4.7))
            light.CreateWidthAttr(2.2);light.CreateHeightAttr(.6);light.CreateIntensityAttr(650.);light.CreateColorAttr(Gf.Vec3f(1.,.94,.85))
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
                    if color is not None:color.text=' '.join(map(str,(*spec.clothing,1)))
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
            item=ET.SubElement(link,tag,name=tag);geometry=ET.SubElement(item,'geometry');ET.SubElement(ET.SubElement(geometry,'box'),'size').text=' '.join(map(str,box.size))
            if tag=='visual':
                color,rough,metal=PALETTE[box.material];mat=ET.SubElement(item,'material');ET.SubElement(mat,'diffuse').text=' '.join(map(str,(*color,1.)))
                texture=root/'surfaces'/box.material
                if texture.exists():
                    pbr=ET.SubElement(ET.SubElement(mat,'pbr'),'metal');ET.SubElement(pbr,'albedo_map').text=(texture/'Diffuse.jpg').as_uri();ET.SubElement(pbr,'roughness_map').text=(texture/'Rough.jpg').as_uri();ET.SubElement(pbr,'normal_map').text=(texture/'nor_gl.jpg').as_uri()
    for prop in props:
        link=model(prop.name,prop.position,prop.yaw)
        for tag in ('visual','collision'):
            item=ET.SubElement(link,tag,name=tag);mesh=ET.SubElement(ET.SubElement(item,'geometry'),'mesh');ET.SubElement(mesh,'uri').text=(root/'props'/prop.asset/'model.dae').as_uri();ET.SubElement(mesh,'scale').text=' '.join([str(prop.scale)]*3)
    ET.indent(tree);return ET.tostring(tree,encoding='unicode')
