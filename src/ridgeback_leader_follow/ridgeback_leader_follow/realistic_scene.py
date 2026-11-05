"""Textured worlds and a varied pedestrian cast, shared by both simulators."""
from dataclasses import dataclass
import json
from pathlib import Path
import re
import shutil
from xml.etree import ElementTree as ET

import numpy as np
from .realistic_layout import EMISSIVE,ENVIRONMENTS,PALETTE,layout


@dataclass(frozen=True)
class Pedestrian:
    model: str
    scale: float
    clothing: tuple|None


# Microsoft Rocketbox (MIT) pedestrians at natural statures: the scale maps
# each publisher skeleton (about 1.74 m women, 1.81 m men including hair) onto
# 1.63-1.85 m people. Clothing is the publisher's own; no tint is applied.
CAST=tuple(Pedestrian(model,scale,None) for model,scale in (
    ('business_female_01',.97),('business_male_02',.97),('female_adult_05',.95),
    ('male_adult_12',1.),('business_female_02',.94),('male_adult_17',.96),
    ('female_adult_11',.96),('male_adult_03',.99),('business_male_05',.97),
    ('female_party_02',.93),('male_adult_08',1.02),('business_female_03',.99),
    ('business_male_04',.95),('male_adult_14',.98)))


def asset_directory():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('ridgeback_leader_follow'))/'sim/assets/realistic'


def human_directory(model):
    from .rocketbox_cast import ROSTER
    if model in ROSTER:return asset_directory()/'rocketbox'/model
    return asset_directory().parent/'maya' if model=='maya' else asset_directory()/'humans'/model


def assign_people(scenario):
    assigned={};index=0
    for person in scenario.people:
        if person.name=='lf_lookalike':
            assigned[person.name]=assigned['lf_leader']
        else:
            assigned[person.name]=CAST[index%len(CAST)];index+=1
    return assigned


def load_scenario(name,crowd_size=4,environment='office'):
    """Six scenario families. The modern office walks them at a natural pace;
    the earlier realistic worlds keep their recorded routes (alone adds turns)."""
    from dataclasses import replace
    from .crowd_scenarios import load_scenario as baseline,Waypoint
    scenario=baseline(name,crowd_size)
    if environment=='modern_office':
        return natural_scenario(name,crowd_size,scenario)
    if name=='leader_alone':
        leader=replace(scenario.people[0],waypoints=tuple(Waypoint(*p) for p in ((0,2,0),(6,4,0),(12,6,1.5),(16,8,1.5),(20,9,0))))
        scenario=replace(scenario,people=(leader,))
    return scenario


# Walking pace for scripted routes. The baked gaits cover 0.95-1.2 m/s at the
# cast's statures; gait phase follows distance, so stance feet stay planted.
PACE=1.1


def route(points,start=0.,pace=PACE,duration=20.,radius=.7):
    """Waypoints along a polyline at `pace`, standing (idle) before and after.

    Interior corners are rounded with short arcs so heading changes gradually.
    """
    from .crowd_scenarios import Waypoint
    points=[np.asarray(p,float) for p in points];path=[points[0]]
    for a,b,c in zip(points,points[1:],points[2:]):
        r=min(radius,np.linalg.norm(b-a)/2,np.linalg.norm(c-b)/2)
        p0=b+(a-b)/np.linalg.norm(a-b)*r;p2=b+(c-b)/np.linalg.norm(c-b)*r
        for t in np.linspace(0,1,6):path.append((1-t)**2*p0+2*(1-t)*t*b+t**2*p2)
    path.append(points[-1])
    times=[start]
    for a,b in zip(path,path[1:]):times.append(times[-1]+max(np.linalg.norm(b-a),1e-3)/pace)
    if times[-1]>duration-.05:raise ValueError('Route does not finish inside the scenario')
    waypoints=([Waypoint(0.,*map(float,path[0]))] if start>0 else [])
    waypoints+=[Waypoint(round(t,4),float(x),float(y)) for t,(x,y) in zip(times,path)]
    return tuple(waypoints)+(Waypoint(duration,*map(float,path[-1])),)


def natural_scenario(name,crowd_size,baseline):
    """Modern-office routes: same families and roles, natural walking speed."""
    from .crowd_scenarios import CrowdScenario,PersonPath
    colour=baseline.people[0].color
    def person(key,role,points,start=0.,pace=PACE):return PersonPath(key,role,colour if role=='leader' or key=='lf_lookalike' else (.5,.5,.5),route(points,start,pace))
    # Routes keep people 2-9 m in front of the stationary robot, where its camera
    # sees them large, and include oncoming faces and close crossings.
    if name=='leader_alone':
        # Away from the robot, a left jog, a turn-around and the walk back.
        people=(person('lf_leader','leader',((1.6,-.3),(5.5,-.3),(7.,1.2),(8.6,1.2),(9.3,.2),(8.4,-.8),(3.,-1.))),)
    elif name=='crossing':
        # The passer crosses about 3.5 m ahead, between the robot and the leader.
        people=(person('lf_leader','leader',((1.6,.2),(6.,.2),(7.2,1.))),
                person('lf_passer','bystander',((3.4,-3.8),(3.5,0.),(3.6,3.8),(6.,4.)),start=1.5))
    elif name=='lookalike_parallel':
        people=(person('lf_leader','leader',((1.6,.45),(8.5,.45),(9.5,1.))),
                person('lf_lookalike','bystander',((1.6,-.45),(8.5,-.45),(9.7,-.1))))
    elif name=='lookalike_crossing':
        # The pair swap sides a few metres ahead and never come closer than 1.5 m.
        people=(person('lf_leader','leader',((1.7,-1.),(8.,1.4),(9.5,1.4))),
                person('lf_lookalike','bystander',((3.,1.4),(8.5,-1.2),(10.,-1.2))))
    elif name=='leader_stop':
        # The leader joins a standing group; a short step sets each member's facing.
        group=(((6.9,1.3),(6.5,.95)),((7.1,-1.1),(6.75,-.75)),((7.9,-.1),(7.45,-.05)))
        people=(person('lf_leader','leader',((1.6,0.),(5.6,.1))),
                *(person(f'lf_standing_{i}','bystander',(a,b),pace=.6) for i,(a,b) in enumerate(group)))
    elif name=='crowd':
        # The explainer page's hall choreography at natural walking speeds: the
        # leader curves away; a brisk crosser cuts between robot and leader; a
        # pair crosses the view; walkers pass the robot on both sides; one walker
        # converges towards the leader; two people stand chatting. Larger sizes
        # add people in this order; everyone stays at least 0.8 m apart.
        cast=(((7.1,-3.4),(7.1,0.),(7.2,4.1)),4.8,1.3),(((19.,2.1),(2.5,2.1),(.6,3.3)),.1,1.1),\
             (((12.2,-3.4),(12.2,3.6)),3.,.85),(((13.2,-3.4),(13.2,3.6)),3.,.85),\
             (((21.,-1.6),(15.,-.7)),6.7,1.),(((19.,-1.6),(2.4,-1.3),(1.,-2.6)),1.3,1.1),\
             (((15.9,3.3),(15.6,3.6)),0.,.5),(((16.7,4.2),(16.4,3.9)),0.,.5)
        people=(person('lf_leader','leader',((1.4,0.),(6.4,0.),(10.,.7),(14.2,1.),(17.4,.5),(19.4,-.1)),pace=1.05),
                *(person(f'lf_crowd_{i}','bystander',points,start=start,pace=pace) for i,(points,start,pace) in enumerate(cast[:crowd_size])))
    else:
        raise ValueError('Unknown scenario '+name)
    return CrowdScenario(name,20.,people)


def manifest_entry(scenario,environment):
    from dataclasses import asdict
    return {'environment':environment,'geometry':'Textured architectural scene, CC0 Poly Haven props and MIT Microsoft Rocketbox pedestrians',
            'appearance':{name:asdict(spec) for name,spec in assign_people(scenario).items()},
            'qualification':'new visual profile; original F1/F2/F3 results do not qualify this population'}


def load_motion(directory):
    """Walk cache plus, when baked, the idle loop played while a person stands."""
    with np.load(directory/'walk.npz') as source:data={k:source[k] for k in source.files}
    if (directory/'idle.npz').exists():
        with np.load(directory/'idle.npz') as source:data['idle']={k:source[k] for k in source.files}
    return data


def scaled_motion(data,scale):
    result=dict(data);result['points']=data['points']*scale;result['gait_speed_mps']=float(data['gait_speed_mps'])*scale
    if 'idle' in data:result['idle']=dict(data['idle'],points=data['idle']['points']*scale)
    return result


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
    texture=fixture_texture(box.material,'Diffuse',path.parent)
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
    # Gazebo's imported material registry is shared across mesh files.
    # Reusing "finish" lets the first fixture's texture leak onto later ones.
    identifiers={item.get('id') for item in root.iter() if item.get('id')}
    prefix=re.sub(r'[^A-Za-z0-9_]', '_', box.name)+'_'
    for item in root.iter():
        for key,value in list(item.attrib.items()):
            if value in identifiers:item.set(key,prefix+value)
            elif value.startswith('#') and value[1:] in identifiers:item.set(key,'#'+prefix+value[1:])
        if item.tag=='init_from' and item.text in identifiers:item.text=prefix+item.text
    path.parent.mkdir(parents=True,exist_ok=True);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return path


def fixture_texture(material,channel,directory):
    """Ogre texture resource basenames must be unique across architectural maps."""
    source=asset_directory()/'surfaces'/material/(channel+'.jpg')
    target=directory/'textures'/(material+'_'+channel+'.jpg')
    if source.exists() and not target.exists():
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
    return target


def clothing_color(name,color):
    # Casual trousers keep their neutral fabric rather than matching every shirt.
    return (.055,.065,.09) if 'original bottom' in name.lower().replace('_',' ') else color


def create_scene(stage,scenario,environment='office'):
    from pxr import Gf,Sdf,UsdGeom,UsdLux,UsdPhysics,UsdShade
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    root=asset_directory();materials={};meshes={}
    def material(name,color=(.7,.7,.7),rough=.65,metal=0.,image=None,normal=None,masked=False,tint=None,orm=None,normal_scale=1.,specular=1.,rough_map=None,opacity_image=None):
        key=(name,tuple(color),rough,metal,image,normal,masked,tint,orm,normal_scale,specular,rough_map,opacity_image)
        if key in materials:return materials[key]
        path='/Realistic/Materials/m'+str(len(materials));mat=UsdShade.Material.Define(stage,path)
        shader=UsdShade.Shader.Define(stage,path+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough)
        shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
        reflectance=(.04*max(0.,min(float(specular),1.)))**.5
        shader.CreateInput('ior',Sdf.ValueTypeNames.Float).Set((1+reflectance)/max(1-reflectance,1e-6))
        if name=='glass':shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.28)
        uv=UsdShade.Shader.Define(stage,path+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2')
        uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
        for field,file,color_space in [('Color',image,'sRGB'),('Normal',normal,'raw'),('Packed',orm,'raw'),('Roughness',rough_map,'raw'),('Opacity',opacity_image,'raw')]:
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
            elif field=='Packed':
                tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1.,rough,metal,1.))
                shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'g')
                shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'b')
            elif field=='Opacity':
                shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'r')
                shader.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(.5)
            elif field=='Roughness':
                shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'r')
            else:
                tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2*normal_scale,2*normal_scale,2,1))
                tex.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-normal_scale,-normal_scale,-1,0))
                shader.CreateInput('normal',Sdf.ValueTypeNames.Normal3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        if name=='glass':
            glass=UsdShade.Shader.Define(stage,path+'/Glass')
            glass.CreateImplementationSourceAttr().Set('sourceAsset')
            glass.SetSourceAsset(Sdf.AssetPath('OmniGlass.mdl'),'mdl');glass.SetSourceAssetSubIdentifier('OmniGlass','mdl')
            glass.CreateInput('glass_color',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.94,.98,1.))
            glass.CreateInput('glass_ior',Sdf.ValueTypeNames.Float).Set(1.5)
            glass.CreateInput('thin_walled',Sdf.ValueTypeNames.Bool).Set(True)
            glass.CreateInput('frosting_roughness',Sdf.ValueTypeNames.Float).Set(.035)
            mat.CreateSurfaceOutput('mdl').ConnectToSource(glass.ConnectableAPI(),'out')
        materials[key]=mat;return mat
    def surface(name):
        color,rough,metal=PALETTE[name];directory=root/'surfaces'/name
        return material(name,color,rough,metal,directory/'Diffuse.jpg' if directory.exists() else None,directory/'nor_gl.jpg' if (directory/'nor_gl.jpg').exists() else None,rough_map=directory/'Rough.jpg' if (directory/'Rough.jpg').exists() else None)
    def mesh(path,data,points):
        obj=UsdGeom.Mesh.Define(stage,path);obj.CreatePointsAttr(points)
        obj.CreateFaceVertexCountsAttr([3]*len(data['faces']));obj.CreateFaceVertexIndicesAttr(data['faces'].reshape(-1))
        obj.CreateNormalsAttr(data['normals'][0] if data['normals'].ndim==3 else data['normals']);obj.SetNormalsInterpolation('vertex')
        obj.CreateSubdivisionSchemeAttr('none');obj.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(obj).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying').Set(data['uv'])
        return obj
    def bind_asset(obj,data,directory,tint=None):
        from .material_assets import material_specs,texture_tint_scale
        specs=material_specs(directory)
        for i,spec in enumerate(specs):
            pbr=spec.get('pbrMetallicRoughness',{});color=pbr.get('baseColorFactor',[.7,.7,.7,1])[:3]
            clothing=tint if tint is not None and _clothing(spec.get('name','')) else None
            if clothing is not None:
                target=clothing_color(spec.get('name',''),clothing)
                if 'image' in spec:clothing=texture_tint_scale(directory/spec['image'],tuple(target))
                else:color=target
            roughness=max(pbr.get('roughnessFactor',.65),.78 if (directory/'walk.npz').exists() and _clothing(spec.get('name','')) else 0.)
            mat=material(str(directory)+str(i),color,roughness,pbr.get('metallicFactor',0.),directory/spec['image'] if 'image' in spec else None,directory/spec['normal_image'] if 'normal_image' in spec else None,spec.get('alphaMode') in ('MASK','BLEND'),clothing,orm=directory/spec['orm_image'] if 'orm_image' in spec else None,normal_scale=spec.get('normal_scale',1.),specular=spec.get('specular_factor',1.),opacity_image=directory/spec['opacity_image'] if 'opacity_image' in spec else None)
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
        if box.name.startswith(('LightHousing','CoveLED')):
            mat=material('luminaire',(.95,.92,.84),.4)
            UsdShade.Shader.Get(stage,str(mat.GetPath())+'/Surface').CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(3.,2.85,2.55))
        elif box.material in EMISSIVE:
            mat=material('emissive_'+box.material,PALETTE[box.material][0],PALETTE[box.material][1])
            UsdShade.Shader.Get(stage,str(mat.GetPath())+'/Surface').CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*EMISSIVE[box.material]))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
        if box.collision:UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    for prop in props:
        directory=root/'props'/prop.asset
        if prop.asset not in meshes:
            with np.load(directory/'mesh.npz') as source:meshes[prop.asset]={k:source[k] for k in source.files}
        data=meshes[prop.asset];obj=mesh('/Realistic/Props/'+prop.name,data,data['points'])
        obj.AddTranslateOp().Set(Gf.Vec3d(*prop.position));obj.AddRotateZOp().Set(prop.yaw);obj.AddScaleOp().Set(Gf.Vec3f(prop.scale))
        bind_asset(obj,data,directory)
        if not prop.collision:continue
        if prop.asset=='tree_small_02':
            center=json.loads((directory/'conversion.json').read_text())['floor_center_translation']
            collider=UsdGeom.Cylinder.Define(stage,str(obj.GetPath())+'/TrunkCollider')
            collider.CreateRadiusAttr(.12);collider.CreateHeightAttr(2.)
            collider.AddTranslateOp().Set(Gf.Vec3d(-center[0],-center[1],1.))
            collider.MakeInvisible();UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
        else:UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    if environment=='modern_office':
        _modern_office_lights(stage);return _people(stage,scenario,meshes,mesh,bind_asset)
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
    if environment=='office':
        for side in (-1,1):
            for i,x in enumerate((-2,3,8,13)):
                light=UsdLux.RectLight.Define(stage,f'/Realistic/Lighting/Cove{side}_{i}'.replace('-','n'))
                light.AddTranslateOp().Set(Gf.Vec3d(x,side*5.5,4.02));light.AddRotateXOp().Set(180.)
                light.CreateWidthAttr(4.8);light.CreateHeightAttr(.08);light.CreateIntensityAttr(1800.)
                light.CreateColorAttr(Gf.Vec3f(1.,.93,.82))
    return _people(stage,scenario,meshes,mesh,bind_asset)


def _people(stage,scenario,meshes,mesh,bind_asset):
    from pxr import UsdGeom
    roots={}
    for person in scenario.people:
        spec=assign_people(scenario)[person.name];directory=human_directory(spec.model)
        if spec.model not in meshes:
            meshes[spec.model]=load_motion(directory)
        data=scaled_motion(meshes[spec.model],spec.scale)
        node=UsdGeom.Xform.Define(stage,'/People/'+person.name);translate=node.AddTranslateOp();rotate=node.AddRotateZOp()
        obj=mesh(str(node.GetPath())+'/human',data,data['points'][0]);bind_asset(obj,data,directory,spec.clothing if spec.model!="maya" else None)
        roots[person.name]=(node,translate,rotate,obj,data)
    return roots


def _modern_office_lights(stage):
    """Daylight through the south glazing plus soft linear ceiling light."""
    from pxr import Gf,Sdf,UsdLux
    from .realistic_layout import CEILING,layout
    root=asset_directory()
    dome=UsdLux.DomeLight.Define(stage,'/Realistic/Lighting/Daylight');dome.CreateIntensityAttr(1600.)
    dome.CreateTextureFileAttr(Sdf.AssetPath(str(root/'daylight.hdr')));dome.CreateTextureFormatAttr('latlong')
    dome.GetPrim().CreateAttribute('visibleInPrimaryRay',Sdf.ValueTypeNames.Bool).Set(True)
    # Low sun from the south-west casting window light across the floor.
    sun=UsdLux.DistantLight.Define(stage,'/Realistic/Lighting/Sun');sun.AddRotateXYZOp().Set(Gf.Vec3f(52,0,-25))
    sun.CreateIntensityAttr(4500.);sun.CreateAngleAttr(.6);sun.CreateColorAttr(Gf.Vec3f(1.,.95,.88))
    # Sky fill just inside the glazing, angled into the room.
    for i,x in enumerate(range(-1,24,5)):
        fill=UsdLux.RectLight.Define(stage,f'/Realistic/Lighting/WindowFill{i}')
        fill.AddTranslateOp().Set(Gf.Vec3d(x,-7.2,1.6));fill.AddRotateXYZOp().Set(Gf.Vec3f(80,0,0))
        fill.CreateWidthAttr(4.8);fill.CreateHeightAttr(2.6);fill.CreateIntensityAttr(2500.)
        fill.CreateColorAttr(Gf.Vec3f(.9,.95,1.))
        fill.GetPrim().CreateAttribute('visibleInPrimaryRay',Sdf.ValueTypeNames.Bool).Set(False)
    boxes,_=layout('modern_office')
    for box in boxes:
        if box.material!='led':continue
        light=UsdLux.RectLight.Define(stage,'/Realistic/Lighting/'+box.name);light.AddTranslateOp().Set(Gf.Vec3d(box.position[0],box.position[1],CEILING-.04))
        light.CreateWidthAttr(box.size[0]);light.CreateHeightAttr(.3);light.CreateIntensityAttr(22000.)
        light.CreateColorAttr(Gf.Vec3f(1.,.97,.92))
        light.GetPrim().CreateAttribute('visibleInPrimaryRay',Sdf.ValueTypeNames.Bool).Set(False)

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
        elif (directory/'idle.dae').exists():
            # Rocketbox: publisher outfit, unique texture names, separate idle clip.
            for clip in ('walk','idle'):
                text=(directory/(clip+'.dae')).read_text()
                for file in [*directory.glob('*.jpg'),*directory.glob('*.png')]:
                    text=text.replace(f'<init_from>{file.name}</init_from>',f'<init_from>{file.as_uri()}</init_from>')
                target=output/(person.name+('' if clip=='walk' else '_idle')+'.dae');target.parent.mkdir(parents=True,exist_ok=True);target.write_text(text)
            skins[person.name]=output/(person.name+'.dae');continue
        else:
            text=(directory/'walk.dae').read_text()
            for file in directory.glob('image_*.png'):text=text.replace(f'<init_from>{file.name}</init_from>',f'<init_from>{file.as_uri()}</init_from>')
            ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};ET.register_namespace('',ns['c']);tree=ET.fromstring(text)
            for effect in tree.findall('.//c:effect',ns):
                if any(word in effect.get('id','').lower() for word in ('original_top','original_bottom','business_fabric')):
                    color=effect.find('.//c:diffuse/c:color',ns)
                    if color is None:
                        diffuse=effect.find('.//c:diffuse',ns)
                        if diffuse is None:continue
                        for old in list(diffuse):diffuse.remove(old)
                        color=ET.SubElement(diffuse,'{'+ns['c']+'}color',sid='diffuse')
                    color.text=' '.join(map(str,(*clothing_color(effect.get('id',''),spec.clothing),1)))
            text=ET.tostring(tree,encoding='unicode')
        path=output/(person.name+'.dae');path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text);skins[person.name]=path
    tree=ET.fromstring(baseline(scenario,skins,1.345));world=tree.find('world')
    for model in list(world.findall('model')):world.remove(model)
    for light in list(world.findall('light')):world.remove(light)
    sun=ET.SubElement(world,'light',name='daylight',type='directional');ET.SubElement(sun,'direction').text='-0.3 0.4 -1';ET.SubElement(sun,'diffuse').text='0.95 0.94 0.9 1'
    scene=ET.SubElement(world,'scene');ET.SubElement(scene,'ambient').text='.45 .45 .45 1';ET.SubElement(scene,'background').text='.58 .72 .87 1';ET.SubElement(scene,'shadows').text='true'
    if environment=='modern_office':
        # Daylight enters through the south glazing; ceiling runs add soft fill.
        sun.find('direction').text='0.35 0.75 -0.55'
        scene.find('ambient').text='.5 .5 .5 1';scene.find('background').text='.7 .8 .92 1'
        for i,(x,y) in enumerate((x,y) for x in (1.,7.,13.,19.) for y in (-3.5,2.)):
            light=ET.SubElement(world,'light',name=f'ceiling_run_{i}',type='point')
            ET.SubElement(light,'pose').text=f'{x} {y} 2.85 0 0 0'
            ET.SubElement(light,'diffuse').text='0.75 0.72 0.66 1';ET.SubElement(light,'specular').text='0.15 0.15 0.15 1'
            ET.SubElement(light,'cast_shadows').text='false'
            attenuation=ET.SubElement(light,'attenuation')
            for key,value in [('range',12),('constant',.7),('linear',.06),('quadratic',.03)]:
                ET.SubElement(attenuation,key).text=str(value)
    elif environment!='courtyard':
        for i,x in enumerate((0,5,10,15)):
            light=ET.SubElement(world,'light',name=f'ceiling_panel_{i}',type='point')
            ET.SubElement(light,'pose').text=f'{x} 0 {3.65 if environment=="office" else 4.65} 0 0 0'
            ET.SubElement(light,'diffuse').text='0.9 0.85 0.75 1'
            ET.SubElement(light,'specular').text='0.3 0.3 0.3 1'
            attenuation=ET.SubElement(light,'attenuation')
            for key,value in [('range',16),('constant',.6),('linear',.03),('quadratic',.02)]:
                ET.SubElement(attenuation,key).text=str(value)
    for actor in world.findall('actor'):
        spec=cast[actor.get('name')];idle=output/(actor.get('name')+'_idle.dae')
        if idle.exists():
            animation=ET.SubElement(actor,'animation',name='idle');ET.SubElement(animation,'filename').text=str(idle)
            ET.SubElement(animation,'interpolate_x').text='false'
        for element in (actor.find('skin'),*actor.findall('animation')):ET.SubElement(element,'scale').text=str(spec.scale)
    controller=next(p for p in world.findall('plugin') if p.get('name')=='leader_follow::CrowdMotion')
    for config in controller.findall('person'):
        spec=cast[config.findtext('name')]
        directory=human_directory(spec.model)
        with np.load(directory/'walk.npz') as data:
            ET.SubElement(config,'gait_speed').text=str(float(data['gait_speed_mps'])*spec.scale)
            ET.SubElement(config,'cycle').text=str(float(data['cycle_seconds']))
        if (directory/'idle.npz').exists():
            from .crowd_geometry import idle_offset
            with np.load(directory/'idle.npz') as data:cycle=float(data['cycle_seconds'])
            ET.SubElement(config,'idle_cycle').text=str(cycle)
            ET.SubElement(config,'idle_offset').text=str(idle_offset(config.findtext('name'),cycle))
            ET.SubElement(config,'walk_offset').text=str(idle_offset(config.findtext('name')+':gait',float(config.findtext('cycle'))))
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
                color,rough,metal=PALETTE[box.material];mat=ET.SubElement(item,'material')
                # Ogre multiplies albedo maps by the diffuse colour; the modern office keeps texture pixels.
                if environment=='modern_office' and (root/'surfaces'/box.material).exists():color=(1.,1.,1.)
                ET.SubElement(mat,'diffuse').text=' '.join(map(str,(*color,1.)))
                if box.name.startswith(('LightHousing','CoveLED')):
                    ET.SubElement(mat,'emissive').text='1 0.94 0.82 1'
                elif box.material in EMISSIVE:
                    ET.SubElement(mat,'emissive').text=' '.join(f'{min(v/3.,1.):.3f}' for v in EMISSIVE[box.material])+' 1'
                texture=root/'surfaces'/box.material
                if texture.exists():
                    pbr=ET.SubElement(ET.SubElement(mat,'pbr'),'metal')
                    ET.SubElement(pbr,'albedo_map').text=fixture_texture(box.material,'Diffuse',output.parent/'fixtures').as_uri()
                    if (texture/'Rough.jpg').exists():ET.SubElement(pbr,'roughness_map').text=fixture_texture(box.material,'Rough',output.parent/'fixtures').as_uri()
                    if (texture/'nor_gl.jpg').exists():ET.SubElement(pbr,'normal_map').text=fixture_texture(box.material,'nor_gl',output.parent/'fixtures').as_uri()
    for prop in props:
        link=model(prop.name,prop.position,prop.yaw)
        for tag in ('visual','collision') if prop.collision else ('visual',):
            item=ET.SubElement(link,tag,name=tag);geometry=ET.SubElement(item,'geometry')
            if tag=='collision' and prop.asset=='tree_small_02':
                center=json.loads((root/'props'/prop.asset/'conversion.json').read_text())['floor_center_translation']
                ET.SubElement(item,'pose').text=f'{-center[0]*prop.scale} {-center[1]*prop.scale} {prop.scale} 0 0 0'
                cylinder=ET.SubElement(geometry,'cylinder');ET.SubElement(cylinder,'radius').text=str(.12*prop.scale);ET.SubElement(cylinder,'length').text=str(2*prop.scale)
            else:
                mesh=ET.SubElement(geometry,'mesh');ET.SubElement(mesh,'uri').text=(root/'props'/prop.asset/'model.dae').as_uri();ET.SubElement(mesh,'scale').text=' '.join([str(prop.scale)]*3)
    ET.indent(tree);return ET.tostring(tree,encoding='unicode')
