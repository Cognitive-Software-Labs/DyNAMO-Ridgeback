"""The richer world must retain usable scenario lanes and deliberate look-alikes."""
import json
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
import pytest
from ridgeback_leader_follow.crowd_scenarios import SCENARIOS
from ridgeback_leader_follow.realistic_layout import ENVIRONMENTS,layout
from ridgeback_leader_follow.realistic_scene import assign_people,asset_directory,human_directory,load_scenario,gazebo_world


@pytest.mark.parametrize('environment',ENVIRONMENTS)
def test_all_scenario_paths_clear_of_static_furniture(environment):
    boxes,props=layout(environment)
    names=[o.name for o in boxes+props]
    assert len(names)==len(set(names))
    obstacles=[]
    for box in boxes:
        if box.collision and box.position[2]+box.size[2]/2>.1 and box.position[2]-box.size[2]/2<1.9:
            obstacles.append((box.name,np.array(box.position[:2]),np.array(box.size[:2])/2))
    for prop in props:
        if not prop.collision:continue
        dimensions=json.loads((asset_directory()/'props'/prop.asset/'conversion.json').read_text())['dimensions_m']
        if prop.position[2]>1.9:continue
        angle=np.deg2rad(prop.yaw);c,s=abs(np.cos(angle)),abs(np.sin(angle))
        half=np.array([[c,s],[s,c]])@np.array(dimensions[:2])*prop.scale/2
        obstacles.append((prop.name,np.array(prop.position[:2]),half))
    for name in SCENARIOS:
        for person in load_scenario(name,8).people:
            for at in np.linspace(0,20,101):
                point=np.array(person.sample(at)[:2])
                for label,center,half in obstacles:
                    separation=np.maximum(abs(point-center)-half,0.)
                    assert np.linalg.norm(separation)>.28,(environment,name,person.name,label,at)


@pytest.mark.parametrize('name',SCENARIOS)
def test_cast_diversity_and_deliberate_twins(name):
    scenario=load_scenario(name,8);cast=assign_people(scenario)
    assert set(cast)=={p.name for p in scenario.people}
    if name.startswith('lookalike'):assert cast['lf_leader']==cast['lf_lookalike']
    if name=='crowd':assert len({p.model for p in cast.values()})==4
    if name=='leader_alone':assert len({p.y for p in scenario.people[0].waypoints})>1
    for spec in cast.values():
        with np.load(human_directory(spec.model)/'walk.npz') as data:
            assert np.isfinite(data['points']).all() and data['points'][:,:,2].min()>=-1e-5
            assert float(data['gait_speed_mps'])>0


@pytest.mark.parametrize('environment',ENVIRONMENTS)
def test_gazebo_preserves_scenario_paths_and_actor_scales(environment,tmp_path):
    scenario=load_scenario('lookalike_crossing');world=ET.fromstring(gazebo_world(scenario,tmp_path,environment)).find('world')
    assert len(world.findall('actor'))==len(scenario.people)
    if environment=='courtyard':
        trees=[model for model in world.findall('model') if model.get('name','').startswith('Tree')]
        assert len(trees)==6
        assert all(tree.find('link/collision/geometry/cylinder') is not None for tree in trees)
        assert all(tree.find('link/visual/geometry/mesh') is not None for tree in trees)
    for actor in world.findall('actor'):
        assert float(actor.findtext('skin/scale'))>0
        skin=Path(actor.findtext('skin/filename'));assert skin.is_file()
        document=ET.parse(skin);ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        for image in document.findall('.//c:library_images/c:image/c:init_from',ns):
            assert Path(image.text.removeprefix('file://')).is_file(),image.text
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    material_ids=[]
    for fixture in (tmp_path.parent/'fixtures').glob('*.dae'):
        document=ET.parse(fixture)
        ids={element.get('id') for element in document.iter() if element.get('id')}
        for element in document.iter():
            for key in ('url','source','target'):
                reference=element.get(key,'')
                if reference.startswith('#'):assert reference[1:] in ids
        material_ids.extend(material.get('id') for material in document.findall('.//c:library_materials/c:material',ns))
    assert material_ids and len(material_ids)==len(set(material_ids))
    texture_paths=[Path(item.text.removeprefix('file://')) for item in world.findall('.//pbr/metal/albedo_map')]
    by_basename={}
    for path in texture_paths:
        assert path.is_file()
        assert by_basename.setdefault(path.name,path)==path
        assert path.name!='Diffuse.jpg'
    if environment!='courtyard':
        assert len([light for light in world.findall('light') if light.get('type')=='point'])==4
        luminous=[model for model in world.findall('model') if model.get('name','').startswith(('LightHousing','CoveLED'))]
        assert luminous and all(model.find('link/visual/material/emissive') is not None for model in luminous)
    plugin=next(p for p in world.findall('plugin') if p.get('name')=='leader_follow::CrowdMotion')
    for person,config in zip(scenario.people,plugin.findall('person'),strict=True):
        assert [w.text for w in config.findall('waypoint')]==[f'{w.time} {w.x} {w.y}' for w in person.waypoints]


def test_wall_texture_u_axis_is_horizontal_and_signs_are_not_tiled():
    from ridgeback_leader_follow.realistic_scene import box_mesh_data
    for size in ((12,.2,4),(.2,12,4)):
        data,points=box_mesh_data(size)
        for face in (2,3,4,5):
            offset=face*6
            # On each side wall U follows its horizontal edge, V rises vertically.
            assert points[offset+1,2]==pytest.approx(points[offset,2])
            assert points[offset+2,2]>points[offset+1,2]
        sign,_=box_mesh_data(size,repeat=False)
        assert sign['uv'].min()==0 and sign['uv'].max()==1


def test_static_props_have_no_wire_edges_and_textures_resolve():
    """Gazebo renders Collada <lines> as unlit white strokes (seen as white tree foliage)."""
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    for model in sorted((asset_directory()/'props').glob('*/model.dae')):
        document=ET.parse(model)
        assert not document.findall('.//c:lines',ns)+document.findall('.//c:linestrips',ns),model
        for image in document.findall('.//c:library_images/c:image',ns):
            reference=image.findtext('c:init_from',namespaces=ns).removeprefix('file://')
            assert (model.parent/reference).is_file(),(model,reference)
