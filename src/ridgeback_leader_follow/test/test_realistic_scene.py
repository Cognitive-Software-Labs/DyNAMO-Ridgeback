"""The richer world must retain usable scenario lanes and deliberate look-alikes."""
import json
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
import pytest


def test_waiting_modern_walkers_face_their_first_step():
    from ridgeback_leader_follow.realistic_scene import load_scenario
    for person in load_scenario('crowd',8,'modern_office').people:
        for a,b in zip(person.waypoints,person.waypoints[1:]):
            if (a.x,a.y)!=(b.x,b.y):
                assert person.sample(0.)[2]==pytest.approx(np.arctan2(b.y-a.y,b.x-a.x))
                break


def test_cached_motion_interpolates_and_wraps_at_loop_boundary():
    from ridgeback_leader_follow.crowd_geometry import motion_values
    clip={'points':np.array([[[0.,0.,0.]],[[2.,0.,0.]]])}
    assert motion_values(clip,.5)[0,0]==pytest.approx(1.)
    assert motion_values(clip,1.5)[0,0]==pytest.approx(1.)
from ridgeback_leader_follow.crowd_scenarios import SCENARIOS
from ridgeback_leader_follow.realistic_layout import ENVIRONMENTS,layout
from ridgeback_leader_follow.realistic_scene import CAST,assign_people,asset_directory,human_directory,load_scenario,gazebo_world


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
        for person in load_scenario(name,8,environment).people:
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
    if name=='crowd':assert len({p.model for p in cast.values()})==len(scenario.people)
    if name=='leader_alone':assert len({p.y for p in scenario.people[0].waypoints})>1
    for spec in cast.values():
        with np.load(human_directory(spec.model)/'walk.npz') as data:
            assert np.isfinite(data['points']).all() and data['points'][:,:,2].min()>=-1e-4
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
        assert len([light for light in world.findall('light') if light.get('type')=='point'])==(8 if environment=='modern_office' else 4)
        luminous=[model for model in world.findall('model') if model.get('name','').startswith(('LightHousing','CoveLED','LEDRun'))]
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


def test_modern_office_routes_walk_at_a_natural_pace_without_contact():
    import itertools
    times=np.linspace(0,20,801)
    for name in SCENARIOS:
        scenario=load_scenario(name,8,'modern_office')
        assert [p.name for p in scenario.people]==[p.name for p in load_scenario(name,8,'office').people]
        for person in scenario.people:
            speeds=[np.hypot(b.x-a.x,b.y-a.y)/(b.time-a.time) for a,b in zip(person.waypoints,person.waypoints[1:])]
            assert max(speeds)<=1.4
            if person.role=='leader':assert max(speeds)>=1.
        tracks={p.name:np.array([p.sample(t)[:2] for t in times]) for p in scenario.people}
        for a,b in itertools.combinations(tracks,2):
            assert np.linalg.norm(tracks[a]-tracks[b],axis=1).min()>.8,(name,a,b)
    stop=load_scenario('leader_stop',4,'modern_office')
    assert all(not p.moving(15.) for p in stop.people)


def test_cast_has_natural_statures_and_varied_people():
    from ridgeback_leader_follow.rocketbox_cast import ROSTER
    assert len({p.model for p in CAST})==len(CAST)>=12 and all(p.model in ROSTER for p in CAST)
    heights=[]
    for spec in CAST:
        with np.load(human_directory(spec.model)/'idle.npz') as data:heights.append(float(data['points'][0][:,2].max())*spec.scale)
    assert 1.55<min(heights) and max(heights)<1.9 and max(heights)-min(heights)>.15


@pytest.mark.parametrize('model',sorted(__import__('ridgeback_leader_follow.rocketbox_cast',fromlist=['ROSTER']).ROSTER))
def test_rocketbox_assets_are_consistent(model):
    directory=human_directory(model)
    receipt=json.loads((directory/'conversion.json').read_text())
    import hashlib
    for name,digest in receipt['sha256'].items():
        assert hashlib.sha256((directory/name).read_bytes()).hexdigest()==digest,name
    with np.load(directory/'walk.npz') as walk,np.load(directory/'idle.npz') as idle:
        assert walk['points'].shape[1]==idle['points'].shape[1]
        assert (walk['faces']==idle['faces']).all()
        assert abs(float(walk['cycle_seconds'])-len(walk['points'])/float(walk['fps']))<1e-6
        assert 0.9<float(walk['gait_speed_mps'])<1.4
        for clip in (walk,idle):assert clip['points'][:,:,2].min()>=-1e-4
        # Standing loop: first and last frames nearly coincide.
        assert np.abs(idle['points'][0]-idle['points'][-1]).max()<.02
    for spec in json.loads((directory/'materials.json').read_text()):
        assert (directory/spec['image']).is_file()
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    for clip in ('walk.dae','idle.dae'):
        document=ET.parse(directory/clip)
        assert len(document.findall('.//c:instance_controller/c:skeleton',ns))==1
        for image in document.findall('.//c:library_images/c:image',ns):
            assert image.get('id')==Path(image.findtext('c:init_from',namespaces=ns)).stem
            assert (directory/image.findtext('c:init_from',namespaces=ns)).is_file()


def test_idle_clip_plays_while_standing_and_gait_while_walking():
    from ridgeback_leader_follow.crowd_geometry import motion_frame
    scenario=load_scenario('leader_stop',4,'modern_office');leader=scenario.people[0]
    from ridgeback_leader_follow.realistic_scene import load_motion
    data=load_motion(human_directory(assign_people(scenario)['lf_leader'].model))
    clip,_=motion_frame(data,leader,2.);assert clip is data
    clip,first=motion_frame(data,leader,15.);assert clip is data['idle']
    _,later=motion_frame(data,leader,16.);assert later!=first


def test_gazebo_actor_switches_to_idle_animation(tmp_path):
    scenario=load_scenario('leader_stop',4,'modern_office')
    world=ET.fromstring(gazebo_world(scenario,tmp_path,'modern_office')).find('world')
    for actor in world.findall('actor'):
        names=[a.get('name') for a in actor.findall('animation')]
        assert names==['walk','idle'] and all(Path(a.findtext('filename')).is_file() for a in actor.findall('animation'))
    plugin=next(p for p in world.findall('plugin') if p.get('name')=='leader_follow::CrowdMotion')
    for config in plugin.findall('person'):
        assert float(config.findtext('idle_cycle'))>0 and float(config.findtext('cycle'))>0
    emissive=[m for m in world.findall('model') if m.get('name','').startswith(('LEDRun','Display'))]
    assert emissive and all(m.find('link/visual/material/emissive') is not None for m in emissive)


@pytest.mark.parametrize('name',SCENARIOS)
def test_modern_office_starts_with_a_person_in_the_front_lidar_gate(name):
    """crowd_qualify requires a leg return 0.65-2 m ahead (within 0.8 rad) at t=0."""
    lidar=np.array([.3922,0.])
    for person in load_scenario(name,8,'modern_office').people:
        offset=np.array(person.sample(0.)[:2])-lidar
        if .8<np.linalg.norm(offset)<1.85 and abs(np.arctan2(offset[1],offset[0]))<.7:return
    pytest.fail('no pedestrian inside the front-lidar start gate')


def test_modern_office_people_pass_close_to_but_clear_of_the_robot():
    times=np.linspace(0,20,401)
    for name in SCENARIOS:
        scenario=load_scenario(name,8,'modern_office')
        distances=[np.hypot(*p.sample(t)[:2]) for p in scenario.people for t in times]
        assert min(distances)>1.4 and min(distances)<2.5,name
        assert max(np.hypot(*p.sample(t)[:2]) for p in scenario.people for t in times)<15.,name


def test_explainer_motion_matches_html_at_every_display_frame():
    import re
    from ridgeback_leader_follow.explainer_motion import load_scenario as illustration
    html=(Path(__file__).resolve().parents[3]/'docs/leader_following/assets/explainer.html').read_text()
    block=html.split('// sim:start',1)[1].split('// sim:end',1)[0]
    leader=json.loads(re.search(r'const leaderAt = path\((\[\[.*?\]\])\)',block).group(1))
    entries=re.findall(r'at:\s*(?:path\((\[\[.*?\]\])\)|\(\)\s*=>\s*(\[[^\]]+\]))',block)
    expected=[leader]+[json.loads(path) if path else [[0,*json.loads(point)],[24,*json.loads(point)]] for path,point in entries]
    scenario=illustration()
    assert len(expected)==len(scenario.people)==11
    assert scenario.duration==24.
    for person,keys in zip(scenario.people,expected,strict=True):
        keys=np.array(keys)
        for t in np.linspace(0,24,721):
            x,y,_=person.sample(t)
            assert x==pytest.approx(np.interp(t,keys[:,0],keys[:,1])-2.2)
            assert y==pytest.approx(4.6-np.interp(t,keys[:,0],keys[:,2]))
    cast=assign_people(scenario)
    assert cast['lf_leader']==cast['lf_lookalike']
    assert sum(not p.moving(12.) for p in scenario.people)==3


def test_explainer_restores_original_office_without_changing_paths():
    from ridgeback_leader_follow.explainer_motion import load_scenario as illustration
    from ridgeback_leader_follow.realistic_scene import scene_layout
    scenario=illustration()
    before=tuple(p.waypoints for p in scenario.people)
    assert scene_layout('modern_office',scenario)==layout('modern_office')
    assert tuple(p.waypoints for p in scenario.people)==before


def test_performance_lighting_preserves_cast_paths_and_office(tmp_path):
    scenario=load_scenario('explainer',8,'modern_office')
    reference=ET.fromstring(gazebo_world(scenario,tmp_path/'reference','modern_office')).find('world')
    fast=ET.fromstring(gazebo_world(scenario,tmp_path/'performance','modern_office','performance')).find('world')
    for tag in ('actor','model','plugin','physics'):
        def normalized(world):
            result=[]
            for element in world.findall(tag):
                text=ET.tostring(element,encoding='unicode')
                result.append(text.replace(str(tmp_path/'reference'),'RUN').replace(str(tmp_path/'performance'),'RUN'))
            return result
        assert normalized(reference)==normalized(fast),tag
    assert fast.find('scene/shadows').text=='false'
    assert len(fast.findall('light'))==3


def test_explainer_weave_only_changes_the_leader_and_stays_natural():
    from ridgeback_leader_follow.explainer_motion import load_scenario as illustration, load_weave_scenario
    from ridgeback_leader_follow.realistic_scene import scene_layout
    base,weave=illustration(),load_weave_scenario()
    assert weave.name=='explainer_weave' and weave.duration==base.duration
    assert weave.people[1:]==base.people[1:]
    leader,original=weave.people[0],base.people[0]
    times=np.linspace(0,24,481)
    path=np.array([leader.sample(t)[:2] for t in times])
    route=np.array([original.sample(t)[:2] for t in times])
    sideways=np.hypot(*(path-route).T)
    assert .5<sideways.max()<=.61 and sideways[0]==pytest.approx(0) and sideways[-1]==pytest.approx(0,abs=1e-6)
    speed=np.hypot(*np.diff(path,axis=0).T)/(times[1]-times[0])
    assert speed.max()<1.25
    for person in weave.people[1:]:
        other=np.array([person.sample(t)[:2] for t in times])
        assert np.hypot(*(path-other).T).min()>.75,person.name
    assert scene_layout('modern_office',weave)==layout('modern_office')


@pytest.mark.parametrize('seed',(0,1,5))
def test_wander_leader_is_random_reproducible_and_clear(seed):
    from ridgeback_leader_follow.wander_scenario import (FURNITURE_CLEARANCE, PEOPLE_CLEARANCE,
                                                         furniture_distance, load_wander_scenario, static_obstacles)
    scenario=load_wander_scenario(seed)
    assert scenario==load_wander_scenario(seed)
    assert scenario.people[0].waypoints!=load_wander_scenario(seed+10).people[0].waypoints
    leader=scenario.people[0]
    times=np.arange(0,scenario.duration,.1)
    path=np.array([leader.sample(t)[:2] for t in times])
    assert path[0]==pytest.approx((1.4,0.))
    assert np.hypot(*np.diff(path,axis=0).T).sum()>20.
    assert (np.hypot(*np.diff(path,axis=0).T)/.1).max()<1.1
    obstacles=static_obstacles()
    assert min(furniture_distance(p,obstacles) for p in path[::5])>FURNITURE_CLEARANCE-.06
    for person in scenario.people[1:]:
        other=np.array([person.sample(t)[:2] for t in times])
        assert np.hypot(*(path-other).T).min()>=PEOPLE_CLEARANCE-1e-6
