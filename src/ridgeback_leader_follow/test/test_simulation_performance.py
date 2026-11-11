"""Reduced-cost scenes preserve geometry, calibration and animation contracts."""
import json
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
import pytest
from ridgeback_leader_follow.crowd_scenarios import PersonPath,Waypoint
from ridgeback_leader_follow.simulation_performance import RuntimeTuning,resolve_tuning,use_distance_lod,minimum_route_distance,prop_bounds


def path(points,name='lf_passer'):
    return PersonPath(name,'bystander',(.5,.5,.5),tuple(Waypoint(i,*p) for i,p in enumerate(points)))


def test_distance_selection_checks_between_waypoints_and_both_observers():
    p=path(((-12.,0.),(12.,0.)))
    assert minimum_route_distance(p,(0.,0.))==0
    assert not use_distance_lod(p,resolve_tuning('balanced'))
    assert not use_distance_lod(path(((-3.,-9.),(-3.,-9.))),resolve_tuning('balanced'))
    assert use_distance_lod(path(((12.,0.),(15.,0.))),resolve_tuning('balanced'))
    assert not use_distance_lod(path(((12.,0.),(15.,0.)),'lf_leader'),resolve_tuning('balanced'))
    assert not use_distance_lod(path(((12.,0.),(15.,0.)),'lf_lookalike'),resolve_tuning('balanced'))


def test_reduced_sensor_intrinsics_keep_projection_and_run_local_settings():
    import yaml
    root=Path(__file__).resolve().parents[3]
    original=yaml.safe_load((root/'clearpath/robot.yaml').read_text())
    before=json.dumps(original,sort_keys=True)
    tuning=resolve_tuning('balanced');p=tuning.camera_profile();full=RuntimeTuning().camera_profile()
    assert p.horizontal_fov_rad==pytest.approx(full.horizontal_fov_rad)
    assert p.vertical_fov_rad==pytest.approx(full.vertical_fov_rad)
    assert p.cx/p.focal_length_px==full.cx/full.focal_length_px
    assert (p.width,p.height,p.fps)==(320,240,15.)
    changed=tuning.setup_robot(original)
    assert json.dumps(original,sort_keys=True)==before
    assert changed['sensors']['camera'][0]['ros_parameters']['intel_realsense']['rgb_camera.color_profile']=='320,240,15'
    assert changed['sensors']['lidar2d']==original['sensors']['lidar2d']


def test_primitive_collider_encloses_visual_vertices():
    from ridgeback_leader_follow.realistic_scene import asset_directory
    for file in (asset_directory()/'props').glob('*/mesh.npz'):
        with np.load(file) as data:
            center,size=prop_bounds(data,.75)
            assert np.all(data['points']*.75>=center-size/2-1e-5)
            assert np.all(data['points']*.75<=center+size/2+1e-5)


def test_all_distance_lods_preserve_outfit_scale_and_gait():
    from ridgeback_leader_follow.realistic_scene import human_directory
    from ridgeback_leader_follow.rocketbox_cast import ROSTER
    for model in ROSTER:
        directory=human_directory(model);lod=directory/'lod1'
        full=json.loads((directory/'conversion.json').read_text());receipt=json.loads((lod/'conversion.json').read_text())
        assert receipt['triangles']<=full['triangles']*.55
        assert receipt['height_m']==pytest.approx(full['height_m'],abs=.02)
        assert receipt['gait_speed_mps']==pytest.approx(full['gait_speed_mps'],abs=1e-6)
        assert receipt['idle_loop_gap_m']<.002 and receipt['max_rekey_deviation_m']<.002
        specs=json.loads((lod/'materials.json').read_text())
        original=json.loads((directory/'materials.json').read_text())
        for a,b in zip(specs,original):
            assert a['name']==b['name']
            assert (lod/a['image']).resolve()==(directory/b['image']).resolve()
        for clip in ('walk','idle'):
            with np.load(lod/(clip+'.npz')) as data:
                assert np.isfinite(data['points']).all()
                assert data['points'][:,:,2].min()>=-1e-4
                assert len(data['uv'])==len(data['faces'])*3
                assert data['faces'].max()<data['points'].shape[1]


def test_simple_collisions_keep_visual_assets_and_scenario_paths(tmp_path):
    from ridgeback_leader_follow.realistic_scene import gazebo_world,load_scenario
    scenario=load_scenario('explainer',8,'modern_office')
    full=ET.fromstring(gazebo_world(scenario,tmp_path/'full','modern_office')).find('world')
    fast=ET.fromstring(gazebo_world(scenario,tmp_path/'fast','modern_office',tuning=RuntimeTuning(collision_quality='simple'))).find('world')
    for a,b in zip(full.findall('model'),fast.findall('model')):
        assert a.get('name')==b.get('name')
        aa=ET.tostring(a.find('link/visual'),encoding='unicode').replace(str(tmp_path/'full'),'RUN')
        bb=ET.tostring(b.find('link/visual'),encoding='unicode').replace(str(tmp_path/'fast'),'RUN')
        assert aa==bb
    assert not fast.findall('.//collision/geometry/mesh')


def test_gazebo_reuses_identical_lookalike_skin_files(tmp_path):
    from ridgeback_leader_follow.realistic_scene import gazebo_world,load_scenario
    world=ET.fromstring(gazebo_world(load_scenario('lookalike_parallel',8,'modern_office'),tmp_path,'modern_office')).find('world')
    leader,twin=world.findall('actor')
    assert leader.findtext('skin/filename')==twin.findtext('skin/filename')
    assert leader.findtext('animation/filename')==twin.findtext('animation/filename')


def test_isaac_step_rate_is_opt_in_and_independent_of_profile():
    assert resolve_tuning().isaac_step_hz is None
    assert resolve_tuning('balanced').isaac_step_hz is None
    tuning=resolve_tuning('reference',isaac_step_hz=60)
    assert tuning.isaac_step_hz==60 and tuning.sensor_quality=='reference' and tuning.human_detail=='full'
