"""Scoring must use the recorded humans/routes rather than a baseline mannequin."""
from dataclasses import asdict
import hashlib,json
import numpy as np
import pytest
from ridgeback_leader_follow.benchmarking.scene_geometry import recorded_scenario,projection_meshes
from ridgeback_leader_follow.realistic_scene import load_scenario


def test_recorded_turns_survive_later_catalog_defaults():
    scenario=load_scenario('leader_alone')
    manifest={'scenario':asdict(scenario),'scene_profile':'realistic','crowd_size':4}
    restored=recorded_scenario(manifest)
    assert restored==scenario
    assert restored.people[0].sample(13)[1]==pytest.approx(1.5)


def test_annotation_uses_each_recorded_model_and_scale_and_checks_hash(tmp_path):
    scenario=load_scenario('crossing')
    manifest={'scene_profile':'realistic','appearance':{},'input_sha256':{}}
    for index,person in enumerate(scenario.people):
        model=['sam','harper'][index%2];scale=1.05 if index==0 else .96
        relative='simulation_inputs/assets/realistic/humans/'+model+'/walk.npz'
        path=tmp_path/relative;path.parent.mkdir(parents=True,exist_ok=True)
        np.savez(path,points=np.full((1,3,3),index+1,dtype=np.float32),faces=np.array([[0,1,2]]),fps=24.,cycle_seconds=2.,gait_speed_mps=1.3)
        manifest['appearance'][person.name]={'model':model,'scale':scale}
        manifest['input_sha256'][relative]=hashlib.sha256(path.read_bytes()).hexdigest()
    meshes=projection_meshes(tmp_path,manifest,scenario)
    assert [meshes[p.name][1] for p in scenario.people]==[1.05,.96]
    assert meshes[scenario.people[0].name][0]['points'][0,0,0]==1
    assert meshes[scenario.people[1].name][0]['points'][0,0,0]==2
    next(iter((tmp_path/'simulation_inputs/assets/realistic/humans').rglob('walk.npz'))).write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash mismatch'):projection_meshes(tmp_path,manifest,scenario)


def test_unknown_profiles_cannot_silently_use_baseline_geometry(tmp_path):
    with pytest.raises(ValueError,match='No projection geometry'):
        projection_meshes(tmp_path,{'scene_profile':'unknown'},load_scenario('crossing'))


def test_realistic_geometry_requires_the_recorded_snapshot(tmp_path):
    scenario=load_scenario('leader_alone')
    manifest={'scene_profile':'realistic','appearance':{scenario.people[0].name:{'model':'sam','scale':1.05}}}
    with pytest.raises(ValueError,match='recorded mesh snapshot'):projection_meshes(tmp_path,manifest,scenario)


def test_qualification_matrix_keeps_worlds_and_crowd_densities_distinct():
    from ridgeback_leader_follow.crowd_qualify import qualification_cases
    baseline=qualification_cases(['gz','isaac'],'baseline')
    realistic=qualification_cases(['gz','isaac'],'realistic',['office','warehouse','courtyard'])
    assert len(baseline)==16 and all(case[1] is None for case in baseline)
    assert len(realistic)==48 and len(set(realistic))==48
    with pytest.raises(ValueError,match='requires realistic'):
        qualification_cases(['gz'],'baseline',['office'])
