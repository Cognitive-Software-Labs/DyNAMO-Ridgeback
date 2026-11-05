"""Cross-backend fixture geometry and deterministic trajectory checks."""
import csv
from dataclasses import replace
import io
import math
from xml.etree import ElementTree as ET

import pytest

from ridgeback_leader_follow.crowd_geometry import asset_directory, gazebo_world
from ridgeback_leader_follow.crowd_runner import FIELDS, main, write_sample
from ridgeback_leader_follow.crowd_scenarios import CROWD_SIZES, SCENARIOS, load_scenario, Waypoint


def test_leader_motion_turn_stop_and_clamping():
    leader = load_scenario('leader_alone').people[0]
    assert leader.sample(-1.) == (1.5, 0., 0.)
    assert leader.sample(6.) == (4.5, 0., 0.)
    assert leader.sample(14.) == (7.5, .75, math.pi / 2)
    assert leader.sample(99.) == (7.5, 1.5, math.pi / 2)


def test_crossing_is_between_observer_and_leader():
    leader, passer = load_scenario('crossing').people
    lx, ly, _ = leader.sample(6.)
    px, py, _ = passer.sample(6.)
    assert ly == py == 0.
    assert 0. < px < lx
    assert passer.sample(0.)[:2] == passer.sample(2.)[:2]


def test_invalid_inputs_fail_before_scene_mutation():
    scenario = load_scenario('crossing')
    with pytest.raises(ValueError):
        replace(scenario, people=(scenario.people[0], scenario.people[0]))
    with pytest.raises(ValueError):
        replace(scenario.people[0], waypoints=(Waypoint(0., 0., 0.), Waypoint(0., 1., 0.)))
    with pytest.raises(ValueError):
        scenario.people[0].sample(float('nan'))
    with pytest.raises(ValueError):
        load_scenario('unknown')
    with pytest.raises(SystemExit):
        main(['--backend', 'gz', '--duration', '0.001'])


def test_gazebo_has_skinned_actors_and_native_truth_plugin():
    from ament_index_python.packages import get_package_prefix
    from pathlib import Path
    if not (Path(get_package_prefix('ridgeback_leader_follow')) / 'lib/ridgeback_leader_follow/libCrowdMotion.so').exists():
        pytest.skip('Optional Gazebo profile is not installed on this host')
    scenario = load_scenario('crossing')
    root = ET.fromstring(gazebo_world(scenario))
    for person in scenario.people:
        actor = root.find(f"world/actor[@name='{person.name}']")
        assert actor.find('skin/filename').text.endswith('walk.dae')
        assert actor.find('animation').attrib['name'] == 'walk'
    plugin = root.find("world/plugin[@name='leader_follow::CrowdMotion']")
    assert len(plugin.findall('person')) == 2


@pytest.mark.parametrize('name', SCENARIOS)
def test_all_f1_scenario_families_have_finite_paths(name):
    for count in CROWD_SIZES:
        scenario = load_scenario(name, count)
        if name == 'crowd':
            assert len(scenario.people) == count + 1
        else:
            assert len(scenario.people) >= 1
        for person in scenario.people:
            for t in (0., 6., 10., 14., 20.):
                x,y,yaw = person.sample(t)
                assert -3 < x < 15 and abs(y) < 2.6 and math.isfinite(yaw)


def test_gait_freezes_when_people_stop():
    scenario = load_scenario('leader_stop')
    assert scenario.people[0].animation_time(8.) == scenario.people[0].animation_time(20.)
    assert scenario.people[1].animation_time(20.) == 0.


def test_human_asset_is_attributed_animated_and_full_height():
    import json, hashlib
    import numpy as np
    folder = asset_directory()
    receipt = json.loads((folder / 'conversion.json').read_text())
    for name, digest in receipt['sha256'].items():
        assert hashlib.sha256((folder / name).read_bytes()).hexdigest() == digest
    assert 'Creative Commons Attribution' in (folder / 'LICENSE.md').read_text()
    data = np.load(folder / 'walk.npz')
    assert data['points'].shape[0] == 48
    assert np.ptp(data['points'][0,:,2]) == pytest.approx(1.75, abs=.001)
    assert np.max(np.abs(data['points'][0] - data['points'][24])) > .1


def test_truth_preserves_observed_pose_separately_from_command():
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    person = load_scenario('leader_alone').people[0]
    write_sample(writer, person, 1., .5, .45, (1.7, .1, 0., .2))
    row = next(csv.DictReader(io.StringIO(stream.getvalue())))
    assert float(row['x']) == 1.7
    assert float(row['command_x']) == 1.725
    assert row['frame'] == 'world'


@pytest.mark.parametrize('fault', ('none', 'missing_person', 'wrong_pose', 'duplicate_time', 'missing_depth', 'wrong_calibration'))
def test_qualification_rejects_incomplete_or_incorrect_evidence(tmp_path, fault):
    import json
    from dataclasses import asdict
    from ridgeback_leader_follow.crowd_qualify import validate_run
    scenario = load_scenario('crossing')
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'status': 'complete', 'scenario': asdict(scenario), 'crowd_size': 4,
        'duration': 20., 'backend': 'gz'}))
    with (tmp_path / 'ground_truth.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=FIELDS)
        writer.writeheader()
        for index in range(1,401):
            time = index * .05
            for person in scenario.people:
                if fault == 'missing_person' and person.role == 'bystander':
                    continue
                x,y,yaw = person.sample(time)
                stamp = time-.05 if fault == 'duplicate_time' and index == 2 else time
                write_sample(writer, person, time, stamp, time,
                             (x+.1 if fault == 'wrong_pose' else x,y,0.,yaw))
    health = {key: {'run_count': 400, 'valid_count': 400,
                    'first_stamp': 0., 'last_stamp': 20.} for key in
              ('rgb','depth','front_scan','rear_scan','camera_info','odom','tf_static')}
    if fault == 'missing_depth':
        health['depth']['run_count'] = 0
    (tmp_path / 'sensor_health.json').write_text(json.dumps(health))
    (tmp_path / 'camera_info.json').write_text(json.dumps({
        'width': 640, 'height': 480, 'k': [443. if fault == 'wrong_calibration' else 384.,0,320.,0,384.,240.,0,0,1]}))
    (tmp_path / 'frames').mkdir()
    (tmp_path / 'frames/rgb_06s.png').touch()
    (tmp_path / 'frames/front_scan_00s.json').write_text(json.dumps({
        'angle_min': 0., 'angle_increment': .01, 'ranges': [1.]}))
    if fault == 'none':
        assert validate_run(tmp_path)['status'] == 'passed'
    else:
        with pytest.raises(ValueError):
            validate_run(tmp_path)


def test_sensor_readiness_requires_complete_extrinsics():
    from ridgeback_leader_follow.crowd_sensors import SensorEvidence
    from types import SimpleNamespace
    monitor=SensorEvidence.__new__(SensorEvidence)
    monitor.stats={k:{'valid_count':3,'frame':k} for k in
                   ('rgb','depth','camera_info','front_scan','rear_scan','odom','tf_static')}
    monitor.tf_buffer=SimpleNamespace(can_transform=lambda target,source,at:source != 'rear_scan')
    assert not monitor.ready
    monitor.tf_buffer=SimpleNamespace(can_transform=lambda target,source,at:True)
    assert monitor.ready
