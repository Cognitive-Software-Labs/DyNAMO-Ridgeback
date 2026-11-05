#!/usr/bin/env python3
"""Run and verify every F1 scene in both simulators; fail on any missing gate."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from ridgeback_leader_follow.crowd_runner import stop_process
from ridgeback_leader_follow.crowd_scenarios import CROWD_SIZES, SCENARIOS, load_scenario


def validate_run(output, scene_profile='baseline'):
    manifest = json.loads((output / 'manifest.json').read_text())
    if manifest['status'] != 'complete':
        raise ValueError(f'Run did not complete: {manifest}')
    if manifest.get('scene_profile', 'baseline') != scene_profile:
        raise ValueError(f'Run used profile {manifest.get("scene_profile")!r}, not {scene_profile!r}')
    if manifest['scenario']['name'] == 'leader_out_of_view' and scene_profile == 'f3':
        from ridgeback_leader_follow.recognition_scenarios import load_recognition_scenario
        scenario = load_recognition_scenario(manifest['scenario']['name'])
    elif scene_profile == 'realistic':
        from ridgeback_leader_follow.realistic_scene import load_scenario as realistic_scenario,manifest_entry
        scenario=realistic_scenario(manifest['scenario']['name'],manifest['crowd_size'],manifest.get('environment','office'))
        expected=json.loads(json.dumps(manifest_entry(scenario,manifest['environment'])))
        if manifest.get('appearance')!=expected['appearance']:
            raise ValueError('Recorded realistic cast differs from its rendering contract')
    else:
        scenario = load_scenario(manifest['scenario']['name'], manifest['crowd_size'])
    if scene_profile == 'f3':
        from ridgeback_leader_follow.f3_scene import manifest_entry
        if manifest.get('appearance') != manifest_entry(scenario):
            raise ValueError('Recorded outfits differ from the deterministic F3 assignment')
    rows = list(csv.DictReader((output / 'ground_truth.csv').open()))
    names = {p.name for p in scenario.people}
    if {r['person'] for r in rows} != names:
        raise ValueError('Missing or unexpected person identities')
    max_error = 0.
    for person in scenario.people:
        samples = [r for r in rows if r['person'] == person.name]
        times = [float(r['scenario_time']) for r in samples]
        if len(samples) < round(manifest['duration'] / .05):
            raise ValueError(f'Missing ground truth for {person.name}: {len(samples)}')
        if times[0] > .075 or times[-1] < manifest['duration'] - .025:
            raise ValueError(f'Incomplete time coverage for {person.name}')
        if any(not 0 < b-a < .075 for a,b in zip(times,times[1:])):
            raise ValueError(f'Non-monotonic or gapped truth for {person.name}')
        for row in samples:
            observed = [float(row[k]) for k in ('x','y','z','yaw')]
            if row['frame'] != 'world' or row['role'] != person.role or not all(map(math.isfinite,observed)):
                raise ValueError('Invalid truth frame, role or pose')
            x,y,yaw = person.sample(float(row['command_time']))
            error = max(math.hypot(observed[0]-x,observed[1]-y),abs(observed[2]),
                        abs(math.atan2(math.sin(observed[3]-yaw),math.cos(observed[3]-yaw))))
            max_error = max(max_error,error)
    if max_error > .005:
        raise ValueError(f'Trajectory/observed transform mismatch: {max_error}')
    health = json.loads((output / 'sensor_health.json').read_text())
    for key in ('rgb','depth','front_scan','rear_scan','camera_info','odom'):
        stat = health.get(key,{})
        if stat.get('run_count',0) < manifest['duration'] * 2 or stat.get('valid_count',0) < 3:
            raise ValueError(f'Insufficient {key} sensor data: {stat}')
        if stat['last_stamp']-stat['first_stamp'] < manifest['duration'] * .9:
            raise ValueError(f'Incomplete {key} sensor coverage')
    if not health.get('tf_static',{}).get('valid_count'):
        raise ValueError('No robot/sensor static transforms')
    calibration = json.loads((output / 'camera_info.json').read_text())
    if (calibration['width'],calibration['height']) != (640,480) or any(
            abs(calibration['k'][i]-384.) > .01 for i in (0,4)):
        raise ValueError('Camera does not match the shared 640x480 D455 profile')
    if not (output / 'frames/rgb_06s.png').exists() and manifest['duration'] > 6:
        raise ValueError('No representative camera frame')
    scan = json.loads((output / 'frames/front_scan_00s.json').read_text())
    nearby = [r for i,r in enumerate(scan['ranges']) if r is not None and .65 < r < 2.
              and abs(scan['angle_min'] + i*scan['angle_increment']) < .8]
    if not nearby:
        raise ValueError('Front lidar has no nearby returns in the initial pedestrian sector')
    return {'backend': manifest['backend'], 'scene_profile': scene_profile,
            **({'environment':manifest['environment']} if scene_profile=='realistic' else {}), 'scenario': scenario.name,
            'crowd_size': manifest['crowd_size'], 'people': len(names),
            'duration': manifest['duration'], 'truth_rows': len(rows),
            'maximum_pose_error': max_error, 'status': 'passed', 'run': str(output)}


def qualification_cases(backends,scene_profile,environments=None,scenarios=SCENARIOS,crowd_sizes=CROWD_SIZES):
    """World names are part of realistic case keys so evidence cannot collide."""
    from ridgeback_leader_follow.realistic_layout import ENVIRONMENTS
    if scene_profile=='realistic':
        worlds=environments or ('modern_office',)
        if any(world not in ENVIRONMENTS for world in worlds):raise ValueError('Unknown realistic environment')
    else:
        if environments:raise ValueError('Environment selection requires realistic profile')
        worlds=(None,)
    return [(backend,world,name,size) for backend in backends for world in worlds
            for name in scenarios for size in (crowd_sizes if name=='crowd' else (4,))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backends', nargs='+', choices=('gz','isaac'), default=['gz','isaac'])
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--ros-domain', type=int, default=87)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--scene-profile', choices=('baseline', 'f3', 'realistic'), default='baseline',
                        help='Scene/sensor gates only; recognition needs separate evaluation')
    parser.add_argument('--environments',nargs='+',choices=('modern_office','office','warehouse','courtyard'))
    parser.add_argument('--scenarios',nargs='+',choices=SCENARIOS,default=SCENARIOS)
    parser.add_argument('--crowd-sizes',nargs='+',type=int,choices=CROWD_SIZES,default=CROWD_SIZES)
    args = parser.parse_args()
    if args.environments and args.scene_profile!='realistic':parser.error('--environments requires --scene-profile realistic')
    label = {'baseline':'F1','f3':'F3 appearance','realistic':'Realistic scenes'}[args.scene_profile]
    root = (args.output_root or Path('artifacts/leader-following') /
            ({'baseline':'f1','f3':'f3-appearance','realistic':'realistic'}[args.scene_profile] + '-qualification-' +
             datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S'))).resolve()
    root.mkdir(parents=True,exist_ok=False)
    cases=qualification_cases(args.backends,args.scene_profile,args.environments,args.scenarios,args.crowd_sizes)
    results = []
    for backend,environment,name,size in cases:
        python = sys.executable if backend == 'gz' else os.environ.get('ISAAC_PYTHON',str(Path.cwd()/'isaac_venv/bin/python3'))
        key=f'{backend}-{environment}-{name}-{size}' if environment else f'{backend}-{name}-{size}'
        output = root / key
        command = [python,'-m','ridgeback_leader_follow.crowd_runner','--backend',backend,
                   '--scenario',name,'--crowd-size',str(size),'--ros-domain',str(args.ros_domain),
                   '--output-dir',str(output),'--scene-profile',args.scene_profile]
        if environment:command.extend(['--environment',environment])
        if args.record:
            command.append('--record')
        print(f'{label} {backend}: {name} ({size} bystanders for crowd)',flush=True)
        with (root / f'{output.name}.log').open('w') as log:
            process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code = process.wait(timeout=360)
            finally:
                stop_process(process)
        if code:
            (root / 'qualification.json').write_text(json.dumps({
                'status': 'failed', 'failed_case': output.name, 'exit_code': code,
                'results': results},indent=2)+'\n')
            raise RuntimeError(f'{output.name} failed (exit {code}); see {output.name}.log')
        results.append(validate_run(output, args.scene_profile))
        (root / 'qualification.json').write_text(json.dumps({
            'status': 'running', 'results': results},indent=2)+'\n')
        print('  passed',flush=True)
    scope = ('F1 scenes, sensors, animation and simulator truth; no F2-F4 claims'
             if args.scene_profile == 'baseline' else
             f'{label}: playback, sensors and simulator truth; no recognition claims')
    (root / 'qualification.json').write_text(json.dumps({
        'status': 'passed', 'scene_profile': args.scene_profile, 'scope': scope,
        'environments':args.environments or (['modern_office'] if args.scene_profile=='realistic' else []),
        'results': results},indent=2)+'\n')
    print(f'{label} qualification passed; evidence: {root}',flush=True)


if __name__ == '__main__':
    main()
