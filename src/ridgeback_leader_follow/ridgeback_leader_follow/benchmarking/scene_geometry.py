"""Evaluation-only recorded route and character geometry; no inference inputs."""
from pathlib import Path
import numpy as np
from ridgeback_leader_follow.crowd_geometry import asset_directory
from ridgeback_leader_follow.crowd_scenarios import load_scenario

def recorded_scenario(manifest):
    """Evaluation uses the route actually recorded, including custom/turn routes."""
    from ridgeback_leader_follow.crowd_scenarios import CrowdScenario,PersonPath,Waypoint
    spec=manifest['scenario']
    if 'people' in spec:
        return CrowdScenario(spec['name'],spec['duration'],tuple(
            PersonPath(p['name'],p['role'],tuple(p['color']),tuple(Waypoint(**w) for w in p['waypoints']))
            for p in spec['people']))
    if spec['name']=='leader_out_of_view' and manifest.get('scene_profile')=='f3':
        from ridgeback_leader_follow.recognition_scenarios import load_recognition_scenario
        return load_recognition_scenario(spec['name'])
    return load_scenario(spec['name'],manifest['crowd_size'])


def projection_meshes(run,manifest,scenario):
    """Per-person recorded rendering geometry; evaluation only, never inference."""
    import hashlib
    profile=manifest.get('scene_profile','baseline')
    if profile not in ('baseline','f3','lab','realistic'):
        raise ValueError('No projection geometry for scene profile '+repr(profile))
    snapshot=run/'simulation_inputs/assets';cache={};assigned={}
    for person in scenario.people:
        scale=1.
        if profile=='realistic':
            spec=manifest.get('appearance',{}).get(person.name)
            if spec is None:raise ValueError('Missing recorded character for '+person.name)
            model=spec['model'];scale=float(spec['scale'])
            if model not in ('maya','sam','harper','jordan') or not np.isfinite(scale) or scale<=0:
                raise ValueError('Invalid recorded character or scale')
            relative=Path('maya') if model=='maya' else Path('realistic/humans')/model
        else:
            relative=Path('maya' if profile in ('f3','lab') else 'cesium_man')
        if relative not in cache:
            path=snapshot/relative/'walk.npz'
            if not path.exists():
                if profile=='realistic':raise ValueError('Realistic scoring requires the recorded mesh snapshot: '+str(path))
                path=asset_directory().parent/relative/'walk.npz'
            expected=manifest.get('input_sha256',{}).get(str(Path('simulation_inputs/assets')/relative/'walk.npz'))
            if profile=='realistic' and expected is None:raise ValueError('Recorded mesh hash absent: '+str(path))
            if expected is not None and hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
                raise ValueError('Recorded mesh hash mismatch: '+str(path))
            with np.load(path) as source:
                cache[relative]={k:source[k] for k in ('points','faces','fps','cycle_seconds','gait_speed_mps') if k in source}
        assigned[person.name]=(cache[relative],scale)
    return assigned

