"""Parking demo scenes: a G1 workstation per demo world, and scenarios relative to it.

Pure Python (no ROS imports), shared by the simulation startup that spawns a scene
and by the offline evaluation tools that read a run's recorded scenario file.

Schema 2 places every scene at a *workstation*: G1 standing at a worktable,
facing it, at a pose in each demo world that the robot cannot see from its
start. Each scenario states G1's turn relative to the workstation heading and
its objects in the workstation frame (``forward`` toward the worktable,
``left`` to G1's left when it faces the table), so the same scenario has the
same layout around G1 in Gazebo and Isaac. Schema 1 (world coordinates, the
P7 scenes) is still read for recorded runs.
"""

from __future__ import annotations

import math

SCHEMA_V1 = 'ridgeback_parking_scenarios/1'
SCHEMA_V2 = 'ridgeback_parking_scenarios/2'
BACKENDS = ('gz', 'isaac')


def _check(data):
    if not isinstance(data, dict) or data.get('schema') not in (SCHEMA_V1, SCHEMA_V2):
        raise ValueError('unsupported parking scenario schema')
    if not isinstance(data.get('scenarios'), dict) or not data['scenarios']:
        raise ValueError('parking scenario file defines no scenarios')
    if data['schema'] == SCHEMA_V2:
        stations = data.get('workstations') or {}
        missing = [b for b in BACKENDS if b not in stations]
        if missing:
            raise ValueError(f'parking scenarios define no workstation for {missing}')
    return data


def load(path):
    import yaml
    with open(path, encoding='utf-8') as stream:
        return _check(yaml.safe_load(stream))


def backends(scene):
    return tuple(scene.get('backends', BACKENDS))


def _place(station, forward, left, yaw_deg):
    heading = math.radians(station['yaw_deg'])
    x = station['x'] + forward * math.cos(heading) - left * math.sin(heading)
    y = station['y'] + forward * math.sin(heading) + left * math.cos(heading)
    yaw = (station['yaw_deg'] + yaw_deg + 180.0) % 360.0 - 180.0
    return round(x, 4), round(y, 4), round(yaw, 4)


def world_layout(data, name, backend):
    """G1 and objects of scenario ``name`` in ``backend``'s world frame.

    Returns ``{'g1': {x, y, yaw_deg}, 'objects': [{name, asset, x, y, yaw_deg}],
    'workstation': {...} or None}``; raises ValueError for an unknown scenario
    or a backend the scenario does not fit.
    """
    _check(data)
    scenarios = data['scenarios']
    if name not in scenarios:
        raise ValueError(f'unknown demo scenario {name!r}; known: {", ".join(scenarios)}')
    scene = scenarios[name]
    if backend not in backends(scene):
        raise ValueError(f'demo scenario {name!r} does not fit the {backend} world; '
                         f'supported: {", ".join(backends(scene))}')
    if data['schema'] == SCHEMA_V1:
        g1 = scene['g1']
        return {'g1': {'x': float(g1['x']), 'y': float(g1['y']),
                       'yaw_deg': float(g1['yaw_deg'])},
                'objects': [dict(o) for o in scene.get('objects') or []],
                'workstation': None}
    station = data['workstations'][backend]
    gx, gy, gyaw = _place(station, 0.0, 0.0, float(scene.get('g1_turn_deg', 0.0)))
    objects = []
    for item in scene.get('objects') or []:
        x, y, yaw = _place(station, float(item['forward']), float(item['left']),
                           float(item['yaw_deg']))
        objects.append({'name': item['name'], 'asset': item['asset'],
                        'x': x, 'y': y, 'yaw_deg': yaw})
    names = [o['name'] for o in objects]
    if len(set(names)) != len(names):
        raise ValueError(f'duplicate object names in demo scenario {name!r}')
    return {'g1': {'x': gx, 'y': gy, 'yaw_deg': gyaw}, 'objects': objects,
            'workstation': dict(station)}
