"""Parking demo scenes: a G1 workstation per demo world, and scenarios relative to it.

Pure Python (no ROS imports), shared by the simulation startup that spawns a scene
and by the offline evaluation tools that read a run's recorded scenario file.

Schema 2 places every scene at a *workstation*: G1 standing at a worktable,
facing it, at a pose in each demo world that the robot cannot see from its
start. Each scenario states G1's turn relative to the workstation heading and
its objects in the workstation frame (``forward`` toward the worktable,
``left`` to G1's left when it faces the table), so the same scenario has the
same layout around G1 in Gazebo and Isaac. Schema 3 keys workstations by demo
world, so both simulators share the hospital ward's (``parking_demo``) and Gazebo's
warehouse (``depot``) keeps its own. Schemas 2 (workstations per backend) and 1
(world coordinates, the P7 scenes) are still read for recorded runs.
"""

from __future__ import annotations

import math

SCHEMA_V1 = 'ridgeback_parking_scenarios/1'
SCHEMA_V2 = 'ridgeback_parking_scenarios/2'
SCHEMA_V3 = 'ridgeback_parking_scenarios/3'
BACKENDS = ('gz', 'isaac')
# The worlds each backend's parking demo may load; the first is its default. Both
# backends' parking_demo is initial_test_world (the hospital ward) without its
# authored G1. Gazebo's depot (Clearpath's warehouse) reproduces the earlier
# Gazebo runs and is selected explicitly with world:=depot.
DEMO_WORLDS = {'gz': ('parking_demo', 'depot'), 'isaac': ('parking_demo',)}
# The floor top of a schema-2 recorded run, by backend (schema 3 records it per world).
LEGACY_FLOOR_Z = {'gz': 0.0, 'isaac': 0.05}


def _check(data):
    if not isinstance(data, dict) or data.get('schema') not in (SCHEMA_V1, SCHEMA_V2, SCHEMA_V3):
        raise ValueError('unsupported parking scenario schema')
    if not isinstance(data.get('scenarios'), dict) or not data['scenarios']:
        raise ValueError('parking scenario file defines no scenarios')
    if data['schema'] == SCHEMA_V2:
        stations = data.get('workstations') or {}
        missing = [b for b in BACKENDS if b not in stations]
        if missing:
            raise ValueError(f'parking scenarios define no workstation for {missing}')
    if data['schema'] == SCHEMA_V3:
        stations = data.get('workstations') or {}
        missing = sorted({w for worlds in DEMO_WORLDS.values() for w in worlds} - set(stations))
        if missing:
            raise ValueError(f'parking scenarios define no workstation for {missing}')
        unfloored = [w for w, station in stations.items() if 'floor_z' not in station]
        if unfloored:
            raise ValueError(f'parking workstations without floor_z: {unfloored}')
    return data


def default_world(backend):
    """The world a backend's parking demo loads unless world:= names another."""
    return DEMO_WORLDS[backend][0]


def workstation(data, backend, world=None):
    """G1's workstation (x, y, yaw_deg, floor_z) in ``world`` (default: the backend's
    default demo world); a schema-2 file has one per backend and ignores ``world``."""
    _check(data)
    if data['schema'] == SCHEMA_V2:
        return {**data['workstations'][backend], 'floor_z': LEGACY_FLOOR_Z[backend]}
    if data['schema'] != SCHEMA_V3:
        raise ValueError('schema 1 parking scenarios place no workstation')
    world = world or default_world(backend)
    if world not in DEMO_WORLDS[backend]:
        raise ValueError(f'{backend} parking demo supports world {", ".join(DEMO_WORLDS[backend])}, '
                         f'not {world!r}')
    return dict(data['workstations'][world])


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


def world_layout(data, name, backend, world=None):
    """G1 and objects of scenario ``name`` in the world frame of ``backend`` loading
    ``world`` (default: its default demo world).

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
    station = workstation(data, backend, world)
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
