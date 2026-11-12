"""Modern office with a leader who wanders the hall at random (seeded).

The ten other walkers keep their explainer routes, so they cross, oncoming
and stand in the leader's way. The leader starts 1.4 m in front of the robot,
facing away, then walks a random rounded route through the open hall at a
random natural pace. Every route keeps clear of furniture and of the other
walkers; the seed makes it reproducible.
"""
import json
import math
import random

import numpy as np

from .crowd_scenarios import CrowdScenario, PersonPath

HALL = ((-2.5, 21.5), (-3.4, 3.4))   # open hall (x, y) in metres, inside a 0.6 m furniture margin
FURNITURE_CLEARANCE = .6             # metres from any collidable furniture
PEOPLE_CLEARANCE = .8                # metres from every other walker at every moment
DURATION = 40.


def static_obstacles(environment='modern_office'):
    """Axis-aligned (centre, half-size) boxes of collidable furniture below 1.9 m."""
    from .realistic_layout import layout
    from .realistic_scene import asset_directory
    boxes, props = layout(environment)
    obstacles = []
    for box in boxes:
        if box.collision and box.name != 'Ground' and box.position[2] + box.size[2] / 2 > .1 \
                and box.position[2] - box.size[2] / 2 < 1.9:
            obstacles.append((np.array(box.position[:2]), np.array(box.size[:2]) / 2))
    for prop in props:
        if not prop.collision or prop.position[2] > 1.9:
            continue
        dimensions = json.loads((asset_directory() / 'props' / prop.asset / 'conversion.json').read_text())['dimensions_m']
        angle = math.radians(prop.yaw)
        c, s = abs(math.cos(angle)), abs(math.sin(angle))
        half = np.array([[c, s], [s, c]]) @ np.array(dimensions[:2]) * prop.scale / 2
        obstacles.append((np.array(prop.position[:2]), half))
    return obstacles


def furniture_distance(point, obstacles):
    return min(np.linalg.norm(np.maximum(abs(point - centre) - half, 0.)) for centre, half in obstacles)


def _random_polyline(rng, start, heading, length, obstacles):
    points = [np.array(start, float)]
    travelled = 0.
    while travelled < length:
        for attempt in range(120):
            # Prefer gentle turns; widen to turning around when boxed in.
            limit = 75 if attempt < 40 else 180
            turn = math.radians(rng.uniform(-limit, limit))
            step = rng.uniform(2.5, 5.)
            candidate_heading = heading + turn
            end = points[-1] + step * np.array([math.cos(candidate_heading), math.sin(candidate_heading)])
            if not (HALL[0][0] <= end[0] <= HALL[0][1] and HALL[1][0] <= end[1] <= HALL[1][1]):
                continue
            samples = [points[-1] + (end - points[-1]) * f for f in np.linspace(0, 1, 12)]
            if all(furniture_distance(p, obstacles) >= FURNITURE_CLEARANCE for p in samples):
                break
        else:
            return None
        points.append(end)
        heading = candidate_heading
        travelled += step
    return points


def _truncate(points, length):
    """Cut a polyline to ``length`` metres."""
    kept, travelled = [points[0]], 0.
    for a, b in zip(points, points[1:]):
        step = float(np.linalg.norm(b - a))
        if travelled + step >= length:
            kept.append(a + (b - a) * (length - travelled) / step)
            return kept
        kept.append(b)
        travelled += step
    return kept


def load_wander_scenario(seed=0, duration=DURATION):
    from .explainer_motion import load_scenario as illustration
    from .realistic_scene import route
    others = illustration().people[1:]
    obstacles = static_obstacles()
    rng = random.Random(seed)
    times = np.arange(0., duration + 1e-9, .1)
    for _ in range(2000):
        pace = rng.uniform(.8, 1.)
        start = .5
        # First leg straight ahead, so the robot sees the leader's back at the start.
        first = (1.4 + rng.uniform(2.5, 4.), 0.)
        rest = _random_polyline(rng, first, 0., pace * (duration - start - 2.) - (first[0] - 1.4), obstacles)
        if rest is None:
            continue
        points = _truncate([np.array((1.4, 0.))] + rest, .95 * pace * (duration - start - 1.))
        try:
            waypoints = route([tuple(p) for p in points], start, pace, duration)
        except ValueError:
            continue
        leader = PersonPath('lf_leader', 'leader', (.15, .35, .85), waypoints, 0.)
        path = np.array([leader.sample(t)[:2] for t in times])
        if any(np.hypot(*(path - np.array([o.sample(t)[:2] for t in times])).T).min() < PEOPLE_CLEARANCE
               for o in others):
            continue
        if min(furniture_distance(p, obstacles) for p in path[::5]) < FURNITURE_CLEARANCE - .05:
            continue
        return CrowdScenario('wander', duration, (leader,) + tuple(others))
    raise RuntimeError(f'no clear wandering route found for seed {seed}')
