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


# Roaming variant: a leader who goes "here and there" among walkers who keep
# moving for the whole scene. Legs are shorter and turns sharper than in
# ``wander``; the leader changes pace between stretches and sometimes stops,
# and never walks back into the robot following them (``follower_gap``).
ROAM_DURATION = 60.
ROAM_PACE = (.6, 1.35)       # m/s, drawn per stretch between stops
ROAM_STOP = (1., 3.)         # seconds standing at a stop
ROAM_STOP_CHANCE = .35       # chance of stopping at each corner
ROAM_LEG = (2., 4.5)         # metres per leg
ROAM_TURN = 120.             # degrees either way at a corner (wider when boxed in)


def ping_pong(person, duration):
    """Walk the recorded route forward, then back, until ``duration``.

    Every walker reverses at the same instants, so the distance between any
    two of them at time T + s equals the distance at T - s: the clearances of
    the original choreography hold for the whole scene.
    """
    from dataclasses import replace
    from .crowd_scenarios import Waypoint
    forward = person.waypoints
    period = forward[-1].time
    points, offset, reverse = list(forward), period, True
    while offset < duration:
        leg = [Waypoint(offset + (period - w.time), w.x, w.y) for w in reversed(forward[:-1])] if reverse \
            else [Waypoint(offset + w.time, w.x, w.y) for w in forward[1:]]
        points += leg
        offset += period
        reverse = not reverse
    return replace(person, waypoints=tuple(_cut(points, duration)))


def _cut(waypoints, end):
    """Waypoints up to time ``end``, ending exactly there."""
    from .crowd_scenarios import Waypoint
    kept = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        if b.time >= end:
            f = (end - a.time) / (b.time - a.time)
            kept.append(Waypoint(end, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)))
            return kept
        kept.append(b)
    return kept


def stop_and_go(rng, points, start, duration):
    """Leader waypoints along ``points``: a new pace per stretch, random stops at corners."""
    from .crowd_scenarios import Waypoint
    from .realistic_scene import route
    waypoints = [Waypoint(0., *map(float, points[0]))]
    t, stretch = start, [points[0]]
    for i, point in enumerate(points[1:], 1):
        stretch.append(point)
        if i < len(points) - 1 and rng.random() >= ROAM_STOP_CHANCE:
            continue
        walked = route(stretch, t, rng.uniform(*ROAM_PACE), 1e6)
        waypoints += [w for w in walked if 0 < w.time < 1e6 and w.time > waypoints[-1].time]
        t = waypoints[-1].time + rng.uniform(*ROAM_STOP)
        stretch = [point]
        if t >= duration:
            break
    if waypoints[-1].time < duration - 1.:
        raise ValueError('route ends early')
    waypoints = _cut(waypoints, duration - 1.)
    return tuple(waypoints) + (Waypoint(duration, waypoints[-1].x, waypoints[-1].y),)


def _roam_polyline(rng, start, heading, length, obstacles):
    points = [np.array(start, float)]
    travelled = 0.
    while travelled < length:
        for attempt in range(150):
            limit = ROAM_TURN if attempt < 60 else 180
            turn = math.radians(rng.uniform(-limit, limit))
            step = rng.uniform(*ROAM_LEG)
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


FOLLOWER_CLEARANCE = .9     # metres: the leader walks around, not into, the robot following them


def follower_gap(path, dt, start=(0., 0.), follow_distance=1.2, gain=.8, max_speed=1.3):
    """Closest leader approach to a point robot that follows ``path`` (sampled every ``dt``).

    The follower drives straight at the leader with follow_controller's pace
    rule and holds still within the follow distance or while the leader walks
    toward it. A scripted leader who would walk into it is rejected: a person
    leading a robot walks around it.
    """
    robot = np.array(start, float)
    closest = math.inf
    for i in range(1, len(path)):
        offset = path[i] - robot
        gap = float(np.linalg.norm(offset))
        closest = min(closest, gap)
        receding = float(np.dot(path[i] - path[i - 1], offset) / dt / gap) if gap > 1e-6 else 0.
        speed = max(0., min(max_speed, receding + gain * (gap - follow_distance)))
        if gap > follow_distance + .15:
            robot += offset / gap * min(speed * dt, gap - follow_distance)
    return closest


def load_roam_scenario(seed=0, duration=ROAM_DURATION):
    """All eleven office walkers moving for ``duration``; the leader roams (seeded)."""
    from .explainer_motion import load_scenario as illustration
    others = tuple(ping_pong(p, duration) for p in illustration().people[1:])
    obstacles = static_obstacles()
    rng = random.Random(seed)
    times = np.arange(0., duration + 1e-9, .1)
    tracks = [np.array([o.sample(t)[:2] for t in times]) for o in others]
    for _ in range(3000):
        first = (1.4 + rng.uniform(2., 3.5), 0.)
        rest = _roam_polyline(rng, first, 0., ROAM_PACE[1] * duration, obstacles)
        if rest is None:
            continue
        try:
            waypoints = stop_and_go(rng, [(1.4, 0.)] + [tuple(p) for p in rest], .5, duration)
        except ValueError:
            continue
        leader = PersonPath('lf_leader', 'leader', (.15, .35, .85), waypoints, 0.)
        path = np.array([leader.sample(t)[:2] for t in times])
        if any(np.hypot(*(path - track).T).min() < PEOPLE_CLEARANCE for track in tracks):
            continue
        if min(furniture_distance(p, obstacles) for p in path[::5]) < FURNITURE_CLEARANCE - .05:
            continue
        if follower_gap(path, .1) < FOLLOWER_CLEARANCE:
            continue
        return CrowdScenario('roam', duration, (leader,) + others)
    raise RuntimeError(f'no clear roaming route found for seed {seed}')
