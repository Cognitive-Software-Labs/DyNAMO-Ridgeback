"""Exact 24-second pedestrian choreography from the explainer's sim block.

SVG coordinates map to metres by x'=x-2.2, y'=4.6-y. The observer
starts at the origin. Robot following in the illustration is not replayed.
"""
import math
from .crowd_scenarios import CrowdScenario,PersonPath,Waypoint

LEADER=((0,3.6,4.6),(5,8.6,4.6),(9,12.2,3.4),(14,16.4,3.),(18,19.6,3.8),(24,21.6,4.8))
OTHERS=(
 ((0,9.3,9.6),(3.5,9.3,9.),(9.5,9.3,0.),(24,9.3,-12.)),
 ((0,14.9,9.1),(24,14.9,-1.7)),
 ((0,14.9,9.7),(24,14.9,-1.1)),
 ((0,17.6,1.4),(24,17.6,1.4)),
 ((0,18.2,1.1),(24,18.2,1.1)),
 ((0,18.15,1.85),(24,18.15,1.85)),
 ((0,26.,7.3),(24,15.5,3.7)),
 ((0,.8,7.8),(24,19.,8.3)),
 ((0,21.,7.6),(24,3.,6.9)),
 ((0,19.,.4),(24,1.5,.6)),
)


def load_scenario():
    def person(name,role,keys):
        points=tuple(Waypoint(t,x-2.2,4.6-y) for t,x,y in keys)
        a,b=points[:2];heading=math.atan2(b.y-a.y,b.x-a.x)
        return PersonPath(name,role,(.15,.35,.85) if name in ('lf_leader','lf_lookalike') else (.5,.5,.5),points,heading)
    return CrowdScenario('explainer',24.,(person('lf_leader','leader',LEADER),)+tuple(
        person('lf_lookalike' if i==6 else f'lf_illustration_{i}','bystander',keys)
        for i,keys in enumerate(OTHERS)))


# Weaving variant: the same eleven walkers, but the leader drifts left and
# right of the illustrated route like a person who is not walking a ruler line.
WEAVE_AMPLITUDE = .6     # metres either side of the illustrated route
WEAVE_WAVELENGTH = 6.    # metres walked per full left-right cycle
WEAVE_PHASE = 1.963     # radians; keeps every other walker at least 0.79 m away
WEAVE_STEP = .2          # seconds between generated waypoints
WEAVE_SMOOTHING = 1.5    # metres of route averaged for the sideways direction


def weave_offset(distance, total, amplitude=WEAVE_AMPLITUDE, wavelength=WEAVE_WAVELENGTH, phase=WEAVE_PHASE):
    """Lateral offset after ``distance`` metres; zero at both ends of the walk."""
    taper = min(1., distance / 2., max(0., total - distance) / 2.)
    return amplitude * taper * math.sin(2 * math.pi * distance / wavelength + phase)


def load_weave_scenario(amplitude=WEAVE_AMPLITUDE, wavelength=WEAVE_WAVELENGTH, phase=WEAVE_PHASE):
    """Explainer scene with a weaving leader; every other walker is unchanged."""
    base = load_scenario()
    leader = base.people[0]
    total = leader.distance(base.duration)
    # Sideways direction from the route smoothed over WEAVE_SMOOTHING metres,
    # so the weave bends gently through the illustration's corners.
    fine = [i * .05 for i in range(round(base.duration / .05) + 1)]
    track = [(leader.distance(t), *leader.sample(t)[:2]) for t in fine]
    def normal(distance):
        near = [(x, y) for d, x, y in track if abs(d - distance) <= WEAVE_SMOOTHING / 2]
        (x0, y0), (x1, y1) = near[0], near[-1]
        length = math.hypot(x1 - x0, y1 - y0)
        return (-(y1 - y0) / length, (x1 - x0) / length) if length > 1e-6 else (0., 1.)
    points = []
    for i in range(round(base.duration / WEAVE_STEP) + 1):
        t = round(i * WEAVE_STEP, 6)
        x, y, _ = leader.sample(t)
        distance = leader.distance(t)
        offset = weave_offset(distance, total, amplitude, wavelength, phase)
        nx, ny = normal(distance)
        points.append(Waypoint(t, x + offset * nx, y + offset * ny))
    kept = [points[0]]
    for point in points[1:]:
        if (point.x, point.y) != (kept[-1].x, kept[-1].y) or point is points[-1]:
            kept.append(point)
    weaving = PersonPath(leader.name, leader.role, leader.color, tuple(kept), leader.initial_heading)
    return CrowdScenario('explainer_weave', base.duration, (weaving,) + base.people[1:])
