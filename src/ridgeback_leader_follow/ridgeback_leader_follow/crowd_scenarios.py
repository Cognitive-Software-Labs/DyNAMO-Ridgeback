"""Deterministic F1 motion fixtures; metres, seconds, world frame, floor at z=0."""
from dataclasses import dataclass
import math

SCENARIOS = ('leader_alone', 'crossing', 'lookalike_parallel',
             'lookalike_crossing', 'leader_stop', 'crowd')
CROWD_SIZES = (2, 4, 8)


@dataclass(frozen=True)
class Waypoint:
    time: float
    x: float
    y: float


@dataclass(frozen=True)
class PersonPath:
    name: str
    role: str
    color: tuple[float, float, float]
    waypoints: tuple[Waypoint, ...]

    def __post_init__(self):
        if not self.name.isidentifier() or self.role not in ('leader', 'bystander'):
            raise ValueError('Person needs an identifier and a leader/bystander role')
        if len(self.color) != 3 or any(not 0 <= c <= 1 for c in self.color):
            raise ValueError('Color must contain three values in [0, 1]')
        if len(self.waypoints) < 2 or self.waypoints[0].time != 0:
            raise ValueError('At least two waypoints, starting at time zero, are required')
        if any(not all(math.isfinite(v) for v in (p.time, p.x, p.y))
               for p in self.waypoints):
            raise ValueError('Waypoints must be finite')
        if any(b.time <= a.time for a, b in zip(self.waypoints, self.waypoints[1:])):
            raise ValueError('Waypoint times must strictly increase')

    def sample(self, time: float) -> tuple[float, float, float]:
        """Clamped linear position and travel heading; retain heading at a stop."""
        if not math.isfinite(time):
            raise ValueError('Time must be finite')
        time = max(0., min(time, self.waypoints[-1].time))
        heading = 0.
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            dx, dy = b.x - a.x, b.y - a.y
            if dx or dy:
                heading = math.atan2(dy, dx)
            if time < b.time:
                fraction = (time - a.time) / (b.time - a.time)
                return a.x + fraction * dx, a.y + fraction * dy, heading
        end = self.waypoints[-1]
        return end.x, end.y, heading

    def distance(self, time: float) -> float:
        """Travelled arc length drives gait and freezes it while standing."""
        total = 0.
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            fraction = max(0., min(1., (time - a.time) / (b.time - a.time)))
            total += fraction * math.hypot(b.x - a.x, b.y - a.y)
        return total

    def moving(self, time: float) -> bool:
        """True while a segment with displacement is being walked; False when standing."""
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            if a.time <= time < b.time:
                return (b.x, b.y) != (a.x, a.y)
        return False

    def animation_time(self, time: float) -> float:
        return self.distance(time) / 0.5


@dataclass(frozen=True)
class CrowdScenario:
    name: str
    duration: float
    people: tuple[PersonPath, ...]

    def __post_init__(self):
        if not math.isfinite(self.duration) or self.duration <= 0:
            raise ValueError('Duration must be positive and finite')
        if len({p.name for p in self.people}) != len(self.people):
            raise ValueError('Person names must be unique')
        if sum(p.role == 'leader' for p in self.people) != 1:
            raise ValueError('Exactly one leader is required')
        if any(p.waypoints[-1].time > self.duration for p in self.people):
            raise ValueError('Paths must end within the scenario duration')


def load_scenario(name: str, crowd_size: int = 4) -> CrowdScenario:
    """All six F1 families; crowd is qualified at 2, 4 and 8 bystanders."""
    if crowd_size not in CROWD_SIZES:
        raise ValueError(f'crowd_size must be one of {CROWD_SIZES}')
    leader = PersonPath('lf_leader', 'leader', (0.15, 0.35, 0.85), (
        Waypoint(0., 1.5, 0.), Waypoint(12., 7.5, 0.),
        Waypoint(16., 7.5, 1.5), Waypoint(20., 7.5, 1.5)))
    if name == 'leader_alone':
        return CrowdScenario(name, 20., (leader,))
    if name == 'crossing':
        # At t=6 the passer crosses x=3.5, between the leader (x=4.5)
        # and a stationary observer at the origin facing +X.
        passer = PersonPath('lf_passer', 'bystander', (0.8, 0.25, 0.12), (
            Waypoint(0., 3.5, -2.), Waypoint(2., 3.5, -2.),
            Waypoint(10., 3.5, 2.), Waypoint(20., 3.5, 2.)))
        return CrowdScenario(name, 20., (leader, passer))
    if name == 'lookalike_parallel':
        twin = PersonPath('lf_lookalike', 'bystander', leader.color, tuple(
            Waypoint(p.time, p.x, p.y - .8) for p in leader.waypoints))
        return CrowdScenario(name, 20., (leader, twin))
    if name == 'lookalike_crossing':
        leader = PersonPath('lf_leader', 'leader', leader.color, (
            Waypoint(0., 1.5, -.7), Waypoint(12., 7.5, .7),
            Waypoint(20., 7.5, .7)))
        twin = PersonPath('lf_lookalike', 'bystander', leader.color, (
            Waypoint(0., 1.5, .7), Waypoint(2., 1.5, .7),
            Waypoint(14., 7.5, -.7), Waypoint(20., 7.5, -.7)))
        return CrowdScenario(name, 20., (leader, twin))
    if name == 'leader_stop':
        leader = PersonPath('lf_leader', 'leader', leader.color, (
            Waypoint(0., 1.5, 0.), Waypoint(8., 4., 0.), Waypoint(20., 4., 0.)))
        standing = tuple(PersonPath(f'lf_standing_{i}', 'bystander', (.5, .5, .5), (
            Waypoint(0., x, y), Waypoint(20., x, y)))
            for i, (x, y) in enumerate(((3.7, -.8), (4.3, .8), (4.9, -.5))))
        return CrowdScenario(name, 20., (leader, *standing))
    if name == 'crowd':
        leader = PersonPath('lf_leader', 'leader', leader.color, (
            Waypoint(0., 1.5, 0.), Waypoint(12., 7.5, 0.), Waypoint(20., 7.5, 0.)))
        lanes = (-.8, .8, -1.7, 1.7)
        people = tuple(PersonPath(f'lf_crowd_{i}', 'bystander', (.5, .5, .5), (
            Waypoint(0., 2. + .65 * i, lanes[i % 4]),
            Waypoint(20., 6. + .65 * i, lanes[i % 4])))
            for i in range(crowd_size))
        return CrowdScenario(name, 20., (leader, *people))
    raise ValueError(f'Unknown scenario {name!r}; choose one of {SCENARIOS}')
