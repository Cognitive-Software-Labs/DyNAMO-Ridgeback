"""Documentation camera views of a parking scene, computed from its layout.

Usage: scene_views.py gz|isaac SCENARIO [OUT.json]
Prints (or writes) the view list for that scene: an overall top-down view of the
start and G1's workstation, a close top-down view of the workstation, a close-up
of G1 and its table, and an oblique view that shows heights. Both simulators get
the same views around the same layout; only the world (and so the background)
differs. The Isaac runner reads the JSON through RIDGEBACK_ISAAC_VIEW_CAMERAS;
capture_scene_views.py spawns Gazebo cameras from the same list.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '../..'))
sys.path.insert(0, f'{ROOT}/src/ridgeback_autonomy')   # the pure scenario module
from ridgeback_autonomy.mission import parking_scenarios  # noqa: E402

SCENARIO_FILE = f'{ROOT}/src/ridgeback_autonomy/config/parking_scenarios.yaml'
# The warehouse has a roof, so its overall view stays below it; Isaac's rooms are open.
OVERVIEW_HEIGHT_M = {'gz': 8.0, 'isaac': 11.0}
START = (0.0, 0.0)
CAPTIONS = {
    'overview': 'Top-down: the robot start (lower left) and G1\'s workstation',
    'workstation': 'Top-down close: G1, its worktable and the floor around it',
    'closeup': 'G1 and its worktable from behind its left shoulder',
    'oblique': 'Oblique: object heights against the robot\'s mast and basket',
}


def _snap(angle):
    return round(angle / (math.pi / 2)) * (math.pi / 2)


def views(backend, scenario, data=None):
    data = data or parking_scenarios.load(SCENARIO_FILE)
    layout = parking_scenarios.world_layout(data, scenario, backend)
    station = layout['workstation']
    gx, gy = station['x'], station['y']
    heading = math.radians(station['yaw_deg'])          # G1 toward its table
    fx, fy = math.cos(heading), math.sin(heading)
    lx, ly = -fy, fx                                     # G1's left at the table

    # Overall view: the start-to-G1 axis runs across the image, the start on the left.
    across = _snap(math.atan2(gy - START[1], gx - START[0]))
    up = (-math.sin(across), math.cos(across), 0.0)
    cx, cy = (START[0] + gx) / 2, (START[1] + gy) / 2
    top = OVERVIEW_HEIGHT_M[backend]

    def at(forward, left, z):
        return [round(gx + forward * fx + left * lx, 3), round(gy + forward * fy + left * ly, 3), z]

    table = at(0.4, 0.0, 0.6)
    return [
        {'name': 'overview', 'eye': [cx, cy, top], 'target': [cx, cy, 0.0], 'up': list(up),
         'hfov_deg': 84, 'width': 1600, 'height': 1000},
        {'name': 'workstation', 'eye': at(0.0, 0.0, 5.5)[:2] + [5.5],
         'target': at(0.0, 0.0, 0.0), 'up': [fx, fy, 0.0],
         'hfov_deg': 62, 'width': 1600, 'height': 1000},
        {'name': 'closeup', 'eye': at(-2.2, 1.7, 2.1), 'target': table,
         'hfov_deg': 58, 'width': 1600, 'height': 1000},
        {'name': 'oblique', 'eye': at(-4.5, -4.0, 4.2), 'target': at(-0.3, 0.0, 0.4),
         'hfov_deg': 62, 'width': 1600, 'height': 1000},
    ]


def gz_pose(view):
    """SDF pose (x y z roll pitch yaw) of a camera looking from eye to target."""
    (ex, ey, ez), (tx, ty, tz) = view['eye'], view['target']
    dx, dy, dz = tx - ex, ty - ey, tz - ez
    horizontal = math.hypot(dx, dy)
    if horizontal < 1e-6:     # straight down: yaw sets which world direction is image-up
        ux, uy = view['up'][:2]
        return ex, ey, ez, 0.0, math.pi / 2, math.atan2(uy, ux)
    return ex, ey, ez, 0.0, math.atan2(-dz, horizontal), math.atan2(dy, dx)


if __name__ == '__main__':
    result = views(sys.argv[1], sys.argv[2])
    text = json.dumps(result, indent=1)
    if len(sys.argv) > 3:
        with open(sys.argv[3], 'w') as stream:
            stream.write(text)
    else:
        print(text)
