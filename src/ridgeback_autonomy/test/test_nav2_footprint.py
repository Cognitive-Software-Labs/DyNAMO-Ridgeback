import math
from pathlib import Path
import struct

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


PARAMS = Path(__file__).resolve().parents[1] / 'config/nav2_params.yaml'
# Robot body: the Clearpath r100 collision meshes widened by the Isaac origin
# offset and mirrored, enclosed along 24 support directions with 5.1 mm
# margin (95.2 x 80.1 cm, angled ends). Its hardware basis is open; see
# docs/robot/collision_model.md.
EXPECTED_OUTLINE = [
    [0.475762, 0.073265], [0.409768, 0.319557], [0.393065, 0.348488],
    [0.367198, 0.374355], [0.339166, 0.390539], [0.302752, 0.400297],
    [-0.302752, 0.400297], [-0.339166, 0.390539], [-0.367198, 0.374355],
    [-0.393065, 0.348488], [-0.409768, 0.319557], [-0.475762, 0.073265],
    [-0.475762, -0.073265], [-0.409768, -0.319557], [-0.393065, -0.348488],
    [-0.367198, -0.374355], [-0.339166, -0.390539], [-0.302752, -0.400297],
    [0.302752, -0.400297], [0.339166, -0.390539], [0.367198, -0.374355],
    [0.393065, -0.348488], [0.409768, -0.319557], [0.475762, -0.073265],
]


def test_costmaps_and_collision_monitor_share_the_body_outline():
    params = yaml.safe_load(PARAMS.read_text())
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    zone = params['collision_monitor']['ros__parameters']['FootprintApproach']

    assert yaml.safe_load(local['footprint']) == EXPECTED_OUTLINE
    assert yaml.safe_load(global_['footprint']) == EXPECTED_OUTLINE
    # The margin is built into the outline, so no runtime padding.
    assert local['footprint_padding'] == pytest.approx(0.0)
    assert global_['footprint_padding'] == pytest.approx(0.0)
    assert zone['footprint_topic'] == 'local_costmap/published_footprint'
    assert 'points' not in zone


def _inside_margin(polygon, x, y):
    """Signed distance to the polygon boundary, positive inside."""
    inside, nearest = False, math.inf
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
        nearest = min(nearest, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return nearest if inside else -nearest


def _stl_xy(path):
    """Plan-view vertices of a binary STL."""
    data = path.read_bytes()
    count, = struct.unpack_from('<I', data, 80)
    points = set()
    for i in range(count):
        values = struct.unpack_from('<12f', data, 84 + 50 * i)
        points.update((round(values[k], 6), round(values[k + 1], 6)) for k in (3, 6, 9))
    return points


def test_outline_encloses_the_clearpath_meshes_in_both_simulators():
    meshes = Path(get_package_share_directory('clearpath_platform_description')) / 'meshes/r100'
    points = _stl_xy(meshes / 'body-collision.stl') | _stl_xy(meshes / 'top.stl')
    # Isaac places the same chassis 4.408 mm back and 0.201 mm aside.
    offsets = [(sx * 0.004408, sy * 0.000201) for sx in (1, -1) for sy in (1, -1)]
    assert min(_inside_margin(EXPECTED_OUTLINE, x + dx, y + dy)
               for x, y in points for dx, dy in offsets) >= 0.005
