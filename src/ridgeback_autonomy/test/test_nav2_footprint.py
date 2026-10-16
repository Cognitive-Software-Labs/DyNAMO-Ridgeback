from pathlib import Path

import pytest
import yaml


PARAMS = Path(__file__).resolve().parents[1] / 'config/nav2_params.yaml'
# Measured r100_0160 outline (2026-10-16): 96.0 x 79.3 cm overall with 5.5 cm
# diagonal corner gaps (13.3 cm rounding), base_link centred between symmetric
# hubs. Straight edges sit 12 mm (the tape uncertainty) outside the faces;
# corner vertices lie on the rounding offset by 12 mm, pushed out by the
# 30-degree chord sag so no segment cuts inside the arc.
EXPECTED_OUTLINE = [
    [0.497107, 0.263718],
    [0.477026, 0.338663],
    [0.422163, 0.393526],
    [0.347218, 0.413607],
    [-0.347218, 0.413607],
    [-0.422163, 0.393526],
    [-0.477026, 0.338663],
    [-0.497107, 0.263718],
    [-0.497107, -0.263718],
    [-0.477026, -0.338663],
    [-0.422163, -0.393526],
    [-0.347218, -0.413607],
    [0.347218, -0.413607],
    [0.422163, -0.393526],
    [0.477026, -0.338663],
    [0.497107, -0.263718],
]


def test_local_and_global_costmaps_share_pinned_footprint_clearance():
    params = yaml.safe_load(PARAMS.read_text())
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']

    assert yaml.safe_load(local['footprint']) == EXPECTED_OUTLINE
    assert yaml.safe_load(global_['footprint']) == EXPECTED_OUTLINE
    assert local['footprint_padding'] == pytest.approx(0.01)
    assert global_['footprint_padding'] == pytest.approx(0.01)


def _inside_margin(polygon, x, y):
    """Signed distance to the polygon boundary, positive inside."""
    import math
    inside, nearest = False, math.inf
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
        nearest = min(nearest, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return nearest if inside else -nearest


def test_footprint_encloses_the_measured_outline_by_its_uncertainty():
    import math
    half_length, half_width, uncertainty = 0.48, 0.3965, 0.012
    radius = 0.055 / (math.sqrt(2) - 1)
    points = []
    for sx in (1, -1):
        for sy in (1, -1):
            for degrees in range(0, 91, 5):
                a = math.radians(degrees)
                points.append((sx * (half_length - radius + radius * math.cos(a)),
                               sy * (half_width - radius + radius * math.sin(a))))
    points += [(half_length, 0.0), (-half_length, 0.0), (0.0, half_width), (0.0, -half_width)]
    # 1e-5 m allows for the six-decimal rounding of the configured vertices.
    assert min(_inside_margin(EXPECTED_OUTLINE, x, y) for x, y in points) >= uncertainty - 1e-5
