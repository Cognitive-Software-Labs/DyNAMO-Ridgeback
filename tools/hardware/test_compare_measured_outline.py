"""Measured-outline comparison against the configured Nav2 footprint."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'compare_measured_outline', Path(__file__).with_name('compare_measured_outline.py'))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

POLYGON, PADDING = tool.footprint(tool.NAV2.read_text())
# Hull matching the Isaac model's bounds: 0.9324 m long, 0.7932 m wide.
HULL = {'length_m': 0.9324, 'width_m': 0.7932, 'uncertainty_m': 0.004}


def rows(points=()):
    return {row['name']: row for row in tool.evaluate({'hull': HULL, 'points': list(points)}, POLYGON, PADDING)}


def test_reads_the_configured_polygon_and_padding():
    assert len(POLYGON) == 16 and PADDING == 0.01


def test_margin_sign_inside_and_outside():
    assert tool.signed_margin(POLYGON, 0.0, 0.0) > 0.3
    assert tool.signed_margin(POLYGON, 0.6, 0.0) < 0


def test_a_model_sized_hull_passes_its_face_midpoints():
    assert all(row['passed'] for row in rows().values())


def test_converts_tape_distances_to_base_link():
    row = rows([{'name': 'mast', 'from_front_m': 0.2, 'from_left_m': 0.4, 'uncertainty_m': 0.005}])['mast']
    assert row['x_m'] == round(0.9324 / 2 - 0.2, 4) and row['y_m'] == round(0.7932 / 2 - 0.4, 4)


def test_a_protrusion_past_the_side_fails():
    row = rows([{'name': 'bracket', 'from_front_m': 0.3, 'from_left_m': -0.03, 'uncertainty_m': 0.005}])['bracket']
    assert row['margin_m'] < 0 and not row['passed']


def test_uncertainty_can_turn_a_marginal_point_into_a_fail():
    # Place the point 10 mm inside the effective left edge instead of assuming
    # a particular deployed footprint width, which can change after a survey.
    left_edge = max(y for _, y in POLYGON)
    near_edge = {'name': 'edge', 'from_front_m': HULL['length_m'] / 2,
                 'from_left_m': HULL['width_m'] / 2 - (left_edge + PADDING - 0.01)}
    assert rows([dict(near_edge, uncertainty_m=0.0)])['edge']['passed']
    assert not rows([dict(near_edge, uncertainty_m=0.02)])['edge']['passed']


def test_a_square_corner_is_outside_the_chamfered_footprint():
    row = rows([{'name': 'square corner', 'from_front_m': 0.0, 'from_left_m': 0.0, 'uncertainty_m': 0.0}])
    assert not row['square corner']['passed']
