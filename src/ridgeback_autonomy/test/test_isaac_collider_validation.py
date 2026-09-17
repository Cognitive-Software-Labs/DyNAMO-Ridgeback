from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pytest


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _load_module():
    path = _repo_root() / 'tools' / 'isaac' / 'validate_chassis_contacts.py'
    spec = importlib.util.spec_from_file_location('validate_chassis_contacts', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


validation = _load_module()


def test_body_command_maps_to_world_normal_at_each_contact_orientation():
    for yaw in (0.0, math.pi / 4.0, math.pi / 2.0):
        body = validation.world_to_body(0.2, 0.0, yaw)
        world = validation.body_to_world(*body, yaw)
        assert world == pytest.approx((0.2, 0.0))


def test_support_and_signed_separation_follow_rotated_geometry():
    rectangle = [(-2.0, -1.0), (-2.0, 1.0), (2.0, -1.0), (2.0, 1.0)]
    assert validation.support_xy(rectangle, 0.0) == pytest.approx(2.0)
    assert validation.support_xy(rectangle, math.pi / 2.0) == pytest.approx(1.0)
    assert validation.signed_separation(
        10.0, (7.5, 0.0), rectangle, 0.0) == pytest.approx(0.5)


def test_convex_hull_removes_interior_points_and_keeps_corners():
    points = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (1, 0)]
    assert set(validation.convex_hull_2d(points)) == {
        (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0),
    }


def test_cylinder_points_respect_authored_axis_and_support():
    points = validation.cylinder_local_points(0.25, 1.0, 'Y')
    assert validation.support_xy(points, 0.0) == pytest.approx(0.25)
    assert max(abs(point[1]) for point in points) == pytest.approx(0.5)


def _passing_samples():
    rows = []
    for index in range(120):
        rows.append({
            'phase': 'hold', 'phase_time': index / 120,
            'x': 0.5, 'y': 0.0, 'yaw': 0.0,
            'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'gap': 0.001,
        })
    for index in range(120):
        rows.append({
            'phase': 'reverse', 'phase_time': index / 120,
            'x': 0.5 - 0.1 * index / 120, 'y': 0.0, 'yaw': 0.0,
            'vx': -0.1, 'vy': 0.0, 'wz': 0.0,
            'gap': 0.001 + 0.1 * index / 120,
        })
    return rows


def test_case_gate_accepts_stable_contact_and_recovery():
    spec = validation.CaseSpec('hull_only', 'front', 0.0, 0.1)
    contact = validation.ContactEvidence(
        sim_time=1.0, paths=['/World/wall', '/World/ridgeback'],
        normal=(1.0, 0.0, 0.0), impulse=1.0)
    result = validation.evaluate_case(
        spec, _passing_samples(), [contact], expected_support=0.5,
        visual_support=0.499)
    assert result.passed


def test_case_gate_rejects_missing_contact_and_excess_penetration():
    samples = _passing_samples()
    for row in samples:
        if row['phase'] == 'hold':
            row['gap'] = -0.02
    spec = validation.CaseSpec('hull_only', 'front', 0.0, 0.1)
    result = validation.evaluate_case(
        spec, samples, [], expected_support=0.5, visual_support=0.5)
    assert not result.passed
    assert not result.checks['penetration']
    assert not result.checks['contact_event']


def test_aggregate_requires_repeatability_and_aabb_control():
    def row(name, stop, expected):
        return {
            'name': name, 'settled_stop_x': stop,
            'expected_stop_x': expected, 'passed': True,
        }

    boots = []
    for index, jitter in enumerate((0.0, 0.001, -0.001), start=1):
        boots.append({
            'boot_index': index,
            'cases': [
                row('hull_only_angle_0.10', 0.5 + jitter, 0.5),
                row('aabb_control_angle_0.10', 0.4 + jitter, 0.4),
            ],
        })
    aggregate, _ = validation.aggregate_boots(boots)
    assert aggregate['physics_passed']
