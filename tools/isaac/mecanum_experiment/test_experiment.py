"""Offline regression checks for geometric assumptions and qualification gates."""

import math
import numpy as np
import pytest
from model import roller_geometry
from run import cases, evaluate


def test_rollers_fit_declared_wheel_envelope():
    radius, width = 0.0759, 0.079
    p, rho, _, _ = roller_geometry(radius, width)
    # Local X is a roller's axle; orient it 45 degrees from the wheel axle.
    axis = np.array([1, 1, 0]) / math.sqrt(2)
    other = np.array([-1, 1, 0]) / math.sqrt(2)
    wheel_points = (
        p[:, 0, None] * axis
        + p[:, 1, None] * other
        + p[:, 2, None] * [0, 0, 1]
        + [0, 0, rho]
    )
    assert np.max(np.linalg.norm(wheel_points[:, [0, 2]], axis=1)) <= radius + 1e-12
    assert np.max(np.abs(wheel_points[:, 1])) <= width / 2
    assert np.max(np.linalg.norm(wheel_points[:, [0, 2]], axis=1)) == pytest.approx(
        radius
    )


def samples(velocity=(0.1, 0, 0), sensor_error=0.0):
    return [
        dict(
            position=[velocity[0] * t, velocity[1] * t, 0.03],
            body_velocity=list(velocity),
            yaw=velocity[2] * t,
            time=t,
            sensor_error=sensor_error,
            sensor_translation_error=sensor_error,
            sensor_rotation_error=0.0,
            linear_velocity=[*velocity[:2], 0.0],
            lowest_z=0.0,
            floor_penetration=0.0,
            targets=[0] * 4,
        )
        for t in np.linspace(0, 5, 151)
    ]


def test_linear_gate_rejects_wrong_direction_and_cross_drift():
    case = dict(kind="linear", command=[0.1, 0, 0])
    assert evaluate(case, samples(), [])["passed"]
    assert not evaluate(case, samples((-0.1, 0, 0)), [])["passed"]
    assert not evaluate(case, samples((0.1, 0.03, 0)), [])["passed"]


def test_sensor_detachment_cannot_pass_motion_gate():
    assert not evaluate(
        dict(kind="linear", command=[0.1, 0, 0]), samples(sensor_error=0.002), []
    )["passed"]


def test_full_matrix_has_both_directions_and_contact_orientations():
    matrix = cases()
    assert sum(c["kind"] == "linear" for c in matrix) == 8
    assert sum(c["kind"] == "rotate" for c in matrix) == 2
    assert sum(c["kind"] == "wall" for c in matrix) == 3
    assert len({c["name"] for c in matrix}) == len(matrix)


def test_timeout_does_not_pass_a_robot_still_moving():
    assert not evaluate(dict(kind="timeout"), samples(), [])["passed"]


def test_wall_proximity_without_contact_report_cannot_pass():
    case = dict(kind="wall")
    rows = samples((0, 0, 0))
    for row in rows:
        row["time"] *= 1.6
        row["wall_gap"] = 0.1 if row["time"] >= 7.5 else 0.0
    assert evaluate(case, rows, [object()])["passed"]
    assert not evaluate(case, rows, [])["passed"]


def test_declared_mass_is_merged_with_parallel_axis_shift(tmp_path):
    from model import Description, WHEELS

    # Two equal, separated fixed masses: centre at zero, Iyy/Izz gain 2*m*d².
    xml = '<robot name="test"><link name="base_link"/>'
    for name, x in [("left", -1), ("right", 1)]:
        xml += f"""<link name="{name}"><inertial><mass value="2"/>
          <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
          </inertial></link><joint name="{name}" type="fixed">
          <parent link="base_link"/><child link="{name}"/>
          <origin xyz="{x} 0 0"/></joint>"""
    for name in WHEELS:
        xml += f"""<link name="{name}_wheel_link"><collision><geometry>
          <cylinder radius="0.0759" length="0.079"/></geometry></collision></link>
          <joint name="{name}_wheel_joint" type="continuous"><parent link="base_link"/>
          <child link="{name}_wheel_link"/></joint>"""
    xml += "</robot>"
    path = tmp_path / "test.urdf"
    path.write_text(xml)
    mass, com, inertia = Description(path).inertial()
    assert mass == 4
    assert com == pytest.approx([0, 0, 0])
    assert inertia == pytest.approx(np.diag([2, 6, 6]))


def test_nonwheel_movable_joint_is_rejected(tmp_path):
    from model import Description

    path = tmp_path / "unsupported.urdf"
    path.write_text("""<robot name="test"><link name="base_link"/><link name="arm"/>
      <joint name="arm" type="revolute"><parent link="base_link"/>
      <child link="arm"/></joint></robot>""")
    with pytest.raises(ValueError, match="unsupported movable joints"):
        Description(path)


def test_aggregate_rejects_missing_boots(tmp_path):
    from aggregate import aggregate

    with pytest.raises(FileNotFoundError):
        aggregate(tmp_path)
