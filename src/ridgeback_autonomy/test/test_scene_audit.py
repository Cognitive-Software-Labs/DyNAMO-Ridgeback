"""Scene audit: rasterizer, metrics and LiDAR cast against closed-form answers.

Every scene here is synthetic (boxes and cylinders at chosen poses), so each
metric has an exact expected value. Principal points are deliberately
non-integer, keeping every projected edge off a pixel sample so the counts are
exact rather than +-1.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from ridgeback_autonomy.benchmarking import scene_audit as sa
from ridgeback_autonomy.benchmarking import scene_certifications as certs
from ridgeback_autonomy.benchmarking.scenarios import ObjectSpec, RobotSpec, Scene

REPO_ROOT = Path(__file__).resolve().parents[3]
MODELS = REPO_ROOT / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'models'

INTR = sa.Intrinsics('test', 200.0, 200.0, 160.3, 120.3, 320, 240)
CAMERA_Z = 0.5
LASER_Z = 0.3
NEAR = 0.05


def _sensors(camera_xyz=(0.0, 0.0, CAMERA_Z)) -> sa.SensorGeometry:
    laser = np.eye(4)
    laser[2, 3] = LASER_Z
    return sa.SensorGeometry(
        name='test', base_link_world_z=0.0,
        camera_in_base=sa.level_optical_frame(*camera_xyz), laser_in_base=laser,
        intrinsics=(INTR,), laser=sa.contract_laser_spec(), near_clip_m=NEAR)


def _box_model(name, size, part='torso_link') -> sa.ModelGeometry:
    pose = sa.pose_matrix(0.0, 0.0, size[2] / 2)
    tris = sa.transform_triangles(sa.box_triangles(size), pose)
    return sa.ModelGeometry(name, '', (sa.Part(part, 'box', tris, {}, pose),))


def _cylinder_part(name, radius, length, x=0.0, y=0.0, z0=0.0, segments=256) -> sa.Part:
    pose = sa.pose_matrix(x, y, z0 + length / 2)
    tris = sa.transform_triangles(sa.cylinder_triangles(radius, length, segments), pose)
    return sa.Part(name, 'cylinder', tris, {'radius': radius, 'length': length}, pose)


def _auditor(target, objects=None, sensors=None) -> sa.SceneAuditor:
    builder = sa.SceneBuilder(target=target, objects=objects or {}, world=[])
    return sa.SceneAuditor(builder, sensors or _sensors())


def _scene(robots, objects=()) -> Scene:
    return Scene('t', tuple(RobotSpec(*r) for r in robots),
                 tuple(ObjectSpec(*o) for o in objects), None)


def _samples(lo: float, hi: float, limit: int | None = None) -> int:
    """Integer sample points in [lo, hi] (optionally clipped to [0, limit - 1])."""
    first, last = math.ceil(lo), math.floor(hi)
    if limit is not None:
        first, last = max(first, 0), min(last, limit - 1)
    return max(0, last - first + 1)


# Target: 0.2 deep x 0.6 wide x 1.0 tall box, front face 2.9 m ahead.
BOX = (0.2, 0.6, 1.0)
TARGET_XY = (3.0, 0.07)


def _front_face_uv(x, y, size=BOX, camera_z=CAMERA_Z):
    z_front = x - size[0] / 2
    u = [INTR.fx * -(y + s * size[1] / 2) / z_front + INTR.cx for s in (1, -1)]
    v = [INTR.fy * -(h - camera_z) / z_front + INTR.cy for h in (size[2], 0.0)]
    return u, v, z_front


def _metrics(auditor, scene):
    result, renders, _ = auditor.audit_scene(scene)
    return result['camera'][INTR.name]['targets'], result['lidar'], renders[INTR.name]


def test_box_pixel_count_matches_its_projected_front_face():
    targets, _, _ = _metrics(_auditor(_box_model('box', BOX)), _scene([(*TARGET_XY, 0.0)]))
    (u_lo, u_hi), (v_lo, v_hi), _ = _front_face_uv(*TARGET_XY)
    expected = _samples(u_lo, u_hi) * _samples(v_lo, v_hi)

    target = targets[0]
    assert target['pixels_alone_in_image'] == expected
    assert target['pixels_visible'] == expected
    assert target['visible_fraction'] == 1.0
    assert target['in_frame_fraction'] == 1.0
    assert target['bbox'] == [math.ceil(u_lo), math.ceil(v_lo), math.floor(u_hi), math.floor(v_hi)]


def test_centre_ratio_is_the_projected_yaw_axis():
    x, y = TARGET_XY
    targets, _, _ = _metrics(_auditor(_box_model('box', BOX)), _scene([(x, y, 0.7)]))
    u_c = INTR.fx * -y / x + INTR.cx
    assert targets[0]['center_ratio'] == pytest.approx(abs(u_c - INTR.cx) / INTR.cx, abs=1e-4)
    assert targets[0]['range_horizontal_m'] == pytest.approx(math.hypot(x, y), abs=1e-4)
    assert targets[0]['bearing_deg'] == pytest.approx(math.degrees(math.atan2(y, x)), abs=1e-3)


def test_nearer_box_hiding_the_left_half_halves_the_visible_fraction():
    (u_lo, u_hi), (v_lo, v_hi), _ = _front_face_uv(*TARGET_XY)
    u_mid = 0.5 * (u_lo + u_hi) + 0.37  # keep the edge off a sample column
    depth, x_occ = 0.002, 1.5
    z_back = x_occ + depth / 2
    # Occluder right edge (min y) projects onto u_mid at its far face, which is
    # the silhouette edge seen from a camera to its right.
    y_edge = (INTR.cx - u_mid) * z_back / INTR.fx
    occluder = _box_model('wall', (depth, 2.0, 2.0), part='visual_0')
    scene = _scene([(*TARGET_XY, 0.0)], [('wall', x_occ, y_edge + 1.0, 0.0)])
    targets, _, _ = _metrics(_auditor(_box_model('box', BOX), {'wall': occluder}), scene)

    rows = _samples(v_lo, v_hi)
    visible_cols = _samples(u_mid, u_hi)
    total_cols = _samples(u_lo, u_hi)
    target = targets[0]
    assert target['pixels_visible'] == visible_cols * rows
    assert target['visible_fraction'] == pytest.approx(visible_cols / total_cols, abs=1e-4)
    assert target['visible_fraction'] == pytest.approx(0.5, abs=1.5 / total_cols)
    assert target['occluded_by'] == {'object:0': (total_cols - visible_cols) * rows}


def test_target_straddling_the_left_edge_is_half_in_frame():
    # A thin board: seen this far off-axis a deep box would also show a side face.
    board = (0.002, BOX[1], BOX[2])
    x = TARGET_XY[0]
    z_front = x - board[0] / 2
    y = (INTR.cx - 0.4) * z_front / INTR.fx  # centre column at u = 0.4
    targets, _, _ = _metrics(_auditor(_box_model('box', board)), _scene([(x, y, 0.0)]))
    (u_lo, u_hi), (v_lo, v_hi), _ = _front_face_uv(x, y, size=board)
    inside = _samples(u_lo, u_hi, INTR.width)
    total = _samples(u_lo, u_hi)

    target = targets[0]
    assert target['pixels_extended'] == total * _samples(v_lo, v_hi)
    assert target['in_frame_fraction'] == pytest.approx(inside / total, abs=1e-4)
    assert target['in_frame_fraction'] == pytest.approx(0.5, abs=1.5 / total)
    assert target['clipped_edges'] == ['left']
    assert target['bbox_extended'][0] == math.ceil(u_lo) < 0


def test_ground_plane_is_clipped_exactly_at_the_near_plane():
    """A plane under the camera, running from behind it to far ahead: only rows
    whose ray meets it at Z >= near are drawn, with perspective-correct depth."""
    height, near, far, half_width = 1.0, 2.0, 50.0, 1000.0
    corners = np.array([[-half_width, height, -5.0], [half_width, height, -5.0],
                        [half_width, height, far], [-half_width, height, far]])
    tris = corners[np.array([[0, 1, 2], [0, 2, 3]])]
    depth, code = sa.rasterize(tris, np.array([7, 7]), sa.Canvas.image(INTR), near=near)

    rows = np.flatnonzero((code == 7).any(axis=1))
    first = math.ceil(INTR.cy + INTR.fy * height / far)
    last = math.floor(INTR.cy + INTR.fy * height / near)
    assert last < INTR.height - 1, 'the clip boundary must fall inside the image'
    assert rows[0] == first and rows[-1] == last
    assert (code[first:last + 1] == 7).all()
    assert (code[last + 1:] == -1).all()
    v = 180
    assert depth[v, 40] == pytest.approx(INTR.fy * height / (v - INTR.cy), rel=1e-9)


def test_clip_near_splits_a_crossing_triangle_at_the_plane():
    tri = np.array([[[0.0, 0.0, 2.0], [1.0, 0.0, -2.0], [-1.0, 0.0, -2.0]]])
    clipped, codes = sa.clip_near(tri, np.array([3]), 1.0)
    assert len(clipped) == 1 and codes.tolist() == [3]
    assert clipped[:, :, 2].min() == pytest.approx(1.0)
    tri = np.array([[[0.0, 0.0, -2.0], [1.0, 0.0, 2.0], [-1.0, 0.0, 2.0]]])
    clipped, _ = sa.clip_near(tri, np.array([3]), 1.0)
    assert len(clipped) == 2
    assert clipped[:, :, 2].min() == pytest.approx(1.0)


def _beams_on_disc(centre, radius, origin=(0.0, 0.0)):
    spec = sa.contract_laser_spec()
    d = math.hypot(centre[0] - origin[0], centre[1] - origin[1])
    bearing = math.atan2(centre[1] - origin[1], centre[0] - origin[0])
    half = math.asin(radius / d)
    return int((np.abs(spec.angles() - bearing) <= half).sum()), math.degrees(2 * half)


def test_lidar_beam_count_on_a_pole_matches_its_subtended_angle():
    pole = sa.ModelGeometry('pole', '', (_cylinder_part('shaft', 0.025, 1.75),))
    scene = _scene([(6.0, -3.0, 0.0)], [('pole', 2.0, 0.3, 0.0)])
    _, lidar, _ = _metrics(_auditor(_box_model('box', BOX), {'pole': pole}), scene)
    expected, _ = _beams_on_disc((2.0, 0.3), 0.025)
    entry = lidar['objects'][0]
    assert abs(entry['beams'] - expected) <= 1
    assert entry['is_pole']
    assert entry['angular_width_deg'] == pytest.approx(
        math.degrees(2 * math.asin(0.025 / math.hypot(2.0, 0.3))), abs=1e-3)
    assert entry['range_from_laser_m'] == pytest.approx(math.hypot(2.0, 0.3), abs=1e-4)


def _legged_target() -> sa.ModelGeometry:
    legs = [_cylinder_part(f'{side}_knee_link', 0.05, 0.6, y=sign * 0.12)
            for side, sign in (('left', 1), ('right', -1))]
    torso_pose = sa.pose_matrix(0.0, 0.0, 0.9)
    torso = sa.Part('torso_link', 'box', sa.transform_triangles(
        sa.box_triangles((0.2, 0.4, 0.6)), torso_pose), {}, torso_pose)
    return sa.ModelGeometry('legged', '', (*legs, torso))


def test_leg_width_ratio_subtracts_a_poles_shadow():
    target = _legged_target()
    leg = np.array([3.0, 0.12])
    d_leg = float(np.hypot(*leg))
    leg_bearing = math.atan2(leg[1], leg[0])
    leg_half = math.asin(0.05 / d_leg)
    # Pole at 1.5 m whose shadow starts at the left leg's bearing: it hides the
    # right half of that leg's angular interval.
    pole_r, pole_d = 0.025, 1.5
    pole_half = math.asin(pole_r / pole_d)
    pole_bearing = leg_bearing - pole_half
    pole_xy = (pole_d * math.cos(pole_bearing), pole_d * math.sin(pole_bearing))
    pole = sa.ModelGeometry('pole', '', (_cylinder_part('shaft', pole_r, 1.75),))
    scene = _scene([(3.0, 0.0, 0.0)], [('pole', *pole_xy, 0.0)])
    _, lidar, _ = _metrics(_auditor(target, {'pole': pole}), scene)

    legs = lidar['targets'][0]['legs']
    full = math.degrees(2 * leg_half)
    hidden = math.degrees(min(leg_bearing + leg_half, pole_bearing + pole_half)
                          - max(leg_bearing - leg_half, pole_bearing - pole_half))
    assert legs['left']['width_alone_deg'] == pytest.approx(full, abs=0.03)
    assert legs['left']['width_deg'] == pytest.approx(full - hidden, abs=0.03)
    assert legs['right']['width_ratio'] == pytest.approx(1.0)
    assert lidar['targets'][0]['legs_meet_criterion'] is (
        (full - hidden) / full >= sa.LEG_WIDTH_CRITERION)


def test_sdf_assets_load_with_their_declared_geometry():
    pole = sa.load_sdf_model(MODELS / 'iv_pole' / 'model.sdf', cylinder_segments=64)
    shaft = next(p for p in pole.parts if p.params.get('radius') == 0.025)
    assert shaft.kind == 'cylinder'
    assert shaft.z_range == pytest.approx((0.05, 1.80))
    assert pole.triangle_count == 2 * 4 * 64

    g1 = sa.load_sdf_model(MODELS / 'g1' / 'model.sdf')
    assert len(g1.parts) == 36
    sides = {sa.leg_side(p.name) for p in g1.parts}
    assert sides == {'left', 'right', None}


def test_certification_comments_parse_into_claims():
    text = '\n'.join([
        'scenes:',
        '  # 2 robots all visible: 1.00, 0.98',
        '  - id: multi_x',
        '',
        '  # far robot vis 0.23 (band 0.15-0.45)',
        '  - id: interocc_moderate_x',
        '',
        '  # 640x480 edge, cut by the left edge; in-frame 0.28, center ratio 1.03',
        '  - id: edge640_half_out_x',
        '',
        '  # R0 vis 0.47, R1 vis 0.75 | table',
        '  - id: chaos_x',
        '',
        '  # near limit, centered: feet clipped by VFOV',
        '  - id: near_clip_x',
    ])
    claims = {(c.scene, c.claim, c.target): c for c in certs.parse_claims(text)}
    assert claims[('multi_x', 'vis', 1)].value == 0.98
    assert claims[('interocc_moderate_x', 'vis', 'far')].band == (0.15, 0.45)
    assert claims[('edge640_half_out_x', 'in_frame_fraction', 0)].band == (0.20, 0.35)
    assert claims[('edge640_half_out_x', 'center_ratio', 0)].value == 1.03
    assert claims[('chaos_x', 'vis', 1)].value == 0.75
    assert ('near_clip_x', 'feet_clipped', 0) in claims


def test_description_camera_pose_composes_the_optical_rotation(tmp_path):
    pytest.importorskip('xacro')
    pytest.importorskip('clearpath_generator_common.description.generator')
    from ridgeback_autonomy import robot_description as rd

    setup = rd.generate_setup(REPO_ROOT / 'clearpath' / 'robot.yaml', tmp_path)
    urdf = rd.expand_description(setup, is_sim=True)
    pose = rd.transform_to(urdf, 'camera_0_color_optical_frame')
    # docs/target_localization/camera_stack.md: nominal colour optical origin.
    assert pose[:3, 3] == pytest.approx((0.28035, -0.0115, 1.0345), abs=1e-9)
    assert pose[:3, :3] == pytest.approx(sa.level_optical_frame(0, 0, 0)[:3, :3], abs=1e-12)


def test_in_box_pole_comments_become_claims():
    text = '\n'.join([
        'scenes:',
        '  # 2 poles: p0 at 3.27 m, 4 beams, +23 px inside the box; p1 at 2.23 m, 5 beams, '
        '1 px outside the box, intersects the G1 by >= 2.5 cm; legs keep >= 93% width',
        '  - id: interfere_pair_x',
        '',
        '  # control: 1 pole at 1.81 m, 7 beams, 55 px outside the box by design; legs unshadowed',
        '  - id: interfere_control_04',
    ])
    claims = {(c.scene, c.claim, c.target): c for c in certs.parse_claims(text)}
    assert claims[('interfere_pair_x', 'pole_range_m', 1)].value == 2.23
    assert claims[('interfere_pair_x', 'pole_inset_px', 0)].value == 23.0
    assert claims[('interfere_pair_x', 'pole_inset_px', 1)].value == -1.0
    assert claims[('interfere_pair_x', 'pole_target_clearance_m', 1)].value == -0.025
    assert claims[('interfere_pair_x', 'min_leg_width_ratio', None)].value == 0.93
    assert claims[('interfere_control_04', 'pole_beams', 0)].value == 7.0
    assert claims[('interfere_control_04', 'min_leg_width_ratio', None)].value == 1.0


def test_every_shipped_in_box_comment_is_checkable():
    text = (REPO_ROOT / 'src' / 'ridgeback_autonomy' / 'config'
            / 'benchmark_scenarios_full.yaml').read_text(encoding='utf-8')
    claimed = {c.scene for c in certs.parse_claims(text)}
    in_box = [s for s, comment in certs.scene_comments(text).items()
              if certs.is_in_box_scene(s) and comment]
    assert len(in_box) == 18
    assert [s for s in in_box if s not in claimed] == []


def test_axis_clearance_measures_the_surface_not_the_vertices():
    # 1 m cube centred at (2, 0, 0.5): its face x = 1.5 has vertices only at the
    # corners (0.5 m off the axis' line), but the face is 0.3 m from the axis.
    cube = sa.transform_triangles(sa.box_triangles((1.0, 1.0, 1.0)), sa.pose_matrix(2, 0, 0.5, 0, 0, 0))
    d = sa._axis_clearance(cube, (1.2, 0.0), (0.2, 0.8))
    assert d == pytest.approx(0.3, abs=1e-9)
    assert sa._axis_clearance(cube, (1.2, 0.0), (1.5, 2.0)) is None


def test_control_scene_pole_outside_the_box_is_not_a_banner_failure():
    pole = {'is_pole': True, 'index': 0, 'beams': 7, 'range_from_laser_m': 1.8,
            'camera': {'640x480': {'inset_in_target_bbox': [{'target': 0, 'inset_px': -54.7}]}},
            'clearance_to_targets': [{'target': 0, 'clearance_m': 0.3}],
            'clearance_to_other_poles': [], 'clearance_to_sensor_robot_m': 1.5, 'shadowed_by': []}
    scene = {'lidar': {'targets': [{'legs_meet_criterion': True, 'min_leg_width_ratio': 1.0}],
                       'objects': [pole]}}
    control = certs.check_in_box_banner({'interfere_control_04': scene}, '640x480')
    pair = certs.check_in_box_banner({'interfere_pair_01': scene}, '640x480')
    assert control[0]['holds'] and not pair[0]['holds']
