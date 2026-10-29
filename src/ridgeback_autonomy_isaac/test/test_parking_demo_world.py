"""Demo target uniqueness and generated TF versus the committed sensor rig; no Kit."""

import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pytest

try:
    from pxr import Usd, UsdGeom
except ImportError:
    Usd = UsdGeom = None

pytestmark = pytest.mark.skipif(Usd is None, reason='run with the Isaac interpreter (no Kit needed)')
ROOT = Path(__file__).resolve().parents[3]
USD = ROOT/'src/ridgeback_autonomy_isaac/sim/isaac/usd'


def test_demo_layer_preserves_environment_and_removes_the_authored_target():
    stage = Usd.Stage.Open(str(USD/'worlds/parking_demo.usda'))
    original = Usd.Stage.Open(str(USD/'worlds/initial_test_world.usda'))
    hidden = '/initial_test_world/default_g1_far_right_room'
    assert not stage.GetPrimAtPath(hidden).IsActive()
    assert original.GetPrimAtPath(hidden).IsActive()
    assert not [p for p in stage.Traverse() if 'g1' in p.GetName().lower()]
    assert {str(p.GetPath()) for p in stage.GetDefaultPrim().GetChildren()} == {
        str(p.GetPath()) for p in original.GetDefaultPrim().GetChildren()
        if str(p.GetPath()) != hidden}


def urdf_transform(joints, frame):
    if frame == 'base_link':
        return np.eye(4)
    joint = joints[frame]
    origin = joint.find('origin')
    x, y, z = map(float, origin.get('xyz', '0 0 0').split())
    r, p, yaw = map(float, origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yaw), math.sin(yaw)
    tf = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, x],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, y],
                   [-sp, cp*sr, cp*cr, z], [0,0,0,1]])
    return urdf_transform(joints, joint.find('parent').get('link')) @ tf


def test_simulation_sensor_frames_match_the_committed_usd(tmp_path):
    from clearpath_generator_common.description.generator import DescriptionGenerator
    import xacro
    shutil.copyfile(ROOT/'clearpath/robot.yaml', tmp_path/'robot.yaml')
    DescriptionGenerator(str(tmp_path)).generate()
    robot = ET.fromstring(xacro.process_file(str(tmp_path/'robot.urdf.xacro'),
        mappings={'is_sim':'true', 'gazebo_controllers':'/dev/null',
                  'camera_support_deck_z':'0.280'}).toxml())
    joints = {j.find('child').get('link'):j for j in robot.findall('joint')}
    stage = Usd.Stage.Open(str(USD/'robots/ridgeback_r100/ridgeback_r100.usda'))
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    for frame in ('camera_0_color_optical_frame', 'lidar2d_0_laser', 'lidar2d_1_laser'):
        prims = [p for p in stage.Traverse() if p.GetName() == frame]
        assert len(prims) == 1
        actual = np.asarray(UsdGeom.Xformable(prims[0]).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())).T
        assert actual == pytest.approx(urdf_transform(joints, frame), abs=1e-7)
