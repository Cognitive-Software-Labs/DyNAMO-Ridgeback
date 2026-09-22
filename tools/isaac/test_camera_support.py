"""Regression gates for the shared support and planar-drive import boundary."""
import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('ridgeback_importer', ROOT / 'tools/isaac/import_ridgeback_urdf.py')
IMPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IMPORTER)
SUPPORT = ROOT / 'src/ridgeback_common/urdf/camera_support.urdf.xacro'


def expanded(tmp_path, deck):
    path = tmp_path / 'support.urdf'
    path.write_bytes(subprocess.check_output(['xacro', str(SUPPORT), f'camera_support_deck_z:={deck}']))
    return path


@pytest.mark.parametrize('deck,length', [(0.280, 0.815), (0.295, 0.800)])
def test_support_fits_retained_deck_and_housing(tmp_path, deck, length):
    path = expanded(tmp_path, deck)
    root = ET.parse(path).getroot()
    # Sensor frames must never become children of the visual support.
    assert [j.find('parent').get('link') for j in root.findall('joint')] == ['chassis_link'] * 2
    for link in root.findall('link'):
        assert float(link.find('inertial/mass').get('value')) > 0
        assert all(float(link.find('inertial/inertia').get(a)) > 0 for a in ('ixx', 'iyy', 'izz'))
    mast, bracket = IMPORTER._take_shared_support(path)
    assert mast['size'][2] == pytest.approx(length)
    assert mast['centre'][2] - mast['size'][2]/2 == pytest.approx(deck)
    assert mast['centre'][2] + mast['size'][2]/2 == pytest.approx(1.095)
    assert bracket['centre'][0] - bracket['size'][0]/2 == pytest.approx(mast['centre'][0] + mast['size'][0]/2)
    assert bracket['centre'][0] + bracket['size'][0]/2 == pytest.approx(0.259)
    assert bracket['centre'][2] == pytest.approx((1.02 + 1.04899953365326)/2)


def test_planar_import_removes_only_support_bodies(tmp_path):
    path = expanded(tmp_path, .280)
    tree = ET.parse(path)
    root = tree.getroot()
    for xml in ('<link name="chassis_link"><inertial><mass value="123"/></inertial></link>',
                '<link name="camera_0_link"/>',
                '<joint name="camera_0_joint" type="fixed"><parent link="default_mount"/><child link="camera_0_link"/></joint>'):
        root.append(ET.fromstring(xml))
    preserved = [ET.tostring(e) for e in list(root)[-3:]]
    tree.write(path)
    assert len(IMPORTER._take_shared_support(path)) == 2
    assert [ET.tostring(e) for e in ET.parse(path).getroot()] == preserved


@pytest.mark.parametrize('change', ['rotated', 'mismatched_collision', 'invalid_parent', 'missing'])
def test_unsupported_geometry_fails_before_mutating_urdf(tmp_path, change):
    path = expanded(tmp_path, .280)
    tree = ET.parse(path)
    root = tree.getroot()
    if change == 'rotated':
        root.find('joint/origin').set('rpy', '0 0 0.1')
    elif change == 'mismatched_collision':
        root.find('link/collision/geometry/box').set('size', '1 1 1')
    elif change == 'invalid_parent':
        root.find('joint/parent').set('link', 'base_link')
    else:
        root.remove(root.find('link'))
    tree.write(path)
    before = path.read_bytes()
    with pytest.raises(ValueError):
        IMPORTER._take_shared_support(path)
    assert path.read_bytes() == before
