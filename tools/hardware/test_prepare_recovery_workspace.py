"""Preparation must fail safely and never install or start the robot stack."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import prepare_recovery_workspace as preparation


class PreparationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.deployment = self.root / 'deployment'
        shutil.copytree(preparation.DEFAULT_ROOT, self.deployment)
        self.upstream = self.root / 'upstream'
        self.upstream.mkdir()
        self.run_git('init', '--quiet')
        files = {'CMakeLists.txt': 'project(ros2_laser_scan_merger)\n',
                 'package.xml': '<package><name>ros2_laser_scan_merger</name></package>\n',
                 'src/main.cpp': 'int main() {return 0;}\n'}
        for relative, content in files.items():
            p = self.upstream / relative
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content)
        self.run_git('add', '.')
        self.run_git('-c', 'user.name=Recovery test', '-c', 'user.email=test@example.invalid',
                     'commit', '--quiet', '-m', 'fixture')
        revision = self.run_git('rev-parse', 'HEAD').decode().strip()
        lock = {'revision': revision, 'repository': 'https://example.invalid/public.git',
                'files': [{'path': relative, 'sha256': hashlib.sha256(content.encode()).hexdigest()}
                          for relative, content in files.items()]}
        (self.deployment / 'reconstruction/scan_merger.lock.json').write_text(json.dumps(lock))
        self.output = self.root / 'prepared'

    def run_git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.upstream), *args], stderr=subprocess.DEVNULL)

    def prepare(self):
        return preparation.prepare(self.upstream, self.output, self.deployment)

    def test_prepared_tree_has_no_vendor_dashboard_and_preserves_merger_override(self):
        result = self.prepare()
        self.assertEqual(result['status'], 'prepared-not-installed')
        self.assertFalse(result['vendor_dashboard_required'])
        destinations = {r['destination'] for r in result['host_files']}
        self.assertFalse(any('mbs-webserver' in p or 'camera-web-ready' in p for p in destinations))
        self.assertNotIn('/etc/systemd/system/ridgeback-startup-recover.service', destinations)
        self.assertNotIn('/etc/systemd/system/robot-map-navi.service', destinations)
        merger = (self.output / preparation.MERGER / 'launch/merge_ridgeback.launch.py').read_text()
        self.assertIn('rmw_fastrtps_cpp', merger)
        camera = (self.output / 'host-files/usr/local/sbin/realsense-camera-start').read_text()
        self.assertIn('source /etc/clearpath/setup.bash', camera)
        self.assertNotIn('source /opt/mybotshop/install', camera)
        for row in result['host_files']:
            self.assertEqual(hashlib.sha256((self.output / row['candidate']).read_bytes()).hexdigest(), row['sha256'])
        self.assertTrue((self.output / 'RECOVERY_PREPARATION.json').is_file())

    def test_existing_output_is_never_overwritten(self):
        self.output.mkdir()
        marker = self.output / 'marker'
        marker.write_text('preserve')
        with self.assertRaisesRegex(ValueError, 'already exists'):
            self.prepare()
        self.assertEqual(marker.read_text(), 'preserve')

    def test_production_output_is_rejected_before_creation(self):
        self.output = Path('/opt/dynamo-hardware-unittest-never-create')
        with self.assertRaisesRegex(ValueError, 'Output must'):
            self.prepare()

    def test_wrong_revision_is_rejected(self):
        lock_path = self.deployment / 'reconstruction/scan_merger.lock.json'
        lock = json.loads(lock_path.read_text());lock['revision'] = '0' * 40
        lock_path.write_text(json.dumps(lock))
        with self.assertRaisesRegex(ValueError, 'recorded revision'):
            self.prepare()
        self.assertFalse(self.output.exists())

    def test_dirty_checkout_is_rejected(self):
        (self.upstream / 'src/main.cpp').write_text('modified')
        with self.assertRaisesRegex(ValueError, 'local changes'):
            self.prepare()
        self.assertFalse(self.output.exists())

    def test_snapshot_tampering_is_rejected_before_output_creation(self):
        (self.deployment / 'files/etc/clearpath/robot.yaml').write_text('changed')
        with self.assertRaisesRegex(ValueError, 'failed verification'):
            self.prepare()
        self.assertFalse(self.output.exists())

    def test_archive_rejects_traversal_and_symlinks_without_writing(self):
        for name, kind in [('../escape', tarfile.REGTYPE), ('link', tarfile.SYMTYPE)]:
            archive = io.BytesIO()
            with tarfile.open(fileobj=archive, mode='w') as t:
                m = tarfile.TarInfo(name);m.type = kind;m.linkname = '/etc/passwd'
                t.addfile(m)
            with self.assertRaisesRegex(ValueError, 'Unsafe entry'):
                preparation.unpack_public_source(archive.getvalue(), self.output)
            self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
