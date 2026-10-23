import tempfile
import unittest
from pathlib import Path
from install_puma_feedback_plugin import dropin, manifest


class FeedbackPluginInstallTests(unittest.TestCase):
    def test_preserves_platform_chain_and_adds_plugin_overlay(self):
        base = ('[Unit]\nAfter=chrony.service\n[Service]\nExecStart=\n'
                "ExecStart=/bin/bash -c 'source /etc/clearpath/setup.bash && source /opt/dynamo-puma/pinned/share/puma_motor_driver/local_setup.bash && exec ros2 launch /usr/local/lib/dynamo-hardware/auxiliary_platform.launch.py'\n"
                'ExecStartPre=/usr/bin/python3 /usr/local/lib/dynamo-hardware/check_clock_ready.py --timeout 900\n')
        result = dropin(base, Path('/opt/dynamo-puma-feedback/verified'))
        self.assertIn('source /opt/dynamo-puma/pinned', result)
        self.assertNotIn('ExecStartPre=', result)
        self.assertIn('export AMENT_PREFIX_PATH=/opt/dynamo-puma-feedback/verified:$AMENT_PREFIX_PATH;', result)
        self.assertEqual(result.count('exec ros2 launch'), 1)
        with self.assertRaises(RuntimeError): dropin(base.replace('auxiliary_platform', 'other_platform'), Path('/opt/x'))

    def test_rejects_out_of_scope_symlink(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)
            (path/'lib').mkdir()
            (path/'lib/libpuma_hardware.so').write_bytes(b'fake')
            (path/'lib/libunexpected.so').symlink_to('/tmp/other.so')
            with self.assertRaises(RuntimeError): manifest(path)


if __name__ == '__main__': unittest.main()
