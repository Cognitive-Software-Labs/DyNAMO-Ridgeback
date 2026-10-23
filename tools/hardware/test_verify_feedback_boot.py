import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
import sys

import verify_feedback_boot as boot


class FeedbackBootTest(unittest.TestCase):
    def test_sudo_verification_loads_ros_before_the_live_probe(self):
        with patch.object(sys, 'argv', ['verify_feedback_boot.py', '--verify',
                                        '--run-dir', '/home/robot/DyNAMO-Ridgeback/artifacts/hardware/retry']), \
             patch.object(boot.os, 'geteuid', return_value=0), \
             patch.object(boot.importlib.util, 'find_spec', return_value=None), \
             patch.dict(boot.os.environ, {}, clear=True), \
             patch.object(boot.os, 'execv', side_effect=RuntimeError('reexec')) as execv:
            with self.assertRaisesRegex(RuntimeError, 'reexec'):
                boot.main()
            self.assertEqual(execv.call_args.args[0], '/bin/bash')

    def test_rejects_a_second_check_in_the_same_boot(self):
        with TemporaryDirectory() as temp:
            run = Path(temp)
            (run / 'before.json').write_text(json.dumps({
                'boot_id': 'same', 'plugin_prefix': '/prefix',
                'package_hash': 'hash', 'dropin_sha256': 'drop',
                'chrony_sha256': 'chrony'}))
            with patch.object(boot, 'load_installed', return_value={
                'prefix': '/prefix', 'package_hash': 'hash'}), \
                 patch.object(boot, 'current_boot_id', return_value='same'), \
                 patch.object(boot, 'digest', side_effect=['drop', 'chrony']):
                with self.assertRaisesRegex(RuntimeError, 'has not rebooted'):
                    boot.verify(run)

    def test_requires_the_exact_plugin_after_reboot(self):
        with TemporaryDirectory() as temp:
            run = Path(temp)
            (run / 'before.json').write_text(json.dumps({
                'boot_id': 'old', 'plugin_prefix': '/prefix',
                'package_hash': 'hash', 'dropin_sha256': 'drop',
                'chrony_sha256': 'chrony'}))
            with patch.object(boot, 'load_installed', return_value={
                'prefix': '/prefix', 'package_hash': 'hash'}), \
                 patch.object(boot, 'current_boot_id', return_value='new'), \
                 patch.object(boot, 'digest', side_effect=['drop', 'chrony']), \
                 patch.object(boot, 'running_plugin', return_value={
                     'pid': 42, 'loaded': ['/opt/ros/jazzy/lib/libpuma_hardware.so']}), \
                 patch.object(boot, 'verify_auxiliary', return_value={
                     'status': 'post-reboot-check-passed'}):
                result = boot.verify(run)
            self.assertEqual(result['status'], 'feedback-clock-post-boot-failed')
            self.assertFalse(result['checks']['plugin_loaded_after_boot'])


if __name__ == '__main__':
    unittest.main()
