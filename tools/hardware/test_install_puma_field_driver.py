import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import install_puma_field_driver as installer

REPAIR = '/opt/dynamo-puma/' + 'a' * 64
FEEDBACK = '/opt/dynamo-puma-feedback/' + 'b' * 64
BASE = ('[Service]\nExecStart=\n'
        f"ExecStart=/bin/bash -c 'source /etc/clearpath/setup.bash && source {REPAIR}/share/puma_motor_driver/"
        f"local_setup.bash && export AMENT_PREFIX_PATH={FEEDBACK}:$AMENT_PREFIX_PATH; "
        f"export LD_LIBRARY_PATH={FEEDBACK}/lib:$LD_LIBRARY_PATH; "
        "exec ros2 launch /usr/local/lib/dynamo-hardware/auxiliary_platform.launch.py'\n")


class DropinTests(unittest.TestCase):
    def test_swaps_only_the_driver_prefix(self):
        prefix = Path('/opt/dynamo-puma-field/' + 'c' * 64)
        result = installer.dropin(BASE, prefix)
        self.assertIn(f'source {prefix}/share/puma_motor_driver/local_setup.bash', result)
        self.assertNotIn(REPAIR, result)
        self.assertEqual(result.count(FEEDBACK), 2)  # plugin overlay kept
        self.assertEqual(result.count('ExecStart=\n'), 1)
        self.assertEqual(result.count('exec ros2 launch'), 1)

    def test_refuses_unexpected_launch_chains(self):
        prefix = Path('/opt/dynamo-puma-field/x')
        for base in (BASE.replace(REPAIR, '/opt/ros/jazzy'),  # apt driver, no pinned repair
                     BASE + BASE.splitlines()[-1] + '\n',     # two ExecStart lines
                     BASE.replace('local_setup.bash &&', f'local_setup.bash && source {REPAIR}/share/'
                                  'puma_motor_driver/local_setup.bash &&')):
            with self.assertRaises(RuntimeError):
                installer.dropin(base, prefix)


class CandidateTests(unittest.TestCase):
    def make(self, root, **changes):
        candidate = root / 'artifacts/hardware/candidate'
        binary = candidate / 'install/puma_motor_driver/lib/puma_motor_driver/multi_puma_node'
        binary.parent.mkdir(parents=True)
        binary.write_bytes(b'\x7fELF driver')
        (root / 'patches').mkdir()
        (root / 'patches/field.patch').write_text('patch')
        record = {'status': installer.BUILT, 'motor_node_started': False,
                  'input_sha256': {'patches/field.patch': installer.digest(root / 'patches/field.patch')},
                  'dependency_versions': 'pinned\n', 'motor_binary_sha256': installer.digest(binary)}
        record.update(changes)
        (candidate / 'candidate.json').write_text(json.dumps(record))
        return candidate

    def check(self, **changes):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            candidate = self.make(root, **changes)
            with mock.patch.object(installer, 'ROOT', root), \
                    mock.patch.object(installer, 'versions', return_value='pinned\n'):
                return installer.check_candidate(candidate)[1]

    def test_accepts_matching_build(self):
        self.assertEqual(self.check()['status'], installer.BUILT)

    def test_refuses_incomplete_or_mismatched_builds(self):
        for change in ({'status': 'offline-candidate-build-in-progress'},
                       {'motor_node_started': True},
                       {'dependency_versions': 'other\n'},
                       {'motor_binary_sha256': '0' * 64},
                       {'input_sha256': {'patches/field.patch': '0' * 64}}):
            with self.assertRaises(ValueError):
                self.check(**change)


class RunDirectoryTests(unittest.TestCase):
    def test_requires_artifact_subdirectory(self):
        with self.assertRaises(ValueError):
            installer.run_directory(Path('/tmp/field'))
        with self.assertRaises(ValueError):
            installer.run_directory(installer.ROOT / 'artifacts/hardware')


if __name__ == '__main__':
    unittest.main()
