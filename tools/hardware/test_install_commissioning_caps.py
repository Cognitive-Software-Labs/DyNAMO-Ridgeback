"""Offline checks for tightly scoped commissioning deployment inputs."""
import importlib.util
import base64
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).parent))
import install_commissioning_caps as installer


class InstallerTests(unittest.TestCase):
    def test_original_manual_profile_preserves_guard_and_routing(self):
        normal=installer.desired_files('r100_0160')
        manual=installer.desired_files('r100_0160',original_manual_speeds=True)
        unit=installer.SYSTEM/installer.GUARD
        self.assertIn(b'--original-manual-speeds',manual[unit])
        self.assertNotIn(b'--require-test-permit',manual[unit])
        for path in normal:
            if path != unit:self.assertEqual(normal[path],manual[path])
        with self.assertRaises(ValueError):
            installer.desired_files('r100_0160',test_interlock=True,original_manual_speeds=True)

    def test_temporary_interlock_only_changes_guard_start_option(self):
        normal=installer.desired_files('r100_0160')
        locked=installer.desired_files('r100_0160',test_interlock=True)
        unit=installer.SYSTEM/installer.GUARD
        self.assertNotIn(b'--require-test-permit',normal[unit])
        self.assertIn(b'--require-test-permit',locked[unit])
        self.assertEqual(set(normal),set(locked))
        for path in normal:
            if path != unit:self.assertEqual(normal[path],locked[path])

    def test_only_four_owned_paths_and_pinned_copy(self):
        files=installer.desired_files('r100_0160')
        self.assertEqual(len(files),4)
        self.assertNotIn(Path('/etc/clearpath/robot.yaml'),files)
        source=files[installer.LIB/'commissioning_speed_guard.py']
        wrapper=files[installer.LIB/'commissioning_platform.launch.py'].decode()
        self.assertIn(installer.digest(source),wrapper)
        self.assertLess(wrapper.index('SetRemap(src='),wrapper.index('IncludeLaunchDescription(Python'))
        compile(wrapper,'wrapper','exec')
        self.assertTrue(all(str(p).startswith(('/usr/local/lib/dynamo-hardware/','/etc/systemd/system/')) for p in files))

    def test_platform_requires_guard_without_robot_or_desktop_restart(self):
        files=installer.desired_files('r100_0160')
        dropin=files[installer.SYSTEM/(installer.PLATFORM+'.d')/'70-dynamo-commissioning.conf'].decode()
        self.assertIn('Requires='+installer.GUARD,dropin)
        self.assertIn('ExecStart=\nExecStart=',dropin)
        for text in [c.decode() for c in files.values()]:
            self.assertNotIn('systemctl restart clearpath-robot',text)
            self.assertNotIn('mbs-webserver',text)

    def test_atomic_write_refuses_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); target=root/'target';link=root/'link'
            target.write_bytes(b'original');link.symlink_to(target)
            with self.assertRaises(RuntimeError):installer.write_file(link,b'new')
            self.assertEqual(target.read_bytes(),b'original')
            installer.write_file(target,b'new')
            self.assertEqual(target.read_bytes(),b'new')
            self.assertEqual(target.stat().st_mode & 0o777,0o644)

    def test_probe_failures_do_not_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            fake=type('Response',(),{'returncode':1,'stdout':'{"ok":false}'})()
            with patch.object(installer.subprocess,'run',return_value=fake),self.assertRaises(RuntimeError):
                installer.run_probe('/r100_0160',Path(directory),'before')
            self.assertTrue((Path(directory)/'before-probe.json').exists())

    def test_clock_margin_rejects_stale_reference_before_restart(self):
        stale=type('Response',(),{'returncode':1,'stdout':'{"ready": false}'})()
        fresh=type('Response',(),{'returncode':0,'stdout':'{"ready": true}'})()
        with patch.object(installer.subprocess,'run',return_value=stale) as run:
            with self.assertRaisesRegex(RuntimeError,'clock startup margin'):
                installer.require_clock_start_margin()
            self.assertEqual(run.call_args_list[0].args[0][-2:],['--max-reference-age','60'])
        with patch.object(installer.subprocess,'run',return_value=fresh):
            installer.require_clock_start_margin()
        burst=type('Response',(),{'returncode':0,'stdout':'200 OK'})()
        with patch.object(installer.subprocess,'run',side_effect=[stale,burst,fresh]) as run:
            installer.require_clock_start_margin()
            self.assertEqual(run.call_args_list[1].args[0],['/usr/bin/chronyc','burst','2/4'])
            self.assertEqual(run.call_args_list[2].args[0][-4:],
                             ['--timeout','30','--max-reference-age','60'])

    def test_upgrade_requires_exact_verified_previous_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=root/'artifacts/hardware/previous';run.mkdir(parents=True)
            target=root/'owned.py';target.write_bytes(b'previous')
            record={'status':'installed-verified-stationary','namespace':'/r100_0160',
                    'files':{str(target):installer.digest(b'previous')}}
            (run/'result.json').write_text(json.dumps(record))
            with patch.object(installer,'ROOT',root):
                self.assertEqual(installer.verify_upgrade('/r100_0160',{target:b'candidate'},run),run)
                target.write_bytes(b'changed')
                with self.assertRaisesRegex(RuntimeError,'differs'):
                    installer.verify_upgrade('/r100_0160',{target:b'candidate'},run)
                target.write_bytes(b'previous')
                for change in [{'namespace':'/wrong'},{'status':'staged'},{'files':{}}]:
                    (run/'result.json').write_text(json.dumps({**record,**change}))
                    with self.assertRaises(RuntimeError):
                        installer.verify_upgrade('/r100_0160',{target:b'candidate'},run)
                with self.assertRaises(ValueError):
                    installer.verify_upgrade('/r100_0160',{target:b'candidate'},root)

    def test_restore_exact_bytes_and_reject_changed_owned_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=root/'artifacts/hardware/run';run.mkdir(parents=True)
            target=root/'owned.py';target.write_bytes(b'installed')
            result={'namespace':'/r100_0160','files':{str(target):installer.digest(b'installed')},
                    'caps':{'planar_m_s':0.05},'test_interlock':True,'original_manual_speeds':False}
            backup={'files':[{'path':str(target),'existed':True,'content_b64':base64.b64encode(b'original').decode(),'mode':0o640,'uid':0,'gid':0}],
                    'services':{installer.PLATFORM:{'ActiveState':'active'},installer.GUARD:{'ActiveState':'inactive'}}}
            (run/'result.json').write_text(json.dumps(result));(run/'backup.json').write_text(json.dumps(backup))
            with patch.object(installer,'ROOT',root),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'desired_files',return_value={target:b'installed'}),patch.object(installer,'state',return_value={'ActiveState':'inactive'}),patch.object(installer,'systemctl') as systemctl,patch.object(installer.os,'chown'):
                target.write_bytes(b'changed')
                with self.assertRaises(RuntimeError):installer.restore(run,True)
                systemctl.assert_not_called()
                target.write_bytes(b'installed')
                restored=installer.restore(run,True)
                self.assertEqual(target.read_bytes(),b'original')
                self.assertEqual(target.stat().st_mode & 0o777,0o640)
                self.assertEqual(restored['status'],'restored-original-caps-removed-platform-starting')
                systemctl.assert_any_call('start','--no-block',installer.PLATFORM)
                self.assertNotIn('caps',restored)
                self.assertNotIn('test_interlock',restored)
                self.assertEqual(restored['replaced_install']['test_interlock'],True)
                self.assertEqual(restored['restored_files'][str(target)],installer.digest(b'original'))


if __name__=='__main__':unittest.main()
