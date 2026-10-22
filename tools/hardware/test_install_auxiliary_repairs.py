import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import install_auxiliary_repairs as installer

class AuxiliaryTests(unittest.TestCase):
    def fixture(self,root):
        run=root/'run';run.mkdir();path=root/'new/helper.py';files={str(path):{'content':'new','mode':0o644}}
        record={'namespace':'/r100_0160','files':files,'inputs':{},'before':{str(path):None},'preserved':{u:'pid unchanged' for u in installer.PRESERVED},'status':'staged-not-installed','intent':[]}
        (run/'result.json').write_text(json.dumps(record));return run,path,record
    def write(self,p,data,mode,*a):p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data);p.chmod(mode)
    def test_success_restarts_only_platform(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d))
            with patch.object(installer,'load',return_value=record),patch.object(installer,'states',return_value=record['preserved']),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'run_probe'),patch.object(installer,'command',return_value='') as calls,patch.object(installer,'write_file',side_effect=self.write):
                self.assertEqual(installer.apply(run,True)['status'],'installed-stationary-awaiting-data-verification')
                for c in calls.call_args_list:self.assertFalse(set(c.args)&set(installer.PRESERVED))
    def test_unpressed_preflight_cannot_write_or_restart(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d))
            with patch.object(installer,'load',return_value=record),patch.object(installer,'states',return_value=record['preserved']),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'run_probe',side_effect=RuntimeError('stop not pressed')),patch.object(installer,'command') as calls:
                self.assertRaisesRegex(RuntimeError,'stop not pressed',installer.apply,run,True);calls.assert_not_called();self.assertFalse(path.exists())
    def test_clock_not_ready_prevents_platform_stop(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d))
            with patch.object(installer,'load',return_value=record),patch.object(installer,'states',return_value=record['preserved']),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'run_probe'),patch.object(installer,'command',side_effect=RuntimeError('clock not ready')) as calls:
                self.assertRaisesRegex(RuntimeError,'clock not ready',installer.apply,run,True);self.assertEqual(calls.call_count,1);self.assertFalse(path.exists())
    def test_postcheck_failure_removes_only_owned_additions(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d))
            with patch.object(installer,'load',return_value=record),patch.object(installer,'states',return_value=record['preserved']),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'run_probe',side_effect=[{},RuntimeError('post failure')]),patch.object(installer,'command',return_value='') as calls,patch.object(installer,'write_file',side_effect=self.write):
                self.assertRaisesRegex(RuntimeError,'post failure',installer.apply,run,True);self.assertFalse(path.exists());self.assertEqual(json.loads((run/'result.json').read_text())['status'],'failed-restored-original')
                for c in calls.call_args_list:self.assertFalse(set(c.args)&set(installer.PRESERVED))
    def test_source_change_prevents_loading_receipt(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d))
            with patch.object(installer,'desired',return_value=('/r100_0160',{},{})):self.assertRaisesRegex(RuntimeError,'source/baseline changed',installer.load,run)

    def test_receipt_cannot_add_unowned_rollback_path(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d));record['intent']=['/etc/other'];(run/'result.json').write_text(json.dumps(record))
            with patch.object(installer,'desired',return_value=(record['namespace'],record['files'],record['inputs'])):
                self.assertRaisesRegex(RuntimeError,'invalid mutation scope',installer.load,run)
    def test_verification_only_finalizes_without_service_mutation(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d));record['status']='applying';record['intent']=[str(path)]
            entry=record['files'][str(path)]
            snapshot={'content_hex':entry['content'].encode().hex(),'mode':entry['mode'],'uid':0,'gid':0}
            from types import SimpleNamespace
            with patch.object(installer,'load',return_value=record),patch.object(installer,'snapshot_file',return_value=snapshot),patch.object(installer,'states',return_value=record['preserved']),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'run_probe',return_value={'ok':True}),patch.object(installer,'command',return_value='{"ready":true}') as commands,patch.object(installer.subprocess,'run',return_value=SimpleNamespace(stdout='{"ok":true}')):
                self.assertEqual(installer.verify(run,True)['status'],'installed-verified-stationary')
                self.assertEqual(commands.call_count,1)
                self.assertEqual(commands.call_args.args[0],'/usr/bin/python3')
    def test_partial_install_cannot_be_finalized(self):
        with tempfile.TemporaryDirectory() as d:
            run,path,record=self.fixture(Path(d));record['status']='applying'
            with patch.object(installer,'load',return_value=record),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'command') as commands:
                self.assertRaisesRegex(RuntimeError,'incomplete installation',installer.verify,run,True);commands.assert_not_called()
