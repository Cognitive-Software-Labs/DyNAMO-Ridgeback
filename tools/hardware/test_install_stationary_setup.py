import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import install_stationary_setup as installer

class StationaryInstallTests(unittest.TestCase):
    def fixture(self,root):
        files={str(root/'host/a'):{'content':'new','mode':0o644},str(root/'host/b'):{'content':'helper','mode':0o755}}
        (root/'host').mkdir();(root/'host/a').write_text('original')
        before={name:installer.snapshot_file(Path(name)) for name in files}
        states={u:{'MainPID':'12','ActiveState':'active','UnitFileState':'enabled'} for u in (*installer.MANAGED,*installer.INDEPENDENT)}
        run=root/'run';run.mkdir()
        for name,entry in files.items():
            p=run/'staged'/name.lstrip('/');p.parent.mkdir(parents=True,exist_ok=True);p.write_text(entry['content'])
        record={'status':'staged-not-installed','files':files,'before':before,'activity':{u:states[u] for u in installer.MANAGED},'independent':{u:states[u] for u in installer.INDEPENDENT},'namespace':'/r100_0160','installed':[]}
        (run/'result.json').write_text(json.dumps(record));return run,files,states
    def write(self,path,data,mode,*args):path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data);path.chmod(mode)
    def test_success_never_controls_hardware_units(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run,files,states=self.fixture(root)
            with patch.object(installer,'desired_files',return_value=files),patch.object(installer,'state',side_effect=lambda u:states[u]),patch.object(installer,'command',return_value='') as cmd,patch.object(installer,'run_probe'),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'write_file',side_effect=self.write),patch.object(installer,'listener_identity',return_value=[{'pid':1}]):
                self.assertEqual(installer.apply(run,True)['status'],'installed-verified-stationary')
                for call in cmd.call_args_list:self.assertFalse(set(call.args)&set(installer.INDEPENDENT))
                self.assertEqual((root/'host/a').read_text(),'new')
    def test_changed_baseline_prevents_any_service_action(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run,files,states=self.fixture(root);(root/'host/a').write_text('other edit')
            with patch.object(installer,'desired_files',return_value=files),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'command') as cmd:
                self.assertRaisesRegex(RuntimeError,'host file changed',installer.apply,run,True);cmd.assert_not_called()
    def test_probe_failure_does_not_stop_desktop(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run,files,states=self.fixture(root)
            with patch.object(installer,'desired_files',return_value=files),patch.object(installer,'state',side_effect=lambda u:states[u]),patch.object(installer,'command') as cmd,patch.object(installer,'run_probe',side_effect=RuntimeError('stop not asserted')),patch.object(installer.os,'geteuid',return_value=0):
                self.assertRaisesRegex(RuntimeError,'stop not asserted',installer.apply,run,True);cmd.assert_not_called()
    def test_post_install_failure_restores_original_files(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run,files,states=self.fixture(root)
            with patch.object(installer,'desired_files',return_value=files),patch.object(installer,'state',side_effect=lambda u:states[u]),patch.object(installer,'command',return_value='') as cmd,patch.object(installer,'run_probe',side_effect=[{},RuntimeError('postcheck failed')]),patch.object(installer.os,'geteuid',return_value=0),patch.object(installer,'write_file',side_effect=self.write),patch.object(installer,'listener_identity',return_value=[]),patch.object(installer.subprocess,'run') as recovery:
                self.assertRaisesRegex(RuntimeError,'postcheck failed',installer.apply,run,True)
                self.assertEqual((root/'host/a').read_text(),'original');self.assertFalse((root/'host/b').exists())
                self.assertEqual(json.loads((run/'result.json').read_text())['status'],'failed-restored-original')
                for call in cmd.call_args_list:self.assertFalse(set(call.args)&set(installer.INDEPENDENT))
                for call in recovery.call_args_list:self.assertFalse(set(call.args[0])&set(installer.INDEPENDENT))
    def test_tampered_staged_payload_cannot_write_outside_scope(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run,files,states=self.fixture(root)
            p=run/'result.json';r=json.loads(p.read_text());r['files']['/etc/other']={'content':'bad','mode':0o644};p.write_text(json.dumps(r))
            with patch.object(installer,'desired_files',return_value=files):self.assertRaisesRegex(RuntimeError,'staged inputs changed',installer.load,run)
