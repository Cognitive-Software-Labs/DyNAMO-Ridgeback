"""Deployment gates and failure/rollback ownership, entirely in temporary files."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).parent))
import install_puma_repair as installer


class PumaInstallTests(unittest.TestCase):
    def fixture(self,root,status='staged'):
        run=root/'artifacts/hardware/run';run.mkdir(parents=True)
        package=run/'package';package.mkdir()
        (package/'binary').write_bytes(b'pinned compiled package')
        manifest=installer.file_manifest(package)
        digest=installer.caps.digest(json.dumps(manifest,sort_keys=True).encode())
        prefix=root/'puma'/digest
        owned={root/'host/wait.py',root/'host/guard.conf',root/'host/platform.conf'}
        files={}
        for path in owned:
            data=('new:'+path.name).encode();files[str(path)]=installer.caps.digest(data)
            target=run/'staged'/str(path).lstrip('/');target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(data)
        record={'status':status,'namespace':'/r100_0160','prefix':str(prefix),
                'package_hash':digest,'package_manifest':manifest,'files':files,
                'baseline_caps':{'old-guard':'unchanged'},'dependency_versions':'versions'}
        (run/'result.json').write_text(json.dumps(record))
        (run/'SOURCE.json').write_text(json.dumps({'patch_application_verified':True}))
        return run,prefix,owned,record

    def contexts(self,root,owned):
        from contextlib import ExitStack
        stack=ExitStack()
        stack.enter_context(patch.object(installer,'ROOT',root))
        stack.enter_context(patch.object(installer,'PREFIX_ROOT',root/'puma'))
        stack.enter_context(patch.object(installer,'OWNED',owned))
        stack.enter_context(patch.object(installer,'check_caps',return_value={'old-guard':'unchanged'}))
        stack.enter_context(patch.object(installer,'versions',return_value='versions'))
        stack.enter_context(patch.object(installer.os,'geteuid',return_value=0))
        stack.enter_context(patch.object(installer.os,'chown'))
        stack.enter_context(patch.object(installer,'secure_prefix'))
        stack.enter_context(patch.object(installer,'root_owned_package'))
        return stack

    def test_manifest_rejects_symlinks_and_writable_package_files(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);file=root/'binary';file.write_bytes(b'bin')
            (root/'alias').symlink_to(file)
            with self.assertRaisesRegex(ValueError,'symlink'):
                installer.file_manifest(root)
            (root/'alias').unlink();file.chmod(0o666)
            with self.assertRaisesRegex(ValueError,'permissions'):
                installer.file_manifest(root)

    def test_old_unverified_build_is_refused_before_service_actions(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);run,prefix,owned,record=self.fixture(root)
            (run/'SOURCE.json').write_text('{}')
            with self.contexts(root,owned),patch.object(installer.caps,'systemctl') as service:
                with self.assertRaisesRegex(ValueError,'lacks verified applied patches'):
                    installer.apply(run,True)
                service.assert_not_called()

    def test_changed_package_or_dependencies_cannot_restart_platform(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);run,prefix,owned,record=self.fixture(root)
            with self.contexts(root,owned),patch.object(installer.caps,'systemctl') as systemctl:
                (run/'package/binary').write_bytes(b'changed')
                with self.assertRaisesRegex(RuntimeError,'package changed'):
                    installer.apply(run,True)
                systemctl.assert_not_called()
                (run/'package/binary').write_bytes(b'pinned compiled package')
                with patch.object(installer,'versions',return_value='new versions'):
                    with self.assertRaisesRegex(RuntimeError,'dependencies changed'):
                        installer.apply(run,True)
                systemctl.assert_not_called()

    def test_post_install_failure_leaves_platform_stopped(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);run,prefix,owned,record=self.fixture(root)
            with self.contexts(root,owned),patch.object(installer.caps,'systemctl') as systemctl,\
                 patch.object(installer.caps,'state',return_value={'ActiveState':'active','MainPID':'123'}),\
                 patch.object(installer.caps,'run_probe',side_effect=[{'ok':True},RuntimeError('post gate')]):
                with self.assertRaisesRegex(RuntimeError,'post gate'):
                    installer.apply(run,True)
                self.assertEqual(systemctl.call_args.args,('stop',installer.caps.PLATFORM))
                for call in systemctl.call_args_list:
                    self.assertFalse(set(call.args)&set(installer.INDEPENDENT))
                self.assertEqual(json.loads((run/'result.json').read_text())['status'],
                                 'failed-platform-left-stopped')
                self.assertTrue(all(path.exists() for path in owned))

    def test_partial_install_rollback_preserves_caps_and_refuses_changed_file(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);run,prefix,owned,record=self.fixture(root,'failed-platform-left-stopped')
            (run/'backup.json').write_text(json.dumps({'baseline_caps':record['baseline_caps'],
                                                      'owned_paths_existed':False}))
            target=next(iter(owned));target.parent.mkdir(parents=True)
            original=(run/'staged'/str(target).lstrip('/')).read_bytes()
            target.write_bytes(b'unrelated change')
            with self.contexts(root,owned),patch.object(installer.caps,'systemctl') as systemctl,\
                 patch.object(installer.caps,'state',return_value={'ActiveState':'inactive'}),\
                 patch.object(installer.caps,'run_probe',return_value={'ok':True}):
                with self.assertRaisesRegex(RuntimeError,'refuse rollback'):
                    installer.restore(run,True)
                systemctl.assert_not_called()
                target.write_bytes(original)
                restored=installer.restore(run,True)
                self.assertEqual(restored['status'],'restored-original-capped-driver')
                self.assertTrue(all(not path.exists() for path in owned))
                for call in systemctl.call_args_list:
                    self.assertFalse(set(call.args)&set(installer.INDEPENDENT))


if __name__=='__main__':unittest.main()
