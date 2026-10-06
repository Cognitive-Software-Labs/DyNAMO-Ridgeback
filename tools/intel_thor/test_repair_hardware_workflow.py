"""Offline transaction and desktop dependency tests; never call real systemd."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import repair_hardware_workflow as repair
import desktop_ready


def isolated(tmp_path, monkeypatch):
    target = tmp_path / 'service'
    target.write_bytes(b'old\x00unit')
    target.chmod(0o640)
    added = tmp_path / 'added'
    files = {target: {'content': 'new service', 'mode': 0o644},
             added: {'content': 'new helper', 'mode': 0o755}}
    monkeypatch.setattr(repair, 'desired_files', lambda: files)
    monkeypatch.setattr(repair.os, 'chown', lambda *args: None)
    directory = tmp_path / 'evidence'
    directory.mkdir()
    instance = repair.Repair(directory)
    instance.before = {str(path): repair.snapshot_file(path) for path in files}
    instance.activity = {unit: 'active' if unit in (repair.WEB, repair.VNC, repair.REMOTE) else 'inactive'
                         for unit in repair.UNITS}
    calls = []
    monkeypatch.setattr(instance, 'command', lambda args, **kwargs: calls.append(args) or '')
    monkeypatch.setattr(instance, 'check_preserved', lambda: None)
    return instance, target, added, calls


def apply(instance):
    instance.changed = True
    for path, entry in instance.files.items():
        instance.applied.append(str(path))
        repair.write_file(path, entry['content'].encode(), entry['mode'])


def test_rollback_restores_bytes_permissions_and_activity(tmp_path, monkeypatch):
    instance, original, added, calls = isolated(tmp_path, monkeypatch)
    apply(instance)
    unrelated = tmp_path / 'unrelated'; unrelated.write_text('keep')
    instance.rollback()
    assert original.read_bytes() == b'old\x00unit'
    assert original.stat().st_mode & 0o777 == 0o640
    assert not added.exists() and unrelated.read_text() == 'keep'
    assert calls[-3:] == [['systemctl', 'start', unit] for unit in (repair.VNC, repair.REMOTE, repair.WEB)]
    assert not instance.changed and instance.stage == 'rolled-back'


def test_rollback_refuses_external_edit_before_service_stop(tmp_path, monkeypatch):
    instance, original, _, calls = isolated(tmp_path, monkeypatch)
    apply(instance)
    original.write_text('external')
    with pytest.raises(RuntimeError, match='External edit'):
        instance.rollback()
    assert calls == []


def test_partial_write_intent_recovers_original(tmp_path, monkeypatch):
    instance, original, added, calls = isolated(tmp_path, monkeypatch)
    # Intent saved, process interrupted before replacing the file.
    instance.changed = True
    instance.applied = [str(original)]
    instance.rollback()
    assert original.read_bytes() == b'old\x00unit' and not added.exists()


def test_atomic_replace_failure_cleans_temporary_and_keeps_original(tmp_path, monkeypatch):
    instance, original, _, _ = isolated(tmp_path, monkeypatch)
    def fail(*args):
        raise OSError('simulated chown failure')
    monkeypatch.setattr(repair.os, 'chown', fail)
    with pytest.raises(OSError):
        repair.write_file(original, b'candidate', 0o644)
    assert original.read_bytes() == b'old\x00unit'
    assert not list(tmp_path.glob('.service.dynamo-*'))


def test_repair_verifies_ownership_across_vendor_restart(tmp_path, monkeypatch):
    instance, _, _, calls = isolated(tmp_path, monkeypatch)
    monkeypatch.setattr(instance, 'prepare', lambda: None)
    monkeypatch.setattr(instance, 'wait_all', lambda: {'vnc': 11, 'proxy': 12, 'remote': 13})
    monkeypatch.setattr(instance, 'wait_vendor_skips', lambda *args: None)
    instance.install()
    proof = json.loads((instance.directory / 'verification.json').read_text())
    assert proof['passed'] and proof['before'] == proof['after']
    assert proof['cold_boot_tested'] is False
    assert ['systemctl', 'restart', repair.WEB] in calls
    assert ['systemctl', 'restart', repair.CAMERA_READY] in calls
    assert all(not any(name in arg for name in ('clearpath-sensors', 'realsense-camera', 'cyclonedds.xml'))
               for call in calls for arg in call)


def test_changed_listener_identity_fails_repair(tmp_path, monkeypatch):
    instance, _, _, _ = isolated(tmp_path, monkeypatch)
    monkeypatch.setattr(instance, 'prepare', lambda: None)
    monkeypatch.setattr(instance, 'wait_vendor_skips', lambda *args: None)
    samples = iter([{'vnc': 11}, {'vnc': 11}, {'vnc': 99}])
    monkeypatch.setattr(instance, 'wait_all', lambda: next(samples))
    with pytest.raises(RuntimeError, match='Vendor restart changed'):
        instance.install()
    assert not (instance.directory / 'verification.json').exists()


@pytest.mark.parametrize('group,accepted', [
    ('/system.slice/ridgeback-vnc.service', True),
    ('/system.slice/mbs-webserver.service', False),
    ('/system.slice/not-ridgeback-vnc.service', False),
])
def test_ready_gate_checks_exact_control_group(tmp_path, group, accepted):
    process = tmp_path / '42'; process.mkdir()
    exe = tmp_path / 'Xtigervnc'; exe.touch()
    (process / 'exe').symlink_to(exe)
    (process / 'cgroup').write_text('0::' + group + '\n')
    (process / 'stat').write_text('42 (Xtigervnc) ' + ' '.join(['S'] + ['0'] * 18 + ['100']))
    from types import SimpleNamespace
    def run(*args, **kwargs):
        return SimpleNamespace(stdout='LISTEN users:(("Xtigervnc",pid=42,fd=3))')
    if accepted:
        assert desktop_ready.listener_identity(5901, repair.VNC, run, tmp_path) == [{'pid': 42, 'start_ticks': 100}]
    else:
        with pytest.raises(RuntimeError, match='owned outside'):
            desktop_ready.listener_identity(5901, repair.VNC, run, tmp_path)


def test_ready_gate_rejects_empty_listener(tmp_path):
    from types import SimpleNamespace
    with pytest.raises(RuntimeError, match='No visible listener'):
        desktop_ready.listener_identity(5901, repair.VNC,
            lambda *a, **kw: SimpleNamespace(stdout=''), tmp_path)


def test_deployment_has_single_foreground_vnc_owner_and_retrying_proxies():
    files = repair.desired_files()
    vnc = files[repair.SYSTEM / repair.VNC]['content']
    assert 'Type=exec' in vnc and ':1 -fg ' in vnc and 'PIDFile=' not in vnc
    assert 'Requires=user@1000.service' in vnc
    assert 'XDG_RUNTIME_DIR=/run/user/1000' in vnc
    assert '50_mesa.json' in vnc
    for path in (repair.SYSTEM / repair.PROXY,
                 repair.SYSTEM / 'ridgeback-remote-screen.service.d/20-independent-vnc.conf'):
        content = files[path]['content']
        assert 'Requires=' not in content and 'PartOf=' not in content
        assert 'Restart=always' in content
    remote = files[repair.SYSTEM / 'ridgeback-remote-screen.service.d/20-independent-vnc.conf']['content']
    assert 'ExecStart=\nExecStart=/usr/bin/websockify' in remote
    assert 'start-remote-screen.sh' not in remote
    web = files[repair.SYSTEM / 'mbs-webserver.service.d/20-independent-vnc.conf']['content']
    assert 'ExecStartPre=' in web and 'desktop_ready.py' in web
    assert Path('/etc/clearpath/cyclonedds.xml') not in files
    startup = files[repair.LIB / 'vnc-xstartup']['content']
    assert 'unset __EGL_VENDOR_LIBRARY_FILENAMES LIBGL_ALWAYS_SOFTWARE' in startup
    assert 'dbus-launch' not in startup
    assert 'exec dbus-run-session -- startxfce4' in startup
    assert 'dbus-update-activation-environment' not in startup
    readiness = files[repair.SYSTEM / 'camera-web-ready.service.d/30-direct-readiness.conf']['content']
    assert 'User=robot' in readiness and 'Environment=HOME=/home/robot' in readiness


def test_recovery_rejects_state_with_arbitrary_root_target(tmp_path, monkeypatch):
    instance, _, _, calls = isolated(tmp_path, monkeypatch)
    instance.state()
    saved = json.loads((instance.directory / 'state.json').read_text())
    saved['desired']['/etc/arbitrary'] = {'content': 'bad', 'mode': 0o644}
    with pytest.raises(RuntimeError, match='allowlist'):
        instance.load_recovery(saved)
    assert not instance.changed and not calls


def test_recovery_rejects_applied_path_outside_allowlist(tmp_path, monkeypatch):
    instance, _, _, calls = isolated(tmp_path, monkeypatch)
    instance.state()
    saved = json.loads((instance.directory / 'state.json').read_text())
    saved['applied'] = ['/etc/arbitrary']
    with pytest.raises(RuntimeError, match='applied-file'):
        instance.load_recovery(saved)
    assert not instance.changed and not calls


def test_state_and_system_file_intent_are_durable(tmp_path, monkeypatch):
    instance, original, _, _ = isolated(tmp_path, monkeypatch)
    syncs = []
    monkeypatch.setattr(repair, 'sync_directory', lambda path: syncs.append(path))
    instance.state()
    repair.write_file(original, b'new', 0o640)
    assert syncs == [instance.directory, original.parent]


def test_proxy_browser_workers_do_not_change_master_identity(tmp_path):
    from types import SimpleNamespace
    executable = tmp_path / Path('/usr/bin/python3').resolve().name; executable.touch()
    for pid in (42, 43, 44):
        process = tmp_path / str(pid); process.mkdir()
        (process / 'exe').symlink_to(executable)
        (process / 'cgroup').write_text('0::/system.slice/ridgeback-remote-screen.service\n')
        (process / 'stat').write_text(f'{pid} (python3) ' + ' '.join(['S'] + ['0'] * 18 + [str(100 + pid)]))
    current = [42]
    def run(argv, **kwargs):
        if argv[0] == 'systemctl':
            return SimpleNamespace(stdout='42\n')
        return SimpleNamespace(stdout=' '.join(f'users:(("websockify",pid={pid},fd=3))' for pid in current))
    before = desktop_ready.listener_identity(9100, repair.REMOTE, run, tmp_path)
    current[:] = [42, 43, 44]
    after = desktop_ready.listener_identity(9100, repair.REMOTE, run, tmp_path)
    assert before == after == [{'pid': 42, 'start_ticks': 142}]
    current[:] = [43]
    with pytest.raises(RuntimeError, match='main process'):
        desktop_ready.listener_identity(9100, repair.REMOTE, run, tmp_path)
