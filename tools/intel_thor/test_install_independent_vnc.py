"""Offline desktop-lifecycle checks; no real services are touched."""
import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location('vnc_installer', Path(__file__).with_name('install_independent_vnc.py'))
vnc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(vnc)


def isolated(tmp_path, monkeypatch):
    files = {tmp_path / path.name: text for path, text in vnc.FILES.items()}
    monkeypatch.setattr(vnc, 'FILES', files)
    directory = tmp_path / 'evidence'
    directory.mkdir()
    installer = vnc.Installer(directory)
    installer.original_active = True
    calls = []
    monkeypatch.setattr(installer, 'command', lambda args, **kwargs: calls.append(args) or '')
    return installer, files, calls


def test_install_verifies_listener_identity_across_webserver_restart(tmp_path, monkeypatch):
    installer, files, calls = isolated(tmp_path, monkeypatch)
    (installer.directory / 'preserved-hashes.json').write_text('{}')
    monkeypatch.setattr(installer, 'validate', lambda: None)
    identities = {'vnc': [{'pid': 42, 'start_ticks': 100}], 'proxy': [{'pid': 43, 'start_ticks': 101}]}
    monkeypatch.setattr(installer, 'wait_listeners', lambda: identities)
    monkeypatch.setattr(installer, 'wait_vendor_skips', lambda since: None)
    installer.install()
    assert installer.stage == 'installed'
    assert all(path.read_text() == text for path, text in files.items())
    proof = json.loads((installer.directory / 'verification.json').read_text())
    assert proof['passed'] and proof['before'] == proof['after']
    assert ['systemctl', 'restart', vnc.WEB] in calls
    assert all('clearpath-sensors.service' not in call for call in calls)


def test_changed_desktop_identity_fails_verification(tmp_path, monkeypatch):
    installer, _, _ = isolated(tmp_path, monkeypatch)
    (installer.directory / 'preserved-hashes.json').write_text('{}')
    monkeypatch.setattr(installer, 'validate', lambda: None)
    monkeypatch.setattr(installer, 'wait_vendor_skips', lambda since: None)
    identities = iter([{'pid': 42}, {'pid': 42}, {'pid': 43}])
    monkeypatch.setattr(installer, 'wait_listeners', lambda: next(identities))
    with pytest.raises(RuntimeError, match='Restarting webserver changed desktop'):
        installer.install()
    assert not (installer.directory / 'verification.json').exists()


def test_rollback_removes_only_owned_files_and_restores_vendor(tmp_path, monkeypatch):
    installer, files, calls = isolated(tmp_path, monkeypatch)
    for path, text in files.items():
        path.write_text(text)
    unrelated = tmp_path / 'unrelated.service'
    unrelated.write_text('keep')
    installer.changed = True
    installer.rollback()
    assert unrelated.read_text() == 'keep'
    assert all(not path.exists() for path in files)
    assert calls[-1] == ['systemctl', 'start', vnc.WEB]
    assert installer.stage == 'rolled-back' and not installer.changed


def test_rollback_refuses_external_edits_before_stopping_anything(tmp_path, monkeypatch):
    installer, files, calls = isolated(tmp_path, monkeypatch)
    changed = next(iter(files))
    changed.write_text('external change')
    with pytest.raises(RuntimeError, match='externally changed'):
        installer.rollback()
    assert calls == []
    assert changed.read_text() == 'external change'


@pytest.mark.parametrize('group,accepted', [
    ('/system.slice/ridgeback-vnc.service', True),
    ('/system.slice/ridgeback-vnc.service/nested.scope', True),
    ('/system.slice/mbs-webserver.service', False),
    ('/system.slice/not-ridgeback-vnc.service', False),
])
def test_listener_must_belong_to_independent_service(tmp_path, monkeypatch, group, accepted):
    installer, _, _ = isolated(tmp_path, monkeypatch)
    proc = tmp_path / 'proc'
    pid = proc / '42'
    pid.mkdir(parents=True)
    executable = tmp_path / 'Xtigervnc'
    executable.touch()
    (pid / 'exe').symlink_to(executable)
    (pid / 'cgroup').write_text('0::' + group + '\n')
    (pid / 'stat').write_text('42 (Xtigervnc) ' + ' '.join(['S'] + ['0'] * 18 + ['1234']))
    original_path = vnc.Path
    monkeypatch.setattr(vnc, 'Path', lambda value: proc if value == '/proc' else original_path(value))
    monkeypatch.setattr(installer, 'command', lambda *args: 'LISTEN users:(("Xtigervnc",pid=42,fd=9))')
    if accepted:
        assert installer.listener(5901, vnc.VNC, 'Xtigervnc') == [{'pid': 42, 'start_ticks': 1234}]
    else:
        with pytest.raises(RuntimeError, match='another service'):
            installer.listener(5901, vnc.VNC, 'Xtigervnc')
