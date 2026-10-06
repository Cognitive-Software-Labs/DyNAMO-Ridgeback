"""Clock installer transactions without package installation or real services."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def tool(tmp_path, monkeypatch):
    loader = SourceFileLoader('time_sync', str(ROOT / 'tools/intel_thor/time_sync'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    calls = []
    def run(*args, check=True):
        calls.append(args)
        return SimpleNamespace(stdout='ntp.ubuntu.com\n' if args[0] == 'timedatectl' else '', returncode=0)
    monkeypatch.setattr(module, 'run', run)
    monkeypatch.setattr(module, 'installed', lambda p: p == 'systemd-timesyncd')
    monkeypatch.setattr(module, 'state', lambda unit: {'enabled': 'enabled' if unit.startswith('systemd') else 'not-found',
                                                     'active': 'active' if unit.startswith('systemd') else 'inactive'})
    return module, module.Host(tmp_path), calls


@pytest.mark.parametrize('role', ['intel', 'thor'])
def test_apply_reapply_and_rollback_keep_original_state(tool, role):
    module, host, calls = tool
    original = host.path('/etc/chrony/chrony.conf')
    original.parent.mkdir(parents=True)
    original.write_text('original configuration\n')
    module.apply(host, role)
    first = host.path(module.BACKUP + '/state.json').read_bytes()
    module.apply(host, role)
    assert host.path(module.BACKUP + '/state.json').read_bytes() == first
    managed = host.path(module.MANAGED).read_text()
    assert '@' not in managed
    if role == 'intel':
        assert 'ntp.ubuntu.com' in managed and 'allow 192.168.131.0/24' in managed
    else:
        assert 'server 192.168.131.1 iburst prefer' in managed and 'ntp.ubuntu.com' not in managed
    assert 'sourcedir' not in host.path(module.MAIN).read_text()
    module.rollback(host)
    assert original.read_text() == 'original configuration\n'
    assert ('apt-get', 'install', '-y', 'systemd-timesyncd') in calls
    assert ('systemctl', 'restart', 'systemd-timesyncd.service') in calls
    assert not host.path(module.BACKUP).exists()


def test_rollback_refuses_external_changes_before_stopping_services(tool):
    module, host, calls = tool
    module.apply(host, 'thor')
    host.path(module.MANAGED).write_text('external edit\n')
    calls.clear()
    with pytest.raises(RuntimeError, match='changed externally'):
        module.rollback(host)
    assert calls == []
    assert host.path(module.BACKUP).exists()


def test_incomplete_snapshot_is_retained(tool):
    module, host, calls = tool
    host.path(module.BACKUP).mkdir(parents=True)
    with pytest.raises(RuntimeError, match='Incomplete'):
        module.apply(host, 'thor')
    assert calls == []
