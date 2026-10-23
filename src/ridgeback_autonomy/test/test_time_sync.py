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
        # Slow internet corrections keep sensor stamps and PTP smooth.
        assert 'maxslewrate 500' in managed
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


def test_boot_step_installs_beside_chrony_and_removes_only_its_own_files(tool):
    module, host, calls = tool
    managed = host.path(module.MANAGED)
    managed.parent.mkdir(parents=True)
    managed.write_text('pool ntp.ubuntu.com iburst maxsources 4 maxpoll 7\n')
    module.boot_step(host, install=True)
    assert 'Before=clearpath-robot.service' in host.path(module.BOOT_UNIT).read_text()
    assert host.path(module.BOOT_SCRIPT).exists()
    assert ('systemctl', 'enable', 'dynamo-boot-clock.service') in calls
    # Neither chrony's profile nor any running service is touched.
    assert managed.read_text() == 'pool ntp.ubuntu.com iburst maxsources 4 maxpoll 7\n'
    assert not any(call[:2] in (('systemctl', 'restart'), ('systemctl', 'start')) for call in calls)
    module.boot_step(host, install=False)
    assert not host.path(module.BOOT_UNIT).exists() and not host.path(module.BOOT_SCRIPT).exists()
    assert managed.exists()


def test_boot_step_removal_refuses_an_edited_unit(tool):
    module, host, calls = tool
    module.boot_step(host, install=True)
    host.path(module.BOOT_UNIT).write_text('edited\n')
    with pytest.raises(RuntimeError, match='differs'):
        module.boot_step(host, install=False)
    assert host.path(module.BOOT_UNIT).exists()


def test_adopting_tuned_upstreams_keeps_them_across_reapply(tool):
    module, host, calls = tool
    module.apply(host, 'intel', ['ntp.ubuntu.com'])
    managed = host.path(module.MANAGED)
    tuned = managed.read_text().replace('server ntp.ubuntu.com iburst', 'pool ntp.ubuntu.com iburst maxsources 4 maxpoll 7')
    managed.write_text(tuned)
    with pytest.raises(RuntimeError, match='changed externally'):
        module.apply(host, 'intel')
    module.adopt_upstreams(host)
    module.apply(host, 'intel')
    assert managed.read_text() == tuned


def test_adoption_refuses_edits_beyond_the_upstream_lines(tool):
    module, host, calls = tool
    module.apply(host, 'intel', ['ntp.ubuntu.com'])
    managed = host.path(module.MANAGED)
    managed.write_text(managed.read_text().replace('maxslewrate 500', 'maxslewrate 83333'))
    with pytest.raises(RuntimeError, match='beyond its upstream lines'):
        module.adopt_upstreams(host)
