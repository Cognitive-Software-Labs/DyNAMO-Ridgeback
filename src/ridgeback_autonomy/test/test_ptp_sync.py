"""PTP installer transactions without package installation or real services."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def tool(tmp_path, monkeypatch):
    loader = SourceFileLoader('ptp_sync', str(ROOT / 'tools/intel_thor/ptp_sync'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    calls = []

    def run(*args, check=True):
        calls.append(args)
        return SimpleNamespace(stdout='', returncode=0)

    monkeypatch.setattr(module, 'run', run)
    monkeypatch.setattr(module, 'installed', lambda package: False)
    monkeypatch.setattr(module, 'state', lambda unit: {'enabled': 'enabled', 'active': 'active'})
    settle = module.settle
    monkeypatch.setattr(module, 'settle', lambda: settle(seconds=0))
    host = module.Host(tmp_path)
    for interface in ('eno1', 'enP2p1s0'):
        host.path(f'/sys/class/net/{interface}').mkdir(parents=True)
    chrony = host.path(module.CHRONY_MANAGED)
    chrony.parent.mkdir(parents=True)
    chrony.write_text('server ntp.ubuntu.com iburst\nmaxslewrate 500\n')
    return module, host, calls


def text(module, host, name):
    return host.path(f'{module.UNIT_DIR}/{name}').read_text()


def test_intel_serves_its_wall_clock_and_keeps_chrony(tool):
    module, host, calls = tool
    module.apply(host, 'intel')

    config = host.path(module.PTP4L_CONF).read_text()
    assert 'serverOnly              1' in config and 'network_transport       L2' in config
    assert 'time_stamping           hardware' in config
    assert '-i eno1' in text(module, host, module.PTP4L_UNIT)
    phc2sys = text(module, host, module.PHC2SYS_UNIT)
    assert '/usr/sbin/phc2sys -s CLOCK_REALTIME -c eno1 -O 0' in phc2sys
    assert 'Conflicts=' not in phc2sys
    assert not any('chrony.service' in call for call in calls)
    for name in (module.PTP4L_UNIT, module.PHC2SYS_UNIT):
        assert '@' not in text(module, host, name)


def test_thor_hands_the_wall_clock_from_chrony_and_back(tool):
    module, host, calls = tool
    module.apply(host, 'thor')

    assert 'clientOnly              1' in host.path(module.PTP4L_CONF).read_text()
    phc2sys = text(module, host, module.PHC2SYS_UNIT)
    assert '/usr/sbin/phc2sys -s enP2p1s0 -c CLOCK_REALTIME -O 0' in phc2sys and ' -w ' in phc2sys
    assert 'Conflicts=chrony.service' in phc2sys
    disable = calls.index(('systemctl', 'disable', '--now', 'chrony.service'))
    assert disable < calls.index(('systemctl', 'restart', module.PHC2SYS_UNIT))

    calls.clear()
    module.rollback(host)
    assert not host.path(module.PTP4L_CONF).exists()
    assert not host.path(f'{module.UNIT_DIR}/{module.PHC2SYS_UNIT}').exists()
    stop = calls.index(('systemctl', 'disable', '--now', module.PHC2SYS_UNIT))
    assert stop < calls.index(('systemctl', 'restart', 'chrony.service'))
    assert ('systemctl', 'enable', 'chrony.service') in calls
    assert ('apt-get', 'remove', '-y', 'linuxptp') in calls
    assert not host.path(module.BACKUP).exists()


def test_intel_requires_the_slow_slew_rate_first(tool):
    module, host, calls = tool
    host.path(module.CHRONY_MANAGED).write_text('server ntp.ubuntu.com iburst\n')
    with pytest.raises(RuntimeError, match='slow slew rate'):
        module.apply(host, 'intel')
    assert calls == [] and not host.path(module.BACKUP).exists()


def test_apply_refuses_on_the_wrong_host(tool):
    module, host, calls = tool
    host.path('/sys/class/net/enP2p1s0').rmdir()
    with pytest.raises(RuntimeError, match='not the thor host'):
        module.apply(host, 'thor')
    assert calls == []


def test_reapply_keeps_the_snapshot_and_external_edits_block_changes(tool):
    module, host, calls = tool
    module.apply(host, 'thor')
    first = host.path(module.BACKUP + '/state.json').read_bytes()
    module.apply(host, 'thor')
    assert host.path(module.BACKUP + '/state.json').read_bytes() == first

    host.path(module.PTP4L_CONF).write_text('external edit\n')
    calls.clear()
    with pytest.raises(RuntimeError, match='changed externally'):
        module.apply(host, 'thor')
    with pytest.raises(RuntimeError, match='changed externally'):
        module.rollback(host)
    assert calls == []
    assert host.path(module.BACKUP).exists()


def test_unmanaged_existing_file_is_not_overwritten(tool):
    module, host, calls = tool
    existing = host.path(module.PTP4L_CONF)
    existing.parent.mkdir(parents=True)
    existing.write_text('someone else\n')
    with pytest.raises(RuntimeError, match='already exists'):
        module.apply(host, 'intel')
    assert existing.read_text() == 'someone else\n' and calls == []


def test_apply_fails_loudly_when_a_daemon_exits(tool, monkeypatch):
    module, host, calls = tool
    monkeypatch.setattr(module, 'state', lambda unit: {
        'enabled': 'enabled', 'active': 'failed' if unit == module.PHC2SYS_UNIT else 'active'})
    with pytest.raises(RuntimeError, match='dynamo-phc2sys.service did not stay active'):
        module.apply(host, 'intel')
    # The snapshot stays for rollback.
    assert host.path(module.BACKUP + '/complete').exists()
