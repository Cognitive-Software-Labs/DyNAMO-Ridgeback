"""Performance-policy installer on a fake sysfs, without real services or nvpmodel."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def tool(tmp_path, monkeypatch):
    loader = SourceFileLoader('host_performance', str(ROOT / 'tools/intel_thor/host_performance'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    calls, mode, units = [], {'id': 1}, {}

    def run(*args, check=True, input=None):
        calls.append(args)
        if args[:2] == ('nvpmodel', '-q'):
            name = 'MAXN' if mode['id'] == 0 else '120W'
            return SimpleNamespace(stdout=f'NV Power Mode: {name}\n{mode["id"]}\n', returncode=0)
        if args[:2] == ('nvpmodel', '-m'):
            mode['id'] = int(args[2])
        if args[:2] == ('systemctl', 'enable'):
            units[args[2]] = 'enabled'
        if args[:2] == ('systemctl', 'is-enabled'):
            return SimpleNamespace(stdout=units.get(args[2], 'not-found'), returncode=0)
        return SimpleNamespace(stdout='', returncode=0)

    monkeypatch.setattr(module, 'run', run)
    host = module.Host(tmp_path)

    def cpus(governor, epp=None, count=2):
        for index in range(count):
            policy = host.path(f'{module.CPUFREQ}/policy{index}')
            policy.mkdir(parents=True)
            (policy / 'scaling_governor').write_text(governor + '\n')
            if epp:
                (policy / 'energy_performance_preference').write_text(epp + '\n')

    return module, host, calls, mode, cpus


def performance(module, host):
    """Stand in for the boot unit's script, which systemctl would run."""
    for policy in host.path(module.CPUFREQ).glob('policy*'):
        (policy / 'scaling_governor').write_text('performance\n')
        if (policy / 'energy_performance_preference').exists():
            (policy / 'energy_performance_preference').write_text('performance\n')


def test_intel_reports_each_difference_then_passes_after_apply(tool, monkeypatch):
    module, host, calls, _, cpus = tool
    cpus('powersave', 'balance_performance')
    rows = {criterion: passed for passed, criterion, _ in module.checks(host, 'intel')}
    assert rows == {'CPU governor': False, 'energy preference': False, 'boot unit': False}
    monkeypatch.setattr(module, 'status', lambda host: performance(module, host) or 0)
    assert module.apply(host, 'intel') == 0
    assert host.path(module.SCRIPT).stat().st_mode & 0o111
    assert 'ExecStart=/usr/local/sbin/dynamo-host-performance' in host.path(f'{module.UNIT_DIR}/{module.UNIT}').read_text()
    assert all(passed for passed, _, _ in module.checks(host, 'intel'))
    assert not any(call[0] == 'nvpmodel' for call in calls)


def test_thor_enters_maxn_and_rollback_restores_mode_and_governors(tool, monkeypatch):
    module, host, calls, mode, cpus = tool
    host.path(module.NVPMODEL_CONF).parent.mkdir(parents=True)
    host.path(module.NVPMODEL_CONF).write_text('< PM_CONFIG DEFAULT=1 >\n')
    cpus('schedutil')
    assert module.detect(host) == 'thor'
    monkeypatch.setattr(module, 'status', lambda host: performance(module, host) or 0)
    module.apply(host, 'thor')
    assert mode['id'] == module.MAXN
    assert all(passed for passed, _, _ in module.checks(host, 'thor'))
    module.rollback(host)
    assert mode['id'] == 1
    assert {state['governor'] for state in module.cpu_state(host).values()} == {'schedutil'}
    assert not host.path(module.SCRIPT).exists() and not host.path(module.BACKUP).exists()


def test_apply_refuses_the_wrong_host_and_unknown_managed_files(tool):
    module, host, _, _, cpus = tool
    cpus('powersave')
    with pytest.raises(RuntimeError, match='not the thor host'):
        module.apply(host, 'thor')
    host.path(module.SCRIPT).parent.mkdir(parents=True)
    host.path(module.SCRIPT).write_text('#!/bin/sh\n')
    with pytest.raises(RuntimeError, match='already exists'):
        module.apply(host, 'intel')


def test_boot_script_sets_every_policy():
    script = (ROOT / 'src/ridgeback_autonomy_hardware/config/intel_thor/performance/dynamo-host-performance.sh').read_text()
    assert 'cpufreq/policy*' in script and 'echo performance > "$policy/scaling_governor" || exit 1' in script
