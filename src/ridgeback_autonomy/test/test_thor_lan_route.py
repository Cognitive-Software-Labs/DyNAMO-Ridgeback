"""Thor's route to Intel's Wi-Fi address, without NetworkManager or a real kernel."""
from importlib.machinery import SourceFileLoader
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
# Thor on 2026-10-21: two profiles for the robot port, one for another port.
BINDINGS = {'Profile 1': 'enP2p1s0', 'Profile 2': 'mgbe0_0', 'Profile 3': 'enP2p1s0'}


@pytest.fixture
def tool(tmp_path, monkeypatch):
    loader = SourceFileLoader('thor_lan_route', str(ROOT / 'tools/intel_thor/thor_lan_route'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    state = {'routes': {name: ['10.0.0.0/8 192.168.131.9'] for name in BINDINGS}, 'active': 'Profile 3',
             'kernel_via': None}

    def run(*args, check=True):
        if args[:5] == ('nmcli', '-t', '-f', 'NAME,TYPE', 'connection'):
            lines = [f'{name}:802-3-ethernet' for name in BINDINGS] + ['ASUS_20:802-11-wireless']
            return SimpleNamespace(stdout='\n'.join(lines) + '\n', returncode=0)
        if args[:3] == ('nmcli', '-g', 'connection.interface-name'):
            return SimpleNamespace(stdout=BINDINGS.get(args[5], 'wlP1p1s0') + '\n', returncode=0)
        if args[:3] == ('nmcli', '-g', 'ipv4.routes'):
            return SimpleNamespace(stdout=', '.join(state['routes'][args[5]]) + '\n', returncode=0)
        if args[:3] == ('nmcli', 'connection', 'modify'):
            name = args[3]
            if args[4] == '+ipv4.routes':
                state['routes'][name].append(args[5])
            else:
                state['routes'][name] = [r for r in args[5].split(', ') if r]
        if args[:3] == ('nmcli', 'device', 'reapply'):
            state['kernel_via'] = module.INTEL_ETHERNET if module.has_route(state['routes'][state['active']]) else None
        if args[:3] == ('ip', 'route', 'get'):
            via = state['kernel_via']
            text = f'{module.INTEL_WIFI} via {via} dev enP2p1s0' if via else f'{module.INTEL_WIFI} dev wlP1p1s0 src 192.168.123.152'
            return SimpleNamespace(stdout=text, returncode=0)
        return SimpleNamespace(stdout='', returncode=0)

    monkeypatch.setattr(module, 'run', run)
    monkeypatch.setattr(module, 'BACKUP', tmp_path / 'backup')
    return module, state


def test_every_profile_on_the_robot_port_gets_the_route_and_rollback_restores(tool):
    module, state = tool
    assert [passed for passed, _, _ in module.checks()] == [False, False, False]
    assert module.apply() == 0
    for name in ('Profile 1', 'Profile 3'):
        assert state['routes'][name] == ['10.0.0.0/8 192.168.131.9', module.ROUTE]
    assert state['routes']['Profile 2'] == ['10.0.0.0/8 192.168.131.9']  # another port: untouched
    assert all(passed for passed, _, _ in module.checks())
    module.apply()  # idempotent: the route is not added twice
    assert state['routes']['Profile 1'].count(module.ROUTE) == 1
    module.rollback()
    assert state['routes']['Profile 1'] == state['routes']['Profile 3'] == ['10.0.0.0/8 192.168.131.9']
    assert not module.BACKUP.exists()


def test_a_profile_switch_no_longer_loses_the_route(tool):
    module, state = tool
    module.apply()
    state['active'] = 'Profile 1'
    module.run('nmcli', 'device', 'reapply', module.DEVICE)
    assert all(passed for passed, _, _ in module.checks())


def test_reads_the_former_single_connection_snapshot(tool):
    module, state = tool
    module.BACKUP.mkdir()
    (module.BACKUP / 'state.json').write_text(json.dumps({'connection': 'Profile 1', 'routes': ['original']}))
    module.apply()
    saved = json.loads((module.BACKUP / 'state.json').read_text())['connections']
    assert saved['Profile 1'] == ['original'] and saved['Profile 3'] == ['10.0.0.0/8 192.168.131.9']
