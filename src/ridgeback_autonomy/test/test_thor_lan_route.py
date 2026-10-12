"""Thor's route to Intel's Wi-Fi address, without NetworkManager or a real kernel."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def tool(tmp_path, monkeypatch):
    loader = SourceFileLoader('thor_lan_route', str(ROOT / 'tools/intel_thor/thor_lan_route'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    state = {'routes': ['10.0.0.0/8 192.168.131.9'], 'kernel_via': None}

    def run(*args, check=True):
        if args[:3] == ('nmcli', '-t', '-f'):
            return SimpleNamespace(stdout='wifi:wlP1p1s0\nProfile 1:enP2p1s0\n', returncode=0)
        if args[:2] == ('nmcli', '-g'):
            return SimpleNamespace(stdout=', '.join(state['routes']) + '\n', returncode=0)
        if args[:4] == ('nmcli', 'connection', 'modify', 'Profile 1'):
            if args[4] == '+ipv4.routes':
                state['routes'].append(args[5])
            else:
                state['routes'] = [r for r in args[5].split(', ') if r]
        if args[:3] == ('nmcli', 'device', 'reapply'):
            state['kernel_via'] = module.INTEL_ETHERNET if module.has_route(state['routes']) else None
        if args[:3] == ('ip', 'route', 'get'):
            via = state['kernel_via']
            text = f'{module.INTEL_WIFI} via {via} dev enP2p1s0' if via else f'{module.INTEL_WIFI} dev wlP1p1s0 src 192.168.123.152'
            return SimpleNamespace(stdout=text, returncode=0)
        return SimpleNamespace(stdout='', returncode=0)

    monkeypatch.setattr(module, 'run', run)
    monkeypatch.setattr(module, 'BACKUP', tmp_path / 'backup')
    return module, state


def test_wifi_path_is_reported_then_apply_routes_over_ethernet_and_rollback_restores(tool):
    module, state = tool
    assert [passed for passed, _, _ in module.checks()] == [False, False]
    assert module.apply() == 0
    assert state['routes'] == ['10.0.0.0/8 192.168.131.9', module.ROUTE]
    assert all(passed for passed, _, _ in module.checks())
    module.apply()  # idempotent: the route is not added twice
    assert state['routes'].count(module.ROUTE) == 1
    module.rollback()
    assert state['routes'] == ['10.0.0.0/8 192.168.131.9']
    assert not module.BACKUP.exists()
