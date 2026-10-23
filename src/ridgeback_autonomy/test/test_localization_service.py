"""Thor localization boot-service installer, on a temporary root without systemd."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def tool(tmp_path, monkeypatch):
    loader = SourceFileLoader('localization_service', str(ROOT / 'tools/intel_thor/localization_service'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    calls = []
    monkeypatch.setattr(module, 'run', lambda *a, check=True: calls.append(a) or SimpleNamespace(stdout='', returncode=0))
    monkeypatch.setattr(module, 'manual_launches', lambda: [])
    monkeypatch.setattr(module.pwd, 'getpwnam', lambda user: SimpleNamespace(pw_dir='/home/' + user))
    checkout = tmp_path / 'checkout'
    (checkout / 'artifacts/colcon/thor-install').mkdir(parents=True)
    (checkout / 'artifacts/colcon/thor-install/local_setup.bash').write_text('')
    return module, module.Host(tmp_path / 'root'), calls, checkout


def test_apply_renders_the_unit_and_settings_and_starts_it(tool):
    module, host, calls, checkout = tool
    module.apply(host, checkout, 'robot')
    unit = host.path(module.UNIT_PATH).read_text()
    assert 'User=robot' in unit and 'KillSignal=SIGTERM' in unit and 'After=network-online.target dynamo-ptp4l' in unit
    defaults = host.path(module.DEFAULTS_PATH).read_text()
    assert f'CHECKOUT={checkout.resolve()}' in defaults and 'MASK_GATE=box' in defaults
    assert 'RIDGEBACK_PERCEPTION_VENV=/home/robot/venvs/dynamo-perception' in defaults
    assert ('systemctl', 'enable', '--now', module.UNIT) in calls
    assert module.status(host, 'robot') == 0


def test_reapply_keeps_operator_edits_to_the_settings(tool):
    module, host, calls, checkout = tool
    module.apply(host, checkout, 'robot')
    host.path(module.DEFAULTS_PATH).write_text('MASK_GATE=silhouette\n')
    module.apply(host, checkout, 'robot')
    assert host.path(module.DEFAULTS_PATH).read_text() == 'MASK_GATE=silhouette\n'


def test_refuses_next_to_a_manual_launch(tool, monkeypatch):
    module, host, calls, checkout = tool
    monkeypatch.setattr(module, 'manual_launches', lambda: ['4242'])
    with pytest.raises(RuntimeError, match='4242'):
        module.apply(host, checkout, 'robot')
    assert not host.path(module.UNIT_PATH).exists() and calls == []


def test_refuses_a_checkout_without_a_thor_build(tool, tmp_path):
    module, host, calls, _ = tool
    with pytest.raises(RuntimeError, match='no Thor build'):
        module.apply(host, tmp_path / 'empty', 'robot')


def test_remove_keeps_settings_unless_purged(tool):
    module, host, calls, checkout = tool
    module.apply(host, checkout, 'robot')
    module.remove(host, purge=False)
    assert not host.path(module.UNIT_PATH).exists() and host.path(module.DEFAULTS_PATH).exists()
    module.remove(host, purge=True)
    assert not host.path(module.DEFAULTS_PATH).exists()
