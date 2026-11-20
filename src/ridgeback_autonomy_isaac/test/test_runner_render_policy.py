"""The repeatability recipe must not change realtime renderer settings."""

import sys
from types import SimpleNamespace

import pytest

import isaac_runner


def launch_config(monkeypatch, mode, renderer, headless, aa=0):
    captured = {}

    def simulation_app(config):
        captured.update(config)
        return SimpleNamespace(close=lambda: None)

    monkeypatch.setitem(sys.modules, 'isaacsim',
                        SimpleNamespace(SimulationApp=simulation_app))
    monkeypatch.setattr(sys, 'argv', [
        'isaac_runner', '--sim-mode', mode, '--renderer', renderer,
        '--headless', headless, '--anti-aliasing', str(aa)])
    monkeypatch.setattr(isaac_runner, 'run', lambda app, args: 0)
    with pytest.raises(SystemExit) as result:
        isaac_runner.main()
    assert result.value.code == 0
    return captured


@pytest.mark.parametrize('headless', ['true', 'false'])
@pytest.mark.parametrize('mode', ['realtime', 'deterministic'])
@pytest.mark.parametrize('renderer', [
    'PathTracing', 'RealTimePathTracing', 'RaytracedLighting'])
def test_cache_policy_and_gui_controls(monkeypatch, mode, renderer, headless):
    config = launch_config(monkeypatch, mode, renderer, headless)
    flags = config.get('extra_args', [])
    repeatable_pt = mode == 'deterministic' and renderer == 'PathTracing'
    assert ('--/rtx/pathtracing/cached/enabled=false' in flags) == repeatable_pt
    assert ('--/rtx-transient/post/aa/limitedOps=false' in flags) == repeatable_pt
    assert not any('lightcache' in flag for flag in flags)
    assert ('--/app/vsync=false' in flags) == (headless == 'false')
    assert ('--/app/runLoops/present/rateLimitEnabled=false' in flags) == (
        headless == 'false')
    assert config['renderer'] == renderer
    assert config['anti_aliasing'] == 0
    assert config.get('denoiser') == (False if renderer == 'PathTracing' else None)


def test_explicit_dlss_request_is_preserved(monkeypatch):
    config = launch_config(monkeypatch, 'deterministic', 'PathTracing', 'false', aa=3)
    assert config['anti_aliasing'] == 3
    assert '--/rtx-transient/post/aa/limitedOps=false' not in config['extra_args']
    assert '--/rtx/pathtracing/cached/enabled=false' in config['extra_args']
