from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[3]
loader = SourceFileLoader('latency_check_clock', str(ROOT / 'tools/intel_thor/check_clock'))
spec = importlib.util.spec_from_loader(loader.name, loader)
TOOL = importlib.util.module_from_spec(spec)
loader.exec_module(TOOL)


def probe():
    class Socket:
        def send(self, value):
            self.sequence = value
        def recv(self, size):
            return self.sequence + b' 1000150000 1500000 1500100'
    result = object.__new__(TOOL.ClockProbe)
    result.socket, result.sequence, result.previous = Socket(), 0, {}
    return result


def test_clock_read_scheduling_delay_does_not_imply_a_wall_clock_step(monkeypatch):
    reads = iter([(1000000000, 1000000, 1200000),
                  (1000300000, 1300000, 1300100)])
    monkeypatch.setattr(TOOL, 'clock_read', lambda: next(reads))
    result = probe().exchange()
    assert result['bound_ms'] == .15
    assert result['offset_ms'] == 0
    assert result['intel_phase_min_ns'] <= result['intel_phase_max_ns']


def test_a_step_outside_read_uncertainty_aborts_the_probe(monkeypatch):
    reads = iter([(1000000000, 1000000, 1000100),
                  (1001000000, 1300000, 1300100)])
    monkeypatch.setattr(TOOL, 'clock_read', lambda: next(reads))
    with pytest.raises(RuntimeError, match='wall clock stepped'):
        probe().exchange()


def test_steps_between_exchanges_abort_even_with_short_round_trip(monkeypatch):
    reads = iter([(1000000000, 1000000, 1000100),
                  (1000300000, 1300000, 1300100)])
    monkeypatch.setattr(TOOL, 'clock_read', lambda: next(reads))
    peer = probe()
    peer.previous = {'thor': (1000000000, 1000000100)}
    with pytest.raises(RuntimeError, match='thor wall clock stepped'):
        peer.exchange()


def test_overlapping_phase_intervals_include_scheduling_uncertainty():
    assert TOOL.phase_gap((0, 300000), (200000, 201000)) == 0
    assert TOOL.phase_gap((0, 100), (200001, 200101)) > 100000


def test_retry_selection_depends_only_on_clock_uncertainty(monkeypatch):
    peer = probe()
    peer.peer, peer.transport = 'test', 'udp'
    samples = iter([{'bound_ms': .2, 'offset_ms': 1},
                    {'bound_ms': .19, 'offset_ms': -2},
                    {'bound_ms': .05, 'offset_ms': 3}])
    peer.exchange = lambda: next(samples)
    monkeypatch.setattr(TOOL.time, 'monotonic', lambda: 0.)
    result = peer.sample(count=2)
    assert len(result['samples']) == 3
    assert result['bound_ms'] == .05
    assert result['offset_ms'] == 3
