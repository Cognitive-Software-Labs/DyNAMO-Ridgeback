from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
loader = SourceFileLoader('sensor_latency_run', str(ROOT / 'tools/intel_thor/sensor_latency_run'))
spec = importlib.util.spec_from_loader(loader.name, loader)
TOOL = importlib.util.module_from_spec(spec)
loader.exec_module(TOOL)


def test_drift_expands_clock_bound_and_invalidates_cross_host_timing():
    clock = TOOL.clock_window({'offset_ms': 1, 'bound_ms': .1}, {'offset_ms': 1.3, 'bound_ms': .1})
    assert clock['offset_ms'] == 1.15
    assert clock['bound_ms'] > .2
    assert clock['resolved'] is False


def test_reduced_rate_and_short_runs_cannot_pass_30hz_targets():
    report = {'fps': 15, 'duration_s': 10, 'clock': {'resolved': True}, 'passed': True,
              'aborted': None, 'timing': {key: {'components': {'network_dds': {'p95_ms': 15},
                                                            'total': {'p95_ms': 35}},
                                            'dds_loss': {'fraction': None}} for key in ('color', 'depth')},
              'pairing': {'color': 1., 'depth': 1.}, 'rate_loss_estimate': {'color': 0., 'depth': 0.}}
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False
    report.update(fps=30, duration_s=60)
    TOOL.qualify(report)
    assert report['working_targets_passed'] is True
    report['clock']['resolved'] = False
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False
