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
    report = {'expected': {'grid': [640, 480]}, 'fps': 15, 'duration_s': 10, 'clock': {'resolved': True}, 'passed': True,
              'aborted': None, 'timing': {key: {'components': {'network_dds': {'p95_ms': 15},
                                                            'total': {'p95_ms': 35}},
                                            'dds_loss': {'fraction': None}} for key in ('color', 'depth')},
              'delivery_loss_vs_intel': {key: {'fraction': 0.} for key in ('color', 'depth')},
              'pairing': {'color': 1., 'depth': 1.}, 'rate_loss_estimate': {'color': 0., 'depth': 0.}}
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False
    report.update(fps=30, duration_s=60)
    TOOL.qualify(report)
    assert report['working_targets_passed'] is True
    report['expected']['grid'] = [1280, 720]
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False
    report['expected']['grid'] = [640, 480]
    report['clock']['resolved'] = False
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False


def test_delivery_loss_excludes_observer_window_boundaries():
    reference = [{'stamp_ns': i} for i in range(10)]
    received = [{'stamp_ns': i} for i in (2, 3, 5, 6, 7, 8, 9, 10)]
    result = TOOL.delivery_loss(reference, received)
    assert result['reference_count'] == 8
    assert result['missing'] == 1
    assert result['fraction'] == 1/8
    assert result['stamp_window_ns'] == [2, 9]
    assert TOOL.delivery_loss(reference, [])['fraction'] is None


def test_unavailable_loss_is_not_a_qualification_pass():
    # An empty common window must remain unresolved, rather than imply zero loss.
    assert TOOL.delivery_loss([{'stamp_ns': 1}], [{'stamp_ns': 2}])['fraction'] is None
