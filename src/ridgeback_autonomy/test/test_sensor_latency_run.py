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
    report = {'streams': {key: {'unique': 1800} for key in ('color', 'depth')}, 'expected': {'grid': [640, 480]}, 'fps': 15, 'duration_s': 10, 'clock': {'resolved': True}, 'passed': True,
              'aborted': None, 'timing': {key: {'components': {'network_dds': {'p95_ms': 15, 'count': 1800},
                                                            'total': {'p95_ms': 35, 'count': 1800}},
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


def compressed_report(network_count):
    report = {'streams': {key: {'unique': 1800} for key in ('color', 'depth')}, 'expected': {'grid': [640, 480]},
              'fps': 30, 'duration_s': 60, 'clock': {'resolved': True}, 'passed': True, 'aborted': None,
              'transports': {'color': 'ffmpeg', 'depth': 'raw'},
              'timing': {key: {'components': {'network_dds': {'p95_ms': 15, 'count': 1800},
                                              'total': {'p95_ms': 35, 'count': 1800}},
                               'dds_loss': {'fraction': None}} for key in ('color', 'depth')},
              'delivery_loss_vs_intel': {key: {'fraction': 0.} for key in ('color', 'depth')},
              'pairing': {'color': 1., 'depth': 1.}}
    report['timing']['color']['components']['network_dds']['count'] = network_count
    return report


def test_compressed_coverage_tolerates_a_few_missed_wire_receipts_only():
    # The compressed message has its own Thor subscriber; the decoded total must be complete.
    report = compressed_report(1790)
    TOOL.qualify(report)
    assert report['working_targets_passed'] is True
    report = compressed_report(1700)
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False
    report = compressed_report(1790)
    report['transports']['color'] = 'raw'
    TOOL.qualify(report)
    assert report['working_targets_passed'] is False


def test_camera_parameter_values_are_typed():
    assert [TOOL.parse_value(text) for text in ('80', '0.5', 'true', 'False', 'rvl', 'preset:p1')] == [
        80, .5, True, False, 'rvl', 'preset:p1']


def test_unknown_parameter_assignment_is_rejected(monkeypatch):
    import pytest
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    with pytest.raises(SystemExit):
        TOOL.main(['--namespace', 'r100_0160', '--camera-param', 'jpeg_quality'])
