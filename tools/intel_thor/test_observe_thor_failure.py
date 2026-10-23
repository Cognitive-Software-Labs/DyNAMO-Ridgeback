"""Outage and rate summary of observe_thor_failure, on synthetic per-second rows."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('observe_thor_failure', Path(__file__).with_name('observe_thor_failure.py'))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def row(t, meas, health='processing', cam=30, scan=40):
    return {'t': t, 'health': health, 'detections': meas, 'measurements': meas,
            'colour': cam, 'depth': cam, 'front': scan, 'rear': scan, 'merged': scan}


def test_finds_an_outage_and_its_recovery():
    rows = [row(t, 3) for t in range(5)] + [row(t, 0, None) for t in range(5, 9)] + [row(t, 3) for t in range(9, 12)]
    summary = tool.summarize(rows)
    assert summary['measurement_outages_s'] == [[5, 9]]
    assert summary['min_per_second']['colour'] == 30


def test_an_unrecovered_outage_is_open_ended():
    rows = [row(t, 3) for t in range(3)] + [row(t, 0) for t in range(3, 6)]
    assert tool.summarize(rows)['measurement_outages_s'] == [[3, None]]


def test_reports_a_local_sensor_dip():
    rows = [row(t, 3) for t in range(6)]
    rows[4] = row(4, 3, scan=20)
    assert tool.summarize(rows)['min_per_second']['front'] == 20
