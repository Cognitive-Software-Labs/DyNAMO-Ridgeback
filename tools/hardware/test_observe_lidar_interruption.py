"""Recovery criteria of observe_lidar_interruption, on synthetic scan arrivals."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'observe_lidar_interruption', Path(__file__).with_name('observe_lidar_interruption.py'))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

FRONT, REAR, MERGED = tool.LIDARS['front'][0], tool.LIDARS['rear'][0], tool.MERGED
END = 80.0
PIDS = {'front driver': '1', 'rear driver': '2', 'scan merger': '3'}


def stream(hz, start=0.0, end=END, holes=()):
    rows, t = [], start
    while t <= end:
        if not any(a <= t < b for a, b in holes):
            rows.append((t, 1000.0 + t))
        t += 1.0 / hz
    return rows


def result(samples, before=PIDS, after=PIDS):
    rows = tool.evaluate(samples, FRONT, REAR, 20.0, END, before, after)
    return {name: ok for name, ok, _ in rows}


def recovered(outage):
    return {FRONT: stream(40, holes=[(25.0, 25.0 + outage)]), REAR: stream(40),
            MERGED: stream(40, holes=[(25.0, 25.0 + outage)])}


def test_a_short_outage_with_full_recovery_passes():
    assert all(result(recovered(2.25)).values())


def test_no_outage_means_the_interruption_was_not_observed():
    names = result(recovered(0.0))
    assert not names['interruption observed']


def test_an_outage_past_the_resume_limit_fails():
    names = result(recovered(tool.RESUME_LIMIT_S + 1))
    assert names['interruption observed'] and not names[f'{FRONT} resumes']


def test_a_driver_restart_fails_even_when_scans_return():
    names = result(recovered(2.25), after=dict(PIDS, **{'front driver': '9'}))
    assert not names['front driver not restarted']


def test_the_other_lidar_gapping_fails():
    samples = recovered(2.25)
    samples[REAR] = stream(40, holes=[(30.0, 30.5)])
    assert not result(samples)[f'{REAR} unaffected']


def test_a_merged_scan_that_never_recovers_fails():
    samples = recovered(2.25)
    samples[MERGED] = stream(40, end=25.0)
    assert not result(samples)[f'{MERGED} settled']


def test_a_late_reset_is_judged_on_the_scans_after_it():
    # The 2026-10-14 case: reset 30 s after the prompt, 27 s recorded after it.
    samples = {FRONT: stream(40, holes=[(50.6, 52.85)], end=100.0), REAR: stream(40, end=100.0),
               MERGED: stream(40, holes=[(50.6, 52.85)], end=100.0)}
    rows = {n: ok for n, ok, _ in tool.evaluate(samples, FRONT, REAR, 20.0, 100.0, PIDS, PIDS)}
    assert all(rows.values())
    short = {n: ok for n, ok, _ in tool.evaluate(samples, FRONT, REAR, 20.0, 80.0, PIDS, PIDS)}
    assert not short['settle window recorded']


def test_recording_stops_only_after_recovery_has_settled():
    samples = recovered(2.25)
    assert not tool.settled(samples, FRONT, 20.0, 40.0)
    assert tool.settled(samples, FRONT, 20.0, 25.0 + 2.25 + tool.SETTLE_S + 0.6)
    assert not tool.settled({FRONT: stream(40)}, FRONT, 20.0, 79.0)
