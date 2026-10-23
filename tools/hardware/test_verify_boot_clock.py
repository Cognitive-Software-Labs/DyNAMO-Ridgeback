"""Boot clock verification criteria on synthetic systemd/chrony readings."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('verify_boot_clock', Path(__file__).with_name('verify_boot_clock.py'))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

UNIT = {'Result': 'success', 'ActiveEnterTimestampMonotonic': '17000000'}
AFTER = {name: {'InactiveExitTimestampMonotonic': '17100000'} for name in tool.CONSUMERS}
TRACK = {'reference': 'B97DBE39', 'name': '185.125.190.57', 'offset_s': 0.0004}


def names(**overrides):
    args = dict(before_boot='a', boot='b', unit=UNIT, consumers=AFTER,
                decision='Source x, offset +350.000 ms: step', tracking=TRACK, platform_active=True)
    args.update(overrides)
    return {n: ok for n, ok, _ in tool.evaluate(**args)}


def test_a_clean_stepped_boot_passes():
    assert all(names().values())


def test_same_boot_fails():
    assert not names(boot='a')['boot changed']


def test_a_consumer_started_before_the_step_fails():
    early = dict(AFTER, **{'dynamo-camera.service': {'InactiveExitTimestampMonotonic': '13800000'}})
    assert not names(consumers=early)['dynamo-camera.service after the step']


def test_local_reference_or_large_offset_fails():
    assert not names(tracking=dict(TRACK, reference='7F7F0101'))['chrony external source within 5 ms']
    assert not names(tracking=dict(TRACK, offset_s=0.02))['chrony external source within 5 ms']
