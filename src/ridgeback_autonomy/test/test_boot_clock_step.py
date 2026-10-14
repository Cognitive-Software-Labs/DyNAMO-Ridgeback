"""Boot clock step decisions, on recorded `chronyc -c tracking` lines."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    'boot_clock_step', ROOT / 'src/ridgeback_autonomy_hardware/config/intel_thor/boot_clock_step.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

# Intel on 2026-10-14: a selected internet source, 0.12 ms offset.
SYNCED = ('B97DBE39,185.125.190.57,3,1791977003.621890186,-0.000121786,0.000070234,'
          '0.000089808,-5.598,-0.001,0.175,0.027657418,0.000885120,130.2,Normal')
BOOT = SYNCED.replace('-0.000121786', '0.350000000')
LOCAL = ('7F7F0101,,10,0.0,0.350000000,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Normal')
UNSYNCED = ('00000000,,0,0.0,0.350000000,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Not synchronised')


def test_parses_the_offset_and_source():
    sample = tool.parse_tracking(SYNCED)
    assert sample['name'] == '185.125.190.57' and sample['stratum'] == 3
    assert sample['offset_s'] == -0.000121786 and tool.external_source(sample)


def test_only_an_external_source_counts():
    assert not tool.external_source(tool.parse_tracking(LOCAL))
    assert not tool.external_source(tool.parse_tracking(UNSYNCED))


def test_a_boot_offset_is_stepped_before_consumers_start():
    assert tool.decide(tool.parse_tracking(BOOT), [], 0.005) == 'step'


def test_a_small_offset_is_left_to_chrony():
    assert tool.decide(tool.parse_tracking(SYNCED), [], 0.005) == 'none'


def test_never_steps_under_a_running_consumer():
    assert tool.decide(tool.parse_tracking(BOOT), ['dynamo-camera.service'], 0.005) == 'refuse'


def test_the_unit_orders_before_every_consumer():
    unit = (ROOT / 'src/ridgeback_autonomy_hardware/config/intel_thor/dynamo-boot-clock.service').read_text()
    before = next(line for line in unit.splitlines() if line.startswith('Before=')).split('=', 1)[1].split()
    assert set(before) == set(tool.CONSUMERS)
