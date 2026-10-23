"""D3 silent-loss verdict on synthetic event timelines."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('d3', Path(__file__).with_name('observe_silent_controller_loss.py'))
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def timeline(held_until, drop_at, stage_until, stop_released=False):
    ev = [(t / 10, 'joy_held', True) for t in range(50, int(held_until * 10) + 1)]
    ev += [(t / 10, s, True) for s in tool.STAGES[:2] for t in range(51, int(stage_until * 10) + 1)]
    ev += [(held_until + .1, 'joy_held', False)]
    ev += [(stage_until + .1, s, False) for s in tool.STAGES]
    ev += [(drop_at, 'link', False), (0.2, 'link', True)]
    ev += [(t / 10, 'stop', True) for t in range(1, 601, 2)]
    if stop_released:
        ev.append((6.0, 'stop', False))
    return ev


def test_zeroed_before_the_drop_and_commands_follow_passes():
    assert tool.evaluate(timeline(20.0, 20.3, 21.0), 60)['passed']


def test_held_input_surviving_the_link_drop_fails():
    assert not tool.evaluate(timeline(25.0, 20.0, 25.5), 60)['passed']


def test_a_command_lingering_after_the_input_fails():
    assert not tool.evaluate(timeline(20.0, 20.3, 23.0), 60)['passed']


def test_no_link_drop_or_a_released_stop_fails():
    assert not tool.evaluate([e for e in timeline(20.0, 20.3, 21.0) if e[1] != 'link'], 60)['passed']
    assert not tool.evaluate(timeline(20.0, 20.3, 21.0, stop_released=True), 60)['passed']


def test_missing_fresh_zero_or_neutral_evidence_fails():
    events = timeline(20.0, 20.3, 21.0)
    assert not tool.evaluate([e for e in events if not (e[1] == tool.STAGES[0] and not e[2])], 60)['passed']
    assert not tool.evaluate([e for e in events if not (e[1] == 'joy_held' and not e[2])], 60)['passed']


def test_drop_before_held_input_fails():
    assert not tool.evaluate(timeline(20.0, 4.0, 21.0), 60)['passed']
    assert not tool.evaluate(timeline(20.0, 30.0, 21.0), 60)['passed']


def test_missing_source_or_nonzero_mux_fails_under_pressed_stop():
    events = timeline(20.0, 20.3, 21.0)
    assert not tool.evaluate([e for e in events if e[1] != tool.STAGES[0]], 60)['passed']
    assert not tool.evaluate(events + [(10.0, tool.STAGES[2], True)], 60)['passed']
    source_only_before_hold = [e for e in events if e[1] != tool.STAGES[0]]
    source_only_before_hold += [(1.0, tool.STAGES[0], True),
                                (21.1, tool.STAGES[0], False)]
    assert not tool.evaluate(source_only_before_hold, 60)['passed']


def test_stale_stop_or_missing_settled_tail_fails():
    events = timeline(20.0, 20.3, 21.0)
    stale = [event for event in events if event[1] != 'stop'] + [(0.1, 'stop', True)]
    assert not tool.evaluate(stale, 60)['passed']
    assert not tool.evaluate(events, 21.2)['passed']


def test_bluetooth_parser_never_turns_poll_failure_into_link_loss():
    class Completed:
        def __init__(self, returncode, stdout):
            self.returncode, self.stdout = returncode, stdout
    assert tool.parse_connected(Completed(0, 'Connected: yes\n')) is True
    assert tool.parse_connected(Completed(0, 'Connected: no\n')) is False
    assert tool.parse_connected(Completed(1, 'Connected: no\n')) is None
    assert tool.parse_connected(Completed(0, '')) is None
