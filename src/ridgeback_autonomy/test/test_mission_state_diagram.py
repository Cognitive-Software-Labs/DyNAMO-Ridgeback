"""Live RViz state-machine panel: a faithful, read-only view of the mission's own states."""

import html
import re


from ridgeback_autonomy.mission import mission_state_diagram as diagram
from ridgeback_autonomy.mission.mission_manager_node import (
    _ALLOWED_TRANSITIONS, MissionState,
)
from test_mission_manager import manager_factory, occupancy_map, planning_manager  # noqa: F401 (fixture)

ALLOWED = {s.name: frozenset(t.name for t in ts) for s, ts in _ALLOWED_TRANSITIONS.items()}
MAIN = ('WAITING_FOR_START_POSE', 'SEARCH_G1', 'STOPPING_EXPLORATION', 'SETTLING_MOTION',
        'LOCALIZE_G1', 'G1_LOCALIZED')
PARKING = ('PARKING_CANDIDATES_READY', 'EVALUATING_PARKING_PATHS', 'PARKING_PATH_FEASIBLE',
           'NAVIGATING_TO_PREPARKING', 'NAVIGATING_TO_PARKING', 'PARKED', 'RETURNING_TO_START')


def walk(states, reasons=None):
    reasons = reasons or {}
    return [(a, b, reasons.get(b, f'to {b}')) for a, b in zip(states, states[1:])]


def rows(text):
    """Plain rows: (mark, name, colour) per state, plus the plain text."""
    plain = html.unescape(re.sub(r'<[^>]+>', '', text)).replace('\xa0', ' ')
    out = {}
    for line in text.split('\n'):
        flat = html.unescape(re.sub(r'<[^>]+>', '', line)).replace('\xa0', ' ')
        m = re.match(r'\s*(\d+) (\S) (\S+)', flat)
        if m:
            colour = tuple(map(int, re.search(r'rgb\((\d+), (\d+), (\d+)\)', line).groups()))
            out[m.group(3)] = (m.group(2), colour, flat)
    return out, plain


def test_every_mission_state_is_one_node():
    assert sorted(diagram.ROWS) == sorted(s.name for s in MissionState)
    assert len(set(diagram.ROWS)) == len(diagram.ROWS)
    assert diagram.FAILURES == {'MOTION_SETTLEMENT_FAILED', 'PARKING_PLANNING_FAILED',
                                'PARKING_EXECUTION_FAILED', 'MISSION_FAILED', 'MISSION_CANCELED'}


def test_drawn_edges_are_exactly_the_allowed_transitions():
    for name in diagram.ROWS:
        down, others = diagram.row_edges(name, ALLOWED)
        index = diagram.ROWS.index(name)
        drawn = set(others) | ({diagram.ROWS[index + 1]} if down else set())
        # "any state with an exit -> cancel" stands for the cancellation edges.
        drawn |= {diagram.CANCELED} if ALLOWED[name] else set()
        assert drawn == ALLOWED[name], name
    # and the rendered numbers say the same
    text, _ = rows(diagram.diagram_text('SEARCH_G1', [], ALLOWED))
    assert text['G1_LOCALIZED'][2].split()[3:] == ['↓', '→9', '✗18']
    assert text['NAVIGATING_TO_PARKING'][2].split()[3:] == ['↓', '→15']
    assert text['EXPLORING_AROUND_G1'][2].split()[3:] == ['→3', 'optional']


def test_active_visited_and_pending_states_are_distinguished():
    history = walk(('WAITING_FOR_START_POSE', 'SEARCH_G1', 'STOPPING_EXPLORATION', 'SETTLING_MOTION'))
    text, plain = rows(diagram.diagram_text('SETTLING_MOTION', history, ALLOWED))
    assert text['SETTLING_MOTION'][:2] == ('▶', diagram.ACTIVE)
    for name in ('WAITING_FOR_START_POSE', 'SEARCH_G1', 'STOPPING_EXPLORATION'):
        assert text[name][:2] == ('✔', diagram.VISITED)
    for name in ('LOCALIZE_G1',) + PARKING + ('MISSION_COMPLETE',):
        assert text[name][:2] == ('·', diagram.PENDING)
    assert 'last: STOPPING_EXPLORATION → SETTLING_MOTION' in plain


def test_skipped_optional_g1_exploration_stays_unvisited():
    history = walk(MAIN + PARKING + ('MISSION_COMPLETE',))
    text, _ = rows(diagram.diagram_text('MISSION_COMPLETE', history, ALLOWED))
    assert text['EXPLORING_AROUND_G1'][0] == text['RETURNING_TO_OBSERVATION'][0] == '·'
    assert all(text[name][0] == '✔' for name in MAIN + PARKING)
    assert text['MISSION_COMPLETE'][:2] == ('▶', diagram.VISITED)   # success end in green


def test_repeated_states_of_the_g1_mapping_loop_are_shown_once():
    loop = MAIN + ('EXPLORING_AROUND_G1', 'STOPPING_EXPLORATION', 'RETURNING_TO_OBSERVATION',
                   'SETTLING_MOTION', 'LOCALIZE_G1', 'G1_LOCALIZED', 'PARKING_CANDIDATES_READY')
    text, plain = rows(diagram.diagram_text('PARKING_CANDIDATES_READY', walk(loop), ALLOWED))
    assert text['EXPLORING_AROUND_G1'][0] == text['RETURNING_TO_OBSERVATION'][0] == '✔'
    assert plain.count('LOCALIZE_G1') == 1 and text['PARKING_CANDIDATES_READY'][0] == '▶'


def test_lidar_veto_and_failure_are_shown_with_the_recorded_reason():
    reason = 'returned to start after: lidar_clearance: lidar_obstacle_on_route'
    history = walk(MAIN + PARKING[:4] + ('RETURNING_TO_START', 'MISSION_FAILED'), {
        'RETURNING_TO_START': 'lidar_clearance: lidar_obstacle_on_route; stopping before the return',
        'MISSION_FAILED': reason})
    text, plain = rows(diagram.diagram_text('MISSION_FAILED', history, ALLOWED))
    assert text['MISSION_FAILED'][:2] == ('▶', diagram.FAILED)
    assert text['NAVIGATING_TO_PARKING'][0] == '·'      # the final leg never started
    assert 'last: RETURNING_TO_START → MISSION_FAILED' in plain
    assert 'lidar_clearance: lidar_obstacle_on_route' in ' '.join(plain.split())   # wrapped


def test_long_reasons_are_wrapped_to_two_lines():
    text = diagram.diagram_text('SEARCH_G1', [('A', 'SEARCH_G1', 'word ' * 60)], ALLOWED)
    tail = [r for r in text.split('\n')][-2:]
    assert all(len(html.unescape(re.sub(r'<[^>]+>', '', r))) <= diagram.REASON_WIDTH + 2 for r in tail)
    assert tail[-1].count('…') == 1


def test_reset_starts_a_new_run_of_visited_states():
    history = walk(MAIN + PARKING + ('MISSION_COMPLETE',)) + [
        ('MISSION_COMPLETE', 'SEARCH_G1', diagram.RESET_REASON)]
    text, plain = rows(diagram.diagram_text('SEARCH_G1', history, ALLOWED))
    assert text['SEARCH_G1'][0] == '▶' and text['WAITING_FOR_START_POSE'][0] == '✔'
    assert all(text[name][0] == '·' for name in PARKING + ('MISSION_COMPLETE',))
    assert 'explicit manager reset' in plain


def test_overlay_is_a_bottom_left_monospace_panel_sized_for_its_rows():
    msg = diagram.state_diagram_overlay('SEARCH_G1', walk(MAIN[:2]), ALLOWED)
    assert msg.horizontal_alignment == msg.LEFT and msg.vertical_alignment == msg.BOTTOM
    assert 'Mono' in msg.font and msg.height >= diagram.LINE_PX * len(msg.text.split('\n'))
    assert re.findall(r'color: rgb\(\d+, \d+, \d+\);', msg.text)   # shadow-safe spans


# The mission node publishes the panel from its own recorded transitions.

def test_mission_publishes_the_panel_on_each_transition_and_resends_it(manager_factory):
    m, published = planning_manager(manager_factory, map_msg=occupancy_map())
    sent = []
    m.state_diagram_publisher.publish = sent.append
    before = len(m.transition_history)
    m._fail_planning('test_failure', 'planner reported a failure')
    assert len(m.transition_history) == before + 1
    text, plain = rows(sent[-1].text)
    assert text[m.state.name][0] == '▶'
    last = m.transition_history[-1]
    assert f'last: {last.previous.name} → {last.current.name}' in plain
    count = len(sent)
    m._publish_state('no transition', event='snapshot')
    assert len(sent) == count                     # redrawn only when the state changes
    m._monotonic.value += 1.1
    m.tick()
    assert len(sent) == count + 1 and sent[-1] is sent[-2]   # resent, unchanged


def test_mission_panel_shows_a_real_lidar_veto(manager_factory, monkeypatch):
    from test_parking_clearance import driving, feed, on_route
    m, handle, published = driving(manager_factory, monkeypatch)
    sent = []
    m.state_diagram_publisher.publish = sent.append
    px, py = on_route(m)
    feed(m, [(px[0], py[0])])
    text, plain = rows(sent[-1].text)
    assert m.state == MissionState.RETURNING_TO_START
    assert text['RETURNING_TO_START'][0] == '▶' and text['NAVIGATING_TO_PREPARKING'][0] == '✔'
    assert 'NAVIGATING_TO_PREPARKING → RETURNING_TO_START' in plain
    assert 'lidar_obstacle_on_route' in plain


def test_mission_reset_redraws_from_the_reset(manager_factory):
    m, _ = planning_manager(manager_factory, map_msg=occupancy_map())
    sent = []
    m.state_diagram_publisher.publish = sent.append
    m.reset_mission()
    text, plain = rows(sent[-1].text)
    assert text[m.state.name][0] == '▶' and 'explicit manager reset' in plain
    assert text['PARKING_CANDIDATES_READY'][0] == '·'
