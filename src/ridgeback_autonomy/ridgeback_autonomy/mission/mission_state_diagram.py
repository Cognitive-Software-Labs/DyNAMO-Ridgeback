"""Live mission state machine as an RViz screen overlay: a read-only view of the mission.

Every MissionState is a numbered node, in mission order. Edges come from the
mission's own allowed-transition table: "↓" is a transition to the next row,
"→n" / "✗n" any other target (✗ for a failure end). Cancellation is allowed from
every state that has an exit, so it is noted once instead of on each row.
Colours follow the recorded transition history since the last reset. The panel
only renders what the mission already recorded; it never drives a transition.

The overlay renders Qt rich text without backgrounds or tables, so nodes are
coloured rows in a monospace font rather than boxes.
"""

from __future__ import annotations

import html

from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA


# Mission order: the main path, with the optional G1-centred mapping where it
# occurs, then the failure ends.
ROWS = (
    'WAITING_FOR_START_POSE', 'SEARCH_G1', 'STOPPING_EXPLORATION', 'SETTLING_MOTION',
    'LOCALIZE_G1', 'G1_LOCALIZED', 'EXPLORING_AROUND_G1', 'RETURNING_TO_OBSERVATION',
    'PARKING_CANDIDATES_READY', 'EVALUATING_PARKING_PATHS', 'PARKING_PATH_FEASIBLE',
    'NAVIGATING_TO_PREPARKING', 'NAVIGATING_TO_PARKING', 'PARKED', 'RETURNING_TO_START',
    'MISSION_COMPLETE',
    'MOTION_SETTLEMENT_FAILED', 'PARKING_PLANNING_FAILED', 'PARKING_EXECUTION_FAILED',
    'MISSION_FAILED', 'MISSION_CANCELED',
)
FAILURES = frozenset(ROWS[16:])
SUCCESS = 'MISSION_COMPLETE'
CANCELED = 'MISSION_CANCELED'
NOTES = {'EXPLORING_AROUND_G1': 'optional', 'RETURNING_TO_OBSERVATION': 'optional',
         'PARKED': '5 s wait'}
RESET_REASON = 'explicit manager reset'

VISITED = (90, 200, 120)
ACTIVE = (80, 170, 255)
PENDING = (125, 125, 125)
FAILED = (255, 85, 85)
TEXT = (235, 235, 235)
DOWN, TO, FAIL = '↓', '→', '✗'
NAME_WIDTH = 26
REASON_WIDTH = 46
LINE_PX = 14   # one 9 pt monospace row


def row_edges(name, allowed):
    """(transition to the next row?, other targets) of one state, cancellation left out."""
    targets = allowed[name] - {CANCELED}
    index = ROWS.index(name)
    following = ROWS[index + 1] if index + 1 < len(ROWS) else None
    down = following in targets
    return down, sorted(targets - {following}, key=ROWS.index)


def visited_states(history):
    """States reached since the last reset (``history``: (previous, current, reason) names)."""
    visited = {'WAITING_FOR_START_POSE'}
    for previous, current, reason in history:
        if reason == RESET_REASON:
            # A reset keeping the saved start pose resumes the search with it.
            visited = {current} | ({'WAITING_FOR_START_POSE'} if current == 'SEARCH_G1' else set())
        else:
            visited |= {previous, current}
    return visited


def _span(text, rgb):
    # The overlay strips "color: ...;" to draw its shadow, so keep the semicolon.
    return f'<span style="color: rgb{rgb};">{text}</span>'


def _cell(text, width=None):
    text = text if width is None else text.ljust(width)
    return html.escape(text, quote=False).replace(' ', '&nbsp;')


def _wrap(text, width, lines):
    words, out = text.split(), ['']
    for word in words:
        if len(out[-1]) + len(word) + (1 if out[-1] else 0) > width:
            if len(out) == lines:
                out[-1] = out[-1][:width - 1] + '…'
                return out
            out.append('')
        out[-1] = (out[-1] + ' ' + word).strip()
    return out


def diagram_text(current, history, allowed):
    """The panel's rich text: header, one row per state, then the last recorded transition."""
    visited = visited_states(history)
    rows = [_span(_cell('MISSION STATE MACHINE  '), TEXT)
            + _span('✔done ', VISITED) + _span('▶now ', ACTIVE)
            + _span('·next ', PENDING) + _span('✗fail', FAILED)]
    for number, name in enumerate(ROWS, start=1):
        if name == current:
            mark, rgb = '▶', (FAILED if name in FAILURES else
                                   VISITED if name == SUCCESS else ACTIVE)
        elif name in visited:
            mark, rgb = ('✗', FAILED) if name in FAILURES else ('✔', VISITED)
        else:
            mark, rgb = '·', PENDING
        down, others = row_edges(name, allowed)
        edges = (DOWN if down else ' ') + ''.join(
            f' {FAIL if t in FAILURES else TO}{ROWS.index(t) + 1}' for t in others)
        note = f' {NOTES[name]}' if name in NOTES else ''
        rows.append(_span(_cell(f'{number:>2} {mark} ') + _cell(name, NAME_WIDTH), rgb)
                    + _span(_cell(f' {edges}{note}'), TEXT if name == current else PENDING))
    rows.append(_span(_cell(f'any state with an exit → ✗{ROWS.index(CANCELED) + 1}'
                            ' on cancel'), PENDING))
    if history:
        previous, latest, reason = history[-1]
        rows.append(_span(_cell('last: '), TEXT) + _span(_cell(previous), PENDING)
                    + _span(_cell(' → '), TEXT)
                    + _span(_cell(latest), FAILED if latest in FAILURES else ACTIVE))
        rows += [_span(_cell(f'  {line}'), TEXT) for line in _wrap(reason, REASON_WIDTH, 2)]
    return '\n'.join(rows)


def _color(rgb, alpha=1.0):
    return ColorRGBA(r=rgb[0] / 255, g=rgb[1] / 255, b=rgb[2] / 255, a=float(alpha))


def state_diagram_overlay(current, history, allowed) -> OverlayText:
    """The panel as a fixed bottom-left RViz overlay (rviz_2d_overlay_plugins)."""
    msg = OverlayText()
    msg.action = OverlayText.ADD
    msg.text = diagram_text(current, history, allowed)
    # Wide enough for the longest 'last:' line (two 24-character state names).
    msg.width, msg.height = 420, LINE_PX * len(msg.text.split('\n')) + 8
    msg.horizontal_alignment, msg.vertical_alignment = OverlayText.LEFT, OverlayText.BOTTOM
    msg.horizontal_distance = msg.vertical_distance = 10
    msg.bg_color = _color((13, 13, 13), 0.8)
    msg.fg_color = _color(TEXT)
    msg.line_width, msg.text_size, msg.font = 1, 9.0, 'DejaVu Sans Mono'
    return msg
