"""RViz layouts derived from the shared exploration layout.

Hardware uses the simulator's displays unchanged but without the goal tool:
on the robot, Nav2's output reaches the twist mux, so one click of
*2D Goal Pose* would command motion even with frontier goals disabled.
It also redraws at HARDWARE_FRAME_RATE instead of 30 fps. On the robot, RViz
renders in software on the VNC desktop, and at 30 fps it took a full core and
broke the camera stream (467-567 ms depth gaps). At 10 fps, together with the
launch's priority and thread limits, it uses about a quarter of a core and the
camera contract passes. Deriving the layout at launch keeps the displays from
drifting between the two files.
"""
from __future__ import annotations

import re
import tempfile

import yaml

GOAL_TOOL_CLASS = 'rviz_default_plugins/SetGoal'
HARDWARE_FRAME_RATE = 10


def without_tool(layout_text: str, tool_class: str) -> str:
    """The layout with every tool of ``tool_class`` removed, text otherwise kept.

    Tools are list items under ``Visualization Manager: Tools:``. The item's
    ``- Class:`` line and the more-indented lines that follow it (its topic
    and options) are dropped; nothing else changes, so the result stays
    diffable against the source.
    """

    lines = layout_text.splitlines(keepends=True)
    kept, skip_indent = [], None
    for line in lines:
        stripped = line.lstrip(' ')
        indent = len(line) - len(stripped)
        if skip_indent is not None:
            if stripped.strip() and indent > skip_indent:
                continue
            skip_indent = None
        if stripped.rstrip() == f'- Class: {tool_class}':
            skip_indent = indent
            continue
        kept.append(line)
    result = ''.join(kept)
    tools = yaml.safe_load(result)['Visualization Manager']['Tools']
    if any(tool.get('Class') == tool_class for tool in tools):
        raise ValueError(f'{tool_class} is still present after removal')
    return result


def with_frame_rate(layout_text: str, frame_rate: int) -> str:
    """The layout with its global ``Frame Rate`` set, text otherwise kept."""

    result, count = re.subn(r'^(\s*Frame Rate:\s*)\d+\s*$', rf'\g<1>{frame_rate}',
                            layout_text, count=1, flags=re.M)
    if count != 1:
        raise ValueError('layout has no global Frame Rate')
    return result


def hardware_layout(source_path: str) -> str:
    """Write the hardware copy of ``source_path`` and return its path."""

    with open(source_path, encoding='utf-8') as source:
        text = with_frame_rate(without_tool(source.read(), GOAL_TOOL_CLASS), HARDWARE_FRAME_RATE)
    with tempfile.NamedTemporaryFile(
            'w', encoding='utf-8', prefix='ridgeback_hardware_', suffix='.rviz',
            delete=False) as output:
        output.write(text)
    return output.name


def hardware_rviz_process(cpu_count: int | None) -> tuple[str, dict]:
    """Launch prefix and environment that keep software RViz off the sensors.

    RViz runs at the lowest CPU priority on the last two cores, and Mesa's
    software renderer gets two threads instead of one per core. Unlimited, its
    per-frame bursts across every core delayed the camera driver's frames.
    """

    prefix = 'nice -n 19'
    if cpu_count and cpu_count >= 4:
        prefix += f' taskset -c {cpu_count - 2},{cpu_count - 1}'
    return prefix, {'LP_NUM_THREADS': '2'}
