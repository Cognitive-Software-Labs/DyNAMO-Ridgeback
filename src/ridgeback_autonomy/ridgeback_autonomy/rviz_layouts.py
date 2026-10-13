"""RViz layouts derived from the shared exploration layout.

Hardware uses the simulator's displays unchanged but without the goal tool:
on the robot, Nav2's output reaches the twist mux, so one click of
*2D Goal Pose* would command motion even with frontier goals disabled.
Deriving the layout at launch keeps the displays from drifting between the
two files.
"""
from __future__ import annotations

import tempfile

import yaml

GOAL_TOOL_CLASS = 'rviz_default_plugins/SetGoal'


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


def hardware_layout(source_path: str) -> str:
    """Write the goal-free copy of ``source_path`` and return its path."""

    with open(source_path, encoding='utf-8') as source:
        text = without_tool(source.read(), GOAL_TOOL_CLASS)
    with tempfile.NamedTemporaryFile(
            'w', encoding='utf-8', prefix='ridgeback_hardware_', suffix='.rviz',
            delete=False) as output:
        output.write(text)
    return output.name
