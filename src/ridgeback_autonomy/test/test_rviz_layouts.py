"""The hardware RViz layout: the exploration layout without the goal tool.

On the robot, Nav2's output reaches the twist mux, so the goal tool would
command motion even with frontier goals off.
"""
from pathlib import Path

import yaml

from ridgeback_autonomy.rviz_layouts import GOAL_TOOL_CLASS, hardware_layout, without_tool

LAYOUT = Path(__file__).resolve().parents[1] / 'sim' / 'rviz' / 'exploration.rviz'


def tool_classes(layout: dict) -> list:
    return [tool['Class'] for tool in layout['Visualization Manager']['Tools']]


def test_only_the_goal_tool_is_removed():
    source = yaml.safe_load(LAYOUT.read_text())
    derived = yaml.safe_load(without_tool(LAYOUT.read_text(), GOAL_TOOL_CLASS))
    assert GOAL_TOOL_CLASS in tool_classes(source)
    assert tool_classes(derived) == [c for c in tool_classes(source) if c != GOAL_TOOL_CLASS]
    del source['Visualization Manager']['Tools'], derived['Visualization Manager']['Tools']
    assert derived == source  # displays, views and window layout unchanged


def test_hardware_layout_writes_a_goal_free_copy(tmp_path, monkeypatch):
    monkeypatch.setattr('tempfile.tempdir', str(tmp_path))
    path = Path(hardware_layout(str(LAYOUT)))
    assert path.parent == tmp_path
    assert GOAL_TOOL_CLASS not in tool_classes(yaml.safe_load(path.read_text()))
