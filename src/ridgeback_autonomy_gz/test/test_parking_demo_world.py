"""The Gazebo parking world is initial_test_world without its authored G1."""

from pathlib import Path
import xml.etree.ElementTree as ET

WORLDS = Path(__file__).resolve().parents[1] / 'sim' / 'worlds'
AUTHORED_G1 = 'default_g1_far_right_room'


def _text(element):
    """An element's XML without the whitespace that follows it."""
    tail, element.tail = element.tail, None
    text = ET.tostring(element)
    element.tail = tail
    return text


def _children(path):
    world = ET.parse(path).getroot().find('world')
    return world, [_text(child) for child in world]


def test_demo_world_preserves_environment_and_removes_the_authored_target():
    demo, demo_children = _children(WORLDS / 'parking_demo.sdf')
    original, original_children = _children(WORLDS / 'initial_test_world.sdf')
    assert demo.get('name') == 'parking_demo'
    kept = [_text(child) for child in original
            if not (child.tag == 'include' and child.findtext('name') == AUTHORED_G1)]
    assert len(kept) == len(original_children) - 1
    assert demo_children == kept
    assert not [uri for uri in demo.iter('uri') if 'g1' in (uri.text or '').lower()]
