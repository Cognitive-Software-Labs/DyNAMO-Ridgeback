"""RViz layout for the parking mission, derived from the shared exploration layout.

The shared ``exploration.rviz`` stays untouched; the mission launch writes a
temporary copy with a MarkerArray display for the parking plan and screen
overlays for its colour legend and the live mission state machine.
"""

from __future__ import annotations

import tempfile

import yaml

PARKING_DISPLAY_NAME = 'Parking Plan'
LEGEND_DISPLAY_NAME = 'Parking Legend'
LEGEND_TOPIC = 'parking/legend'
STATE_DIAGRAM_DISPLAY_NAME = 'Mission State Machine'
STATE_DIAGRAM_TOPIC = 'parking_mission/state_diagram'
PARKING_NAMESPACES = (
    'parking/g1_position', 'parking/g1_keepout', 'parking/donut_boundaries',
    'parking/radial_1deg',
    'parking/selected_pose', 'parking/selected_footprint', 'parking/selected_task_point',
    'parking/status', 'parking/selected_path',
)
PARKING_HIDDEN_NAMESPACES = ()


def with_parking_display(layout: dict, topic: str, legend_topic: str = LEGEND_TOPIC,
                         state_diagram_topic: str = STATE_DIAGRAM_TOPIC) -> dict:
    """The layout with transient-local parking marker, legend and state-machine displays."""

    displays = layout['Visualization Manager']['Displays']
    if any(d.get('Name') == PARKING_DISPLAY_NAME for d in displays):
        return layout
    displays.append({
        'Class': 'rviz_default_plugins/MarkerArray',
        'Enabled': True,
        'Name': PARKING_DISPLAY_NAME,
        'Namespaces': {**{ns: True for ns in PARKING_NAMESPACES},
                       **{ns: False for ns in PARKING_HIDDEN_NAMESPACES}},
        'Topic': {
            'Depth': 1,
            # The mission publishes each plan once, latched.
            'Durability Policy': 'Transient Local',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': topic,
        },
        'Value': True,
    })
    for name, overlay_topic in ((LEGEND_DISPLAY_NAME, legend_topic),
                                (STATE_DIAGRAM_DISPLAY_NAME, state_diagram_topic)):
        displays.append({
            'Class': 'rviz_2d_overlay_plugins/TextOverlay',
            'Enabled': True,
            'Name': name,
            'Topic': {
                'Depth': 1,
                # Latched by the mission manager, so a late RViz still gets it.
                'Durability Policy': 'Transient Local',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Value': overlay_topic,
            },
            'Value': True,
        })
    return layout


def parking_layout(source_path: str, topic: str) -> str:
    """Write the parking copy of ``source_path`` and return its path."""

    with open(source_path, encoding='utf-8') as source:
        layout = yaml.safe_load(source)
    with tempfile.NamedTemporaryFile(
            'w', encoding='utf-8', prefix='ridgeback_parking_', suffix='.rviz',
            delete=False) as output:
        yaml.safe_dump(with_parking_display(layout, topic), output, sort_keys=False)
    return output.name
