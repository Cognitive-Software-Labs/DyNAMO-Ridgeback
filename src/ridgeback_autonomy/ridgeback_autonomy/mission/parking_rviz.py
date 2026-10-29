"""RViz layout for the parking mission, derived from the shared exploration layout.

The shared ``exploration.rviz`` stays untouched; the mission launch writes a
temporary copy with a MarkerArray display for the parking plan and a screen
overlay for its colour legend.
"""

from __future__ import annotations

import tempfile

import yaml

PARKING_DISPLAY_NAME = 'Parking Plan'
LEGEND_DISPLAY_NAME = 'Parking Legend'
LEGEND_TOPIC = 'parking/legend'
PARKING_NAMESPACES = (
    'parking/g1_position', 'parking/g1_keepout', 'parking/donut_boundaries',
    'parking/radial_1deg', 'parking/candidate_task_points',
    'parking/selected_pose', 'parking/selected_footprint', 'parking/selected_task_point',
    'parking/status',
    'parking/path_candidates', 'parking/selected_path',
)
# Published but off by default: the radial lines already show where parking fails.
PARKING_HIDDEN_NAMESPACES = ('parking/rejected_task_points',)


def with_parking_display(layout: dict, topic: str, legend_topic: str = LEGEND_TOPIC) -> dict:
    """The layout with transient-local parking marker and legend displays appended."""

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
    displays.append({
        'Class': 'rviz_2d_overlay_plugins/TextOverlay',
        'Enabled': True,
        'Name': LEGEND_DISPLAY_NAME,
        'Topic': {
            'Depth': 1,
            # Published once by the mission manager, latched.
            'Durability Policy': 'Transient Local',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': legend_topic,
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
