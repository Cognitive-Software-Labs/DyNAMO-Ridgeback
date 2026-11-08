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


# Presentation defaults for a 1920 x 1080 screen: the scene gets the window. The
# Displays dock starts hidden (RViz's dock arrow, or Panels > Displays, reopens it)
# and the camera-image dock at the bottom is lowered from 306 to 150 px, which leaves
# a 1080-line screen (and a 959-line remote session) room for the left overlays.
WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX = 1920, 1040
BOTTOM_DOCK_STATE = ('00000a0000000132', '0000078000000096')   # Qt state: width, height
VIEW_AREA_PX = (1800.0, 620.0)   # the 3D view left by the docks, toolbars and status bar
VIEW_MARGIN_M = 2.5              # around the start and G1's workstation


def task_view(start_xy, g1_xy):
    """A top-down view framing the robot's start and G1's workstation.

    The view turns in 90-degree steps so that the start-to-G1 axis runs across
    the wide screen; it stays in the map frame, so pan and zoom work as usual.
    """
    import math

    dx, dy = g1_xy[0] - start_xy[0], g1_xy[1] - start_xy[1]
    angle = round(math.atan2(dy, dx) / (math.pi / 2)) * (math.pi / 2)
    along = abs(dx * math.cos(angle) + dy * math.sin(angle)) + 2 * VIEW_MARGIN_M
    across = abs(-dx * math.sin(angle) + dy * math.cos(angle)) + 2 * VIEW_MARGIN_M
    scale = min(VIEW_AREA_PX[0] / along, VIEW_AREA_PX[1] / across)
    return {
        'Class': 'rviz_default_plugins/TopDownOrtho',
        'Name': 'Current View',
        'Value': 'TopDownOrtho',
        'Target Frame': 'map',
        'Near Clip Distance': 0.01,
        'X': round((start_xy[0] + g1_xy[0]) / 2, 3),
        'Y': round((start_xy[1] + g1_xy[1]) / 2, 3),
        'Angle': round(angle, 4),
        'Scale': math.floor(scale * 10) / 10,   # down, so both ends stay in view
    }


# Docks the parking view starts without. RViz ignores 'Hide Left Dock' when it restores
# this window state (2026-11-08, reproduced with a standalone rviz2), so each dock's
# visible flag in the saved Qt state is cleared instead; Panels > <name> reopens it.
HIDDEN_DOCKS = ('Displays', 'Selection', 'Tool Properties', 'Views')


def hide_docks(state_hex: str, names=HIDDEN_DOCKS) -> str:
    """Clear the visible flag that follows each named dock in a QMainWindow state."""
    for name in names:
        encoded = name.encode('utf-16-be').hex()
        marker = 'fb' + f'{len(encoded) // 2:08x}' + encoded
        state_hex = state_hex.replace(marker + '01', marker + '00')
    return state_hex


def with_presentation(layout: dict, view: dict | None = None) -> dict:
    """The layout with the Displays dock hidden, a lower image dock and ``view``."""

    window = layout.setdefault('Window Geometry', {})
    window['Hide Left Dock'] = True
    window['Width'], window['Height'] = WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX
    state = str(window.get('QMainWindow State', ''))
    if BOTTOM_DOCK_STATE[0] in state:
        state = state.replace(*BOTTOM_DOCK_STATE, 1)
    window['QMainWindow State'] = hide_docks(state)
    if view is not None:
        layout['Visualization Manager'].setdefault('Views', {})['Current'] = dict(view)
    return layout


def parking_layout(source_path: str, topic: str, view: dict | None = None) -> str:
    """Write the parking copy of ``source_path`` and return its path."""

    with open(source_path, encoding='utf-8') as source:
        layout = yaml.safe_load(source)
    layout = with_presentation(with_parking_display(layout, topic), view)
    with tempfile.NamedTemporaryFile(
            'w', encoding='utf-8', prefix='ridgeback_parking_', suffix='.rviz',
            delete=False) as output:
        yaml.safe_dump(layout, output, sort_keys=False)
    return output.name
