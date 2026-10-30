"""RViz markers for a parking plan: radial donut lines and the selected pose.

The donut is drawn as 360 radial lines at 1-degree steps, each one solid colour
from the inner to the outer reach limit (``radial_profile``): green where the
direction has at least one parking pose that passed the footprint, keep-out and
reach-budget checks; otherwise orange where unmapped space leaves it undecided,
red where obstacles block every otherwise valid pose, grey where keep-out or reach
rules out every pose. Green is geometric validity only; Nav2 path feasibility is
shown by the selected pose and its route.

Every array starts with a DELETEALL marker so a new plan, a failed plan or a
reset never leaves geometry from an earlier target on screen. The colour legend
is a separate, latched screen overlay built from the same colour tables, so it
survives those clears and cannot drift from the markers.
"""

from __future__ import annotations

import html
import math

from geometry_msgs.msg import Point
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ridgeback_autonomy.mission.parking_geometry import rotated_footprint_vertices
from ridgeback_autonomy.mission.parking_planner import ParkingPlan, PlannerConfig


GREEN = (0.1, 0.9, 0.2, 1.0)
RED = (1.0, 0.0, 0.0, 1.0)
UNKNOWN_ORANGE = (1.0, 0.55, 0.0, 1.0)
SELECTED = (0.0, 0.6, 1.0, 1.0)
G1_COLOR = (0.8, 0.2, 0.9, 1.0)
RING = (0.85, 0.85, 0.85, 0.5)        # donut reach boundaries, secondary to the rays
POSE_INVALID = (0.6, 0.6, 0.6, 1.0)  # keep-out or reach rules out every pose on the ray
# Marker id per radial class: one LINE_LIST each so RViz can toggle them. Opaque, so
# neighbouring 1-degree rays do not blend into one shade.
RAY_CLASSES = (('valid', GREEN), ('occupied', RED), ('unknown', UNKNOWN_ORANGE),
               ('pose_invalid', POSE_INVALID))
# Colours of the shared layout's Nav2 displays (sim/rviz/exploration.rviz), shown in the legend.
NAV2_PLAN = (0.0, 1.0, 0.0, 1.0)
MPPI_TRAJECTORY = (1.0, 170 / 255, 0.0, 1.0)
COLLISION_ZONE = (1.0, 25 / 255, 0.0, 1.0)
CIRCLE_SEGMENTS = 72  # the original donut outline resolution


def _color(rgba):
    return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))


def _marker(ns, marker_id, marker_type, frame_id, stamp, rgba=(1, 1, 1, 1), scale=(0.05, 0.05, 0.05)):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
    m.color = _color(rgba)
    return m


def _circle(cx, cy, radius, z):
    return [Point(x=cx + radius * math.cos(2 * math.pi * k / CIRCLE_SEGMENTS),
                  y=cy + radius * math.sin(2 * math.pi * k / CIRCLE_SEGMENTS), z=z)
            for k in range(CIRCLE_SEGMENTS + 1)]


def clear_markers() -> MarkerArray:
    clear = Marker()
    clear.action = Marker.DELETEALL
    return MarkerArray(markers=[clear])


def plan_markers(plan: ParkingPlan, config: PlannerConfig, stamp) -> MarkerArray:
    frame = plan.target.frame_id
    t = plan.target
    array = clear_markers()
    add = array.markers.append

    g1 = _marker('parking/g1_position', 0, Marker.CYLINDER, frame, stamp, G1_COLOR, (0.08, 0.08, 0.6))
    g1.pose.position.x, g1.pose.position.y, g1.pose.position.z = t.x, t.y, 0.3
    add(g1)
    keepout = _marker('parking/g1_keepout', 0, Marker.LINE_STRIP, frame, stamp, RED, (0.02, 0, 0))
    keepout.points = _circle(t.x, t.y, plan.keepout_radius_m, 0.03)
    add(keepout)
    for marker_id, radius in enumerate((config.reach_min_m, config.reach_max_m)):
        ring = _marker('parking/donut_boundaries', marker_id, Marker.LINE_STRIP, frame, stamp,
                       RING, (0.006, 0, 0))
        ring.points = _circle(t.x, t.y, radius, 0.04)
        add(ring)

    for marker_id, (status, rgba) in enumerate(RAY_CLASSES):
        # 4 mm wide: rays 1 degree apart are 5.2 mm apart at the inner limit.
        rays = _marker('parking/radial_1deg', marker_id, Marker.LINE_LIST, frame, stamp, rgba, (0.004, 0, 0))
        for angle, ray_status in plan.radial_rays:
            if ray_status == status:
                c, s = math.cos(angle), math.sin(angle)
                rays.points += [Point(x=t.x + r * c, y=t.y + r * s, z=0.05)
                                for r in (config.reach_min_m, config.reach_max_m)]
        add(rays)

    if plan.selected_path is not None:
        path = _marker('parking/selected_path', 0, Marker.LINE_STRIP, frame, stamp,
                       SELECTED, (0.025, 0, 0))
        path.points = [Point(x=p.pose.position.x, y=p.pose.position.y, z=0.15)
                       for p in plan.selected_path.poses]
        add(path)

    if plan.selected is not None:
        sel = plan.selected
        arrow = _marker('parking/selected_pose', 0, Marker.ARROW, frame, stamp, SELECTED, (0.4, 0.06, 0.06))
        arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = sel.base_x, sel.base_y, 0.1
        arrow.pose.orientation.z = math.sin(sel.yaw / 2.0)
        arrow.pose.orientation.w = math.cos(sel.yaw / 2.0)
        add(arrow)
        outline = _marker('parking/selected_footprint', 0, Marker.LINE_STRIP, frame, stamp, SELECTED, (0.02, 0, 0))
        vertices = rotated_footprint_vertices(config.footprint, sel.base_x, sel.base_y, sel.yaw)
        outline.points = [Point(x=x, y=y, z=0.1) for x, y in vertices + vertices[:1]]
        add(outline)
        task = _marker('parking/selected_task_point', 0, Marker.SPHERE, frame, stamp, SELECTED, (0.06, 0.06, 0.06))
        task.pose.position.x, task.pose.position.y, task.pose.position.z = sel.task_x, sel.task_y, 0.12
        add(task)

    label = _marker('parking/status', 0, Marker.TEXT_VIEW_FACING, frame, stamp, (1, 1, 1, 1), (0, 0, 0.12))
    # Beyond the outer ring so the text never covers the donut in a top-down view.
    label.pose.position.x = t.x
    label.pose.position.y = t.y + config.reach_max_m + 0.25
    label.pose.position.z = 1.0
    counts = plan.counts()
    status = ('path-feasible parking pose' if plan.selected_path is not None else
              'geometric parking pose' if plan.ok else
              'path evaluation pending' if plan.failure_reason is None else
              'NO PARKING POSE: ' + str(plan.failure_reason))
    passing = sum(status == 'valid' for _, status in plan.radial_rays)
    label.text = (f"{status}\n"
                  f"{t.reference} (keep-out {plan.keepout_radius_m:.2f} m), "
                  f"valid {counts['valid']}/{counts['candidates']}, "
                  f"rays passing {passing}/{len(plan.radial_rays)}")
    add(label)
    return array


# Screen legend: one latched overlay, generated from the colour tables above.

RAY_LABELS = {'valid': 'A valid parking pose exists (no path check)',
              'occupied': 'No valid pose: obstacles block it',
              'unknown': 'No valid pose: unmapped space, undecided',
              'pose_invalid': 'No valid pose: keep-out / reach'}
LINE, DOT, ARROW, OUTLINE, RING_GLYPH = '\u2501\u2501', '\u25cf', '\u279c', '\u25ad', '\u25ef'
LEGEND_TEXT_SIZE = 9
LEGEND_LINE_PX = 15   # one 9 pt row, with slack for font differences


def legend_sections(config: PlannerConfig):
    """(title, [(glyph, rgba, label)]) for every coloured parking marker and shared Nav2 display."""
    return (
        (f'Donut rays: one colour per 1\u00b0 ray ({config.reach_min_m:.2f}-{config.reach_max_m:.2f} m)',
         [(LINE, rgba, RAY_LABELS[status]) for status, rgba in RAY_CLASSES]),
        ('Selected parking (Nav2-checked)', [(ARROW, SELECTED, 'Parking pose and heading'),
                              (OUTLINE, SELECTED, 'Robot footprint at the pose'),
                              (DOT, SELECTED, 'Rear-bin task point'),
                              (LINE, SELECTED, 'Validated route to the pose')]),
        ('G1', [(DOT, G1_COLOR, 'G1 position estimate'),
                (RING_GLYPH, RED, 'G1 keep-out (protected region)'),
                (RING_GLYPH, RING, 'Reach limits (donut boundaries)')]),
        ('Nav2 (shared displays)', [(LINE, NAV2_PLAN, 'Live Nav2 plan'),
                                    (LINE, MPPI_TRAJECTORY, 'MPPI trajectory'),
                                    (OUTLINE, COLLISION_ZONE, 'Collision-monitor zone')]),
    )


def _swatch(glyph, rgba):
    r, g, b = (int(round(v * 255)) for v in rgba[:3])
    # The overlay strips "color: ...;" to draw its shadow, so keep the semicolon.
    return f'<span style="color: rgb({r}, {g}, {b});">{glyph}</span>'


def legend_overlay(config: PlannerConfig) -> OverlayText:
    """The colour legend as a fixed bottom-right RViz overlay (rviz_2d_overlay_plugins)."""
    rows = ['PARKING LEGEND']
    for title, entries in legend_sections(config):
        rows.append(f'<span style="color: rgb(150, 200, 255);">{html.escape(title)}</span>')
        rows += [f'&nbsp;&nbsp;{_swatch(glyph, rgba)}&nbsp;&nbsp;{html.escape(label)}'
                 for glyph, rgba, label in entries]
    msg = OverlayText()
    msg.action = OverlayText.ADD
    msg.width, msg.height = 330, LEGEND_LINE_PX * len(rows) + 10
    msg.horizontal_alignment, msg.vertical_alignment = OverlayText.RIGHT, OverlayText.BOTTOM
    msg.horizontal_distance = msg.vertical_distance = 10
    msg.bg_color = _color((0.05, 0.05, 0.05, 0.8))
    msg.fg_color = _color((0.93, 0.93, 0.93, 1.0))
    msg.line_width, msg.text_size, msg.font = 1, float(LEGEND_TEXT_SIZE), 'DejaVu Sans'
    msg.text = '\n'.join(rows)
    return msg
