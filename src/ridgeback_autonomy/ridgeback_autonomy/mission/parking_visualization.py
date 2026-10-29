"""RViz markers for a non-driving plan: radial donut lines and candidate validity.

The donut is drawn as 360 radial lines at 1-degree steps, each split where the
planner's classification of the cells it crosses changes: green where a parking
pose with its task point there passed the footprint, keep-out and reach-budget
checks, red where the line crosses an obstacle, orange where the map is unknown,
grey on free floor where the parked footprint would fail. Green candidate points
mean the same footprint validity. Neither layer certifies Nav2 path reachability.

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


GREEN = (0.1, 0.9, 0.2, 0.85)
RED = (1.0, 0.0, 0.0, 0.9)
UNKNOWN_RED = (1.0, 0.35, 0.0, 0.8)   # original: unknown in an orange-red shade of red
REJECTED = (1.0, 0.78, 0.05, 0.9)     # original "rejected" amber for non-collision reasons
SELECTED = (0.0, 0.6, 1.0, 1.0)
G1_COLOR = (0.8, 0.2, 0.9, 1.0)
RING = (1.0, 1.0, 1.0, 0.9)           # donut reach boundaries
POSE_INVALID = (0.55, 0.55, 0.55, 0.7)  # free floor, but no valid parking pose there
# Marker id per radial class: one LINE_LIST each so RViz can toggle them.
RAY_CLASSES = (('valid', GREEN), ('occupied', RED), ('unknown', UNKNOWN_RED), ('pose_invalid', POSE_INVALID))
# (namespace, id, candidate status, colour): geometric validity of each candidate task point.
# Valid candidates stay on top of the lines; the rejected ones repeat what the lines
# show, so they get their own namespace (hidden by default in the parking layout).
CANDIDATE_CLASSES = (('parking/candidate_task_points', 0, 'valid', GREEN),
                     ('parking/rejected_task_points', 0, 'collision', RED),
                     ('parking/rejected_task_points', 1, 'rejected', REJECTED))
# Marker id per Nav2 path-check result of a geometrically valid candidate.
PATH_CHECK_CLASSES = (('unchecked', REJECTED), ('rejected', RED), ('valid', GREEN))
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
                       RING, (0.015, 0, 0))
        ring.points = _circle(t.x, t.y, radius, 0.04)
        add(ring)

    for marker_id, (status, rgba) in enumerate(RAY_CLASSES):
        rays = _marker('parking/radial_1deg', marker_id, Marker.LINE_LIST, frame, stamp, rgba, (0.005, 0, 0))
        for angle, runs in plan.radial_rays:
            c, s = math.cos(angle), math.sin(angle)
            rays.points += [Point(x=t.x + r * c, y=t.y + r * s, z=0.05)
                            for r0, r1, run_status in runs if run_status == status for r in (r0, r1)]
        add(rays)

    for ns, marker_id, status, rgba in CANDIDATE_CLASSES:
        task_points = _marker(ns, marker_id, Marker.SPHERE_LIST, frame, stamp, rgba, (0.018, 0.018, 0.018))
        task_points.points = [Point(x=c.task_x, y=c.task_y, z=0.08) for c in plan.candidates
                              if c.status == status]
        add(task_points)

    for marker_id, (status, rgba) in enumerate(PATH_CHECK_CLASSES):
        points = _marker('parking/path_candidates', marker_id, Marker.SPHERE_LIST,
                         frame, stamp, rgba, (0.025, 0.025, 0.025))
        points.points = [Point(x=c.task_x, y=c.task_y, z=0.13) for c in plan.candidates
                         if c.status in ('valid', 'selected') and c.path_status == status]
        add(points)

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
    passing = sum(any(run[2] == 'valid' for run in runs) for _, runs in plan.radial_rays)
    label.text = (f"{status}\n"
                  f"{t.reference} (keep-out {plan.keepout_radius_m:.2f} m), "
                  f"valid {counts['valid']}/{counts['candidates']}, "
                  f"rays passing {passing}/{len(plan.radial_rays)}")
    add(label)
    return array


# Screen legend: one latched overlay, generated from the colour tables above.

RAY_LABELS = {'valid': 'Valid parking pose (no path check)', 'occupied': 'Obstacle on the ray',
              'unknown': 'Unknown or unmapped', 'pose_invalid': 'Free, parking pose invalid'}
CANDIDATE_LABELS = {'valid': 'Valid candidate', 'collision': 'Rejected: collision / G1 keep-out *',
                    'rejected': 'Rejected: unknown / reach *'}
PATH_CHECK_LABELS = {'unchecked': 'Path not checked yet', 'rejected': 'No safe path',
                     'valid': 'Path feasible (Nav2 + swept footprint)'}
LINE, DOT, ARROW, OUTLINE, RING_GLYPH = '\u2501\u2501', '\u25cf', '\u279c', '\u25ad', '\u25ef'
LEGEND_TEXT_SIZE = 9
LEGEND_LINE_PX = 15   # one 9 pt row, with slack for font differences


def legend_sections(config: PlannerConfig):
    """(title, [(glyph, rgba, label)]) for every coloured parking marker and shared Nav2 display."""
    return (
        (f'Donut rays (1\u00b0, {config.reach_min_m:.2f}-{config.reach_max_m:.2f} m)',
         [(LINE, rgba, RAY_LABELS[status]) for status, rgba in RAY_CLASSES]),
        ('Candidates (small dots)',
         [(DOT, rgba, CANDIDATE_LABELS[status]) for _, _, status, rgba in CANDIDATE_CLASSES]),
        ('Nav2 path check (large dots)',
         [(DOT, rgba, PATH_CHECK_LABELS[status]) for status, rgba in PATH_CHECK_CLASSES]),
        ('Selected parking', [(ARROW, SELECTED, 'Parking pose and heading'),
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
    rows.append('* hidden by default (Parking Plan namespaces)')
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
