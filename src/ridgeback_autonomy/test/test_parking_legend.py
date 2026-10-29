"""RViz colour legend: generated from the marker colour tables, latched apart from the markers."""

import re

import yaml
from visualization_msgs.msg import Marker

from ridgeback_autonomy.mission import parking_visualization as viz
from ridgeback_autonomy.mission.parking_planner import plan_parking
from test_mission_manager import manager_factory, occupancy_map, planning_manager  # noqa: F401 (fixture)
from test_parking_planner import APPROACH_FROM_WEST, CONFIG, fill, grid, target
from test_launch_layout import _package_root


def entries():
    return {label: (glyph, rgba) for _, rows in viz.legend_sections(CONFIG) for glyph, rgba, label in rows}


def rgb(rgba):
    return tuple(int(round(v * 255)) for v in rgba[:3])


def marker_rgb(m):
    return rgb((m.color.r, m.color.g, m.color.b))


def test_donut_legend_matches_the_four_ray_classes():
    rows = dict(viz.legend_sections(CONFIG))['Donut rays (1°, 0.30-0.60 m)']
    assert [(label, rgb(rgba)) for _, rgba, label in rows] == [
        ('Valid parking pose (no path check)', rgb(viz.GREEN)),
        ('Obstacle on the ray', rgb(viz.RED)),
        ('Unknown or unmapped', rgb(viz.UNKNOWN_RED)),
        ('Free, parking pose invalid', rgb(viz.POSE_INVALID))]
    # Green on the donut is geometric validity; the path check has its own section.
    assert 'Nav2 path check (large dots)' in dict(viz.legend_sections(CONFIG))


def test_every_coloured_parking_marker_has_its_legend_entry():
    import rclpy.time
    g = grid()
    fill(g, 3.40, 2.80, 3.50, 3.20, 100)       # an obstacle across the donut
    fill(g, 2.40, 2.95, 2.50, 3.05, -1)        # and an unknown patch
    plan = plan_parking(g, target(), CONFIG, APPROACH_FROM_WEST)
    for c in plan.candidates[:3]:
        c.path_status = 'rejected'
    markers = viz.plan_markers(plan, CONFIG, rclpy.time.Time(seconds=1).to_msg()).markers
    legend = entries()
    expected = {('parking/radial_1deg', i): viz.RAY_LABELS[s] for i, (s, _) in enumerate(viz.RAY_CLASSES)}
    expected.update({(ns, i): viz.CANDIDATE_LABELS[s] for ns, i, s, _ in viz.CANDIDATE_CLASSES})
    expected.update({('parking/path_candidates', i): viz.PATH_CHECK_LABELS[s]
                     for i, (s, _) in enumerate(viz.PATH_CHECK_CLASSES)})
    expected.update({('parking/selected_pose', 0): 'Parking pose and heading',
                     ('parking/selected_footprint', 0): 'Robot footprint at the pose',
                     ('parking/selected_task_point', 0): 'Rear-bin task point',
                     ('parking/g1_position', 0): 'G1 position estimate',
                     ('parking/g1_keepout', 0): 'G1 keep-out (protected region)',
                     ('parking/donut_boundaries', 0): 'Reach limits (donut boundaries)',
                     ('parking/donut_boundaries', 1): 'Reach limits (donut boundaries)'})
    seen = set()
    for m in markers[1:]:
        if m.ns == 'parking/status':
            continue                               # white text, explains itself
        key = (m.ns, m.id)
        assert key in expected, f'marker {key} has no legend entry'
        assert marker_rgb(m) == rgb(legend[expected[key]][1]), key
        seen.add(key)
    assert {('parking/radial_1deg', 1), ('parking/radial_1deg', 2)} <= seen   # obstacle and unknown drawn


def test_selected_pose_route_and_feasibility_are_explained_with_their_colours():
    legend = entries()
    for label in ('Parking pose and heading', 'Robot footprint at the pose', 'Rear-bin task point',
                  'Validated route to the pose'):
        assert rgb(legend[label][1]) == rgb(viz.SELECTED)
    assert rgb(legend['Path feasible (Nav2 + swept footprint)'][1]) == rgb(viz.GREEN)
    assert rgb(legend['No safe path'][1]) == rgb(viz.RED)
    assert rgb(legend['Path not checked yet'][1]) == rgb(viz.REJECTED)
    assert rgb(legend['Rejected: collision / G1 keep-out *'][1]) == rgb(viz.RED)


def test_legend_overlay_is_a_bottom_right_rich_text_panel_sized_for_its_rows():
    msg = viz.legend_overlay(CONFIG)
    rows = msg.text.split('\n')
    assert msg.action == msg.ADD and msg.horizontal_alignment == msg.RIGHT
    assert msg.vertical_alignment == msg.BOTTOM
    assert msg.height >= viz.LEGEND_LINE_PX * len(rows) and msg.bg_color.a > 0.5
    # Each swatch keeps "color: ...;" so the overlay can strip it for its shadow.
    swatches = re.findall(r'color: rgb\((\d+), (\d+), (\d+)\);', msg.text)
    labels = sum(len(r) for _, r in viz.legend_sections(CONFIG))
    assert len(swatches) == labels + len(viz.legend_sections(CONFIG))   # entries + section titles
    for _, rgba, label in (e for _, r in viz.legend_sections(CONFIG) for e in r):
        r, g, b = rgb(rgba)
        assert f'<span style="color: rgb({r}, {g}, {b});">' in msg.text
        assert label in msg.text
    assert '0.30-0.60 m' in msg.text


def test_shared_display_colours_match_the_exploration_layout():
    layout = yaml.safe_load((_package_root() / 'sim' / 'rviz' / 'exploration.rviz').read_text())
    colours = {d['Name']: tuple(int(v) for v in d['Color'].split(';'))
               for d in layout['Visualization Manager']['Displays'] if 'Color' in d}
    legend = entries()
    assert colours['Global Plan'] == rgb(legend['Live Nav2 plan'][1])
    assert colours['MPPI Optimal'] == rgb(legend['MPPI trajectory'][1])
    assert colours['Collision Zone (approach)'] == rgb(legend['Collision-monitor zone'][1])


def test_legend_is_latched_once_and_survives_plan_updates_and_clears(manager_factory):
    m, published = planning_manager(manager_factory, map_msg=occupancy_map())
    legends = []
    m.parking_legend_publisher.publish = legends.append
    assert m.parking_legend_publisher.qos_profile.durability.name == 'TRANSIENT_LOCAL'
    # Plans and clears go to the marker topic only; the legend topic is never cleared.
    assert published['markers'] and published['markers'][-1].markers[0].action == Marker.DELETEALL
    m._fail_planning('test_failure', 'cleared by a failed plan')
    assert legends == []
    assert all(mk.ns != 'parking/legend' for arr in published['markers'] for mk in arr.markers)


def test_mission_publishes_the_legend_at_startup(manager_factory, monkeypatch):
    sent = []
    from rclpy.publisher import Publisher
    original = Publisher.publish
    monkeypatch.setattr(Publisher, 'publish', lambda self, msg: (
        sent.append(msg) if self.topic_name.endswith('parking/legend') else None) or original(self, msg))
    m = manager_factory()
    assert len(sent) == 1 and 'PARKING LEGEND' in sent[0].text
    assert m.parking_legend_publisher.topic_name.endswith('parking/legend')


def test_legend_is_resent_each_second_so_the_overlay_can_place_it(manager_factory):
    m = manager_factory()
    sent = []
    m.parking_legend_publisher.publish = sent.append
    m.tick()
    m._monotonic.value += 0.5
    m.tick()
    assert len(sent) == 1                    # at most once a second
    m._monotonic.value += 0.6
    m.tick()
    assert len(sent) == 2 and sent[0] is sent[1]   # the same, unchanging legend
