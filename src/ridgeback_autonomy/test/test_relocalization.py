"""G1 relocalization after mapping around it (2026-11-21 Gazebo): the view is chosen from a
confirmed stop, an in-place view is judged by what the stopped camera sees, a localization
is a reliable fallback view only when its head region showed G1 from 1.3-3.5 m, and the
robot never drives back to a distant view that gave none; it fails as
``g1_relocalization_failed`` instead. Every failed-parking return is the guarded one."""
import math

import numpy as np
import pytest
from rclpy.parameter import Parameter

from ridgeback_autonomy.mission.mission_manager_node import MissionState
from test_g1_exploration import (
    PLAN_NOW_NS, back_at_observation, exploring_manager, finish_round_away_from_the_observation_pose,
    occupancy_map, settle_motion, stand, stop_before_the_view)
from test_mission_manager import localize, manager_factory  # noqa: F401 (fixture)
from test_g1_position_check import mission_at, view
from test_parking_mission_navigation import answer, navigation_result


def test_the_view_is_chosen_only_after_a_confirmed_stop(manager_factory):
    m = exploring_manager(manager_factory)
    m._on_map(occupancy_map())                         # mapping done: relocalize
    m.tick()
    assert m.state == MissionState.RETURNING_TO_OBSERVATION and m._retry_after_stop
    m.tick()
    assert m.navigation_client.calls == [] and m._goal_checker_set_client.calls == []
    m._monotonic.value += m.motion_settlement_timeout_s + 1.0     # never stopped
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'stop_not_confirmed'
    assert 'after mapping around G1' in m.transition_history[-1].reason
    assert m.navigation_client.calls == []


def test_a_small_drift_after_an_in_place_turn_still_relocalizes_there(manager_factory):
    # 2026-11-21: stopped 0.043 m from the turn's pose (limit 0.040 m) with G1 centred 2.4 m
    # away, then drove back to a 4.3 m view; the view, not the pose, decides now.
    m = finish_round_away_from_the_observation_pose(manager_factory)
    goal = back_at_observation(m)
    stand(m, goal[0] + 0.043, goal[1], goal[2] + 0.01)
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    assert len(m.navigation_client.calls) == 1                     # no drive back


def test_an_in_place_view_beyond_the_relocalization_range_is_not_used(manager_factory):
    m = finish_round_away_from_the_observation_pose(manager_factory)
    goal = back_at_observation(m)
    t = m.target_position
    bearing = math.atan2(t.map_y_m - goal[1], t.map_x_m - goal[0])
    stand(m, t.map_x_m - 3.8 * math.cos(bearing), t.map_y_m - 3.8 * math.sin(bearing), bearing)
    settle_motion(m)
    assert m.state == MissionState.RETURNING_TO_OBSERVATION         # the reliable fallback
    assert 'G1 3.80 m from the camera' in m.transition_history[-1].reason


def unreliable_first_view(m):
    """The first localization came from 4.3 m with the head region on the background."""
    m._observation_reliable, m._observation_view = False, (False, 4.3)


def test_an_unreliable_distant_view_is_never_driven_back_to(manager_factory):
    m = finish_round_away_from_the_observation_pose(manager_factory)
    unreliable_first_view(m)
    goal = back_at_observation(m)
    m.tf_buffer.yaw = goal[2]
    settle_motion(m)
    assert m.state == MissionState.LOCALIZE_G1
    m._monotonic.value += m.localization_timeout_s + 1.0           # not localized in place
    m.tick()
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'g1_relocalization_failed'
    assert 'G1 4.30 m from the camera' in m.last_issue.reason
    assert len(m.navigation_client.calls) == 1                     # stayed where it stopped


def test_no_view_in_place_and_no_reliable_pose_fails_without_moving(manager_factory):
    m = exploring_manager(manager_factory)
    t = m.target_position
    stand(m, t.map_x_m - 4.2, t.map_y_m, 0.0)                     # too far to turn toward
    unreliable_first_view(m)
    m._on_map(occupancy_map())
    m.tick()
    stop_before_the_view(m)
    assert m.state == MissionState.MISSION_FAILED
    assert m.last_issue.code == 'g1_relocalization_failed'
    assert m.navigation_client.calls == [] and m._goal_checker_set_client.calls == []


def test_a_reliable_view_is_kept_over_a_later_unreliable_one(manager_factory):
    m = exploring_manager(manager_factory)
    kept = m._observation_pose
    assert m._observation_reliable                                 # 1.3-3.5 m, no depth check here
    stand(m, kept[0] + 1.0, kept[1], kept[2])
    m._last_view = (False, 4.3)
    m._remember_observation_pose()
    assert m._observation_pose == kept and m._observation_reliable
    m._last_view = (True, 2.4)
    m._remember_observation_pose()
    assert m._observation_pose != kept and m._observation_reliable


def test_a_head_region_on_the_background_never_makes_a_reliable_view(manager_factory):
    v = view('ward_close')                                         # head region on the wall
    m = mission_at(manager_factory, v)
    assert m._observe_target_body() is None                       # torso and axis agree
    assert m._last_view[0] is False
    for name in ('ward_front', 'ward_behind'):                     # clear views
        from test_g1_position_check import head
        from ridgeback_autonomy.mission import target_orientation as to
        clear = dict(view(name))
        clear['surface'] = np.array(to.head_axis(head(clear), clear['camera_translation'][:2]))
        m = mission_at(manager_factory, clear)
        assert m._observe_target_body() is None
        reliable, camera_range = m._last_view
        low, high = m.g1_reacquire_range_m
        assert reliable == (low <= camera_range <= high), (name, camera_range)


def test_an_invalid_depth_relocalization_keeps_waiting_then_fails_with_its_reason(manager_factory):
    m = finish_round_away_from_the_observation_pose(manager_factory)
    unreliable_first_view(m)
    m.tf_buffer.yaw = back_at_observation(m)[2]
    settle_motion(m)
    m.target_body_check = True
    m._observe_target_body = lambda: 'no consistent depth in the head region of the detection box'
    second = PLAN_NOW_NS + 10
    m.test_ros_ns = m.tf_buffer.stamp_ns = second + 4
    localize(m, first_stamp_ns=second)
    assert m.state == MissionState.LOCALIZE_G1 and m.last_issue.code == 'target_position_unverified'
    m._monotonic.value += m.localization_timeout_s + 1.0
    m.tick()
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'g1_relocalization_failed'


def mounted_camera(m, ahead=0.6):
    """The camera 0.6 m ahead of the base, looking along it (the turn moves it)."""
    tf = m.tf_buffer
    m._camera_view = lambda: (tf.x + ahead * math.cos(tf.yaw), tf.y + ahead * math.sin(tf.yaw), tf.yaw)


def facing_away(m, base_range):
    t = m.target_position
    m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw = t.map_x_m - base_range, t.map_y_m, math.pi
    mounted_camera(m)


def test_a_turn_that_would_bring_the_camera_too_close_returns_to_the_reliable_view(manager_factory):
    # 2026-11-21: 1.51 m from G1 before the turn, 0.94 m after it, then rejected.
    m = exploring_manager(manager_factory)
    kept = m._observation_pose
    facing_away(m, 1.75)                                      # camera 2.35 m now, 1.15 m after
    m._on_map(occupancy_map())
    m.tick()
    stop_before_the_view(m)
    assert 'would leave the camera 1.15 m from it' in m._view_choice
    assert back_at_observation(m) == pytest.approx(kept)      # no turn in place first
    assert len(m.navigation_client.calls) == 1


def test_a_too_close_turn_without_a_reliable_view_fails_without_moving(manager_factory):
    m = exploring_manager(manager_factory)
    unreliable_first_view(m)
    facing_away(m, 1.75)
    m._on_map(occupancy_map())
    m.tick()
    stop_before_the_view(m)
    assert m.state == MissionState.MISSION_FAILED and m.last_issue.code == 'g1_relocalization_failed'
    assert m.navigation_client.calls == []


def test_a_turn_that_keeps_the_camera_in_range_is_made_and_aims_the_camera(manager_factory):
    m = exploring_manager(manager_factory)
    t = m.target_position
    m.tf_buffer.x, m.tf_buffer.y, m.tf_buffer.yaw = t.map_x_m - 2.5, t.map_y_m - 0.8, math.pi
    mounted_camera(m)
    m._on_map(occupancy_map())
    m.tick()
    assert 'turning toward G1 in place' in (stop_before_the_view(m) or m._view_choice)
    goal = back_at_observation(m)
    camera = (goal[0] + 0.6 * math.cos(goal[2]), goal[1] + 0.6 * math.sin(goal[2]))
    bearing = math.atan2(t.map_y_m - camera[1], t.map_x_m - camera[0])
    assert abs(math.remainder(bearing - goal[2], 2 * math.pi)) < 0.01   # optical axis on G1
    assert 1.3 <= math.dist(camera, (t.map_x_m, t.map_y_m)) <= 3.5
