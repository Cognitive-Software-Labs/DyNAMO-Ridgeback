import math

import pytest

from ridgeback_leader_follow.follow_controller import (
    FollowPlanner, FollowSettings, quaternion_to_yaw, yaw_to_quaternion)

PLANNER = FollowPlanner(FollowSettings(follow_distance=1.2, hold_tolerance=.3, stale_timeout=.5))


def test_goal_is_follow_distance_short_of_the_leader_and_faces_it():
    decision = PLANNER.decide(True, (5., 0.), .1, (0., 0., 0.))
    assert decision.mode == 'following'
    x, y, yaw = decision.goal
    assert (x, y) == pytest.approx((3.8, 0.)) and yaw == pytest.approx(0.)
    decision = PLANNER.decide(True, (1., 4.), .1, (1., 1., 0.))
    assert decision.goal == pytest.approx((1., 2.8, math.pi / 2))


def test_close_leader_only_turns_the_robot_in_place():
    decision = PLANNER.decide(True, (0., 1.4), .1, (0., 0., 0.))
    assert decision.mode == 'holding'
    assert decision.goal == pytest.approx((0., 0., math.pi / 2))


@pytest.mark.parametrize('locked,leader,age,robot,reason', [
    (False, (5., 0.), .1, (0., 0., 0.), 'leader_not_locked'),
    (True, None, .1, (0., 0., 0.), 'leader_state_stale'),
    (True, (5., 0.), .6, (0., 0., 0.), 'leader_state_stale'),
    (True, (5., 0.), float('nan'), (0., 0., 0.), 'leader_state_stale'),
    (True, (5., 0.), .1, None, 'no_robot_pose'),
    (True, (float('inf'), 0.), .1, (0., 0., 0.), 'invalid_position'),
])
def test_every_uncertain_input_stops(locked, leader, age, robot, reason):
    decision = PLANNER.decide(locked, leader, age, robot)
    assert decision.goal is None and decision.mode == 'stopped' and decision.reason == reason


def test_settings_reject_invalid_values_and_quaternion_round_trips():
    with pytest.raises(ValueError):
        FollowSettings(follow_distance=0.)
    for yaw in (-3., -1., 0., 1.2, 3.):
        assert quaternion_to_yaw(*yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_goal_leads_a_moving_leader_and_clamps_speed():
    from ridgeback_leader_follow.follow_controller import FollowPlanner, FollowSettings
    planner = FollowPlanner(FollowSettings(follow_distance=1.2, lead_time=1.))
    decision = planner.decide(True, (3., 0.), .2, (0., 0., 0.), velocity=(.5, 0.))
    assert decision.goal == pytest.approx((3. + .5 * 1.2 - 1.2, 0., 0.))
    fast = planner.decide(True, (3., 0.), 0., (0., 0., 0.), velocity=(10., 0.))
    assert fast.goal[0] == pytest.approx(3. + 2. - 1.2)


def test_leader_velocity_is_smoothed_and_resets_on_track_change_or_gap():
    from ridgeback_leader_follow.follow_controller import LeaderVelocity
    estimate = LeaderVelocity(smoothing=1.)
    assert estimate.update(4, 0., (0., 0.)) == (0., 0.)
    assert estimate.update(4, .1, (.05, 0.)) == pytest.approx((.5, 0.))
    assert estimate.update(5, .2, (1., 0.)) == (0., 0.)
    assert estimate.update(5, 1., (2., 0.)) == (0., 0.)


def test_gazebo_world_real_time_factor_is_rewritten_once():
    from ridgeback_leader_follow.crowd_gazebo import apply_real_time_factor
    world = '<physics><real_time_factor>1</real_time_factor></physics>'
    assert apply_real_time_factor(world, .5) == '<physics><real_time_factor>0.5</real_time_factor></physics>'
    with pytest.raises(ValueError):
        apply_real_time_factor('<physics/>', .5)


def test_heading_rate_turns_toward_the_leader_and_saturates():
    from ridgeback_leader_follow.follow_controller import heading_rate
    assert heading_rate((0., 0., 0.), (2., 0.), 2., 1.) == pytest.approx(0.)
    assert heading_rate((0., 0., 0.), (2., .4), 2., 1.) == pytest.approx(2. * math.atan2(.4, 2.))
    assert heading_rate((0., 0., 0.), (0., 2.), 2., 1.) == pytest.approx(1.)
    assert heading_rate((0., 0., 3.), (-2., -.1), 2., 1.) == pytest.approx(
        max(-1., min(1., 2. * math.atan2(math.sin(math.atan2(-.1, -2.) - 3.), math.cos(math.atan2(-.1, -2.) - 3.)))))


def test_gazebo_drive_localization_takes_yaw_from_the_imu_only():
    from ridgeback_leader_follow.crowd_gazebo import drive_localization
    source = {'platform': {'extras': {'ros_parameters': {}}}}
    ekf = drive_localization(source)['platform']['extras']['ros_parameters']['ekf_node']
    assert source == {'platform': {'extras': {'ros_parameters': {}}}}
    assert ekf['odom0_config'][5] is False and ekf['odom0_config'][11] is False
    assert ekf['odom0_config'][6] and ekf['odom0_config'][7] and ekf['imu0_config'][11]
    with pytest.raises(ValueError):
        drive_localization({'platform': {'extras': {'ros_parameters': {'ekf_node': {}}}}})


def test_pace_matches_the_leader_and_closes_the_gap():
    from ridgeback_leader_follow.follow_controller import pace_speed
    # At the follow distance the robot walks at the leader's pace.
    assert pace_speed(1.2, 1., 1.2, .8, 1.3) == pytest.approx(1.)
    # Lagging 1 m behind a walking leader: faster, capped at the platform limit.
    assert pace_speed(2.2, 1., 1.2, .8, 1.3) == pytest.approx(1.3)
    # Leader stopped 0.5 m beyond the follow distance: a slow approach.
    assert pace_speed(1.7, 0., 1.2, .8, 1.3) == pytest.approx(.4)
    # Too close or invalid input: no translation.
    assert pace_speed(.9, 0., 1.2, .8, 1.3) == 0.
    assert pace_speed(math.nan, 1., 1.2, .8, 1.3) == 0.
    # Leader turned and walks back toward the robot from 2 m: stop, do not speed up.
    assert pace_speed(2., -1.3, 1.2, .8, 1.3) == 0.


def test_receding_speed_is_the_component_away_from_the_robot():
    from ridgeback_leader_follow.follow_controller import receding_speed
    assert receding_speed((0., 0.), (2., 0.), (1., 0.)) == pytest.approx(1.)
    assert receding_speed((0., 0.), (2., 0.), (-1.3, 0.)) == pytest.approx(-1.3)
    assert receding_speed((0., 0.), (2., 0.), (0., 1.)) == pytest.approx(0.)


def test_speed_limit_keeps_nav2_direction():
    from ridgeback_leader_follow.follow_controller import limit_speed
    vx, vy = limit_speed(.6, .8, .5)
    assert math.hypot(vx, vy) == pytest.approx(.5)
    assert math.atan2(vy, vx) == pytest.approx(math.atan2(.8, .6))
    assert limit_speed(.3, 0., .5) == (.3, 0.)
