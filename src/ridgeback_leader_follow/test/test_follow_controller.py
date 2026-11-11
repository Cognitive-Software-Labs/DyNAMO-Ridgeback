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
