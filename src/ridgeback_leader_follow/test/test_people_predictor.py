import math

import pytest

from ridgeback_leader_follow.people_predictor import (
    PredictionSettings, braking_travel, bystanders, clearing_ring, obstacle_points, speed_limit)

S = PredictionSettings()


def closed_loop_clearance(person, robot_speed, settings=S, step=.05, duration=5.):
    """Robot driving along +x at up to max_speed, capped by speed_limit every step."""
    x, v, nearest = 0., robot_speed, math.inf
    for k in range(int(duration / step)):
        t = k * step
        px, py = person[0] + person[2] * t, person[1] + person[3] * t
        cap = speed_limit((x, 0.), (v, 0.), [(px, py, person[2], person[3])], settings)
        target = min(settings.max_speed, cap)
        v = min(v + settings.acceleration * step, target) if target > v else max(target, v - settings.acceleration * step)
        x += v * step
        nearest = min(nearest, math.hypot(px + person[2] * step - x, py + person[3] * step))
    return nearest


def test_braking_travel_respects_the_acceleration_limit():
    travel, speeds = braking_travel(1., 0., [0., .5, 1., 2.], 1.)
    assert list(travel) == pytest.approx([0., .375, .5, .5])
    assert list(speeds) == pytest.approx([1., .5, 0., 0.])
    assert list(braking_travel(0., 1., [2.], 1.)[0]) == pytest.approx([1.5])


def test_speed_limit_brakes_only_when_braking_widens_the_gap():
    assert speed_limit((0., 0.), (.9, 0.), [], S) == S.max_speed
    # Passing wide (1.5 m): no slowing.
    assert speed_limit((0., 0.), (.9, 0.), [(4., 1.5, -1., 0.)], S) == S.max_speed
    # Head-on and unavoidable: brake as hard as the platform can (0.2 m/s in this step).
    assert speed_limit((0., 0.), (.9, 0.), [(1., .1, -1., 0.)], S) <= .7
    # Crossing just ahead of the robot: brake.
    assert speed_limit((0., 0.), (1.1, 0.), [(2.2, 2.7, 0., -1.35)], S) < .8
    # Crossing just behind where the robot will be: braking would meet them, so no cap.
    assert speed_limit((0., 0.), (1.1, 0.), [(1.2, 2.7, 0., -1.35)], S) >= 1.1
    # Nearly still: the heading gives the direction of travel.
    assert speed_limit((0., 0.), (0., 0.), [(1.2, 1.35, 0., -1.35)], S, heading=0.) < .5
    assert speed_limit((0., 0.), (0., 0.), [(1.2, 1.35, 0., -1.35)], S, heading=math.pi) == S.max_speed


@pytest.mark.parametrize('crossing_x', [1.2, 1.95, 2.2, 3.2])
def test_fast_side_crosser_is_not_touched_in_closed_loop(crossing_x):
    # The v7 Gazebo crowd near miss: robot at 1.1 m/s, a walker at 1.35 m/s from the
    # side reaching the robot's line 2 s later. Without a cap the robot passes 0.28 m
    # from them at crossing_x 2.2.
    assert closed_loop_clearance((crossing_x, 2.7, 0., -1.35), 1.1) >= S.contact


def test_obstacles_follow_prediction_and_exclude_the_leader():
    people = bystanders([(3, 2., 0., 1., 0.), (7, 5., 5., 0., 0.)], leader_id=7)
    assert people == [(2., 0., 1., 0.)]
    points = obstacle_points(people, S)
    xs = [x for x, _ in points]
    assert min(xs) == pytest.approx(2. - S.body_radius) and max(xs) == pytest.approx(3. + S.body_radius)
    ring = clearing_ring(S)
    assert len(ring) == S.clear_points and all(math.hypot(*p) == pytest.approx(S.clear_radius) for p in ring)


def test_settings_validate():
    with pytest.raises(ValueError):
        PredictionSettings(contact=1.5, comfort=1.2)


def test_away_from_drops_only_people_near_the_centre():
    from ridgeback_leader_follow.people_predictor import away_from
    people=[(0.,0.,1.,0.),(.5,.5,0.,0.),(3.,0.,0.,0.)]
    assert away_from(people,(0.,0.),1.) == [(3.,0.,0.,0.)]
    assert away_from(people,None,1.) == people


def test_unavoidable_contact_is_met_slowly():
    # A walker heading straight for the robot: braking cannot avoid them, only soften it.
    capped = closed_loop_clearance((3., 0., -1.3, 0.), 1.1)
    assert capped < S.contact
    x, v, step = 0., 1.1, .05
    for k in range(200):
        t = k * step
        cap = speed_limit((x, 0.), (v, 0.), [(3. - 1.3 * t, 0., -1.3, 0.)], S)
        v = max(min(S.max_speed, cap), v - S.acceleration * step) if cap < v else min(v + S.acceleration * step, cap)
        x += v * step
        if abs(3. - 1.3 * (t + step) - x) < S.contact:
            break
    assert v < .5
