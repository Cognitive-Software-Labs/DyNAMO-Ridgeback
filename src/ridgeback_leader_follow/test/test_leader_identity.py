import math

from geometry_msgs.msg import Transform
from leader_follow_interfaces.msg import LeaderState, People, Person
from ridgeback_leader_follow.acquisition import nearest_in_front
from ridgeback_leader_follow.leader_identity_node import LeaderIdentity, apply_planar, planar_transform
from ridgeback_leader_follow.lock_on import LeaderLock
from ridgeback_leader_follow.tracks import Track

IDENTITY = (0.0, 0.0, 0.0)


def people(*entries):
    msg = People()
    msg.header.frame_id = 'odom'
    for track_id, x, y, appearance in entries:
        person = Person(id=track_id, confidence=0.9, appearance=appearance)
        person.position.x, person.position.y = x, y
        msg.people.append(person)
    return msg


def test_nearest_in_front_ignores_people_behind_and_to_the_side():
    near, far, behind, side = (Track(i, 1.0, None) for i in range(4))
    chosen, reason = nearest_in_front([(far, 1.8, 0.0), (behind, -0.5, 0.0), (side, 0.3, 1.0), (near, 1.0, 0.2)])
    assert (chosen, reason) == (near, 'nearest_in_front')
    assert nearest_in_front([(behind, -0.5, 0.0)]) == (None, 'nobody_in_front')


def test_planar_transform_round_trip():
    transform = Transform()
    transform.translation.x, transform.translation.y = 1.0, 2.0
    transform.rotation.z, transform.rotation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)  # yaw 90°
    frame = planar_transform(transform)
    x, y = apply_planar(1.0, 0.0, frame)
    assert math.isclose(x, 1.0, abs_tol=1e-9) and math.isclose(y, 3.0, abs_tol=1e-9)


def test_leader_in_front_is_locked_and_published():
    identity = LeaderIdentity(LeaderLock(acquire_frames=3), 2.0, math.radians(30.0))
    scene = people((4, 1.2, 0.1, [1.0, 0.0]), (5, 0.8, 1.5, [0.0, 1.0]))
    for _ in range(3):
        out = identity.update(scene, IDENTITY)
    assert (out.state, out.reason, out.has_leader, out.track_id) == (LeaderState.LOCKED, 'identity_verified', True, 4)
    assert out.header.frame_id == 'odom' and math.isclose(out.position.x, 1.2, abs_tol=1e-6)


def test_without_a_base_transform_nobody_is_acquired():
    identity = LeaderIdentity(LeaderLock(), 2.0, math.radians(30.0))
    out = identity.update(people((4, 1.2, 0.1, [1.0, 0.0])), None)
    assert (out.state, out.reason, out.has_leader) == (LeaderState.SEARCHING, 'no_base_transform', False)
