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


def test_lidar_continuity_and_motion_gated_recovery():
    identity = LeaderIdentity(LeaderLock(acquire_frames=3), 2., math.radians(30))
    def update(at, *entries):
        scene = people(*entries)
        scene.header.stamp.sec = int(at)
        scene.header.stamp.nanosec = round((at-int(at))*1e9)
        return identity.update(scene, IDENTITY)
    for at in (0., .05, .1):
        out = update(at, (1, 1.2, 0., [1.,0.]))
    assert out.state == LeaderState.LOCKED
    out = update(.15, (1, 1.2, 0., []))
    assert out.state == LeaderState.LOCKED
    assert out.reason == 'identity_held_by_track_continuity'
    update(.2)
    for at in (.25,.3,.35):
        out = update(at, (7, 1.2, 0., [1.,0.]), (8, 5., 0., [0.,1.]))
    assert out.state == LeaderState.LOCKED and out.track_id == 7
    out = update(.4, (7, 5., 0., [1.,0.]))
    assert out.state == LeaderState.UNCERTAIN
    for at in (.45,.5,.55):
        out = update(at, (7, 5., 0., [1.,0.]))
    assert out.state != LeaderState.LOCKED


def test_crossing_with_two_equally_plausible_motion_tracks_becomes_uncertain():
    identity = LeaderIdentity(LeaderLock(acquire_frames=3), 2., math.radians(30))
    for index in range(3):
        msg = people((1, 1.2, 0., [1.,0.]))
        msg.header.stamp.nanosec = index*50000000
        identity.update(msg, IDENTITY)
    msg = people((1, 1.2, .02, []), (2, 1.2, -.02, [1.,0.]))
    msg.header.stamp.nanosec = 150000000
    out = identity.update(msg, IDENTITY)
    assert out.state == LeaderState.UNCERTAIN
    assert out.reason == 'track_discontinuity'


def test_brief_lidar_gap_can_recover_only_original_track_without_appearance():
    identity = LeaderIdentity(LeaderLock(acquire_frames=3), 2., math.radians(30))
    def update(at, *entries):
        msg=people(*entries);msg.header.stamp.nanosec=round(at*1e9)
        return identity.update(msg, IDENTITY)
    for at in (0.,.05,.1):update(at,(1,1.2,0.,[1.,0.]))
    update(.15)
    for at in (.2,.25,.3):out=update(at,(1,1.2,0.,[]))
    assert out.state==LeaderState.LOCKED
    update(.35,(1,1.2,0.,[0.,1.]))
    for at in (.4,.45,.5):out=update(at,(1,1.2,0.,[]))
    assert out.state!=LeaderState.LOCKED


def test_motion_separates_identical_appearances_during_recovery():
    identity=LeaderIdentity(LeaderLock(acquire_frames=3),2.,math.radians(30))
    def update(at,*entries):
        msg=people(*entries);msg.header.stamp.nanosec=round(at*1e9)
        return identity.update(msg,IDENTITY)
    for at in (0.,.05,.1):update(at,(1,1.2,0.,[1.,0.]))
    update(.15)
    for at in (.2,.25,.3):
        out=update(at,(7,1.2,0.,[1.,0.]),(8,1.2,.8,[1.,0.]))
    assert out.state==LeaderState.LOCKED and out.track_id==7


def test_known_bystander_track_is_not_promoted_after_leader_loss():
    identity=LeaderIdentity(LeaderLock(acquire_frames=3),3.,math.radians(30))
    def update(at,*entries):
        msg=people(*entries);msg.header.stamp.nanosec=round(at*1e9)
        return identity.update(msg,IDENTITY)
    for at in (0.,.05,.1):update(at,(1,1.2,0.,[1.,0.]),(2,2.,0.,[1.,0.]))
    update(.15)
    for at in (.2,.25,.3):out=update(at,(2,1.2,0.,[1.,0.]))
    assert out.state!=LeaderState.LOCKED
    for at in (.35,.4,.45):out=update(at,(7,1.2,0.,[1.,0.]),(2,2.,0.,[1.,0.]))
    assert out.state==LeaderState.LOCKED and out.track_id==7


def test_distinct_appearance_can_verify_leader_after_tracker_id_swap():
    identity=LeaderIdentity(LeaderLock(acquire_frames=3),3.,math.radians(30))
    def update(at,*entries):
        msg=people(*entries);msg.header.stamp.nanosec=round(at*1e9)
        return identity.update(msg,IDENTITY)
    for at in (0.,.05,.1):update(at,(1,1.2,0.,[1.,0.]),(2,2.,0.,[0.,1.]))
    update(.15)
    for at in (.2,.25,.3):out=update(at,(2,1.2,0.,[1.,0.]))
    assert out.state==LeaderState.LOCKED and out.track_id==2
    assert 2 not in identity.nonleader_ids


def test_ros_boundary_waits_for_exact_tf_without_blocking_or_processing_early():
    from collections import deque
    from types import SimpleNamespace
    from rclpy.time import Time
    from ridgeback_leader_follow.leader_identity_node import LeaderIdentityNode
    msg=people((1,1.2,0.,[1.,0.]));msg.header.stamp.nanosec=100000000
    published=[];calls=[];ready=[False]
    node=SimpleNamespace(pending=deque(),last_received_stamp=-float('inf'),tf_wait_timeout=.25,
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=.2)),
        to_base=lambda m:IDENTITY if ready[0] else None,
        identity=SimpleNamespace(update=lambda m,t:calls.append((m,t)) or SimpleNamespace(reason='ok')),
        publisher=SimpleNamespace(publish=published.append))
    LeaderIdentityNode.on_people(node,msg)
    LeaderIdentityNode.process_pending(node)
    assert not published and len(node.pending)==1
    ready[0]=True
    LeaderIdentityNode.process_pending(node)
    assert len(published)==1 and calls[0]==(msg,IDENTITY) and not node.pending


def test_ros_boundary_expires_missing_tf_with_empty_observation():
    from collections import deque
    from types import SimpleNamespace
    from rclpy.time import Time
    from ridgeback_leader_follow.leader_identity_node import LeaderIdentityNode
    msg=people((1,1.2,0.,[1.,0.]));published=[];observed=[]
    node=SimpleNamespace(pending=deque([msg]),tf_wait_timeout=.25,
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=1)),to_base=lambda m:None,
        identity=SimpleNamespace(update=lambda m,t:observed.append(m) or SimpleNamespace(reason='old')),
        publisher=SimpleNamespace(publish=published.append))
    LeaderIdentityNode.process_pending(node)
    assert not observed[0].people and published[0].reason=='no_base_transform'


def test_f3_acquisition_tie_is_deterministic_and_prefers_centre_before_side():
    a,b,c=(Track(i,1.,None) for i in (1,2,3))
    left=(b,1.5,.65);right=(a,1.6,-.65)
    for candidates in ([left,right],[right,left]):
        selected,_=nearest_in_front(candidates,distance_tolerance=.25,lateral_tolerance=.1)
        assert selected.track_id==1
    selected,_=nearest_in_front([(a,1.5,-.8),(c,1.5,0.)],distance_tolerance=.25,lateral_tolerance=.1)
    assert selected.track_id==3
    assert nearest_in_front([left,right])[0].track_id==2
