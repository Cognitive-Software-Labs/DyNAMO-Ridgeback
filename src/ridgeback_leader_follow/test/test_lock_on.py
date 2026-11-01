import numpy as np

from ridgeback_leader_follow.lock_on import LeaderLock
from ridgeback_leader_follow.tracks import Track

LEADER = np.array([1.0, 0.0, 0.0], np.float32)
OTHER = np.array([0.0, 1.0, 0.0], np.float32)


def person(track_id, feature, score=0.9):
    return Track(track_id, score, feature)


def propose(track_id):
    return lambda tracks: (next((t for t in tracks if t.track_id == track_id), None), 'proposed')


def nobody(_tracks):
    return None, 'nobody_proposed'


def lock_on(lock, frames=3):
    for _ in range(frames):
        update = lock.update([person(1, LEADER), person(2, OTHER)], propose(1))
    return update


def test_locks_after_consecutive_confirmations():
    lock = LeaderLock(acquire_frames=3)
    lock.update([person(1, LEADER)], propose(1))
    assert (lock.state, lock.reason) == ('SEARCHING', 'confirming_stable_identity')
    update = lock_on(lock, frames=2)
    assert lock.state == 'LOCKED'
    assert update.valid(lock.state)
    assert update.selected.track_id == 1


def test_missing_leader_is_never_replaced_by_a_different_person():
    lock = LeaderLock(acquire_frames=3, lost_frames=4)
    lock_on(lock)
    for _ in range(10):
        # The other person is proposed, but does not match the leader's gallery.
        assert not lock.update([person(2, OTHER)], propose(2)).valid(lock.state)
    assert (lock.state, lock.reason) == ('LOST', 'candidate_appearance_differs')


def test_leader_under_a_new_track_id_is_recovered_after_confirmation():
    lock = LeaderLock(acquire_frames=3)
    lock_on(lock)
    lock.update([], nobody)
    assert (lock.state, lock.reason) == ('UNCERTAIN', 'target_missing')
    for _ in range(3):
        update = lock.update([person(7, LEADER), person(2, OTHER)], propose(7))
    assert (lock.state, lock.reason, update.selected.track_id) == ('LOCKED', 'identity_recovered', 7)


def test_appearance_change_drops_the_lock():
    lock = LeaderLock()
    lock_on(lock)
    lock.update([person(1, OTHER)], nobody)
    assert (lock.state, lock.reason) == ('UNCERTAIN', 'appearance_mismatch')


def test_gallery_is_bounded():
    lock = LeaderLock(gallery_size=4)
    lock_on(lock)
    for _ in range(20):
        lock.update([person(1, LEADER)], nobody)
    assert len(lock.gallery) == 4


def test_person_without_appearance_is_never_locked():
    lock = LeaderLock(acquire_frames=1)
    for _ in range(5):
        lock.update([person(1, np.empty(0, np.float32))], propose(1))
    assert lock.state == 'SEARCHING'
