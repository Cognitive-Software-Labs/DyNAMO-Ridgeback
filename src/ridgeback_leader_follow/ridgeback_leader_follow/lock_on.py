"""Leader identity over time: lock on once, never hand the lock to someone else.

States: SEARCHING -> LOCKED -> UNCERTAIN -> LOST (see tracks.LeaderStateName).
Only LOCKED may drive motion.  Two callables decide who counts as the leader:

* ``acquire(tracks)`` proposes a leader while SEARCHING (every
  ``acquire_interval`` frames).  The proposal must then match its own
  appearance gallery for ``acquire_frames`` consecutive frames.
* ``confirm(tracks)`` independently proposes a person during recovery; the
  proposal is accepted only if it also matches the stored gallery, for
  ``acquire_frames`` consecutive frames.  Appearance alone, or the proposal
  alone, never hands the lock to a different person.

A verified original lidar track may recover after a brief observation gap
when the supplied motion-continuity check confirms it repeatedly; a visible
appearance contradiction disables that exception.

Each returns ``(track or None, reason)``.  The leader_identity node passes the
nearest person in front of the robot for both (acquisition.py).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from ridgeback_leader_follow.tracks import LeaderStateName, Track

Proposal = Tuple[Optional[Track], str]
Proposer = Callable[[List[Track]], Proposal]


def cosine_similarity(first: np.ndarray, second: np.ndarray) -> float:
    from ridgeback_leader_follow.appearance import normalize_embedding
    first, second = normalize_embedding(first), normalize_embedding(second)
    if first is None or second is None or first.shape != second.shape:
        return -1.0
    return float(np.clip(np.dot(first, second), -1., 1.))


@dataclass(frozen=True)
class LockUpdate:
    selected: Optional[Track]     # leader, or the candidate being confirmed / last seen
    similarity: Optional[float]         # selected's best match against the gallery

    def valid(self, state: LeaderStateName) -> bool:
        return state == "LOCKED" and self.selected is not None


class LeaderLock:
    def __init__(
        self,
        acquire_interval: int = 8,
        acquire_frames: int = 3,
        lost_frames: int = 8,
        match_threshold: float = 0.72,
        gallery_update_threshold: float = 0.85,
        gallery_size: int = 8,
        recovery_margin: float = 0.05,
        min_update_score: float = 0.50,
        continuity_overrides_appearance: bool = False,
    ) -> None:
        self.acquire_interval = max(1, int(acquire_interval))
        self.acquire_frames = max(1, int(acquire_frames))
        self.lost_frames = max(2, int(lost_frames))
        self.match_threshold = float(match_threshold)
        self.gallery_update_threshold = float(gallery_update_threshold)
        self.gallery_size = max(1, int(gallery_size))
        self.recovery_margin = float(recovery_margin)
        self.min_update_score = float(min_update_score)
        # When true, a visible appearance mismatch does not drop a lock that the
        # continuity check still confirms (same track, consistent motion, no
        # competing person): the leader turning to face the camera changes
        # appearance without changing identity. The gallery is not updated.
        self.continuity_overrides_appearance = bool(continuity_overrides_appearance)
        self.reset()

    def reset(self) -> None:
        """Forget the leader; the next frame starts a fresh search."""
        self.state: LeaderStateName = "SEARCHING"
        self.reason = "waiting_for_candidate"
        self.frame_index = 0
        self.target_track_id: Optional[int] = None
        self.pending_track_id: Optional[int] = None
        self.pending_frames = 0
        self.missing_frames = 0
        self.recovery_track_id: Optional[int] = None
        self.recovery_frames = 0
        self.gallery: List[np.ndarray] = []
        self.continuity_recovery_allowed = False

    def similarity(self, feature: np.ndarray) -> Optional[float]:
        if not self.gallery:
            return None
        return max(cosine_similarity(feature, reference) for reference in self.gallery)

    def update(self, tracks: List[Track], acquire: Proposer,
               confirm: Optional[Proposer] = None,
               continuity: Optional[Callable[[Track], bool]] = None,
               recovery_filter: Optional[Callable[[List[Track]], List[Track]]] = None) -> LockUpdate:
        confirm = confirm or acquire
        self.frame_index += 1
        selected = find_track(tracks, self.target_track_id)
        similarity = self.similarity(selected.feature) if selected is not None else None

        if self.state == "LOCKED":
            if selected is not None and continuity is not None and not continuity(selected):
                self._begin_uncertain("track_discontinuity")
            elif (selected is not None and np.asarray(selected.feature).size == 0
                    and continuity is not None and continuity(selected)):
                self.missing_frames = 0
                self.reason = "identity_held_by_track_continuity"
            elif (selected is not None and self.continuity_overrides_appearance and continuity is not None
                    and (similarity is None or similarity < self.match_threshold)):
                # Continuity was confirmed by the first branch's check.
                self.missing_frames = 0
                self.reason = "appearance_mismatch_held_by_track_continuity"
            elif selected is None or similarity is None or similarity < self.match_threshold:
                self._begin_uncertain("target_missing" if selected is None else "appearance_mismatch")
            else:
                self.missing_frames = 0
                self.reason = "identity_verified"
                self._update_gallery(selected)
        elif self.state in ("UNCERTAIN", "LOST"):
            if (selected is not None and np.asarray(selected.feature).size > 0
                    and (similarity is None or similarity < self.match_threshold)):
                self.continuity_recovery_allowed = False
            candidate, candidate_reason = self._recovery_candidate(tracks, selected, similarity, confirm, continuity, recovery_filter)
            if candidate is None:
                self.recovery_track_id = None
                self.recovery_frames = 0
                self.missing_frames += 1
                if self.missing_frames >= self.lost_frames:
                    self.state = "LOST"
                self.reason = candidate_reason
            else:
                if candidate.track_id == self.recovery_track_id:
                    self.recovery_frames += 1
                else:
                    self.recovery_track_id = candidate.track_id
                    self.recovery_frames = 1
                selected = candidate
                similarity = self.similarity(candidate.feature)
                self.reason = "verifying_recovery"
                if self.recovery_frames >= self.acquire_frames:
                    self.target_track_id = candidate.track_id
                    self.state = "LOCKED"
                    self.reason = "identity_recovered"
                    self.missing_frames = 0
                    self.recovery_track_id = None
                    self.recovery_frames = 0
        else:
            pending = find_track(tracks, self.pending_track_id)
            pending_similarity = self.similarity(pending.feature) if pending is not None else None
            if pending is not None and pending_similarity is not None and pending_similarity >= self.match_threshold:
                self.pending_frames += 1
                selected = pending
                similarity = pending_similarity
                self.reason = "confirming_stable_identity"
                if self.pending_frames >= self.acquire_frames:
                    self.target_track_id = pending.track_id
                    self.pending_track_id = None
                    self.pending_frames = 0
                    self.state = "LOCKED"
                    self.reason = "identity_verified"
            else:
                if self.pending_track_id is not None:
                    # Confirmation requires consecutive, consistent observations.
                    self.pending_track_id = None
                    self.pending_frames = 0
                    self.gallery = []
                    self.reason = "confirmation_interrupted"
                if self.frame_index == 1 or self.frame_index % self.acquire_interval == 0:
                    proposed, proposal_reason = acquire(tracks)
                    if proposed is not None:
                        self.pending_track_id = proposed.track_id
                        self.pending_frames = 1
                        self.gallery = [np.asarray(proposed.feature, dtype=np.float32).copy()]
                        self.reason = "confirming_stable_identity"
                        selected = proposed
                        similarity = 1.0
                    else:
                        self.reason = proposal_reason
        return LockUpdate(selected, similarity)

    def _begin_uncertain(self, reason: str) -> None:
        self.state = "UNCERTAIN"
        self.continuity_recovery_allowed = reason == "target_missing"
        self.reason = reason
        self.missing_frames = 1
        self.recovery_track_id = None
        self.recovery_frames = 0

    def _recovery_candidate(self, tracks: List[Track], original: Optional[Track],
                            original_similarity: Optional[float], confirm: Proposer,
                            continuity: Optional[Callable[[Track], bool]] = None,
                            recovery_filter: Optional[Callable[[List[Track]], List[Track]]] = None) -> Proposal:
        """Require a separated gallery match and an independent recovery proposal."""
        from ridgeback_leader_follow.appearance import match_gallery
        if (self.continuity_recovery_allowed and original is not None
                and np.asarray(original.feature).size == 0
                and continuity is not None and continuity(original)):
            return original, "original_lidar_track_reappeared"
        if not tracks:
            return None, "waiting_for_original_identity"
        plausible = recovery_filter(tracks) if recovery_filter is not None else tracks
        match = match_gallery(self.gallery, {t.track_id: t.feature for t in plausible},
                              threshold=self.match_threshold, minimum_margin=self.recovery_margin)
        if match.reason == 'ambiguous':
            return None, "ambiguous_appearance_recovery"
        proposed, proposal_reason = confirm(tracks)
        if proposed is None:
            return None, f"recovery_unconfirmed:{proposal_reason}"
        similarity = self.similarity(proposed.feature)
        if similarity is None or similarity < self.match_threshold or proposed.track_id != match.track_id:
            return None, "candidate_appearance_differs"
        return proposed, "appearance_and_candidate_match"

    def _update_gallery(self, track: Track) -> None:
        similarity = self.similarity(track.feature)
        if similarity is not None and similarity >= self.gallery_update_threshold and track.score >= self.min_update_score:
            self.gallery.append(np.asarray(track.feature, dtype=np.float32).copy())
            self.gallery = self.gallery[-self.gallery_size:]


def find_track(tracks: Sequence[Track], track_id: Optional[int]) -> Optional[Track]:
    if track_id is None:
        return None
    return next((track for track in tracks if track.track_id == int(track_id)), None)
