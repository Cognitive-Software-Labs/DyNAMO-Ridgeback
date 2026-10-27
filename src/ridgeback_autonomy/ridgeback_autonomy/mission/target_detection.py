"""Validated target-detection input for the future parking mission manager."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable

from ridgeback_common.stamps import stamp_to_nanoseconds
from ridgeback_interfaces.msg import TargetDetections


HUMANOID_LABEL = 'humanoid robot'
ORIGINAL_G1_DETECTION_THRESHOLD = 0.50
DETECTION_VOTE_WINDOW_SIZE = 5
DETECTION_VOTE_REQUIRED = 3


@dataclass(frozen=True)
class TargetDetectionCandidate:
    """One mission-relevant detection selected from a valid batch."""

    index: int
    label: str
    score: float
    bbox_xyxy: tuple[float, float, float, float]


@dataclass(frozen=True)
class TargetDetectionResult:
    """Outcome of validating and voting on one detector observation."""

    message_valid: bool
    positive: bool
    confirmed: bool
    selected: TargetDetectionCandidate | None
    vote_window: tuple[bool, ...]
    positive_votes: int
    localization_reset_requested: bool
    reason: str
    stamp_ns: int


class TargetDetectionAdapter:
    """Validate generic detections and preserve the original G1 vote policy.

    The original mission confirmed as soon as at least three positive results
    existed in its rolling five-result deque; it did not wait for the deque to
    fill. It also requested localization-state cleanup after five consecutive
    negative observations. This class preserves both details.

    The old search callback had no wall-age timeout. The adapter therefore does
    not invent one, but it rejects messages older than the last accepted source
    timestamp so delayed traffic cannot move the vote history backwards.
    """

    def __init__(
        self,
        *,
        target_labels: Iterable[str] = (HUMANOID_LABEL,),
        minimum_score: float = ORIGINAL_G1_DETECTION_THRESHOLD,
    ) -> None:
        labels = {
            self._normalize_label(label)
            for label in target_labels
            if self._normalize_label(label)
        }
        if not labels:
            raise ValueError('target_labels must contain at least one label')
        if not math.isfinite(minimum_score) or not 0.0 <= minimum_score <= 1.0:
            raise ValueError('minimum_score must be finite and within [0, 1]')

        self.target_labels = frozenset(labels)
        self.minimum_score = float(minimum_score)
        self._votes: deque[bool] = deque(maxlen=DETECTION_VOTE_WINDOW_SIZE)
        self._last_stamp_ns: int | None = None

    @property
    def vote_window(self) -> tuple[bool, ...]:
        return tuple(self._votes)

    def reset(self) -> None:
        """Begin a fresh search cycle with no votes or timestamp history."""

        self._votes.clear()
        self._last_stamp_ns = None

    def observe(self, msg: TargetDetections) -> TargetDetectionResult:
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            return self._result_without_vote(
                stamp_ns=stamp_ns,
                reason='stale_timestamp',
            )

        candidates, invalid_reason = self._validated_candidates(msg)
        if invalid_reason is not None:
            return self._result_without_vote(
                stamp_ns=stamp_ns,
                reason=invalid_reason,
            )

        self._last_stamp_ns = stamp_ns
        selected = self._select_relevant_candidate(candidates)
        positive = selected is not None
        self._votes.append(positive)
        window = tuple(self._votes)
        positives = sum(window)
        localization_reset_requested = (
            len(window) == DETECTION_VOTE_WINDOW_SIZE and positives == 0
        )
        confirmed = (
            len(window) >= DETECTION_VOTE_REQUIRED
            and positives >= DETECTION_VOTE_REQUIRED
        )
        result = TargetDetectionResult(
            message_valid=True,
            positive=positive,
            confirmed=confirmed,
            selected=selected,
            vote_window=window,
            positive_votes=positives,
            localization_reset_requested=localization_reset_requested,
            reason='accepted' if positive else 'no_relevant_detection',
            stamp_ns=stamp_ns,
        )
        if confirmed:
            # The original target-found transition cleared its vote deque.
            self._votes.clear()
        return result

    def _validated_candidates(
        self,
        msg: TargetDetections,
    ) -> tuple[list[TargetDetectionCandidate], str | None]:
        count = int(msg.count)
        width = int(msg.image_width)
        height = int(msg.image_height)
        if count < 0:
            return [], 'negative_count'
        if width <= 0 or height <= 0:
            return [], 'invalid_image_size'
        if bool(msg.detected) != (count > 0):
            return [], 'inconsistent_detected_count'
        if len(msg.bbox_xyxy) != count * 4:
            return [], 'bbox_count_mismatch'
        if len(msg.labels) != count:
            return [], 'label_count_mismatch'
        if len(msg.scores) != count:
            return [], 'score_count_mismatch'

        candidates: list[TargetDetectionCandidate] = []
        for index in range(count):
            start = index * 4
            bbox = tuple(float(value) for value in msg.bbox_xyxy[start:start + 4])
            score = float(msg.scores[index])
            if not all(math.isfinite(value) for value in bbox):
                return [], 'nonfinite_bbox'
            if not math.isfinite(score) or not 0.0 <= score <= 1.0:
                return [], 'invalid_score'
            x1, y1, x2, y2 = bbox
            if not (
                0.0 <= x1 < x2 <= width
                and 0.0 <= y1 < y2 <= height
            ):
                return [], 'invalid_bbox'
            label = str(msg.labels[index]).strip()
            if not label:
                return [], 'empty_label'
            candidates.append(TargetDetectionCandidate(
                index=index,
                label=label,
                score=score,
                bbox_xyxy=bbox,
            ))
        return candidates, None

    def _select_relevant_candidate(
        self,
        candidates: list[TargetDetectionCandidate],
    ) -> TargetDetectionCandidate | None:
        relevant = (
            candidate
            for candidate in candidates
            if self._normalize_label(candidate.label) in self.target_labels
            and candidate.score >= self.minimum_score
        )
        return max(relevant, key=lambda candidate: candidate.score, default=None)

    def _result_without_vote(self, *, stamp_ns: int, reason: str) -> TargetDetectionResult:
        window = tuple(self._votes)
        return TargetDetectionResult(
            message_valid=False,
            positive=False,
            confirmed=False,
            selected=None,
            vote_window=window,
            positive_votes=sum(window),
            localization_reset_requested=False,
            reason=reason,
            stamp_ns=stamp_ns,
        )

    @staticmethod
    def _normalize_label(label: str) -> str:
        return str(label).strip().casefold()
