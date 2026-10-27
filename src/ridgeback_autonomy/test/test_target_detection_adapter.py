from __future__ import annotations

import pytest

from ridgeback_interfaces.msg import TargetDetections
from ridgeback_autonomy.mission.target_detection import TargetDetectionAdapter


def detection_message(
    *,
    stamp_ns: int,
    detections: list[tuple[str, float, tuple[float, float, float, float]]],
    detected: bool | None = None,
) -> TargetDetections:
    msg = TargetDetections()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = 'camera_0_color_optical_frame'
    msg.image_width = 640
    msg.image_height = 480
    msg.detected = bool(detections) if detected is None else detected
    msg.count = len(detections)
    for label, score, bbox in detections:
        msg.labels.append(label)
        msg.scores.append(score)
        msg.bbox_xyxy.extend(bbox)
    return msg


def humanoid_message(stamp_ns: int, score: float = 0.80) -> TargetDetections:
    return detection_message(
        stamp_ns=stamp_ns,
        detections=[('humanoid robot', score, (100.0, 60.0, 300.0, 460.0))],
    )


def empty_message(stamp_ns: int) -> TargetDetections:
    return detection_message(stamp_ns=stamp_ns, detections=[])


def test_valid_humanoid_detection_is_a_positive_observation() -> None:
    result = TargetDetectionAdapter().observe(humanoid_message(1))

    assert result.message_valid
    assert result.positive
    assert not result.confirmed
    assert result.selected.label == 'humanoid robot'
    assert result.selected.bbox_xyxy == (100.0, 60.0, 300.0, 460.0)
    assert result.vote_window == (True,)


def test_no_detections_is_a_valid_negative_observation() -> None:
    result = TargetDetectionAdapter().observe(empty_message(1))

    assert result.message_valid
    assert not result.positive
    assert result.reason == 'no_relevant_detection'
    assert result.vote_window == (False,)


def test_detection_below_original_threshold_is_negative() -> None:
    result = TargetDetectionAdapter().observe(humanoid_message(1, score=0.49))

    assert result.message_valid
    assert not result.positive
    assert result.selected is None


def test_multiple_detections_select_highest_scoring_relevant_humanoid() -> None:
    msg = detection_message(
        stamp_ns=1,
        detections=[
            ('person', 0.99, (10.0, 10.0, 80.0, 200.0)),
            ('humanoid robot', 0.70, (100.0, 50.0, 250.0, 450.0)),
            (' Humanoid Robot ', 0.92, (300.0, 40.0, 500.0, 470.0)),
        ],
    )

    result = TargetDetectionAdapter().observe(msg)

    assert result.positive
    assert result.selected.index == 2
    assert result.selected.score == pytest.approx(0.92)


def test_original_policy_confirms_immediately_on_three_positive_observations() -> None:
    adapter = TargetDetectionAdapter()

    results = [adapter.observe(humanoid_message(stamp)) for stamp in (1, 2, 3)]

    assert [result.confirmed for result in results] == [False, False, True]
    assert results[-1].vote_window == (True, True, True)
    assert results[-1].positive_votes == 3
    assert adapter.vote_window == ()


def test_interrupted_sequence_confirms_with_three_positives_in_recent_window() -> None:
    adapter = TargetDetectionAdapter()
    messages = [humanoid_message(1), empty_message(2), humanoid_message(3), humanoid_message(4)]

    results = [adapter.observe(message) for message in messages]

    assert [result.confirmed for result in results] == [False, False, False, True]
    assert results[-1].vote_window == (True, False, True, True)
    assert results[-1].positive_votes == 3


def test_interrupted_sequence_without_three_votes_does_not_confirm() -> None:
    adapter = TargetDetectionAdapter()
    messages = [
        humanoid_message(1),
        empty_message(2),
        empty_message(3),
        humanoid_message(4),
        empty_message(5),
    ]

    result = None
    for message in messages:
        result = adapter.observe(message)

    assert not result.confirmed
    assert result.positive_votes == 2
    assert result.vote_window == (True, False, False, True, False)


def test_rolling_window_drops_old_positive_observations() -> None:
    adapter = TargetDetectionAdapter()
    messages = [
        humanoid_message(1),
        humanoid_message(2),
        empty_message(3),
        empty_message(4),
        empty_message(5),
        humanoid_message(6),
    ]

    result = None
    for message in messages:
        result = adapter.observe(message)

    assert not result.confirmed
    assert result.vote_window == (True, False, False, False, True)
    assert result.positive_votes == 2


def test_out_of_order_message_is_stale_and_does_not_change_votes() -> None:
    adapter = TargetDetectionAdapter()
    first = adapter.observe(humanoid_message(20))

    stale = adapter.observe(humanoid_message(19))

    assert first.vote_window == (True,)
    assert not stale.message_valid
    assert stale.reason == 'stale_timestamp'
    assert stale.vote_window == (True,)


def test_original_search_has_no_wall_age_expiry() -> None:
    adapter = TargetDetectionAdapter()

    result = adapter.observe(humanoid_message(1))

    assert result.message_valid
    assert result.positive


def test_malformed_parallel_arrays_are_rejected_without_a_vote() -> None:
    adapter = TargetDetectionAdapter()
    msg = humanoid_message(1)
    msg.scores = []

    result = adapter.observe(msg)

    assert not result.message_valid
    assert result.reason == 'score_count_mismatch'
    assert result.vote_window == ()


def test_detected_and_count_must_be_consistent() -> None:
    adapter = TargetDetectionAdapter()
    msg = empty_message(1)
    msg.detected = True

    result = adapter.observe(msg)

    assert not result.message_valid
    assert result.reason == 'inconsistent_detected_count'


def test_five_negative_observations_request_original_localization_reset() -> None:
    adapter = TargetDetectionAdapter()

    results = [adapter.observe(empty_message(stamp)) for stamp in range(1, 6)]

    assert not any(result.localization_reset_requested for result in results[:4])
    assert results[-1].localization_reset_requested
    assert results[-1].vote_window == (False, False, False, False, False)


def test_reset_starts_a_fresh_search_cycle() -> None:
    adapter = TargetDetectionAdapter()
    adapter.observe(humanoid_message(20))

    adapter.reset()
    result = adapter.observe(humanoid_message(1))

    assert result.message_valid
    assert result.vote_window == (True,)
