from __future__ import annotations

import math

import pytest
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

from ridgeback_common.miss_reason import MissReason
from ridgeback_interfaces.msg import TargetMeasurements
from ridgeback_autonomy.mission.target_localization import (
    GENERIC_TARGET_REFERENCE,
    TargetAssociation,
    TargetLocalizationAdapter,
)


BBOX = (100.0, 50.0, 300.0, 450.0)


class FakeTfBuffer:
    def __init__(
        self,
        *,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        failures: int = 0,
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.failures = failures
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, lookup_time):
        self.calls.append((target_frame, source_frame, lookup_time.nanoseconds))
        if self.failures:
            self.failures -= 1
            raise TransformException('transform not available')
        transform = TransformStamped()
        transform.header.frame_id = target_frame
        transform.child_frame_id = source_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        return transform


def measurement_message(
    *,
    stamp_ns: int,
    detections: list[tuple[str, float, tuple[float, float, float, float]]]
    | None = None,
    source_values: dict[str, list[tuple[float, float, float, int | None]]]
    | None = None,
) -> TargetMeasurements:
    detections = detections if detections is not None else [
        ('humanoid robot', 0.80, BBOX),
    ]
    source_values = source_values or {}
    msg = TargetMeasurements()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    # This is the source RGB frame, not the coordinate frame of the planar
    # measurement. The adapter deliberately does not use it as a TF frame.
    msg.header.frame_id = 'camera_0_color_optical_frame'
    msg.image_width = 640
    msg.image_height = 480
    msg.detected = bool(detections)
    msg.count = len(detections)
    for label, score, bbox in detections:
        msg.labels.append(label)
        msg.scores.append(score)
        msg.bbox_xyxy.extend(bbox)

    for source in (
        'pointcloud',
        'projective_ranging',
        'euclidean_reconstruction',
        'polar_profiling',
    ):
        values = source_values.get(source, [])
        lateral = getattr(msg, f'{source}_lateral_m')
        forward = getattr(msg, f'{source}_forward_m')
        distance = getattr(msg, f'{source}_distance_m')
        status = getattr(msg, f'{source}_status', None)
        for index in range(len(detections)):
            if index < len(values):
                value_lateral, value_forward, value_distance, value_status = values[index]
            else:
                value_lateral = value_forward = value_distance = float('nan')
                value_status = int(MissReason.UNSET)
            lateral.append(value_lateral)
            forward.append(value_forward)
            distance.append(value_distance)
            if status is not None:
                status.append(
                    int(MissReason.UNSET)
                    if value_status is None
                    else int(value_status)
                )
    return msg


def association(stamp_ns: int, *, label: str = 'humanoid robot', bbox=BBOX):
    return TargetAssociation(stamp_ns=stamp_ns, label=label, bbox_xyxy=bbox)


def adapter(tf_buffer=None, **kwargs) -> TargetLocalizationAdapter:
    return TargetLocalizationAdapter(
        tf_buffer or FakeTfBuffer(),
        map_frame='map',
        base_frame='r100_0001/robot/base_link',
        **kwargs,
    )


def euclidean_values(lateral=0.2, forward=2.0, status=MissReason.OK):
    return {
        'euclidean_reconstruction': [
            (lateral, forward, math.hypot(lateral, forward), int(status)),
        ],
    }


def observe(
    localizer: TargetLocalizationAdapter,
    msg: TargetMeasurements,
    *,
    now_ns: int | None = None,
    target: TargetAssociation | None = None,
):
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    return localizer.observe(
        msg,
        association=target or association(stamp_ns),
        now_ns=stamp_ns if now_ns is None else now_ns,
    )


def test_valid_euclidean_measurement_removes_front_offset_once() -> None:
    tf_buffer = FakeTfBuffer(x=10.0, y=-3.0)
    localizer = adapter(tf_buffer)
    msg = measurement_message(stamp_ns=1_000, source_values=euclidean_values())

    result = observe(localizer, msg)

    assert result.message_valid
    assert result.accepted
    assert result.observation.source == 'euclidean_reconstruction'
    assert result.observation.front_forward_m == pytest.approx(2.0)
    assert result.observation.base_x_m == pytest.approx(2.25)
    assert result.observation.base_y_m == pytest.approx(0.2)
    assert result.observation.map_x_m == pytest.approx(12.25)
    assert result.observation.map_y_m == pytest.approx(-2.8)
    assert tf_buffer.calls == [('map', 'r100_0001/robot/base_link', 1_000)]


def test_projective_is_deterministic_fallback_when_primary_status_is_not_ok() -> None:
    msg = measurement_message(
        stamp_ns=2_000,
        source_values={
            'euclidean_reconstruction': [
                (0.1, 2.0, 2.0, int(MissReason.TOO_FEW_VALID_POINTS)),
            ],
            'projective_ranging': [(0.3, 2.2, 2.3, int(MissReason.OK))],
        },
    )

    result = observe(adapter(), msg)

    assert result.accepted
    assert result.observation.source == 'projective_ranging'
    assert result.source_failures == (
        ('euclidean_reconstruction', 'estimator_status_31'),
    )


def test_mask_estimator_requires_explicit_ok_status() -> None:
    msg = measurement_message(
        stamp_ns=3_000,
        source_values={
            'euclidean_reconstruction': [(0.1, 2.0, 2.0, int(MissReason.UNSET))],
        },
    )

    result = observe(adapter(source_order=('euclidean_reconstruction',)), msg)

    assert result.message_valid
    assert not result.accepted
    assert result.reason == 'no_valid_localization_source'
    assert result.source_failures == (
        ('euclidean_reconstruction', 'estimator_status_255'),
    )


def test_pointcloud_has_no_status_requirement() -> None:
    msg = measurement_message(
        stamp_ns=4_000,
        source_values={'pointcloud': [(0.1, 1.5, 1.51, None)]},
    )

    result = observe(adapter(source_order=('pointcloud',)), msg)

    assert result.accepted
    assert result.observation.source == 'pointcloud'


@pytest.mark.parametrize('bad_value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_estimator_values_are_rejected(bad_value: float) -> None:
    msg = measurement_message(
        stamp_ns=5_000,
        source_values={
            'euclidean_reconstruction': [
                (0.1, bad_value, 2.0, int(MissReason.OK)),
            ],
        },
    )

    result = observe(adapter(source_order=('euclidean_reconstruction',)), msg)

    assert not result.accepted
    assert result.source_failures == (
        ('euclidean_reconstruction', 'nonfinite_measurement'),
    )


def test_missing_measurement_array_entry_is_rejected() -> None:
    msg = measurement_message(stamp_ns=6_000, source_values=euclidean_values())
    msg.euclidean_reconstruction_forward_m = []

    result = observe(adapter(source_order=('euclidean_reconstruction',)), msg)

    assert not result.accepted
    assert result.source_failures == (
        ('euclidean_reconstruction', 'missing_measurement_field'),
    )


@pytest.mark.parametrize(
    ('yaw', 'expected_x', 'expected_y'),
    [
        (0.0, 3.25, 2.5),
        (math.pi / 2.0, 0.5, 4.25),
        (math.pi, -1.25, 1.5),
    ],
)
def test_map_transform_at_multiple_robot_orientations(
    yaw: float,
    expected_x: float,
    expected_y: float,
) -> None:
    localizer = adapter(FakeTfBuffer(x=1.0, y=2.0, yaw=yaw))
    msg = measurement_message(
        stamp_ns=7_000,
        source_values=euclidean_values(lateral=0.5, forward=2.0),
    )

    result = observe(localizer, msg)

    assert result.observation.map_x_m == pytest.approx(expected_x)
    assert result.observation.map_y_m == pytest.approx(expected_y)


def test_missing_tf_rejects_without_falling_back_to_latest_and_can_retry() -> None:
    tf_buffer = FakeTfBuffer(failures=1)
    localizer = adapter(tf_buffer)
    msg = measurement_message(stamp_ns=8_000, source_values=euclidean_values())

    first = observe(localizer, msg)
    second = observe(localizer, msg)

    assert not first.accepted
    assert first.reason == 'transform_unavailable'
    assert second.accepted
    assert [call[2] for call in tf_buffer.calls] == [8_000, 8_000]


def test_stale_and_future_messages_are_rejected_before_tf() -> None:
    tf_buffer = FakeTfBuffer()
    localizer = adapter(tf_buffer, max_measurement_age_s=1.0)
    stale = measurement_message(stamp_ns=1_000_000_000, source_values=euclidean_values())
    future = measurement_message(stamp_ns=4_000_000_000, source_values=euclidean_values())

    stale_result = observe(localizer, stale, now_ns=2_000_000_001)
    future_result = observe(localizer, future, now_ns=3_999_999_999)

    assert stale_result.reason == 'stale_timestamp'
    assert future_result.reason == 'future_timestamp'
    assert tf_buffer.calls == []


def test_measurement_must_match_detection_stamp_label_and_bbox() -> None:
    msg = measurement_message(stamp_ns=9_000, source_values=euclidean_values())
    localizer = adapter()

    wrong_stamp = observe(localizer, msg, target=association(8_999))
    wrong_label = observe(localizer, msg, target=association(9_000, label='person'))
    wrong_bbox = observe(
        localizer,
        msg,
        target=association(9_000, bbox=(101.0, 50.0, 300.0, 450.0)),
    )

    assert wrong_stamp.reason == 'association_timestamp_mismatch'
    assert wrong_label.reason == 'target_not_associated'
    assert wrong_bbox.reason == 'target_not_associated'


def test_multiple_detections_use_bbox_identity_instead_of_array_position() -> None:
    other_bbox = (320.0, 40.0, 500.0, 460.0)
    msg = measurement_message(
        stamp_ns=10_000,
        detections=[
            ('humanoid robot', 0.95, other_bbox),
            ('humanoid robot', 0.80, BBOX),
        ],
        source_values={
            'euclidean_reconstruction': [
                (1.0, 5.0, 5.1, int(MissReason.OK)),
                (0.2, 2.0, 2.1, int(MissReason.OK)),
            ],
        },
    )

    result = observe(adapter(), msg)

    assert result.accepted
    assert result.observation.target_index == 1
    assert result.observation.front_forward_m == pytest.approx(2.0)


def test_five_map_frame_samples_stabilize_with_median() -> None:
    localizer = adapter()
    results = []
    for stamp, forward in enumerate((2.0, 2.1, 1.9, 2.05, 2.0), start=1):
        msg = measurement_message(
            stamp_ns=stamp,
            source_values=euclidean_values(lateral=0.2, forward=forward),
        )
        results.append(observe(localizer, msg))

    assert [result.stabilized is not None for result in results] == [
        False,
        False,
        False,
        False,
        True,
    ]
    stable = results[-1].stabilized
    assert stable.source == 'euclidean_reconstruction'
    assert stable.map_x_m == pytest.approx(2.25)
    assert stable.map_y_m == pytest.approx(0.2)
    assert stable.sample_count == 5


def test_outlier_is_rejected_and_does_not_count_toward_stabilization() -> None:
    localizer = adapter()
    for stamp in range(1, 3):
        observe(
            localizer,
            measurement_message(stamp_ns=stamp, source_values=euclidean_values()),
        )
    outlier = observe(
        localizer,
        measurement_message(
            stamp_ns=3,
            source_values=euclidean_values(forward=4.0),
        ),
    )

    assert not outlier.accepted
    assert outlier.reason == 'sample_jump_exceeded'
    assert localizer.sample_count_for('euclidean_reconstruction') == 2

    older = observe(
        localizer,
        measurement_message(
            stamp_ns=2,
            source_values=euclidean_values(),
        ),
    )
    assert older.reason == 'nonincreasing_timestamp'


def test_samples_from_different_estimators_are_never_combined() -> None:
    localizer = adapter(sample_count=3)
    for stamp in (1, 2):
        observe(
            localizer,
            measurement_message(stamp_ns=stamp, source_values=euclidean_values()),
        )
    projective = {
        'euclidean_reconstruction': [
            (0.0, 0.0, 0.0, int(MissReason.NO_DEPTH_FRAME)),
        ],
        'projective_ranging': [(0.2, 2.0, 2.1, int(MissReason.OK))],
    }
    for stamp in (3, 4):
        result = observe(
            localizer,
            measurement_message(stamp_ns=stamp, source_values=projective),
        )

    assert result.stabilized is None
    assert localizer.sample_count_for('euclidean_reconstruction') == 2
    assert localizer.sample_count_for('projective_ranging') == 2

    final = observe(
        localizer,
        measurement_message(stamp_ns=5, source_values=euclidean_values()),
    )
    assert final.stabilized.source == 'euclidean_reconstruction'


def test_generic_estimate_is_explicitly_not_a_foot_midpoint() -> None:
    result = observe(
        adapter(),
        measurement_message(stamp_ns=11_000, source_values=euclidean_values()),
    )

    assert result.observation.reference == GENERIC_TARGET_REFERENCE
    assert not result.observation.is_foot_midpoint


def test_reset_discards_samples_and_timestamp_history() -> None:
    localizer = adapter()
    observe(
        localizer,
        measurement_message(stamp_ns=20, source_values=euclidean_values()),
    )

    localizer.reset()
    result = observe(
        localizer,
        measurement_message(stamp_ns=10, source_values=euclidean_values()),
    )

    assert result.accepted
    assert localizer.sample_count_for('euclidean_reconstruction') == 1


def test_no_detection_is_rejected_without_a_sample() -> None:
    msg = measurement_message(stamp_ns=12_000, detections=[])

    result = observe(adapter(), msg, target=association(12_000))

    assert not result.accepted
    assert result.reason == 'no_detections'


def test_detected_and_count_must_be_consistent() -> None:
    msg = measurement_message(stamp_ns=13_000, detections=[])
    msg.detected = True

    result = observe(adapter(), msg, target=association(13_000))

    assert not result.accepted
    assert result.reason == 'inconsistent_detected_count'
