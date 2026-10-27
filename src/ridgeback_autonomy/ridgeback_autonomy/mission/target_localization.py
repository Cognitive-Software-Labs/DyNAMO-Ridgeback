"""Validated target-position input for the future parking mission manager.

The shared localization stack reports planar positions relative to Ridgeback's
front reference.  This adapter converts one explicitly associated target into
a base-origin observation, transforms it into ``map`` at the source timestamp,
and stabilizes repeated observations without mixing estimator families.

None of the current generic estimators has G1 foot-midpoint semantics.  The
output therefore labels its reference as ``generic_target_position`` and must
not be presented as a foot midpoint without separate runtime validation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Iterable

from rclpy.time import Time
from tf2_ros import TransformException

from ridgeback_common.miss_reason import MissReason
from ridgeback_common.stamps import stamp_to_nanoseconds
from ridgeback_interfaces.msg import TargetMeasurements
from ridgeback_localization.core.vehicle_frame import (
    remove_vehicle_front_offset,
    yaw_from_quaternion,
)
from ridgeback_localization.estimator_registry import (
    ESTIMATOR_POSITION_ATTRS,
    ESTIMATOR_STATUS_FIELD_KEYS,
    PUBLIC_ESTIMATOR_ORDER,
)


DEFAULT_LOCALIZATION_SOURCES = (
    'euclidean_reconstruction',
    'projective_ranging',
)
GENERIC_TARGET_REFERENCE = 'generic_target_position'
ORIGINAL_LOCALIZATION_SAMPLE_COUNT = 5
ORIGINAL_BODY_SAMPLE_MAX_JUMP_M = 0.75
ORIGINAL_SENSOR_BUFFER_MAX_AGE_S = 2.0
ORIGINAL_G1_DETECTION_THRESHOLD = 0.50


@dataclass(frozen=True)
class TargetAssociation:
    """The detection identity that a measurement batch must reproduce.

    ``TargetMeasurements`` has no persistent object ID.  Association is
    consequently exact and frame-local: the detection timestamp, normalized
    label, and bbox must all match.  A future mission manager should construct
    this from the corresponding accepted ``TargetDetections`` observation.
    """

    stamp_ns: int
    label: str
    bbox_xyxy: tuple[float, float, float, float]


@dataclass(frozen=True)
class TargetPositionObservation:
    """One accepted map-frame observation from one estimator."""

    source: str
    reference: str
    is_foot_midpoint: bool
    stamp_ns: int
    target_index: int
    label: str
    bbox_xyxy: tuple[float, float, float, float]
    front_forward_m: float
    front_lateral_m: float
    distance_m: float
    base_x_m: float
    base_y_m: float
    map_x_m: float
    map_y_m: float
    base_frame: str
    map_frame: str


@dataclass(frozen=True)
class StabilizedTargetPosition:
    """Median of qualified observations from exactly one estimator source."""

    source: str
    reference: str
    is_foot_midpoint: bool
    map_x_m: float
    map_y_m: float
    sample_count: int
    first_stamp_ns: int
    last_stamp_ns: int


@dataclass(frozen=True)
class TargetLocalizationResult:
    """Outcome of validating and accumulating one measurement batch."""

    message_valid: bool
    accepted: bool
    reason: str
    stamp_ns: int
    observation: TargetPositionObservation | None
    stabilized: StabilizedTargetPosition | None
    source_failures: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class _EstimatorMeasurement:
    source: str
    lateral_m: float
    forward_m: float
    distance_m: float


class TargetLocalizationAdapter:
    """Validate, map-project, and stabilize generic target measurements.

    The default source contract is the status-bearing mask pipeline:
    ``euclidean_reconstruction`` first and ``projective_ranging`` second.
    Pointcloud measurements are supported when explicitly configured, but have
    no per-estimator status in the current message schema and are therefore
    validated by array presence and finite numeric values only.
    """

    def __init__(
        self,
        tf_buffer,
        *,
        map_frame: str,
        base_frame: str,
        source_order: Iterable[str] = DEFAULT_LOCALIZATION_SOURCES,
        minimum_score: float = ORIGINAL_G1_DETECTION_THRESHOLD,
        max_measurement_age_s: float = ORIGINAL_SENSOR_BUFFER_MAX_AGE_S,
        sample_count: int = ORIGINAL_LOCALIZATION_SAMPLE_COUNT,
        max_sample_jump_m: float = ORIGINAL_BODY_SAMPLE_MAX_JUMP_M,
    ) -> None:
        sources = tuple(source_order)
        if not sources:
            raise ValueError('source_order must contain at least one estimator')
        if len(set(sources)) != len(sources):
            raise ValueError('source_order must not contain duplicates')
        unknown = [source for source in sources if source not in PUBLIC_ESTIMATOR_ORDER]
        if unknown:
            raise ValueError(f'unsupported localization source: {unknown[0]}')
        if not str(map_frame).strip() or not str(base_frame).strip():
            raise ValueError('map_frame and base_frame must be explicit TF frame IDs')
        if not math.isfinite(minimum_score) or not 0.0 <= minimum_score <= 1.0:
            raise ValueError('minimum_score must be finite and within [0, 1]')
        if not math.isfinite(max_measurement_age_s) or max_measurement_age_s < 0.0:
            raise ValueError('max_measurement_age_s must be finite and non-negative')
        if sample_count < 2:
            raise ValueError('sample_count must be at least 2')
        if not math.isfinite(max_sample_jump_m) or max_sample_jump_m <= 0.0:
            raise ValueError('max_sample_jump_m must be finite and positive')

        self.tf_buffer = tf_buffer
        self.map_frame = str(map_frame).strip()
        self.base_frame = str(base_frame).strip()
        self.source_order = sources
        self.minimum_score = float(minimum_score)
        self.max_measurement_age_ns = int(max_measurement_age_s * 1_000_000_000)
        self.sample_count = int(sample_count)
        self.max_sample_jump_m = float(max_sample_jump_m)

        self._samples_by_source: dict[str, list[TargetPositionObservation]] = {
            source: [] for source in sources
        }
        self._last_processed_stamp_ns: int | None = None
        self._stabilized: StabilizedTargetPosition | None = None

    @property
    def stabilized(self) -> StabilizedTargetPosition | None:
        return self._stabilized

    def sample_count_for(self, source: str) -> int:
        return len(self._samples_by_source.get(source, ()))

    def reset(self) -> None:
        """Begin a fresh localization attempt without retaining stale samples."""

        for samples in self._samples_by_source.values():
            samples.clear()
        self._last_processed_stamp_ns = None
        self._stabilized = None

    def observe(
        self,
        msg: TargetMeasurements,
        *,
        association: TargetAssociation,
        now_ns: int,
    ) -> TargetLocalizationResult:
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if self._stabilized is not None:
            return self._result(
                stamp_ns,
                reason='already_stabilized',
                stabilized=self._stabilized,
            )
        if stamp_ns != int(association.stamp_ns):
            return self._result(stamp_ns, reason='association_timestamp_mismatch')
        if int(now_ns) < stamp_ns:
            return self._result(stamp_ns, reason='future_timestamp')
        if int(now_ns) - stamp_ns > self.max_measurement_age_ns:
            return self._result(stamp_ns, reason='stale_timestamp')
        if (
            self._last_processed_stamp_ns is not None
            and stamp_ns <= self._last_processed_stamp_ns
        ):
            return self._result(stamp_ns, reason='nonincreasing_timestamp')

        indices, invalid_reason = self._matching_indices(msg, association)
        if invalid_reason is not None:
            return self._result(stamp_ns, reason=invalid_reason)
        if len(indices) != 1:
            reason = 'target_not_associated' if not indices else 'ambiguous_target_association'
            return self._result(stamp_ns, reason=reason)
        index = indices[0]

        measurement, failures = self._select_measurement(msg, index)
        if measurement is None:
            return self._result(
                stamp_ns,
                reason='no_valid_localization_source',
                message_valid=True,
                source_failures=failures,
            )

        lateral_base_m, forward_base_m = remove_vehicle_front_offset(
            measurement.lateral_m,
            measurement.forward_m,
        )
        if not all(math.isfinite(value) for value in (lateral_base_m, forward_base_m)):
            return self._result(stamp_ns, reason='invalid_base_origin_coordinates')

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time.from_msg(msg.header.stamp),
            )
        except TransformException:
            return self._result(
                stamp_ns,
                reason='transform_unavailable',
                message_valid=True,
            )

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_values = (
            translation.x,
            translation.y,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in transform_values):
            return self._result(stamp_ns, reason='invalid_transform')

        yaw = yaw_from_quaternion(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        map_x_m = (
            float(translation.x)
            + math.cos(yaw) * float(forward_base_m)
            - math.sin(yaw) * float(lateral_base_m)
        )
        map_y_m = (
            float(translation.y)
            + math.sin(yaw) * float(forward_base_m)
            + math.cos(yaw) * float(lateral_base_m)
        )
        if not math.isfinite(map_x_m) or not math.isfinite(map_y_m):
            return self._result(stamp_ns, reason='invalid_map_coordinates')

        bbox = self._bbox_at(msg, index)
        observation = TargetPositionObservation(
            source=measurement.source,
            reference=GENERIC_TARGET_REFERENCE,
            is_foot_midpoint=False,
            stamp_ns=stamp_ns,
            target_index=index,
            label=str(msg.labels[index]).strip(),
            bbox_xyxy=bbox,
            front_forward_m=measurement.forward_m,
            front_lateral_m=measurement.lateral_m,
            distance_m=measurement.distance_m,
            base_x_m=float(forward_base_m),
            base_y_m=float(lateral_base_m),
            map_x_m=map_x_m,
            map_y_m=map_y_m,
            base_frame=self.base_frame,
            map_frame=self.map_frame,
        )
        # The source-time TF lookup completed, so this observation is consumed
        # even if the stabilization jump gate rejects its coordinates below.
        self._last_processed_stamp_ns = stamp_ns

        source_samples = self._samples_by_source[measurement.source]
        if source_samples:
            center_x = float(median(sample.map_x_m for sample in source_samples))
            center_y = float(median(sample.map_y_m for sample in source_samples))
            jump_m = math.hypot(map_x_m - center_x, map_y_m - center_y)
            if jump_m > self.max_sample_jump_m:
                return self._result(
                    stamp_ns,
                    reason='sample_jump_exceeded',
                    message_valid=True,
                    observation=observation,
                    source_failures=failures,
                )

        source_samples.append(observation)
        stabilized = None
        if len(source_samples) >= self.sample_count:
            selected_samples = source_samples[-self.sample_count:]
            stabilized = StabilizedTargetPosition(
                source=measurement.source,
                reference=GENERIC_TARGET_REFERENCE,
                is_foot_midpoint=False,
                map_x_m=float(median(sample.map_x_m for sample in selected_samples)),
                map_y_m=float(median(sample.map_y_m for sample in selected_samples)),
                sample_count=self.sample_count,
                first_stamp_ns=selected_samples[0].stamp_ns,
                last_stamp_ns=selected_samples[-1].stamp_ns,
            )
            self._stabilized = stabilized

        return self._result(
            stamp_ns,
            reason='stabilized' if stabilized is not None else 'accepted',
            message_valid=True,
            accepted=True,
            observation=observation,
            stabilized=stabilized,
            source_failures=failures,
        )

    def _matching_indices(
        self,
        msg: TargetMeasurements,
        association: TargetAssociation,
    ) -> tuple[list[int], str | None]:
        count = int(msg.count)
        width = int(msg.image_width)
        height = int(msg.image_height)
        if count < 0:
            return [], 'negative_count'
        if bool(msg.detected) != (count > 0):
            return [], 'inconsistent_detected_count'
        if count == 0:
            return [], 'no_detections'
        if width <= 0 or height <= 0:
            return [], 'invalid_image_size'
        if len(msg.bbox_xyxy) != count * 4:
            return [], 'bbox_count_mismatch'
        if len(msg.labels) != count:
            return [], 'label_count_mismatch'
        if len(msg.scores) != count:
            return [], 'score_count_mismatch'

        association_label = self._normalize_label(association.label)
        if not association_label:
            return [], 'invalid_association_label'
        association_bbox = tuple(float(value) for value in association.bbox_xyxy)
        if len(association_bbox) != 4 or not all(
            math.isfinite(value) for value in association_bbox
        ):
            return [], 'invalid_association_bbox'

        matches: list[int] = []
        for index in range(count):
            bbox = self._bbox_at(msg, index)
            score = float(msg.scores[index])
            if not all(math.isfinite(value) for value in bbox):
                return [], 'nonfinite_bbox'
            x1, y1, x2, y2 = bbox
            if not (0.0 <= x1 < x2 <= width and 0.0 <= y1 < y2 <= height):
                return [], 'invalid_bbox'
            if not math.isfinite(score) or not 0.0 <= score <= 1.0:
                return [], 'invalid_score'
            if (
                self._normalize_label(msg.labels[index]) == association_label
                and score >= self.minimum_score
                and self._same_bbox(bbox, association_bbox)
            ):
                matches.append(index)
        return matches, None

    def _select_measurement(
        self,
        msg: TargetMeasurements,
        index: int,
    ) -> tuple[_EstimatorMeasurement | None, tuple[tuple[str, str], ...]]:
        failures: list[tuple[str, str]] = []
        for source in self.source_order:
            measurement, reason = self._measurement_for_source(msg, index, source)
            if measurement is not None:
                return measurement, tuple(failures)
            failures.append((source, reason))
        return None, tuple(failures)

    def _measurement_for_source(
        self,
        msg: TargetMeasurements,
        index: int,
        source: str,
    ) -> tuple[_EstimatorMeasurement | None, str]:
        forward_attr, lateral_attr = ESTIMATOR_POSITION_ATTRS[source]
        distance_attr = f'{source}_distance_m'
        values = {
            'forward': getattr(msg, forward_attr),
            'lateral': getattr(msg, lateral_attr),
            'distance': getattr(msg, distance_attr),
        }
        if any(index >= len(array) for array in values.values()):
            return None, 'missing_measurement_field'

        forward_m = float(values['forward'][index])
        lateral_m = float(values['lateral'][index])
        distance_m = float(values['distance'][index])
        if not all(math.isfinite(value) for value in (forward_m, lateral_m, distance_m)):
            return None, 'nonfinite_measurement'
        if forward_m <= 0.0 or distance_m <= 0.0:
            return None, 'nonpositive_measurement'

        status_attr = ESTIMATOR_STATUS_FIELD_KEYS.get(source)
        if status_attr is not None:
            statuses = getattr(msg, status_attr)
            if index >= len(statuses):
                return None, 'missing_status'
            if int(statuses[index]) != int(MissReason.OK):
                return None, f'estimator_status_{int(statuses[index])}'

        return _EstimatorMeasurement(
            source=source,
            lateral_m=lateral_m,
            forward_m=forward_m,
            distance_m=distance_m,
        ), 'accepted'

    def _result(
        self,
        stamp_ns: int,
        *,
        reason: str,
        message_valid: bool = False,
        accepted: bool = False,
        observation: TargetPositionObservation | None = None,
        stabilized: StabilizedTargetPosition | None = None,
        source_failures: tuple[tuple[str, str], ...] = (),
    ) -> TargetLocalizationResult:
        return TargetLocalizationResult(
            message_valid=message_valid,
            accepted=accepted,
            reason=reason,
            stamp_ns=stamp_ns,
            observation=observation,
            stabilized=stabilized,
            source_failures=source_failures,
        )

    @staticmethod
    def _bbox_at(
        msg: TargetMeasurements,
        index: int,
    ) -> tuple[float, float, float, float]:
        start = index * 4
        return tuple(float(value) for value in msg.bbox_xyxy[start:start + 4])

    @staticmethod
    def _same_bbox(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> bool:
        return all(math.isclose(a, b, abs_tol=1e-4) for a, b in zip(left, right))

    @staticmethod
    def _normalize_label(label: str) -> str:
        return str(label).strip().casefold()
