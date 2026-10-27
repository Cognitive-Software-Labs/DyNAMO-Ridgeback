#!/usr/bin/env python3
"""Non-driving mission coordinator for detection, handoff, and localization.

Phase 2D deliberately stops in ``G1_LOCALIZED``.  This node owns no Nav2 action
client and cannot start parking or return-to-start motion.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements
from ridgeback_localization.contracts import (
    MASK_MEASUREMENTS_TOPIC,
    RAW_DETECTIONS_TOPIC,
)
from ridgeback_autonomy.mission.target_detection import (
    TargetDetectionAdapter,
    TargetDetectionCandidate,
)
from ridgeback_autonomy.mission.target_localization import (
    StabilizedTargetPosition,
    TargetAssociation,
    TargetLocalizationAdapter,
)


class MissionState(Enum):
    WAITING_FOR_START_POSE = 'waiting_for_start_pose'
    SEARCH_G1 = 'search_g1'
    STOPPING_EXPLORATION = 'stopping_exploration'
    LOCALIZE_G1 = 'localize_g1'
    G1_LOCALIZED = 'g1_localized'
    MISSION_FAILED = 'mission_failed'
    MISSION_CANCELED = 'mission_canceled'


class FailureClass(Enum):
    RECOVERABLE = 'recoverable'
    TERMINAL = 'terminal'
    OPERATOR_CANCELED = 'operator_canceled'


@dataclass(frozen=True)
class MissionIssue:
    classification: FailureClass
    code: str
    reason: str


@dataclass(frozen=True)
class StateTransition:
    previous: MissionState
    current: MissionState
    reason: str
    monotonic_s: float


_ALLOWED_TRANSITIONS = {
    MissionState.WAITING_FOR_START_POSE: {
        MissionState.SEARCH_G1,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.SEARCH_G1: {
        MissionState.STOPPING_EXPLORATION,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.STOPPING_EXPLORATION: {
        MissionState.LOCALIZE_G1,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.LOCALIZE_G1: {
        MissionState.G1_LOCALIZED,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.G1_LOCALIZED: {MissionState.MISSION_CANCELED},
    MissionState.MISSION_FAILED: set(),
    MissionState.MISSION_CANCELED: set(),
}


class MissionManagerNode(Node):
    """Coordinate the safe exploration-to-localization mission prefix."""

    def __init__(
        self,
        *,
        tf_buffer=None,
        pause_client=None,
        monotonic=time.monotonic,
        parameter_overrides=None,
    ) -> None:
        super().__init__(
            'mission_manager_node',
            parameter_overrides=parameter_overrides,
        )
        self._lock = threading.RLock()
        self._monotonic = monotonic

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('target_detection_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('target_measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('exploration_pause_service', 'mission/pause')
        self.declare_parameter('exploration_status_topic', 'mission/status')
        self.declare_parameter('state_topic', 'parking_mission/state')
        self.declare_parameter('handoff_timeout_s', 10.0)
        self.declare_parameter('target_localization_timeout_s', 5.0)
        self.declare_parameter('association_history_size', 32)
        self.declare_parameter('control_period_s', 0.2)

        self.map_frame = str(self.get_parameter('map_frame').value).strip()
        self.base_frame = str(self.get_parameter('robot_base_frame').value).strip()
        self.detection_topic = str(
            self.get_parameter('target_detection_topic').value
        ).strip()
        self.measurement_topic = str(
            self.get_parameter('target_measurement_topic').value
        ).strip()
        self.pause_service_name = str(
            self.get_parameter('exploration_pause_service').value
        ).strip()
        self.guard_status_topic = str(
            self.get_parameter('exploration_status_topic').value
        ).strip()
        self.state_topic = str(self.get_parameter('state_topic').value).strip()
        self.handoff_timeout_s = float(
            self.get_parameter('handoff_timeout_s').value
        )
        self.localization_timeout_s = float(
            self.get_parameter('target_localization_timeout_s').value
        )
        self.association_history_size = int(
            self.get_parameter('association_history_size').value
        )
        control_period_s = float(self.get_parameter('control_period_s').value)
        if not self.map_frame or not self.base_frame:
            raise ValueError('map_frame and robot_base_frame must be explicit')
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (
                self.handoff_timeout_s,
                self.localization_timeout_s,
                control_period_s,
            )
        ):
            raise ValueError('mission timeouts and control period must be positive')
        if self.association_history_size < 1:
            raise ValueError('association_history_size must be positive')

        self.tf_buffer = tf_buffer if tf_buffer is not None else Buffer(node=self)
        self.tf_listener = (
            None
            if tf_buffer is not None
            else TransformListener(self.tf_buffer, self)
        )
        self.detection_adapter = TargetDetectionAdapter()
        self.localization_adapter = TargetLocalizationAdapter(
            self.tf_buffer,
            map_frame=self.map_frame,
            base_frame=self.base_frame,
        )

        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_publisher = self.create_publisher(
            String,
            self.state_topic,
            status_qos,
        )
        self.detection_subscription = self.create_subscription(
            TargetDetections,
            self.detection_topic,
            self._on_detections,
            10,
        )
        self.measurement_subscription = self.create_subscription(
            TargetMeasurements,
            self.measurement_topic,
            self._on_measurements,
            10,
        )
        self.guard_status_subscription = self.create_subscription(
            String,
            self.guard_status_topic,
            self._on_guard_status,
            status_qos,
        )
        self.pause_client = (
            pause_client
            if pause_client is not None
            else self.create_client(Trigger, self.pause_service_name)
        )

        self.state = MissionState.WAITING_FOR_START_POSE
        self.start_pose: PoseStamped | None = None
        self.confirmed_detection: TargetDetectionCandidate | None = None
        self.target_committed = False
        self.target_position: StabilizedTargetPosition | None = None
        self.last_issue: MissionIssue | None = None
        self.transition_history: list[StateTransition] = []
        self._associations: OrderedDict[int, TargetAssociation] = OrderedDict()

        self._handoff_generation = 0
        self._handoff_deadline_s: float | None = None
        self._pause_request_in_flight = False
        self._pause_response_received = False
        self._pause_service_seen_ready = False
        self._pause_request_failed = False
        self._pause_status_min_sequence = 0
        self._guard_status_sequence = 0
        self._latest_guard_status: dict | None = None
        self._guard_failure_signature: tuple[str, bool] | None = None
        self._guard_failure_count = 0
        self._localization_deadline_s: float | None = None

        self.control_timer = self.create_timer(control_period_s, self.tick)
        self._publish_state('mission manager initialized', event='snapshot')

    def tick(self, now_s: float | None = None) -> None:
        """Advance timeout/service polling without blocking ROS callbacks."""

        with self._lock:
            now_s = self._monotonic() if now_s is None else float(now_s)
            if self.state == MissionState.WAITING_FOR_START_POSE:
                self._try_capture_start_pose()
                return
            if self.state == MissionState.STOPPING_EXPLORATION:
                if self._handoff_deadline_s is not None and now_s >= self._handoff_deadline_s:
                    if not self._pause_service_seen_ready:
                        code = 'pause_service_unavailable'
                        reason = 'MissionGuard pause service was unavailable'
                    elif self._pause_request_failed and not self._pause_response_received:
                        code = 'pause_request_failed'
                        reason = 'MissionGuard pause requests failed'
                    else:
                        code = 'exploration_handoff_timeout'
                        reason = 'Exploration ownership remained unresolved'
                    self._fail_terminal(
                        code,
                        f'{reason} before the handoff timeout',
                    )
                    return
                self._request_exploration_pause()
                return
            if (
                self.state == MissionState.LOCALIZE_G1
                and self._localization_deadline_s is not None
                and now_s >= self._localization_deadline_s
            ):
                self._fail_terminal(
                    'target_localization_timeout',
                    'Target localization did not stabilize before the timeout',
                )

    def process_detection(self, msg: TargetDetections) -> None:
        with self._lock:
            if self.state not in (
                MissionState.SEARCH_G1,
                MissionState.STOPPING_EXPLORATION,
                MissionState.LOCALIZE_G1,
            ):
                self._guard_failed(
                    'target detection',
                    f'detections are ignored in {self.state.name}',
                )
                return

            result = self.detection_adapter.observe(msg)
            if not result.message_valid:
                self._record_recoverable('invalid_detection', result.reason)
                return
            if result.selected is not None:
                self._remember_association(result.stamp_ns, result.selected)

            if self.state != MissionState.SEARCH_G1:
                return
            if not result.confirmed:
                return
            if result.selected is None:
                self._fail_terminal(
                    'invalid_detection_confirmation',
                    'Detection adapter confirmed without a selected target',
                )
                return

            self.confirmed_detection = result.selected
            self.target_committed = True
            self.get_logger().info(
                'Target confirmed by detection adapter: '
                f'positives={result.positive_votes}/{len(result.vote_window)}, '
                f'label={result.selected.label}, score={result.selected.score:.3f}'
            )
            if self._transition(
                MissionState.STOPPING_EXPLORATION,
                'target confirmed; requesting explorer ownership release',
            ):
                self._begin_exploration_handoff()

    def process_guard_status(self, msg: String) -> None:
        with self._lock:
            self._guard_status_sequence += 1
            sequence = self._guard_status_sequence
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                self._record_recoverable(
                    'invalid_guard_status',
                    'mission/status is not valid JSON',
                )
                return
            if not self._valid_guard_status(payload):
                self._record_recoverable(
                    'invalid_guard_status',
                    'mission/status is missing required ownership fields',
                )
                return
            self._latest_guard_status = payload
            if self.state != MissionState.STOPPING_EXPLORATION:
                return
            if not self._pause_response_received:
                return
            if sequence <= self._pause_status_min_sequence:
                return
            self._evaluate_guard_status(payload)

    def process_measurement(
        self,
        msg: TargetMeasurements,
        *,
        now_ns: int | None = None,
    ) -> None:
        with self._lock:
            if self.state != MissionState.LOCALIZE_G1:
                self._guard_failed(
                    'target measurement',
                    f'measurements are ignored in {self.state.name}',
                )
                return
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            association = self._associations.get(stamp_ns)
            if association is None:
                self._record_recoverable(
                    'measurement_without_detection_association',
                    'TargetMeasurements has no matching validated detection frame',
                )
                return

            result = self.localization_adapter.observe(
                msg,
                association=association,
                now_ns=(
                    self.get_clock().now().nanoseconds
                    if now_ns is None
                    else int(now_ns)
                ),
            )
            if not result.accepted:
                details = result.reason
                if result.source_failures:
                    details += ': ' + ', '.join(
                        f'{source}={reason}'
                        for source, reason in result.source_failures
                    )
                self._record_recoverable('localization_sample_rejected', details)
                return

            if result.stabilized is None:
                sample_count = self.localization_adapter.sample_count_for(
                    result.observation.source
                )
                self.get_logger().info(
                    'Target localization sample accepted: '
                    f'{sample_count}/{self.localization_adapter.sample_count}, '
                    f'source={result.observation.source}'
                )
                return

            self.target_position = result.stabilized
            self._localization_deadline_s = None
            self._clear_recoverable_issue()
            self.get_logger().info(
                'Target position stabilized: '
                f'x={result.stabilized.map_x_m:.3f}, '
                f'y={result.stabilized.map_y_m:.3f}, '
                f'source={result.stabilized.source}, '
                f'reference={result.stabilized.reference}, '
                f'is_foot_midpoint={result.stabilized.is_foot_midpoint}'
            )
            if self._transition(
                MissionState.G1_LOCALIZED,
                'generic target position stabilized; parking execution deferred',
            ):
                self.get_logger().warn(
                    'Phase 2D terminal reached: generic target position is not a '
                    'validated G1 foot midpoint; no parking or return goal will be sent'
                )

    def reset_mission(self, *, clear_start_pose: bool = True) -> None:
        """Reset manager-local state; MissionGuard resume remains explicit."""

        with self._lock:
            previous = self.state
            self._handoff_generation += 1
            self.detection_adapter.reset()
            self.localization_adapter.reset()
            self.confirmed_detection = None
            self.target_committed = False
            self.target_position = None
            self.last_issue = None
            self._associations.clear()
            self._clear_handoff_state()
            self._localization_deadline_s = None
            if clear_start_pose:
                self.start_pose = None
            self.state = (
                MissionState.WAITING_FOR_START_POSE
                if self.start_pose is None
                else MissionState.SEARCH_G1
            )
            self.transition_history.append(StateTransition(
                previous=previous,
                current=self.state,
                reason='explicit manager reset',
                monotonic_s=self._monotonic(),
            ))
            self.get_logger().info(
                f'Mission state transition: {previous.name} -> {self.state.name}; '
                'reason=explicit manager reset; explorer resume is still explicit'
            )
            self._publish_state('explicit manager reset')

    def cancel_mission(self, reason: str = 'explicit operator cancellation') -> None:
        with self._lock:
            if self.state in (
                MissionState.MISSION_FAILED,
                MissionState.MISSION_CANCELED,
            ):
                return
            self.last_issue = MissionIssue(
                FailureClass.OPERATOR_CANCELED,
                'operator_canceled',
                reason,
            )
            self._transition(MissionState.MISSION_CANCELED, reason)

    def _try_capture_start_pose(self) -> bool:
        if self.start_pose is not None:
            return True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            self._guard_failed(
                'start pose capture',
                f'{self.map_frame} -> {self.base_frame} transform unavailable',
            )
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        norm = math.sqrt(
            float(rotation.x) ** 2
            + float(rotation.y) ** 2
            + float(rotation.z) ** 2
            + float(rotation.w) ** 2
        )
        if not all(math.isfinite(float(value)) for value in values) or norm < 1e-9:
            self._record_recoverable(
                'invalid_start_pose_transform',
                'map-to-base transform contains invalid values',
            )
            return False

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = float(translation.x)
        pose.pose.position.y = float(translation.y)
        pose.pose.position.z = float(translation.z)
        pose.pose.orientation.x = float(rotation.x) / norm
        pose.pose.orientation.y = float(rotation.y) / norm
        pose.pose.orientation.z = float(rotation.z) / norm
        pose.pose.orientation.w = float(rotation.w) / norm
        self.start_pose = pose
        self._clear_recoverable_issue()
        self.get_logger().info(
            'Saved mission start pose: '
            f'frame={self.map_frame}, x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}'
        )
        self.detection_adapter.reset()
        return self._transition(
            MissionState.SEARCH_G1,
            'valid initial map-to-base transform captured',
        )

    def _begin_exploration_handoff(self) -> None:
        self._handoff_generation += 1
        self._handoff_deadline_s = self._monotonic() + self.handoff_timeout_s
        self._pause_request_in_flight = False
        self._pause_response_received = False
        self._pause_service_seen_ready = False
        self._pause_request_failed = False
        self._pause_status_min_sequence = self._guard_status_sequence
        self._request_exploration_pause()

    def _request_exploration_pause(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        if self._pause_request_in_flight or self._pause_response_received:
            return
        if not self.pause_client.service_is_ready():
            return

        self._pause_service_seen_ready = True
        generation = self._handoff_generation
        self._pause_request_in_flight = True
        self.get_logger().info('Requesting exploration pause from MissionGuard')
        try:
            future = self.pause_client.call_async(Trigger.Request())
        except Exception as error:
            self._pause_request_in_flight = False
            self._pause_request_failed = True
            self._record_recoverable('pause_request_failed', str(error))
            return
        future.add_done_callback(
            lambda done, token=generation: self._on_pause_response(done, token)
        )

    def _on_pause_response(self, future, generation: int) -> None:
        with self._lock:
            if generation != self._handoff_generation:
                return
            if self.state != MissionState.STOPPING_EXPLORATION:
                return
            self._pause_request_in_flight = False
            try:
                response = future.result()
            except Exception as error:
                self._pause_request_failed = True
                self._record_recoverable('pause_request_failed', str(error))
                return

            self._pause_request_failed = False
            self._pause_response_received = True
            self._pause_status_min_sequence = self._guard_status_sequence
            if response.success:
                self.get_logger().info(
                    'MissionGuard acknowledged an immediate safe exploration handoff'
                )
                self._enter_localization('explorer ownership released by pause response')
                return
            self.get_logger().info(
                'Exploration pause latched; waiting for terminal ownership status: '
                f'{response.message}'
            )

    def _evaluate_guard_status(self, payload: dict) -> None:
        cancellation = payload['cancellation']
        if cancellation in ('failed', 'rejected') or payload['cancellation_overdue']:
            signature = (cancellation, payload['cancellation_overdue'])
            if signature == self._guard_failure_signature:
                self._guard_failure_count += 1
            else:
                self._guard_failure_signature = signature
                self._guard_failure_count = 1
            if self._guard_failure_count < 2:
                self._record_recoverable(
                    'guard_failure_pending_confirmation',
                    'MissionGuard reported an ownership failure; waiting for one '
                    'fresh repeated status before failing the mission',
                )
                return
            if payload['cancellation_overdue']:
                self._fail_terminal(
                    'exploration_cancellation_timeout',
                    f'MissionGuard cancellation is overdue: '
                    f'{payload.get("reason", "no reason")}',
                )
            else:
                self._fail_terminal(
                    'exploration_cancellation_failed',
                    f'MissionGuard cancellation state is {cancellation}: '
                    f'{payload.get("reason", "no reason")}',
                )
            return
        self._guard_failure_signature = None
        self._guard_failure_count = 0
        if not payload['pause_requested'] or not payload['handoff_safe']:
            return
        if (
            payload['goal_pending']
            or payload['goal_active']
            or cancellation != 'none'
        ):
            self._record_recoverable(
                'inconsistent_guard_status',
                'handoff_safe was true while explorer ownership remained unresolved',
            )
            return
        # A transient-local sample can be delivered late. Verify the reported
        # safe state with a fresh idempotent service call before transferring
        # Nav2 ownership.
        self._pause_response_received = False
        self._pause_status_min_sequence = self._guard_status_sequence
        self.get_logger().info(
            'MissionGuard reported handoff_safe; verifying with a fresh pause request'
        )
        self._request_exploration_pause()

    def _enter_localization(self, reason: str) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            self._guard_failed('localization transition', reason)
            return
        self._clear_handoff_state()
        self.localization_adapter.reset()
        self._clear_recoverable_issue()
        self._localization_deadline_s = self._monotonic() + self.localization_timeout_s
        self._transition(MissionState.LOCALIZE_G1, reason)

    def _remember_association(
        self,
        stamp_ns: int,
        candidate: TargetDetectionCandidate,
    ) -> None:
        if self.confirmed_detection is not None and (
            candidate.label.strip().casefold()
            != self.confirmed_detection.label.strip().casefold()
        ):
            return
        association = TargetAssociation(
            stamp_ns=int(stamp_ns),
            label=candidate.label,
            bbox_xyxy=candidate.bbox_xyxy,
        )
        self._associations[int(stamp_ns)] = association
        self._associations.move_to_end(int(stamp_ns))
        while len(self._associations) > self.association_history_size:
            self._associations.popitem(last=False)

    def _transition(self, new_state: MissionState, reason: str) -> bool:
        if new_state == self.state:
            self._guard_failed(
                'state transition',
                f'already in {self.state.name}: {reason}',
            )
            return False
        if new_state not in _ALLOWED_TRANSITIONS[self.state]:
            self._guard_failed(
                'state transition',
                f'{self.state.name} -> {new_state.name} is not allowed: {reason}',
            )
            return False
        previous = self.state
        self.state = new_state
        transition = StateTransition(
            previous=previous,
            current=new_state,
            reason=reason,
            monotonic_s=self._monotonic(),
        )
        self.transition_history.append(transition)
        self.get_logger().info(
            f'Mission state transition: {previous.name} -> {new_state.name}; '
            f'reason={reason}'
        )
        self._publish_state(reason)
        return True

    def _record_recoverable(self, code: str, reason: str) -> None:
        self.last_issue = MissionIssue(FailureClass.RECOVERABLE, code, reason)
        self.get_logger().warning(f'Recoverable mission issue [{code}]: {reason}')
        self._publish_state(reason, event='recoverable_error')

    def _clear_recoverable_issue(self) -> None:
        if (
            self.last_issue is not None
            and self.last_issue.classification == FailureClass.RECOVERABLE
        ):
            self.last_issue = None

    def _fail_terminal(self, code: str, reason: str) -> None:
        if self.state in (
            MissionState.MISSION_FAILED,
            MissionState.MISSION_CANCELED,
            MissionState.G1_LOCALIZED,
        ):
            return
        self.last_issue = MissionIssue(FailureClass.TERMINAL, code, reason)
        self._clear_handoff_state()
        self._localization_deadline_s = None
        self.get_logger().error(f'Terminal mission failure [{code}]: {reason}')
        self._transition(MissionState.MISSION_FAILED, reason)

    def _guard_failed(self, operation: str, reason: str) -> None:
        self.get_logger().debug(f'Mission guard rejected {operation}: {reason}')
        self._publish_state(reason, event='guard_rejected')

    def _clear_handoff_state(self) -> None:
        self._handoff_deadline_s = None
        self._pause_request_in_flight = False
        self._pause_response_received = False
        self._pause_service_seen_ready = False
        self._pause_request_failed = False
        self._pause_status_min_sequence = self._guard_status_sequence
        self._guard_failure_signature = None
        self._guard_failure_count = 0

    def _publish_state(self, reason: str, *, event: str = 'transition') -> None:
        issue = self.last_issue
        position = self.target_position
        payload = {
            'state': self.state.name,
            'state_value': self.state.value,
            'event': event,
            'reason': reason,
            'target_committed': self.target_committed,
            'start_pose_saved': self.start_pose is not None,
            'parking_navigation_enabled': False,
            'failure_class': None if issue is None else issue.classification.value,
            'failure_code': None if issue is None else issue.code,
            'target_reference': None if position is None else position.reference,
            'target_source': None if position is None else position.source,
            'is_foot_midpoint': None if position is None else position.is_foot_midpoint,
        }
        self.state_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def _valid_guard_status(payload) -> bool:
        if not isinstance(payload, dict):
            return False
        bool_fields = (
            'pause_requested',
            'handoff_safe',
            'goal_pending',
            'goal_active',
            'cancellation_overdue',
        )
        return (
            all(type(payload.get(field)) is bool for field in bool_fields)
            and isinstance(payload.get('cancellation'), str)
        )

    def _on_detections(self, msg: TargetDetections) -> None:
        self.process_detection(msg)

    def _on_measurements(self, msg: TargetMeasurements) -> None:
        self.process_measurement(msg)

    def _on_guard_status(self, msg: String) -> None:
        self.process_guard_status(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
