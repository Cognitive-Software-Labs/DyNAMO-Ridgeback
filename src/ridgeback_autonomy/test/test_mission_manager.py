"""Phase 2D non-driving mission coordinator tests."""

from __future__ import annotations

import json
import math
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from ridgeback_common.miss_reason import MissReason
from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements
from ridgeback_autonomy.mission.mission_manager_node import (
    FailureClass,
    MissionManagerNode,
    MissionState,
)


BBOX = (100.0, 60.0, 300.0, 460.0)


class WallClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


class FakeTfBuffer:
    def __init__(
        self,
        *,
        x=1.0,
        y=2.0,
        yaw=0.25,
        available=True,
        valid_rotation=True,
        stamp_ns=0,
    ):
        self.stamp_ns = stamp_ns
        self.x = x
        self.y = y
        self.yaw = yaw
        self.available = available
        self.valid_rotation = valid_rotation
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, lookup_time):
        self.calls.append((target_frame, source_frame, lookup_time.nanoseconds))
        if not self.available:
            raise TransformException('transform unavailable')
        transform = TransformStamped()
        transform.header.frame_id = target_frame
        transform.header.stamp.sec = self.stamp_ns // 1_000_000_000
        transform.header.stamp.nanosec = self.stamp_ns % 1_000_000_000
        transform.child_frame_id = source_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        if self.valid_rotation:
            transform.transform.rotation.z = math.sin(self.yaw / 2.0)
            transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        else:
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 0.0
        return transform


class FakePauseClient:
    def __init__(self, response=None, *, ready=True):
        self.ready = ready
        self.response = response
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        if isinstance(self.response, Exception):
            future.set_exception(self.response)
        elif self.response is not None:
            future.set_result(self.response)
        return future


class FakePathClient:
    def server_is_ready(self):
        return False


def trigger_response(success, message=''):
    response = Trigger.Response()
    response.success = success
    response.message = message
    return response


@pytest.fixture
def manager_factory():
    rclpy.init()
    managers = []

    def create(*, tf_buffer=None, pause_client=None, clock=None, overrides=None, path_client=None):
        manager = MissionManagerNode(
            tf_buffer=tf_buffer or FakeTfBuffer(),
            pause_client=pause_client or FakePauseClient(),
            compute_path_client=path_client or FakePathClient(),
            monotonic=clock or WallClock(),
            parameter_overrides=overrides,
        )
        manager.test_ros_ns = 4_000_000_004
        manager.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=manager.test_ros_ns))
        managers.append(manager)
        return manager

    yield create
    for manager in managers:
        manager.destroy_node()
    rclpy.shutdown()


def detection(stamp_ns, *, score=0.8, detected=True):
    msg = TargetDetections()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = 'camera_0_color_optical_frame'
    msg.image_width = 640
    msg.image_height = 480
    msg.detected = detected
    msg.count = 1 if detected else 0
    if detected:
        msg.labels = ['humanoid robot']
        msg.scores = [score]
        msg.bbox_xyxy = list(BBOX)
    return msg


def measurement(stamp_ns, *, valid=True):
    msg = TargetMeasurements()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = 'camera_0_color_optical_frame'
    msg.image_width = 640
    msg.image_height = 480
    msg.detected = True
    msg.count = 1
    msg.labels = ['humanoid robot']
    msg.scores = [0.8]
    msg.bbox_xyxy = list(BBOX)
    lateral = 0.2 if valid else float('nan')
    forward = 2.0 if valid else float('nan')
    distance = math.hypot(lateral, forward) if valid else float('nan')
    msg.euclidean_reconstruction_lateral_m = [lateral]
    msg.euclidean_reconstruction_forward_m = [forward]
    msg.euclidean_reconstruction_distance_m = [distance]
    msg.euclidean_reconstruction_status = [
        int(MissReason.OK if valid else MissReason.TOO_FEW_VALID_POINTS)
    ]
    msg.projective_ranging_lateral_m = [float('nan')]
    msg.projective_ranging_forward_m = [float('nan')]
    msg.projective_ranging_distance_m = [float('nan')]
    msg.projective_ranging_status = [int(MissReason.UNSET)]
    return msg


def guard_status(**updates):
    payload = {
        'pause_requested': True,
        'handoff_safe': False,
        'goal_pending': False,
        'goal_active': True,
        'cancellation': 'awaiting_result',
        'cancellation_overdue': False,
        'reason': 'settling',
    }
    payload.update(updates)
    return String(data=json.dumps(payload))


def start_search(manager):
    manager.tick()
    assert manager.state == MissionState.SEARCH_G1


def confirm_target(manager, *, first_stamp_ns=1_000_000_000):
    for offset in range(3):
        manager.process_detection(detection(first_stamp_ns + offset))


def enter_localization(manager):
    start_search(manager)
    confirm_target(manager)
    assert manager.state == MissionState.SETTLING_MOTION
    settle_motion(manager)
    assert manager.state == MissionState.LOCALIZE_G1


def settle_motion(manager):
    for i in range(21):
        stamp_ns = 2_000_000_000 + i * 100_000_000
        manager._monotonic.value = i * 0.1
        manager.motion.observe((0.,) * 6, stamp_ns, stamp_ns, manager._monotonic())
    manager.tick()


def localize(manager, *, first_stamp_ns=4_000_000_000):
    for offset in range(5):
        stamp_ns = first_stamp_ns + offset
        manager.process_detection(detection(stamp_ns))
        manager.process_measurement(measurement(stamp_ns), now_ns=stamp_ns)


def test_initial_start_pose_is_captured_once_in_configured_map_frame(manager_factory):
    tf_buffer = FakeTfBuffer(x=3.0, y=-2.0, yaw=0.5)
    manager = manager_factory(tf_buffer=tf_buffer)

    manager.tick()
    saved = manager.start_pose
    tf_buffer.x = 99.0
    manager.tick()

    assert manager.state == MissionState.SEARCH_G1
    assert saved is manager.start_pose
    assert saved.header.frame_id == 'map'
    assert saved.pose.position.x == pytest.approx(3.0)
    assert saved.pose.position.y == pytest.approx(-2.0)
    assert tf_buffer.calls == [('map', 'base_link', 0)]


def test_missing_start_pose_tf_keeps_manager_waiting(manager_factory):
    manager = manager_factory(tf_buffer=FakeTfBuffer(available=False))

    manager.tick()

    assert manager.state == MissionState.WAITING_FOR_START_POSE
    assert manager.start_pose is None


def test_invalid_start_pose_transform_is_retried_and_not_saved(manager_factory):
    tf_buffer = FakeTfBuffer(valid_rotation=False)
    manager = manager_factory(tf_buffer=tf_buffer)

    manager.tick()

    assert manager.state == MissionState.WAITING_FOR_START_POSE
    assert manager.start_pose is None
    assert manager.last_issue.code == 'invalid_start_pose_transform'

    tf_buffer.valid_rotation = True
    manager.tick()
    assert manager.state == MissionState.SEARCH_G1
    assert manager.last_issue is None


def test_detection_confirmation_uses_adapter_and_requests_pause(manager_factory):
    client = FakePauseClient()
    manager = manager_factory(pause_client=client)
    start_search(manager)

    confirm_target(manager)

    assert manager.state == MissionState.STOPPING_EXPLORATION
    assert manager.target_committed
    assert manager.confirmed_detection.label == 'humanoid robot'
    assert len(client.calls) == 1


def test_detection_outside_search_does_not_commit_or_transition(manager_factory):
    manager = manager_factory()

    for stamp in range(1, 5):
        manager.process_detection(detection(stamp))

    assert manager.state == MissionState.WAITING_FOR_START_POSE
    assert not manager.target_committed
    assert manager.confirmed_detection is None


def test_repeated_detection_after_commit_does_not_request_another_pause(
        manager_factory):
    client = FakePauseClient()
    manager = manager_factory(pause_client=client)
    start_search(manager)
    confirm_target(manager)
    assert manager.state == MissionState.STOPPING_EXPLORATION

    for stamp in range(10, 15):
        manager.process_detection(detection(stamp))

    assert len(client.calls) == 1
    assert manager.state == MissionState.STOPPING_EXPLORATION


def test_immediate_successful_pause_enters_localization(manager_factory):
    client = FakePauseClient(trigger_response(True, 'safe'))
    manager = manager_factory(pause_client=client)

    enter_localization(manager)

    assert manager.transition_history[-1].previous == MissionState.SETTLING_MOTION
    assert manager.transition_history[-1].current == MissionState.LOCALIZE_G1


def test_pending_pause_waits_for_post_response_safe_status(manager_factory):
    client = FakePauseClient()
    manager = manager_factory(pause_client=client)
    start_search(manager)
    confirm_target(manager)
    _, future = client.calls[-1]
    future.set_result(trigger_response(False, 'waiting for terminal result'))

    manager.process_guard_status(guard_status())
    assert manager.state == MissionState.STOPPING_EXPLORATION
    manager.process_guard_status(guard_status(
        handoff_safe=True,
        goal_active=False,
        cancellation='none',
    ))
    assert manager.state == MissionState.STOPPING_EXPLORATION
    assert len(client.calls) == 2
    _, verification_future = client.calls[-1]
    verification_future.set_result(trigger_response(True, 'safe'))

    assert manager.state == MissionState.SETTLING_MOTION
    settle_motion(manager)
    assert manager.state == MissionState.LOCALIZE_G1


def test_cancellation_failure_is_terminal_not_return_home(manager_factory):
    client = FakePauseClient(trigger_response(False, 'pending'))
    manager = manager_factory(pause_client=client)
    start_search(manager)
    confirm_target(manager)

    manager.process_guard_status(guard_status(
        cancellation='failed',
        reason='cancel transport failed',
    ))
    assert manager.state == MissionState.STOPPING_EXPLORATION
    manager.process_guard_status(guard_status(
        cancellation='failed',
        reason='cancel transport failed',
    ))

    assert manager.state == MissionState.MISSION_FAILED
    assert manager.last_issue.classification == FailureClass.TERMINAL
    assert manager.last_issue.code == 'exploration_cancellation_failed'


def test_cancellation_timeout_status_is_terminal(manager_factory):
    client = FakePauseClient(trigger_response(False, 'pending'))
    manager = manager_factory(pause_client=client)
    start_search(manager)
    confirm_target(manager)

    manager.process_guard_status(guard_status(
        cancellation_overdue=True,
        reason='cancellation overdue',
    ))
    assert manager.state == MissionState.STOPPING_EXPLORATION
    manager.process_guard_status(guard_status(
        cancellation_overdue=True,
        reason='cancellation overdue',
    ))

    assert manager.state == MissionState.MISSION_FAILED
    assert manager.last_issue.code == 'exploration_cancellation_timeout'


def test_stale_latched_safe_status_does_not_acknowledge_new_pause(manager_factory):
    client = FakePauseClient()
    manager = manager_factory(pause_client=client)
    start_search(manager)
    stale = guard_status(
        handoff_safe=True,
        goal_active=False,
        cancellation='none',
    )
    manager.process_guard_status(stale)
    confirm_target(manager)
    _, future = client.calls[-1]
    future.set_result(trigger_response(False, 'pending'))

    assert manager.state == MissionState.STOPPING_EXPLORATION


def test_missing_pause_service_fails_after_bounded_timeout(manager_factory):
    clock = WallClock()
    client = FakePauseClient(ready=False)
    manager = manager_factory(pause_client=client, clock=clock)
    start_search(manager)
    confirm_target(manager)

    clock.value = manager.handoff_timeout_s + 0.1
    manager.tick()

    assert manager.state == MissionState.MISSION_FAILED
    assert manager.last_issue.code == 'pause_service_unavailable'
    assert client.calls == []


def test_failed_pause_requests_are_distinct_from_missing_service(manager_factory):
    clock = WallClock()
    client = FakePauseClient(RuntimeError('service transport failed'))
    manager = manager_factory(pause_client=client, clock=clock)
    start_search(manager)
    confirm_target(manager)

    clock.value = manager.handoff_timeout_s + 0.1
    manager.tick()

    assert manager.state == MissionState.MISSION_FAILED
    assert manager.last_issue.code == 'pause_request_failed'
    assert client.calls


def test_localization_success_stops_at_non_driving_g1_localized(manager_factory):
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )
    enter_localization(manager)

    localize(manager)

    assert manager.state == MissionState.G1_LOCALIZED
    assert manager.target_position.sample_count == 5
    assert manager.target_position.reference == 'generic_target_position'
    assert not manager.target_position.is_foot_midpoint
    assert not hasattr(manager, 'navigate_to_pose_client')
    assert not hasattr(manager, 'return_navigation_client')


def test_localization_timeout_is_technical_failure_not_success(manager_factory):
    clock = WallClock()
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
        clock=clock,
    )
    enter_localization(manager)

    clock.value += manager.localization_timeout_s + 0.1
    manager.tick()

    assert manager.state == MissionState.MISSION_FAILED
    assert manager.last_issue.code == 'target_localization_timeout'
    assert manager.last_issue.classification == FailureClass.TERMINAL


def test_invalid_localization_is_recoverable_while_timeout_remains(manager_factory):
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )
    enter_localization(manager)
    stamp_ns = 4_000_000_000
    manager.process_detection(detection(stamp_ns))

    manager.process_measurement(measurement(stamp_ns, valid=False), now_ns=stamp_ns)

    assert manager.state == MissionState.LOCALIZE_G1
    assert manager.last_issue.classification == FailureClass.RECOVERABLE
    assert manager.last_issue.code == 'localization_sample_rejected'


def test_successful_localization_clears_old_recoverable_issue(manager_factory):
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )
    enter_localization(manager)
    rejected_stamp_ns = 3_000_000_000
    manager.process_detection(detection(rejected_stamp_ns))
    manager.process_measurement(
        measurement(rejected_stamp_ns, valid=False),
        now_ns=rejected_stamp_ns,
    )
    assert manager.last_issue.classification == FailureClass.RECOVERABLE

    localize(manager)

    assert manager.state == MissionState.G1_LOCALIZED
    assert manager.last_issue is None


def test_localization_tf_failure_is_recoverable(manager_factory):
    tf_buffer = FakeTfBuffer()
    manager = manager_factory(
        tf_buffer=tf_buffer,
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )
    enter_localization(manager)
    tf_buffer.available = False
    stamp_ns = 4_000_000_000
    manager.process_detection(detection(stamp_ns))

    manager.process_measurement(measurement(stamp_ns), now_ns=stamp_ns)

    assert manager.state == MissionState.LOCALIZE_G1
    assert 'transform_unavailable' in manager.last_issue.reason


def test_state_transition_order_is_deterministic(manager_factory):
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )

    enter_localization(manager)
    localize(manager)

    assert [transition.current for transition in manager.transition_history] == [
        MissionState.SEARCH_G1,
        MissionState.STOPPING_EXPLORATION,
        MissionState.SETTLING_MOTION,
        MissionState.LOCALIZE_G1,
        MissionState.G1_LOCALIZED,
    ]


def test_repeated_measurement_does_not_duplicate_localization_sample(manager_factory):
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )
    enter_localization(manager)
    stamp_ns = 4_000_000_000
    manager.process_detection(detection(stamp_ns))
    msg = measurement(stamp_ns)

    manager.process_measurement(msg, now_ns=stamp_ns)
    manager.process_measurement(msg, now_ns=stamp_ns)

    assert manager.localization_adapter.sample_count_for(
        'euclidean_reconstruction'
    ) == 1
    assert manager.state == MissionState.LOCALIZE_G1


def test_manager_reset_clears_commitment_samples_and_start_pose(manager_factory):
    manager = manager_factory(
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
    )
    enter_localization(manager)
    stamp_ns = 4_000_000_000
    manager.process_detection(detection(stamp_ns))
    manager.process_measurement(measurement(stamp_ns), now_ns=stamp_ns)

    manager.reset_mission()

    assert manager.state == MissionState.WAITING_FOR_START_POSE
    assert manager.start_pose is None
    assert not manager.target_committed
    assert manager.confirmed_detection is None
    assert manager.localization_adapter.sample_count_for(
        'euclidean_reconstruction'
    ) == 0


def test_late_pause_response_after_reset_cannot_advance_old_attempt(manager_factory):
    client = FakePauseClient()
    manager = manager_factory(pause_client=client)
    start_search(manager)
    confirm_target(manager)
    _, old_future = client.calls[-1]

    manager.reset_mission(clear_start_pose=False)
    old_future.set_result(trigger_response(True, 'old attempt safe'))

    assert manager.state == MissionState.SEARCH_G1
    assert not manager.target_committed
    assert manager.target_position is None


def test_operator_cancel_is_distinct_from_terminal_failure(manager_factory):
    manager = manager_factory()
    start_search(manager)

    manager.cancel_mission()

    assert manager.state == MissionState.MISSION_CANCELED
    assert manager.last_issue.classification == FailureClass.OPERATOR_CANCELED


def test_state_diagnostics_identify_non_driving_phase(manager_factory):
    manager = manager_factory()
    messages = []
    manager.state_publisher.publish = messages.append

    manager.tick()

    payload = json.loads(messages[-1].data)
    assert payload['state'] == 'SEARCH_G1'
    assert payload['parking_navigation_enabled'] is False
    assert payload['start_pose_saved'] is True


def test_no_return_or_mission_complete_states_exist_in_phase_2d():
    assert 'RETURNING_TO_START' not in MissionState.__members__
    assert 'MISSION_COMPLETE' not in MissionState.__members__


# Final localization sample time; map/TF must not be dated after planning.
PLAN_NOW_NS = 4_000_000_004


def occupancy_map(*, value=0, stamp_ns=PLAN_NOW_NS, frame='map', size=120):
    from nav_msgs.msg import OccupancyGrid

    grid = OccupancyGrid()
    grid.header.frame_id = frame
    grid.header.stamp.sec = stamp_ns // 1_000_000_000
    grid.header.stamp.nanosec = stamp_ns % 1_000_000_000
    grid.info.resolution = 0.05
    grid.info.width = size
    grid.info.height = size
    grid.info.origin.orientation.w = 1.0
    grid.data = [value] * (size * size)
    return grid


def planning_manager(manager_factory, *, map_msg=None, clock=None):
    manager = manager_factory(
        tf_buffer=FakeTfBuffer(stamp_ns=PLAN_NOW_NS),
        pause_client=FakePauseClient(trigger_response(True, 'safe')),
        clock=clock,
    )
    published = {'markers': [], 'state': []}
    manager.parking_marker_publisher.publish = published['markers'].append
    manager.state_publisher.publish = published['state'].append
    if map_msg is not None:
        manager._on_map(map_msg)
    enter_localization(manager)
    localize(manager)
    return manager, published


def test_localization_with_fresh_map_generates_parking_candidates(manager_factory):
    manager, published = planning_manager(manager_factory, map_msg=occupancy_map())

    assert manager.state == MissionState.EVALUATING_PARKING_PATHS
    plan = manager.parking_plan
    assert not plan.ok and plan.selected is None
    assert plan.counts()['valid'] > 0
    assert plan.target.reference == 'generic_target_position'
    assert plan.keepout_radius_m == pytest.approx(0.29)
    assert published['markers'][-1].markers[0].action == 3   # DELETEALL first
    payload = json.loads(published['state'][-1].data)
    assert payload['state'] == 'EVALUATING_PARKING_PATHS'
    assert payload['parking_navigation_enabled'] is False
    assert payload['parking_plan']['counts']['valid'] > 0
    assert not hasattr(manager, 'navigate_to_pose_client')
    assert isinstance(manager.compute_path_client, FakePathClient)


def test_missing_map_waits_then_fails_explicitly(manager_factory):
    clock = WallClock()
    manager, published = planning_manager(manager_factory, clock=clock)

    assert manager.state == MissionState.G1_LOCALIZED
    assert manager.last_issue is None
    assert json.loads(published['state'][-1].data)['event'] == 'planning_pending'

    clock.value += manager.parking_input_timeout_s + 0.1
    manager.try_plan_parking(clock.value, now_ns=PLAN_NOW_NS)

    assert manager.state == MissionState.PARKING_PLANNING_FAILED
    assert manager.last_issue.code == 'parking_inputs_invalid'
    assert 'map_unavailable' in manager.last_issue.reason


def test_stale_target_fails_without_planning(manager_factory):
    manager, _ = planning_manager(manager_factory)
    manager._on_map(occupancy_map(stamp_ns=PLAN_NOW_NS + 6_000_000_000))
    manager.tf_buffer.stamp_ns = PLAN_NOW_NS + 6_000_000_000

    manager.try_plan_parking(0.0, now_ns=PLAN_NOW_NS + 6_000_000_000)

    assert manager.state == MissionState.PARKING_PLANNING_FAILED
    assert 'target_stale' in manager.last_issue.reason
    assert manager.parking_plan is None


def test_blocked_donut_reports_planning_failure_reason(manager_factory):
    manager, published = planning_manager(manager_factory, map_msg=occupancy_map(value=100))

    assert manager.state == MissionState.PARKING_PLANNING_FAILED
    assert manager.last_issue.code == 'no_free_donut_cells'
    markers = published['markers'][-1].markers
    assert markers[0].action == 3 and not any(m.ns.startswith('parking/selected') for m in markers)


def test_reset_clears_parking_markers_and_plan(manager_factory):
    manager, published = planning_manager(manager_factory, map_msg=occupancy_map())
    assert manager.state == MissionState.EVALUATING_PARKING_PATHS

    manager.reset_mission()

    assert manager.parking_plan is None and manager.approach_pose is None
    cleared = published['markers'][-1].markers
    assert len(cleared) == 1 and cleared[0].action == 3


@pytest.mark.parametrize('problem', ['short_data', 'origin_nan', 'zero_quaternion', 'resolution_nan'])
def test_malformed_map_fails_planning_without_escaping_callback(manager_factory, problem):
    msg = occupancy_map()
    if problem == 'short_data':
        msg.data = [0]
    elif problem == 'origin_nan':
        msg.info.origin.position.x = float('nan')
    elif problem == 'zero_quaternion':
        msg.info.origin.orientation.w = 0.0
    else:
        msg.info.resolution = float('nan')
    manager, published = planning_manager(manager_factory, map_msg=msg)
    assert manager.state == MissionState.PARKING_PLANNING_FAILED
    assert manager.last_issue.code == 'parking_inputs_invalid'
    assert 'map_invalid' in manager.last_issue.reason
    assert manager.parking_plan is None
    assert published['markers'][-1].markers[0].action == 3


@pytest.mark.parametrize('problem', ['missing', 'zero_quaternion'])
def test_planning_waits_for_valid_robot_tf_then_recovers(manager_factory, problem):
    manager, _ = planning_manager(manager_factory)
    manager._on_map(occupancy_map())
    if problem == 'missing':
        manager.tf_buffer.available = False
    else:
        manager.tf_buffer.valid_rotation = False
    manager.try_plan_parking(0.0, now_ns=PLAN_NOW_NS)
    assert manager.state == MissionState.G1_LOCALIZED
    assert manager.parking_plan is None
    manager.tf_buffer.available = manager.tf_buffer.valid_rotation = True
    manager.try_plan_parking(0.1, now_ns=PLAN_NOW_NS)
    assert manager.state == MissionState.EVALUATING_PARKING_PATHS


def test_repeated_callbacks_and_cancel_do_not_regenerate_plan(manager_factory):
    manager, published = planning_manager(manager_factory, map_msg=occupancy_map())
    plan = manager.parking_plan
    marker_count = len(published['markers'])
    manager._on_map(occupancy_map(value=100))
    manager.process_detection(detection(PLAN_NOW_NS + 1))
    manager.process_measurement(measurement(PLAN_NOW_NS + 1), now_ns=PLAN_NOW_NS + 1)
    manager.try_plan_parking(1.0, now_ns=PLAN_NOW_NS + 1)
    assert manager.parking_plan is plan
    assert len(published['markers']) == marker_count
    manager.cancel_mission()
    manager.try_plan_parking(2.0, now_ns=PLAN_NOW_NS + 2)
    assert manager.state == MissionState.MISSION_CANCELED
    assert len(published['markers']) == marker_count + 1
    assert published['markers'][-1].markers[0].action == 3


@pytest.mark.parametrize('source', ['map', 'robot_pose'])
def test_future_planning_input_is_a_technical_failure(manager_factory, source):
    manager, _ = planning_manager(manager_factory)
    manager._on_map(occupancy_map(
        stamp_ns=PLAN_NOW_NS + (1 if source == 'map' else 0)))
    if source == 'robot_pose':
        manager.tf_buffer.stamp_ns = PLAN_NOW_NS + 1
    manager.try_plan_parking(0.0, now_ns=PLAN_NOW_NS)
    assert manager.state == MissionState.PARKING_PLANNING_FAILED
    assert manager.last_issue.code == 'parking_inputs_invalid'
    assert f'{source}_from_future' in manager.last_issue.reason
    assert manager.parking_plan is None
