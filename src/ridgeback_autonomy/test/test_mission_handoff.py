"""Explorer-to-mission Nav2 ownership handoff tests."""

import json
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_srvs.srv import Trigger

from ridgeback_autonomy.frontier_explorer.frontier_explorer_node import (
    FrontierExplorerNode,
)
from ridgeback_autonomy.frontier_explorer.mission_guard import MissionGuard
from ridgeback_interfaces.msg import LocalizationHealth


class CancelHandle:
    def __init__(self):
        self.cancel_future = Future()
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


@pytest.fixture
def guard_factory():
    rclpy.init()
    nodes = []

    def create(required=False):
        node = Node(
            f'mission_handoff_test_{len(nodes)}',
            parameter_overrides=[Parameter('localization_required', value=required)],
        )
        nodes.append(node)
        return MissionGuard(node)

    yield create
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def pause(guard):
    return guard.request_pause(Trigger.Request(), Trigger.Response())


def health(count=1, state='processing'):
    return LocalizationHealth(
        session_id='test',
        state=state,
        streams=['image', 'detections'],
        progress=[count, count],
        source_stamps=[Time(sec=count), Time(sec=count)],
    )


def make_healthy(guard):
    guard.observe(health(1))
    guard.observe(health(2))
    assert guard.resume(Trigger.Request(), Trigger.Response()).success


def settle_cancel_request(handle):
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))


def test_pause_while_idle_is_immediately_acknowledged_and_idempotent(guard_factory):
    guard = guard_factory()
    messages = []
    guard.publisher.publish = messages.append

    first = pause(guard)
    second = pause(guard)

    assert first.success and second.success
    assert guard.handoff_safe()
    assert not guard.allowed()
    status = json.loads(messages[-1].data)
    assert status['pause_requested'] is True
    assert status['handoff_safe'] is True
    assert status['goal_active'] is False


def test_pause_during_active_goal_waits_for_terminal_result(guard_factory):
    guard = guard_factory()
    handle = CancelHandle()
    guard.accepted(handle)

    response = pause(guard)
    assert not response.success
    assert handle.cancel_calls == 1
    assert guard.cancel_state == 'request_pending'
    assert not guard.handoff_safe()

    settle_cancel_request(handle)
    assert guard.cancel_state == 'awaiting_result'
    assert not pause(guard).success
    assert handle.cancel_calls == 1

    guard.finished(handle)
    assert guard.handoff_safe()
    assert pause(guard).success


def test_pause_during_goal_acceptance_cancels_late_accepted_goal(guard_factory):
    guard = guard_factory()
    guard.goal_pending = True

    assert not pause(guard).success
    assert not guard.handoff_safe()

    handle = CancelHandle()
    guard.accepted(handle)
    assert handle.cancel_calls == 1
    assert guard.handle is handle
    assert guard.cancel_state == 'request_pending'
    assert not guard.handoff_safe()


@pytest.mark.parametrize(
    'status',
    [
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    ],
)
def test_any_terminal_result_settles_goal_during_pause(guard_factory, status):
    guard = guard_factory()
    handle = CancelHandle()
    guard.accepted(handle)
    assert not pause(guard).success
    settle_cancel_request(handle)

    logger = SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)
    owner = SimpleNamespace(
        mission_guard=guard,
        _current_goal_handle=handle,
        current_goal=None,
        _visited_goals=[],
        _abort_retry_delay=1.0,
        _retry_after_wall_time=0.0,
        get_logger=lambda: logger,
    )
    result = Future()
    result.set_result(SimpleNamespace(status=status))

    FrontierExplorerNode._handle_goal_result(owner, result, handle)

    assert guard.handoff_safe()
    assert owner._current_goal_handle is None


@pytest.mark.parametrize('outcome', ['rejected', 'exception'])
def test_cancel_failure_never_claims_safe_handoff(guard_factory, outcome):
    guard = guard_factory()
    handle = CancelHandle()
    guard.accepted(handle)
    assert not pause(guard).success

    if outcome == 'rejected':
        handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[]))
        assert guard.cancel_state == 'rejected'
        expected_reason = 'Cancellation rejected; awaiting terminal result'
    else:
        handle.cancel_future.set_exception(RuntimeError('cancel transport failed'))
        assert guard.cancel_state == 'failed'
        expected_reason = 'Cancellation failed: cancel transport failed'

    assert not guard.handoff_safe()
    assert not pause(guard).success
    assert handle.cancel_calls == 1
    assert guard.reason == expected_reason
    assert not guard.resume(Trigger.Request(), Trigger.Response()).success


def test_cancellation_timeout_is_reported_but_never_releases_ownership(
        guard_factory, monkeypatch):
    guard = guard_factory()
    clock = [100.0]
    monkeypatch.setattr(
        'ridgeback_autonomy.frontier_explorer.mission_guard.time.monotonic',
        lambda: clock[0],
    )
    handle = CancelHandle()
    guard.accepted(handle)
    assert not pause(guard).success
    settle_cancel_request(handle)

    clock[0] += guard.cancel_timeout + 0.1
    guard.tick()

    assert guard.cancel_overdue
    assert guard.cancel_state == 'awaiting_result'
    assert guard.handle is handle
    assert not guard.handoff_safe()


def test_pending_goal_acceptance_timeout_is_not_a_safe_handoff(
        guard_factory, monkeypatch):
    guard = guard_factory()
    clock = [200.0]
    monkeypatch.setattr(
        'ridgeback_autonomy.frontier_explorer.mission_guard.time.monotonic',
        lambda: clock[0],
    )
    guard.goal_pending = True
    assert not pause(guard).success

    clock[0] += guard.cancel_timeout + 0.1
    guard.tick()

    assert guard.cancel_overdue
    assert guard.goal_pending
    assert not guard.handoff_safe()


def test_resume_after_safe_pause_allows_explorer_again(guard_factory):
    guard = guard_factory()
    assert pause(guard).success

    response = guard.resume(Trigger.Request(), Trigger.Response())

    assert response.success
    assert not guard.pause_requested
    assert not guard.handoff_safe()
    assert guard.allowed()


def test_resume_is_blocked_when_required_localization_is_unhealthy(guard_factory):
    guard = guard_factory(required=True)
    make_healthy(guard)
    assert pause(guard).success
    guard.observe(health(3, state='failure'))

    response = guard.resume(Trigger.Request(), Trigger.Response())

    assert not response.success
    assert guard.paused
    assert guard.pause_requested


def test_explorer_cannot_send_overlapping_navigation_goal(guard_factory):
    guard = guard_factory()
    guard.handle = object()
    sends = []
    owner = SimpleNamespace(
        mission_guard=guard,
        _send_goal_to_nav2=lambda target: sends.append(target) or True,
    )

    assert not FrontierExplorerNode.send_goal_to_nav2(owner, (1.0, 2.0))
    assert sends == []


def test_internal_cancel_keeps_handle_until_terminal_result(guard_factory):
    guard = guard_factory()
    handle = CancelHandle()
    guard.accepted(handle)
    owner = SimpleNamespace(mission_guard=guard)

    FrontierExplorerNode._cancel_current_goal(owner)

    assert handle.cancel_calls == 1
    assert guard.handle is handle
    assert guard.cancel_state == 'request_pending'
