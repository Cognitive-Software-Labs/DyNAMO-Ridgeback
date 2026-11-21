"""Camera observation turns at frontier stops (parking mission only): bounded Nav2 Spin turns
owned by the explorer's MissionGuard, a brief wait for usable frames at each view, and no
retries. A pause (the mission's G1 confirmation) cancels a turn through the guard."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_srvs.srv import Trigger

from ridgeback_interfaces.msg import TargetDetections

POSE = (5.0, 5.0, 0.0)
OFFSET = (0.27, 0.0)


def grid_msg(size_m=10.0, value=0, resolution=0.05):
    n = int(round(size_m / resolution))
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width = msg.info.height = n
    msg.info.origin.orientation.w = 1.0
    msg.data = [value] * (n * n)
    return msg


class SpinClient:
    def __init__(self, ready=True):
        self.ready, self.calls = ready, []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        future = Future()
        self.calls.append((goal, future))
        return future


class Handle:
    def __init__(self, accepted=True):
        self.accepted, self.cancels = accepted, 0
        self.result = Future()

    def cancel_goal_async(self):
        self.cancels += 1
        return Future()

    def get_result_async(self):
        return self.result


@pytest.fixture
def explorer():
    from ridgeback_autonomy.frontier_explorer.frontier_explorer_node import FrontierExplorerNode
    rclpy.init()
    nodes = []

    def create(enabled=True, **overrides):
        params = [Parameter('camera_observation_enabled', value=enabled)]
        params += [Parameter('camera_observation_' + k, value=v) for k, v in overrides.items()]
        node = FrontierExplorerNode(parameter_overrides=params)
        nodes.append(node)
        if not enabled:
            return node
        o = node.observer
        node.current_costmap = grid_msg()
        o.on_map(grid_msg())
        o.spin_client = SpinClient()
        state = {'pose': POSE, 'camera_yaw': POSE[2], 'turning': False}
        o._robot_pose = lambda: state['pose']
        o._offset = lambda: OFFSET
        # A turning camera sweeps at 1 rad/s; a steady one points at camera_yaw.
        o.camera_pose = lambda stamp, s=state: (s['pose'][0], s['pose'][1], s['camera_yaw'] + (
            stamp.nanoseconds * 1e-9 if s['turning'] else 0.0))
        o.motion.ready = lambda *a: True
        node.test_state = state
        return node

    yield create
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def arrive(node):
    """A frontier goal succeeded; the stop is confirmed on the next poll."""
    with node.mission_guard.lock:
        started = node.observer.begin(focused=False)
        if started:
            node.observer.poll()
    return started


def accept(node, *, accepted=True):
    goal, future = node.observer.spin_client.calls[-1]
    handle = Handle(accepted)
    future.set_result(handle)
    return goal, handle


def end_turn(node, handle, status=GoalStatus.STATUS_SUCCEEDED):
    view = node.observer.round['views'][node.observer.round['index']] if node.observer.round else None
    if view is not None and status == GoalStatus.STATUS_SUCCEEDED:
        node.test_state['camera_yaw'] = view[0]
    handle.result.set_result(SimpleNamespace(status=status, result=SimpleNamespace(error_code=0)))


def frames(node, count, *, after_ns=1):
    now = node.get_clock().now().nanoseconds
    for k in range(count):
        msg = TargetDetections()
        stamp = now + after_ns + k
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        node.observer.on_frame(msg)
    with node.mission_guard.lock:
        node.observer.poll()


def test_off_by_default_and_for_other_exploration_users(explorer):
    node = explorer(enabled=False)
    assert not node.observer.enabled and not node.observer.begin(focused=False)
    assert not node.observer.active


def test_a_stop_turns_toward_unseen_directions_through_the_guard(explorer):
    node = explorer()
    assert arrive(node)
    o, guard = node.observer, node.mission_guard
    assert len(o.round['views']) == 3 and len(o.spin_client.calls) == 1
    assert guard.goal_pending                                   # the guard owns the turn
    goal, handle = accept(node)
    assert guard.handle is handle and not guard.goal_pending
    assert goal.target_yaw == pytest.approx(o.round['views'][0][1])
    assert goal.time_allowance.sec == 15
    end_turn(node, handle)
    assert guard.handle is None and o.round['phase'] == 'dwelling'
    frames(node, 5)
    assert o.round['seen'] == 1 and len(o.spin_client.calls) == 2   # the next view's turn
    for _ in range(2):
        _, handle = accept(node)
        end_turn(node, handle)
        frames(node, 5)
    assert not o.active and o.history[-1]['views_seen'] == 3
    assert len(o.spin_client.calls) == 3                         # at most three views


def test_a_view_without_usable_frames_is_not_counted_and_never_retried_here(explorer):
    node = explorer(dwell_timeout_s=0.01, max_views=1)
    arrive(node)
    o = node.observer
    _, handle = accept(node)
    heading = o.round['views'][0][0]
    end_turn(node, handle)
    frames(node, 2)                                              # too few, then the wait ends
    o.round['deadline'] = 0.0
    with node.mission_guard.lock:
        o.poll()
    assert not o.active and o.history[-1]['views_seen'] == 0
    assert any(abs(h - heading) < 1e-9 for _, _, h in o.attempts)
    arrive(node)                                                 # the same stop again
    assert all(abs(math.remainder(h - heading, 2 * math.pi)) >= o.coverage.model.half_fov_rad
               for h, _, _ in o.round['views'])


def test_frames_while_turning_fast_or_before_the_turn_ended_do_not_count(explorer):
    node = explorer(max_views=1)
    arrive(node)
    o = node.observer
    _, handle = accept(node)
    end_turn(node, handle)
    seen = int(o.coverage.seen.sum())
    node.test_state['turning'] = True                           # camera still sweeping
    now = node.get_clock().now().nanoseconds
    for k in range(6):
        msg = TargetDetections()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now + 2 * k + 1, 1_000_000_000)
        o.on_frame(msg)
    assert o.round['frames'] == 0 and int(o.coverage.seen.sum()) == seen
    node.test_state['turning'] = False
    frames(node, 3, after_ns=-10_000_000_000)                   # taken before the turn ended
    assert o.round is not None and o.round['frames'] == 0


def test_covered_directions_are_not_observed_again(explorer):
    node = explorer()
    o = node.observer
    for k in range(36):
        yaw = math.radians(10 * k)
        o.coverage.mark(o.map, POSE[0] + OFFSET[0] * math.cos(yaw), POSE[1] + OFFSET[0] * math.sin(yaw), yaw)
    arrive(node)
    assert not o.active and o.spin_client.calls == []
    assert o.history[-1]['outcome'] == 'no useful unseen direction'


def test_a_turn_the_footprint_cannot_sweep_is_never_sent(explorer):
    node = explorer()
    msg = grid_msg()
    n = msg.info.width
    data = np.array(msg.data, dtype=np.int16).reshape(n, n)
    data[int(5.45 / 0.05), int(5.0 / 0.05) - 2:int(5.0 / 0.05) + 3] = 100   # left, inside the sweep
    data[int(4.55 / 0.05), int(5.0 / 0.05) - 2:int(5.0 / 0.05) + 3] = 100   # and right
    msg.data = data.ravel().tolist()
    node.current_costmap = msg
    arrive(node)
    o = node.observer
    assert not o.active and o.spin_client.calls == []
    assert 'not safe' in o.history[-1]['outcome'] and o.attempts


@pytest.mark.parametrize('status', [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED])
def test_an_aborted_or_cancelled_turn_ends_the_stop_without_a_retry(explorer, status):
    node = explorer()
    arrive(node)
    _, handle = accept(node)
    end_turn(node, handle, status)
    o = node.observer
    assert not o.active and len(o.spin_client.calls) == 1
    assert node.mission_guard.handle is None and 'ended: status' in o.history[-1]['outcome']


def test_a_rejected_or_unavailable_spin_ends_the_stop(explorer):
    node = explorer()
    arrive(node)
    accept(node, accepted=False)
    assert not node.observer.active and not node.mission_guard.goal_pending
    assert node.observer.history[-1]['outcome'] == 'spin rejected by Nav2'
    node.observer.spin_client.ready = False
    arrive(node)
    assert node.observer.history[-1]['outcome'] == 'Nav2 spin server not ready'


def test_an_overdue_turn_is_cancelled_once_through_the_guard(explorer):
    node = explorer()
    arrive(node)
    _, handle = accept(node)
    node.observer.round['deadline'] = 0.0
    for _ in range(3):
        with node.mission_guard.lock:
            node.observer.poll()
    assert handle.cancels == 1
    end_turn(node, handle, GoalStatus.STATUS_CANCELED)
    assert not node.observer.active and node.mission_guard.handle is None


def test_a_g1_confirmation_pause_cancels_the_turn_and_releases_ownership(explorer):
    node = explorer()
    arrive(node)
    _, handle = accept(node)
    guard = node.mission_guard
    response = guard.request_pause(Trigger.Request(), Trigger.Response())  # the mission's pause
    assert not response.success and handle.cancels == 1           # handoff waits for the turn
    end_turn(node, handle, GoalStatus.STATUS_CANCELED)
    assert guard.handoff_safe() and not node.observer.active
    node.exploration_callback()                                   # paused: nothing new is sent
    assert len(node.observer.spin_client.calls) == 1


def test_a_pause_while_waiting_for_frames_ends_the_stop(explorer):
    node = explorer()
    arrive(node)
    _, handle = accept(node)
    end_turn(node, handle)
    node.mission_guard.request_pause(Trigger.Request(), Trigger.Response())
    node.exploration_callback()
    assert not node.observer.active and node.observer.history[-1]['outcome'] == 'exploration paused'
    assert node.mission_guard.handoff_safe()


def test_no_frontier_is_selected_and_no_completion_reported_while_observing(explorer):
    node = explorer()
    arrive(node)
    node._startup_delay = 0.0
    node.robot_awareness_map = np.zeros((4, 4), dtype=np.int8)
    node.get_robot_position = lambda: (0, 0)
    selected = []
    node.select_new_frontier_goal = lambda pos: selected.append(pos)
    node.exploration_callback()
    assert selected == [] and node.observer.active


def test_unconfirmed_stop_focus_and_budget_skip_the_observations(explorer):
    node = explorer()
    o = node.observer
    o.motion.ready = lambda *a: False
    with node.mission_guard.lock:
        assert o.begin(focused=False)
        o.round['deadline'] = 0.0
        o.poll()
    assert not o.active and o.history[-1]['outcome'].startswith('stop not confirmed')
    assert o.spin_client.calls == []
    with node.mission_guard.lock:
        assert not o.begin(focused=True)                          # mapping around a found G1
        o.spent_s = o.budget_s
        assert not o.begin(focused=False)
    assert 'budget' in o.history[-1]['reason']
