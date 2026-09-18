from types import SimpleNamespace
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
from ridgeback_interfaces.msg import LocalizationHealth
from ridgeback_localization.health import PipelineHealth
from ridgeback_autonomy.frontier_explorer.mission_guard import MissionGuard


def test_pipeline_startup_stale_stalled_failure_and_empty_current_output():
    h = PipelineHealth(['image'], ['detections', 'mask'], now=0, startup_timeout=2,
                       input_timeout=1, progress_timeout=2, source_age=3)
    assert h.state(0, 0)[0] == 'startup'
    assert h.state(3, 0)[0] == 'stale_input'
    h.observe('image', 3*10**9, 3)
    assert h.state(3, 3*10**9)[0] == 'stalled'
    for name in ('detections','mask'): h.observe(name, 3*10**9, 3)
    # Count/target presence is deliberately absent: an empty current result is healthy.
    assert h.state(3, 3*10**9)[0] == 'processing'
    h.observe('image', 6*10**9, 6)
    assert h.state(6, 6*10**9)[0] == 'stalled'
    h.failures['detector'] = 'OOM'
    assert h.state(6, 6*10**9)[0] == 'failure'


def test_republished_old_source_stamp_does_not_refresh_progress():
    h = PipelineHealth(['image'], ['detections'], now=0, startup_timeout=1, input_timeout=1)
    for name in h.streams: h.observe(name, 10**9, 0)
    assert h.state(0, 10**9)[0] == 'processing'
    for name in h.streams: h.observe(name, 10**9, 2)
    assert h.state(2, 3*10**9)[0] == 'stale_input'


@pytest.fixture
def guard():
    rclpy.init()
    node = Node('guard_test', parameter_overrides=[Parameter('localization_required', value=True)])
    g = MissionGuard(node)
    yield g
    node.destroy_node()
    rclpy.shutdown()


def health(count=1, state='processing', epoch='one'):
    return LocalizationHealth(session_id=epoch, state=state, streams=['image','detections'],
        progress=[count,count], source_stamps=[Time(sec=count), Time(sec=count)])


def make_ready(g):
    g.observe(health(1)); g.observe(health(2))
    assert g.resume(None, Trigger.Response()).success


def test_startup_and_recovery_require_explicit_resume(guard):
    assert not guard.allowed()
    assert not guard.resume(None, Trigger.Response()).success
    make_ready(guard)
    assert guard.allowed()
    guard.observe(health(3, 'stale_input'))
    guard.observe(health(4))
    assert not guard.allowed()
    assert guard.resume(None, Trigger.Response()).success
    assert guard.allowed()


def test_missing_health_frozen_progress_and_process_restart_latch_pause(guard, monkeypatch):
    make_ready(guard)
    guard.last_health = time.monotonic() - 4
    guard.tick()
    assert guard.paused
    make_ready(guard)
    guard.progress = {k:(v[0], v[1], time.monotonic()-16, True) for k,v in guard.progress.items()}
    guard.observe(health(2)) # heartbeat with frozen progress
    guard.tick()
    assert guard.paused and not guard.healthy()
    guard.observe(health(3, epoch='restarted'))
    guard.observe(health(4, epoch='restarted'))
    assert guard.healthy() and guard.paused


@pytest.mark.parametrize('outcome', ['accepted','rejected','exception','pending'])
def test_cancellation_never_unlocks_before_terminal_result(guard, outcome):
    make_ready(guard)
    future = Future()
    handle = SimpleNamespace(cancel_goal_async=lambda: future)
    guard.accepted(handle)
    guard.observe(health(3, 'failure'))
    assert guard.cancel_state == 'request_pending'
    if outcome == 'exception': future.set_exception(RuntimeError('transport lost'))
    elif outcome != 'pending': future.set_result(SimpleNamespace(goals_canceling=[1] if outcome=='accepted' else []))
    guard.observe(health(4))
    assert not guard.resume(None, Trigger.Response()).success
    assert guard.paused
    guard.finished(handle)
    assert guard.resume(None, Trigger.Response()).success


def test_health_loss_during_pending_goal_cancels_on_late_acceptance(guard):
    make_ready(guard)
    guard.goal_pending = True
    guard.observe(health(3, 'failure'))
    assert not guard.resume(None, Trigger.Response()).success
    calls=[]
    handle=SimpleNamespace(cancel_goal_async=lambda: calls.append(True) or Future())
    guard.accepted(handle)
    assert calls == [True]
    assert guard.cancel_state == 'request_pending'


def test_fake_nav2_server_receives_cancel_and_terminal_result(guard):
    import threading
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import NavigateToPose
    node = guard.node
    canceled = threading.Event()
    def execute(handle):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if handle.is_cancel_requested:
                handle.canceled(); canceled.set()
                return NavigateToPose.Result()
            time.sleep(.01)
        handle.abort()
        return NavigateToPose.Result()
    group=ReentrantCallbackGroup()
    server=ActionServer(node, NavigateToPose, 'fake_navigate', execute_callback=execute,
        cancel_callback=lambda _:CancelResponse.ACCEPT, callback_group=group)
    client=ActionClient(node, NavigateToPose, 'fake_navigate', callback_group=group)
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert client.wait_for_server(timeout_sec=3)
        make_ready(guard)
        sent=client.send_goal_async(NavigateToPose.Goal())
        deadline=time.monotonic()+3
        while not sent.done() and time.monotonic()<deadline: time.sleep(.01)
        assert sent.done()
        handle=sent.result(); assert handle.accepted
        guard.accepted(handle)
        result=handle.get_result_async()
        result.add_done_callback(lambda _:guard.finished(handle))
        guard.observe(health(3, 'stalled'))
        assert canceled.wait(3)
        deadline=time.monotonic()+3
        while guard.handle is not None and time.monotonic()<deadline: time.sleep(.01)
        assert guard.handle is None and guard.paused
        guard.observe(health(4))
        assert guard.resume(None, Trigger.Response()).success
    finally:
        executor.shutdown();thread.join(3)
        client.destroy();server.destroy()


def test_frontier_owner_blocks_goals_until_resume_and_keeps_unknown_result_paused():
    from ridgeback_autonomy.frontier_explorer.frontier_explorer_node import FrontierExplorerNode
    rclpy.init()
    node=FrontierExplorerNode(parameter_overrides=[Parameter('localization_required',value=True)])
    try:
        assert not node.send_goal_to_nav2((0.0,0.0))
        make_ready(node.mission_guard)
        node.current_costmap=SimpleNamespace(header=SimpleNamespace(frame_id='map'))
        sent=[]
        def discover(**kwargs):
            node.mission_guard.last_health=time.monotonic()-4
            return True
        node.nav_client.wait_for_server=discover
        node.nav_client.send_goal_async=lambda *args,**kwargs:sent.append(True)
        assert not node.send_goal_to_nav2((0.0,0.0))
        assert not sent
        make_ready(node.mission_guard)
        handle=SimpleNamespace(cancel_goal_async=lambda:Future())
        node._current_goal_handle=handle
        node.mission_guard.accepted(handle)
        future=Future();future.set_exception(RuntimeError('result connection lost'))
        node._goal_result_callback(future,handle)
        assert node.mission_guard.paused
        assert node.mission_guard.handle is handle
        assert not node.mission_guard.resume(None,Trigger.Response()).success
    finally:
        node.destroy_node();rclpy.shutdown()


def test_late_cancel_reply_cannot_modify_a_new_goal(guard):
    make_ready(guard)
    reply=Future()
    old=SimpleNamespace(cancel_goal_async=lambda:reply)
    guard.accepted(old);guard.pause('lost health');guard.finished(old)
    guard.observe(health(3));assert guard.resume(None,Trigger.Response()).success
    new=SimpleNamespace(cancel_goal_async=lambda:Future())
    guard.accepted(new)
    reply.set_result(SimpleNamespace(goals_canceling=[]))
    assert guard.handle is new and guard.cancel_state=='none' and not guard.paused


def test_health_node_reports_selected_contract_and_dead_worker_without_inference():
    import json
    from std_msgs.msg import String
    from ridgeback_localization.health_node import LocalizationHealthNode
    from rclpy.serialization import serialize_message, deserialize_message
    rclpy.init()
    node=LocalizationHealthNode(parameter_overrides=[Parameter('estimators',value='projective_ranging')])
    try:
        node.count_publishers=lambda _:1
        now=node.get_clock().now().nanoseconds
        for name in node.health.streams: node.health.observe(name,now,time.monotonic())
        messages=[];node.pub.publish=messages.append
        node.publish_health()
        message=messages[-1]
        assert message.state=='processing'
        assert list(message.estimators)==['projective_ranging']
        assert list(message.labels)==['humanoid robot']
        assert deserialize_message(serialize_message(message),LocalizationHealth)==message
        node.worker_event(String(data=json.dumps(dict(component='detector',epoch='worker1',state='processing'))))
        node.count_publishers=lambda _:0
        node.publish_health()
        assert messages[-1].state=='failure'
        assert 'Worker publisher missing' in messages[-1].detail
    finally:
        node.destroy_node();rclpy.shutdown()
