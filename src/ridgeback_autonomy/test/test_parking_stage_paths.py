"""Two distinct terminal ComputePathToPose results, never NavigateToPose."""

from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from rclpy.task import Future

from ridgeback_autonomy.mission.parking_execution import ExecutionState
from ridgeback_autonomy.mission.parking_stage_paths import ParkingStagePaths
from test_parking_execution import World, checked


class Handle:
    def __init__(self,accepted=True):
        self.accepted=accepted; self.result=Future(); self.cancels=0
    def get_result_async(self): return self.result
    def cancel_goal_async(self): self.cancels+=1; return Future()


class Client:
    def __init__(self): self.calls=[]; self.ready=True
    def server_is_ready(self): return self.ready
    def send_goal_async(self,goal):
        f=Future(); self.calls.append((goal,f)); return f


def setup():
    w=World(enabled=False); client=Client()
    stage=ParkingStagePaths(client,w.exec,w.context,'GridBased')
    return w,client,stage


def finish(w,client,*,status=GoalStatus.STATUS_SUCCEEDED,code=0,empty=False):
    goal,future=client.calls[-1]; handle=Handle(); future.set_result(handle)
    path=w.path((goal.start.pose.position.x,goal.start.pose.position.y,3.141592653589793),
                (goal.goal.pose.position.x,goal.goal.pose.position.y,3.141592653589793))
    if empty: path.poses=[]
    handle.result.set_result(checked(path,status,code))
    return handle


def test_preparking_has_its_own_start_goal_and_qualified_path_without_motion():
    w,c,s=setup()
    assert s.request()
    req,_=c.calls[-1]
    assert req.use_start and req.planner_id=='GridBased'
    assert req.start.pose.position.x==2.5
    assert req.goal.pose.position.x==pytest.approx(4.62)
    finish(w,c)
    assert not s.busy and w.exec.state==ExecutionState.PREPARED
    assert s.reason=='stage_path_validated' and not w.client.calls


@pytest.mark.parametrize('status,code,empty',[(GoalStatus.STATUS_ABORTED,0,False),
    (GoalStatus.STATUS_SUCCEEDED,1,False),(GoalStatus.STATUS_SUCCEEDED,0,True)])
def test_rejected_or_empty_stage_path_does_not_become_a_prepared_goal(status,code,empty):
    w,c,s=setup(); assert s.request(); finish(w,c,status=status,code=code,empty=empty)
    assert not s.busy and w.exec.state!=ExecutionState.PREPARED and not w.client.calls


def test_final_path_starts_at_measured_preparking_pose_and_revalidates_new_map():
    w=World(); w.start(); w.result(); w.settle()
    c=Client(); s=ParkingStagePaths(c,w.exec,w.context,'GridBased')
    assert s.request(final=True)
    req,_=c.calls[-1]
    assert req.start.pose.position.x==pytest.approx(4.62)
    assert req.goal.pose.position.x==pytest.approx(5.12)
    w.grid.data[80,100]=100
    finish(w,c)
    assert w.exec.state==ExecutionState.READY_FOR_FINAL
    assert 'occupied' in s.reason
    assert len(w.client.calls)==1  # Only the fake pre-parking navigation.


def test_final_path_cannot_be_requested_before_preparking_verification():
    w,c,s=setup(); assert not s.request(final=True)
    assert s.reason=='preparking_not_verified' and not c.calls


def test_timeout_pending_acceptance_cancels_late_handle_without_releasing_ownership():
    w,c,s=setup(); s.request(); w.wall+=4.; s.poll()
    assert s.busy and not s.reset()
    handle=Handle(); c.calls[-1][1].set_result(handle)
    assert handle.cancels==1 and s.busy
    handle.result.set_result(checked(w.plan.selected_path,GoalStatus.STATUS_CANCELED))
    assert not s.busy and not w.client.calls


def test_serial_requests_and_bounded_explicit_retries():
    w,c,s=setup()
    for _ in range(2):
        assert s.request()
        assert not s.request()
        c.calls[-1][1].set_result(Handle(accepted=False))
    assert not s.request() and s.reason=='stage_path_attempt_limit'
    assert len(c.calls)==2 and not w.client.calls


def test_unavailable_server_has_bounded_wait_without_fabricating_an_action_result():
    w,c,s=setup(); c.ready=False
    assert s.request() and not c.calls
    w.wall+=4.; s.poll()
    assert not s.busy and 'timeout' in s.reason


def test_map_to_odom_shift_invalidates_pending_path():
    w,c,s=setup(); s.request()
    w.map_odom=(.1,0.,0.,w.now); s.poll()
    assert s.busy and 'changed' in s.reason
    handle=Handle(); c.calls[-1][1].set_result(handle)
    assert handle.cancels==1


def test_cancelled_first_request_cannot_cancel_a_newer_handle():
    w,c,s=setup(); s.request(); s.cancel()
    old=Handle(); c.calls[-1][1].set_result(old)
    old.result.set_result(checked(w.plan.selected_path,GoalStatus.STATUS_CANCELED))
    assert s.reset() and s.request()
    s.cancel(); new=Handle(); c.calls[-1][1].set_result(new)
    assert old.cancels==1 and new.cancels==1


def test_failed_requalification_does_not_leave_previous_path_executable():
    w,c,s=setup(); assert s.request(); finish(w,c)
    assert w.exec.state==ExecutionState.PREPARED
    assert s.request()
    finish(w,c,empty=True)
    assert w.exec.state!=ExecutionState.PREPARED
    assert not w.exec.start() and not w.client.calls
