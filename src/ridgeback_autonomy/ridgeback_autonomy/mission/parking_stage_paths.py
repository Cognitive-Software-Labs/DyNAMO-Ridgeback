"""Bounded, non-driving stage-path requests using the existing path validator."""

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose

from ridgeback_autonomy.mission.parking_execution import ExecutionState
from ridgeback_autonomy.mission.parking_feasibility import PlanningSnapshot
from ridgeback_autonomy.mission.parking_geometry import preparking_pose


class ParkingStagePaths:
    def __init__(self, client, executor, context, planner_id, *, timeout_s=3., max_attempts=2):
        self.client, self.executor, self.context = client, executor, context
        self.planner_id, self.timeout_s, self.max_attempts = planner_id, timeout_s, max_attempts
        self.busy = False
        self.handle = None
        self.sent = self.cancel_requested = False
        self.reason = 'stage_path_not_requested'
        self.attempts = {False: 0, True: 0}
        self.token = 0

    def request(self, *, final=False):
        if self.busy or self.executor.busy:
            return False
        if final and self.executor.state != ExecutionState.READY_FOR_FINAL:
            self.reason = 'preparking_not_verified'
            return False
        if not final and self.executor.state not in (ExecutionState.IDLE, ExecutionState.PREPARED):
            self.reason = 'stage_state_invalid'
            return False
        if self.attempts[final] >= self.max_attempts:
            self.reason = 'stage_path_attempt_limit'
            return False
        ctx = self.context()
        reason = self.executor._inputs(ctx, stationary=True, at_origin=not final)
        if reason:
            self.reason = reason
            return False
        if self.executor.state == ExecutionState.PREPARED and not self.executor.reset():
            self.reason = 'previous_stage_not_settled'
            return False
        self.attempts[final] += 1
        c = ctx.plan.selected
        goal = (c.base_x, c.base_y, c.yaw)
        if not final:
            goal = preparking_pose(*goal, self.executor.limits.preparking_offset_m)
        self.goal, self.start, self.final = goal, ctx.robot, final
        self.snapshot = PlanningSnapshot(ctx.robot, ctx.map_odom)
        self.deadline = ctx.wall_s + self.timeout_s
        self.token += 1
        self.busy, self.sent, self.cancel_requested = True, False, False
        self.handle_cancel_sent = False
        self.reason = 'stage_path_pending'
        self.poll()
        return True

    def poll(self):
        if not self.busy:
            return
        ctx = self.context()
        reason = self.executor._inputs(ctx, stationary=True, at_origin=not self.final)
        reason = reason or self.snapshot.changed(ctx.robot, ctx.map_odom,
            self.executor.limits.frame_shift_m, self.executor.limits.frame_yaw_rad)
        if ctx.wall_s >= self.deadline:
            reason = 'stage_path_timeout'
        if reason:
            self.cancel(reason)
            return
        if not self.sent and self.client.server_is_ready():
            request = ComputePathToPose.Goal()
            request.start = self.executor._pose(self.start, ctx)
            request.goal = self.executor._pose(self.goal, ctx)
            request.use_start, request.planner_id = True, self.planner_id
            self.sent = True
            token = self.token
            try:
                self.client.send_goal_async(request).add_done_callback(lambda f: self._accepted(f, token))
            except Exception:
                self.reason = 'stage_path_acceptance_unknown'
                self.cancel_requested = True
                # Dispatch could have happened. Do not free the slot or retry.

    def _accepted(self, future, token):
        if token != self.token or self.handle is not None or not self.busy:
            return
        try:
            handle = future.result()
        except Exception:
            self.reason, self.cancel_requested = 'stage_path_acceptance_unknown', True
            return
        if not handle.accepted:
            self.busy = False
            self.reason = 'stage_path_goal_rejected'
            return
        self.handle = handle
        try:
            handle.get_result_async().add_done_callback(lambda f: self._result(f, token, handle))
        except Exception:
            self.cancel('stage_path_result_unknown')
            return
        if self.cancel_requested:
            self._cancel_handle()

    def _result(self, future, token, handle):
        if token != self.token or handle is not self.handle:
            return
        try:
            result = future.result()
            if result.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                     GoalStatus.STATUS_CANCELED):
                raise ValueError('nonterminal result')
        except Exception:
            self.cancel('stage_path_result_unknown')
            return
        self.handle, self.busy = None, False
        if self.cancel_requested:
            return
        ctx = self.context()
        if ctx.wall_s >= self.deadline:
            self.reason = 'stage_path_late_result'
            return
        reason = self.snapshot.changed(ctx.robot, ctx.map_odom,
            self.executor.limits.frame_shift_m, self.executor.limits.frame_yaw_rad)
        if reason:
            self.reason = reason
            return
        ready = self.executor.prepare(result, final=self.final)
        self.reason = 'stage_path_validated' if ready else self.executor.reason

    def _cancel_handle(self):
        if self.handle is not None and not getattr(self, 'handle_cancel_sent', False):
            self.handle_cancel_sent = True
            try:
                self.handle.cancel_goal_async()
            except Exception:
                self.reason += '; cancel_transport_failed_ownership_unresolved'

    def cancel(self, reason='stage_path_canceled'):
        self.reason, self.cancel_requested = reason, True
        if not self.sent:
            self.busy = False
        self._cancel_handle()

    def reset(self):
        if self.busy:
            return False
        self.token += 1
        self.attempts = {False: 0, True: 0}
        self.handle_cancel_sent = False
        self.reason = 'stage_path_not_requested'
        return True
