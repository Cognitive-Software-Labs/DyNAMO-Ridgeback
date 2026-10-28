"""Guarded two-stage Nav2 execution, independent of simulator and ROS spinning.

Preparation is not authorization. The caller must supply fresh, independently
verified interlocks; absent inputs fail closed. No retries or return goals.
"""

from copy import deepcopy
from dataclasses import dataclass, fields, replace
from enum import Enum
import math
from uuid import uuid4

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from unique_identifier_msgs.msg import UUID

from ridgeback_autonomy.mission.parking_feasibility import path_rejection, pose_xyyaw
from ridgeback_autonomy.mission.parking_geometry import (
    normalize_angle, preparking_pose, rotated_footprint_vertices, base_pose_from_task_point,
    task_point_from_base_pose,
)
from ridgeback_autonomy.mission.parking_planner import (
    validate_inputs, footprint_rejection_reason, points_to_polygon_distance, keepout_radius_m,
)


class ExecutionState(Enum):
    IDLE = 'idle'
    PREPARED = 'prepared'
    SENDING = 'sending'
    NAVIGATING = 'navigating'
    CANCELING = 'canceling'
    SETTLING = 'settling'
    READY_FOR_FINAL = 'ready_for_final'
    PARKED = 'parked'
    FAILED = 'failed'


@dataclass(frozen=True)
class ExecutionSafety:
    """Evidence from a future trusted ownership/platform integration, not params.

    controller_keepout_verified includes the controller's collision envelope,
    generic-reference uncertainty, braking/latency and local trajectory safety.
    No production publisher/provider of this evidence is assumed in Phase 2G.1.
    """

    stamp_ns: int
    received_s: float
    ownership_released: bool = False
    navigation_clear: bool = False
    manual_control_clear: bool = False
    emergency_stop_clear: bool = False
    controller_keepout_verified: bool = False
    goal_checker_verified: bool = False


@dataclass(frozen=True)
class ExecutionLimits:
    xy_tolerance_m: float = 0.08
    yaw_tolerance_rad: float = 0.12
    preparking_offset_m: float = 0.50
    navigation_timeout_s: float = 90.0
    progress_timeout_s: float = 20.0
    progress_distance_m: float = 0.05
    feedback_timeout_s: float = 2.0
    cancellation_timeout_s: float = 5.0
    settlement_timeout_s: float = 10.0
    safety_max_age_s: float = 0.5
    max_target_age_s: float = 5.0
    max_map_age_s: float = 5.0
    max_pose_age_s: float = 1.0
    frame_shift_m: float = 0.05
    frame_yaw_rad: float = 0.03

    def __post_init__(self):
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) <= 0
               for f in fields(self)):
            raise ValueError('execution limits must be finite and positive')


@dataclass(frozen=True)
class ExecutionContext:
    plan: object
    grid: object
    robot: tuple | None
    map_odom: tuple | None
    snapshot: object
    motion: object
    now_ns: int
    wall_s: float
    safety: ExecutionSafety | None = None
    path_request_unresolved: bool = False
    map_frame: str = 'map'
    odom_frame: str = 'odom'
    base_frame: str = 'base_link'
    odometry: object | None = None
    odometry_received_s: float | None = None
    target_robot: tuple | None = None


class ParkingExecutor:
    def __init__(self, client, context, config, behavior_tree, *, enabled=False,
                 limits=None, on_change=None, safety_check=None):
        self.client, self.context, self.config = client, context, config
        self.behavior_tree = str(behavior_tree)
        self.enabled = enabled
        self.limits = limits or ExecutionLimits()
        self.on_change = on_change or (lambda: None)
        self.safety_check = safety_check or (lambda ctx, moving: None)
        self.state = ExecutionState.IDLE
        self.reason = 'execution_disabled' if not enabled else 'not_prepared'
        self.stage = 'preparking'
        self.goal = self.final_goal = None
        self.route = self.route_start = self.identity = None
        self.busy = False
        self.handle = None
        self.generation = 0
        self.cancel_requested = False
        self.stop_reason = None
        self.collision = None
        self.last_feedback_s = self.last_progress_s = None
        self.best_distance = None
        self.distance_remaining = None
        self.arrival_error = None
        self.requires_settlement = False
        self.goal_uuid = None

    def _set(self, state, reason):
        self.state, self.reason = state, reason
        self.on_change()

    def summary(self):
        return dict(state=self.state.name, stage=self.stage, reason=self.reason,
                    enabled=self.enabled, navigation_unresolved=self.busy,
                    goal=self.goal, final_goal=self.final_goal,
                    distance_remaining=self.distance_remaining,
                    arrival_error=self.arrival_error,
                    post_goal_stop_pending=self.requires_settlement,
                    motion_ownership_retained=self.busy or self.requires_settlement)

    def reset(self):
        if self.busy or self.state in (ExecutionState.CANCELING, ExecutionState.SETTLING):
            return False
        ctx = self.context()
        if self.requires_settlement and not ctx.motion.ready(ctx.now_ns, ctx.wall_s):
            return False
        self.generation += 1
        self.goal = self.final_goal = self.route = self.identity = None
        self.stage = 'preparking'
        self.stop_reason = None
        self.requires_settlement = False
        self.goal_uuid = None
        self._set(ExecutionState.IDLE, 'not_prepared')
        return True

    @staticmethod
    def _identity(plan):
        c, t = plan.selected, plan.target
        return (c.base_x, c.base_y, c.yaw, t.x, t.y, t.reference,
                t.is_foot_midpoint, t.source, plan.keepout_radius_m)

    def _inputs(self, ctx, *, stationary, at_origin=False):
        if (ctx is None or ctx.plan is None or ctx.snapshot is None
                or ctx.snapshot.robot is None or ctx.snapshot.map_odom is None):
            return 'planning_context_missing'
        plan = ctx.plan
        if not plan.ok or plan.selected is None or plan.selected_path is None:
            return 'selected_plan_unavailable'
        if plan.target is None:
            return 'target_unavailable'
        if (plan.selected.status != 'selected' or plan.selected.path_status != 'valid'
                or not any(c is plan.selected for c in plan.candidates)):
            return 'candidate_not_path_valid'
        if self.identity is not None and self.identity != self._identity(plan):
            return 'selected_plan_changed'
        reasons = validate_inputs(plan.target, ctx.grid, ctx.robot, now_ns=ctx.now_ns,
            map_frame=ctx.map_frame, max_target_age_s=self.limits.max_target_age_s,
            max_map_age_s=self.limits.max_map_age_s, max_pose_age_s=self.limits.max_pose_age_s)
        if reasons:
            return ', '.join(reasons)
        if min(ctx.now_ns, plan.target.stamp_ns, ctx.grid.stamp_ns, ctx.robot[3]) <= 0:
            return 'execution_source_timestamp_invalid'
        c, t = plan.selected, plan.target
        if not all(math.isfinite(v) for v in (c.base_x, c.base_y, c.yaw, c.task_x, c.task_y,
                                              plan.keepout_radius_m)) or plan.keepout_radius_m <= 0:
            return 'candidate_nonfinite_or_invalid_keepout'
        if plan.keepout_radius_m + 1e-12 < keepout_radius_m(t, self.config):
            return 'g1_keepout_margin_invalid'
        expected = base_pose_from_task_point(c.task_x, c.task_y, t.x, t.y,
            self.config.task_point_x_base_m, self.config.task_point_y_base_m)
        if (math.hypot(expected[0] - c.base_x, expected[1] - c.base_y) > 1e-6
                or abs(normalize_angle(expected[2] - c.yaw)) > 1e-6
                or not self.config.reach_min_m <= math.hypot(c.task_x - t.x, c.task_y - t.y) <= self.config.reach_max_m):
            return 'candidate_geometry_invalid'
        if ctx.map_odom is None or not all(math.isfinite(v) for v in ctx.map_odom):
            return 'execution_tf_unavailable'
        tf_stamp = ctx.map_odom[3]
        if tf_stamp <= 0 or not 0 <= (ctx.now_ns - tf_stamp) * 1e-9 <= self.limits.max_pose_age_s:
            return 'execution_tf_stale'
        reason = ctx.snapshot.changed(ctx.robot if at_origin else ctx.snapshot.robot,
            ctx.map_odom, self.limits.frame_shift_m, self.limits.frame_yaw_rad)
        if reason:
            return reason
        if ctx.path_request_unresolved:
            return 'path_request_unresolved'
        odom = ctx.odometry
        if odom is None or ctx.odometry_received_s is None:
            return 'odometry_missing'
        twist = odom.twist.twist
        velocities = (twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z)
        ns = odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec
        if (odom.header.frame_id != ctx.odom_frame or odom.child_frame_id != ctx.base_frame
                or not all(math.isfinite(v) for v in velocities)):
            return 'odometry_invalid'
        if (ns <= 0 or not 0 <= (ctx.now_ns - ns) * 1e-9 <= ctx.motion.max_age_s
                or not 0 <= ctx.wall_s - ctx.odometry_received_s <= ctx.motion.max_age_s):
            return 'odometry_stale'
        if stationary and not ctx.motion.ready(ctx.now_ns, ctx.wall_s):
            return f'motion_not_settled: {ctx.motion.reason}'
        if stationary and (math.hypot(*velocities[:3]) >= ctx.motion.linear_limit
                           or math.hypot(*velocities[3:]) >= ctx.motion.angular_limit):
            return 'robot_still_moving'
        if at_origin:
            reason = path_rejection(plan.selected_path, c, ctx.snapshot.robot, ctx.grid,
                                    t, self.config, plan.keepout_radius_m)
            if reason:
                return reason
        polygon = rotated_footprint_vertices(self.config.footprint, *ctx.robot[:3])
        reason = footprint_rejection_reason(polygon, ctx.grid, self.config.occupied_threshold)
        if reason:
            return f'current_footprint_{reason}'
        if points_to_polygon_distance([plan.target.x], [plan.target.y], polygon)[0] < plan.keepout_radius_m:
            return 'current_g1_keepout'
        return None

    def _safety(self, ctx):
        s = ctx.safety
        if s is None:
            return 'execution_interlocks_unavailable'
        if (not math.isfinite(s.received_s) or s.stamp_ns <= 0
                or not 0 <= (ctx.now_ns - s.stamp_ns) * 1e-9 <= self.limits.safety_max_age_s
                or not 0 <= ctx.wall_s - s.received_s <= self.limits.safety_max_age_s):
            return 'execution_interlocks_stale'
        names = ['ownership_released', 'navigation_clear', 'manual_control_clear', 'emergency_stop_clear',
                 'controller_keepout_verified', 'goal_checker_verified']
        for name in names:
            if getattr(s, name) is not True:
                return name
        if self.collision is None:
            return 'collision_monitor_unavailable'
        action, received_ns, wall_s = self.collision
        if (not 0 <= (ctx.now_ns - received_ns) * 1e-9 <= self.limits.safety_max_age_s
                or not 0 <= ctx.wall_s - wall_s <= self.limits.safety_max_age_s):
            return 'collision_monitor_stale'
        if action not in (0, 2, 3, 4):
            return 'collision_monitor_stop'
        return None

    def _route_reason(self, ctx, *, fresh=False):
        if self.route is None:
            return 'preparking_path_required' if self.stage == 'preparking' else 'final_path_required'
        if fresh:
            stamp = self.route.header.stamp
            ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if ns <= 0 or not 0 <= (ctx.now_ns - ns) * 1e-9 <= self.limits.max_pose_age_s:
                return 'execution_path_stale'
        endpoint = replace(ctx.plan.selected, base_x=self.goal[0],
                           base_y=self.goal[1], yaw=self.goal[2])
        return path_rejection(self.route, endpoint, self.route_start, ctx.grid,
                              ctx.plan.target, self.config, ctx.plan.keepout_radius_m)

    def prepare(self, path_result=None, *, final=False):
        if self.busy or self.state in (ExecutionState.CANCELING, ExecutionState.SETTLING):
            return False
        if final and self.state != ExecutionState.READY_FOR_FINAL:
            return False
        if not final and self.state not in (ExecutionState.IDLE, ExecutionState.PREPARED):
            return False
        if self.state == ExecutionState.PREPARED:
            self._set(ExecutionState.IDLE, 'previous preparation superseded')
        ctx = self.context()
        reason = self._inputs(ctx, stationary=True, at_origin=not final)
        if reason:
            self.reason = reason
            return False
        path = None
        if path_result is not None:
            try:
                if path_result.status != GoalStatus.STATUS_SUCCEEDED or path_result.result.error_code != 0:
                    self.reason = 'stage_path_nav2_failed'
                    return False
                path = path_result.result.path
            except AttributeError:
                self.reason = 'stage_path_result_invalid'
                return False
        c = ctx.plan.selected
        self.final_goal = (c.base_x, c.base_y, c.yaw)
        self.goal = self.final_goal if final else preparking_pose(
            *self.final_goal, self.limits.preparking_offset_m)
        self.stage = 'parking' if final else 'preparking'
        self.route, self.route_start = deepcopy(path), tuple(ctx.robot[:3])
        self.identity = self._identity(ctx.plan)
        # The Phase 2F route ends at the final pose, not at pre-parking. It
        # cannot substitute for either newly requested stage path.
        reason = self._route_reason(ctx, fresh=True)
        if reason:
            self.reason = reason
            return False
        self._set(ExecutionState.PREPARED, 'validated_stage_path; execution requires explicit authorization')
        return True

    def start(self):
        if not self.enabled:
            self.reason = 'execution_disabled'
            self.on_change()
            return False
        if self.state != ExecutionState.PREPARED or self.busy:
            return False
        ctx = self.context()
        reason = self._inputs(ctx, stationary=True, at_origin=self.stage == 'preparking')
        if reason is None:
            reason = self._safety(ctx) or self.safety_check(ctx, False) or self._route_reason(ctx, fresh=True)
        if reason is None and (math.hypot(ctx.robot[0] - self.route_start[0],
                                         ctx.robot[1] - self.route_start[1]) > self.limits.frame_shift_m
                              or abs(normalize_angle(ctx.robot[2] - self.route_start[2])) > self.limits.frame_yaw_rad):
            reason = 'stage_start_pose_changed'
        if reason:
            self._set(ExecutionState.FAILED, reason)
            return False
        if not self.behavior_tree or not self.client.server_is_ready():
            self.reason = 'parking_bt_or_action_server_unavailable'
            return False
        request = NavigateToPose.Goal()
        request.pose = self._pose(self.goal, ctx)
        request.behavior_tree = self.behavior_tree
        self.generation += 1
        token = self.generation
        self.busy = True
        goal_uuid = UUID(uuid=list(uuid4().bytes))
        self.goal_uuid = tuple(goal_uuid.uuid)
        self.requires_settlement = True
        self.stop_reason = None
        self.cancel_requested = False
        self.last_feedback_s = self.last_progress_s = ctx.wall_s
        self.distance_remaining = self.best_distance = None
        self.deadline_s = ctx.wall_s + self.limits.navigation_timeout_s
        self.clock_ns, self.clock_advanced_s = ctx.now_ns, ctx.wall_s
        self._set(ExecutionState.SENDING, f'{self.stage} goal acceptance pending')
        try:
            future = self.client.send_goal_async(request,
                feedback_callback=lambda msg: self._feedback(msg, token), goal_uuid=goal_uuid)
            future.add_done_callback(lambda f: self._accepted(f, token))
        except Exception as exc:
            # Transport failure can occur after dispatch: ownership is unknown.
            self.cancel(f'navigation_acceptance_unknown: {exc}')
        return True

    @staticmethod
    def _pose(goal, ctx):
        msg = PoseStamped()
        msg.header.frame_id = ctx.plan.target.frame_id
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ctx.now_ns, 1_000_000_000)
        msg.pose.position.x, msg.pose.position.y = map(float, goal[:2])
        msg.pose.orientation.z = math.sin(goal[2] / 2)
        msg.pose.orientation.w = math.cos(goal[2] / 2)
        return msg

    def _accepted(self, future, token):
        if token != self.generation or not self.busy or self.handle is not None:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.cancel(f'navigation_acceptance_unknown: {exc}')
            return
        if not handle.accepted:
            self.busy = False
            self._begin_settlement(False, 'navigation_goal_rejected')
            return
        self.handle = handle
        try:
            handle.get_result_async().add_done_callback(lambda f: self._result(f, token, handle))
        except Exception as exc:
            self.cancel(f'navigation_result_unknown: {exc}')
            return
        if self.stop_reason is not None:
            self._request_cancel()
        elif self.busy:
            self._set(ExecutionState.NAVIGATING, f'navigating to {self.stage}')

    def _result(self, future, token, handle):
        if token != self.generation or handle is not self.handle:
            return
        try:
            result = future.result()
            if result.status not in (GoalStatus.STATUS_SUCCEEDED,
                                    GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                raise ValueError('nonterminal action result')
            code, message = result.result.error_code, result.result.error_msg
        except Exception as exc:
            self.cancel(f'navigation_result_unknown: {exc}')
            return
        self.busy, self.handle = False, None
        timely = self.context().wall_s < self.deadline_s
        success = (not self.stop_reason and timely
                   and result.status == GoalStatus.STATUS_SUCCEEDED
                   and code == 0)
        reason = self.stop_reason or ('navigation_timeout' if not timely else
            f'nav2_status_{result.status}_error_{code}: {message}')
        self._begin_settlement(success, 'Nav2 success; verify actual stop and pose' if success else reason)

    def _begin_settlement(self, success, reason):
        self.result_success = success
        self.result_reason = reason
        ctx = self.context()
        ctx.motion.reset()
        self.settlement_deadline_s = ctx.wall_s + self.limits.settlement_timeout_s
        self._set(ExecutionState.SETTLING, reason)

    def collision_state(self, action, now_ns, wall_s):
        self.collision = (action, now_ns, wall_s)
        if action == 1 and self.busy:
            self.cancel('collision_monitor_stop')

    def _feedback(self, msg, token):
        if token != self.generation or not self.busy or self.stop_reason:
            return
        ctx = self.context()
        try:
            f = msg.feedback
            pose_xyyaw(f.current_pose.pose)
            stamp = f.current_pose.header.stamp
            ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if (f.current_pose.header.frame_id != ctx.plan.target.frame_id
                    or ns <= 0 or not 0 <= (ctx.now_ns - ns) * 1e-9 <= self.limits.max_pose_age_s
                    or not math.isfinite(f.distance_remaining) or f.distance_remaining < 0):
                raise ValueError('invalid navigation feedback')
        except (AttributeError, ValueError, TypeError):
            self.cancel('navigation_feedback_invalid')
            return
        self.last_feedback_s = ctx.wall_s
        self.distance_remaining = float(f.distance_remaining)
        if self.best_distance is None or self.distance_remaining <= self.best_distance - self.limits.progress_distance_m:
            self.best_distance = self.distance_remaining
            self.last_progress_s = ctx.wall_s

    def cancel(self, reason='operator_canceled'):
        if not self.busy:
            if self.state not in (ExecutionState.SETTLING, ExecutionState.FAILED):
                self._set(ExecutionState.FAILED, reason)
            return
        if self.stop_reason is None:
            self.stop_reason = reason
            self.cancel_deadline_s = self.context().wall_s + self.limits.cancellation_timeout_s
        self._set(ExecutionState.CANCELING, self.stop_reason)
        self._request_cancel()

    def _request_cancel(self):
        if self.handle is not None and not self.cancel_requested:
            self.cancel_requested = True
            try:
                token, handle = self.generation, self.handle
                handle.cancel_goal_async().add_done_callback(
                    lambda f: self._cancel_response(f, token, handle))
            except Exception:
                self.reason = f'{self.stop_reason}; cancellation transport failed, ownership unresolved'

    def _cancel_response(self, future, token, handle):
        if token != self.generation or handle is not self.handle:
            return
        try:
            if not future.result().goals_canceling:
                self.reason = f'{self.stop_reason}; cancellation rejected, awaiting terminal result'
        except Exception:
            self.reason = f'{self.stop_reason}; cancellation response unavailable'
        # Only _result(), never a cancel response, releases ownership.

    def poll(self):
        ctx = self.context()
        if self.state == ExecutionState.PREPARED:
            reason = (self._inputs(ctx, stationary=True, at_origin=self.stage == 'preparking')
                      or self._route_reason(ctx, fresh=True))
            if reason:
                self._set(ExecutionState.FAILED, reason)
            return
        if self.state == ExecutionState.CANCELING:
            if ctx.wall_s >= self.cancel_deadline_s:
                self._set(ExecutionState.FAILED, f'{self.stop_reason}; cancellation_timeout_ownership_unresolved')
            return
        if self.state in (ExecutionState.SENDING, ExecutionState.NAVIGATING):
            reason = self._inputs(ctx, stationary=False)
            reason = reason or self.safety_check(ctx, True) or self._safety(ctx) or self._route_reason(ctx)
            if ctx.now_ns > self.clock_ns:
                self.clock_ns, self.clock_advanced_s = ctx.now_ns, ctx.wall_s
            elif ctx.now_ns < self.clock_ns or ctx.wall_s - self.clock_advanced_s > ctx.motion.max_age_s:
                reason = reason or 'execution_clock_not_advancing'
            if reason is None and ctx.wall_s >= self.deadline_s:
                reason = 'navigation_timeout'
            if reason is None and ctx.wall_s - self.last_feedback_s > self.limits.feedback_timeout_s:
                reason = 'navigation_feedback_timeout'
            if reason is None and ctx.wall_s - self.last_progress_s > self.limits.progress_timeout_s:
                reason = 'navigation_progress_timeout'
            if reason:
                self.cancel(reason)
        elif self.state == ExecutionState.SETTLING:
            if ctx.motion.ready(ctx.now_ns, ctx.wall_s):
                self.requires_settlement = False
                reason = self._inputs(ctx, stationary=True) or self._safety(ctx) or self.safety_check(ctx, False)
                if not self.result_success:
                    reason = self.result_reason
                if reason:
                    self._set(ExecutionState.FAILED, reason)
                    return
                xy = math.hypot(ctx.robot[0] - self.goal[0], ctx.robot[1] - self.goal[1])
                yaw = abs(normalize_angle(ctx.robot[2] - self.goal[2]))
                self.arrival_error = dict(xy_error_m=xy, yaw_error_rad=yaw)
                if xy > self.limits.xy_tolerance_m or yaw > self.limits.yaw_tolerance_rad:
                    self._set(ExecutionState.FAILED, 'arrival_pose_outside_parking_tolerance')
                else:
                    if self.stage == 'parking':
                        tx, ty = task_point_from_base_pose(*ctx.robot[:3],
                            self.config.task_point_x_base_m, self.config.task_point_y_base_m)
                        reach = math.hypot(tx - ctx.plan.target.x, ty - ctx.plan.target.y)
                        self.arrival_error['actual_task_reach_m'] = reach
                        if not self.config.reach_min_m <= reach <= self.config.reach_max_m:
                            self._set(ExecutionState.FAILED, 'arrival_task_point_outside_reach')
                            return
                    self._set(ExecutionState.READY_FOR_FINAL if self.stage == 'preparking'
                              else ExecutionState.PARKED, 'fresh actual pose and physical settlement verified')
            elif ctx.wall_s >= self.settlement_deadline_s:
                self._set(ExecutionState.FAILED, f'post_navigation_motion_timeout: {ctx.motion.reason}')
