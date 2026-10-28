"""Parking-owned vetoes and keep-out mask generation, never a velocity source.

Predictive bounds are conditional on verified deceleration/latency. Publishing
a mask or observing an idle command channel is not an ownership certificate.
"""

from dataclasses import dataclass
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from ridgeback_localization.estimator_registry import PUBLIC_ESTIMATOR_ORDER

from ridgeback_autonomy.mission.parking_geometry import rotated_footprint_vertices
from ridgeback_autonomy.mission.parking_planner import points_to_polygon_distance


LOCK_TOPICS = ('platform/emergency_stop', 'platform/safety_stop', 'joy_teleop/bt_quality_stop')
MANUAL_TOPICS = ('joy_teleop/cmd_vel', 'twist_marker_server/cmd_vel', 'rc_teleop/cmd_vel')
COMMAND_TOPICS = ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel', 'platform/cmd_vel')


def target_execution_reason(target):
    """Reference admission only; never infer calibrated uncertainty from a label.

    Generic observations remain usable for non-driving planning, but one
    head-on accuracy trial cannot authorize their use as a safety reference.
    """
    if target is None:
        return 'target_unavailable'
    if target.reference == 'generic_target_position':
        return ('target_reference_inconsistent' if target.is_foot_midpoint else
                'generic_target_uncertainty_unqualified')
    if target.reference != 'foot_midpoint' or target.is_foot_midpoint is not True:
        return 'target_reference_inconsistent_or_unsupported'
    if not isinstance(target.source, str) or not target.source.strip() or target.source in PUBLIC_ESTIMATOR_ORDER:
        return 'foot_reference_source_unqualified'
    # No qualified foot producer/uncertainty bound exists in this integration.
    # An unfamiliar source name and a Boolean are not a calibration certificate.
    return 'foot_reference_uncertainty_unqualified'


def fresh(stamp_ns, received_s, now_ns, wall_s, age_s):
    return (stamp_ns > 0 and math.isfinite(received_s)
            and 0 <= (now_ns - stamp_ns) * 1e-9 <= age_s
            and 0 <= wall_s - received_s <= age_s)


@dataclass(frozen=True)
class ProtectionLimits:
    latency_s: float = 0.30
    linear_deceleration: float = 0.5
    angular_deceleration: float = 0.5
    command_max_age_s: float = 0.5
    health_max_age_s: float = 1.5
    max_unobserved_displacement_m: float = 0.50
    footprint_padding_m: float = 0.01

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('protection bounds must be finite and positive')


class ParkingSafetyMonitor:
    def __init__(self, limits=None):
        self.limits = limits or ProtectionLimits()
        self.locks, self.manual, self.commands, self.action_status = {}, {}, {}, {}
        self.stop_status = self.controller = None
        self.last_reason = 'stop_inputs_unavailable'
        self.last_clearance = self.last_required_clearance = None
        self.last_moving_poll_s = None

    def observe_lock(self, topic, active, now_ns, wall_s):
        self.locks[topic] = (bool(active), now_ns, wall_s)

    def observe_stop_status(self, msg, wall_s):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # external_stop_present is a connector indication, not a release signal.
        self.stop_status = (bool(msg.stop_power_status and not msg.needs_reset), ns, wall_s)

    def observe_controller(self, active, now_ns, wall_s):
        self.controller = (bool(active), now_ns, wall_s)

    def observe_command(self, topic, msg, now_ns, wall_s):
        if topic in MANUAL_TOPICS:
            # Even a zero teleop message owns that mux input for its timeout.
            self.manual[topic] = wall_s
            return
        t = msg.twist
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.commands[topic] = ((t.linear.x, t.linear.y, t.angular.z), ns, wall_s,
                                msg.header.frame_id)

    def observe_action_status(self, topic, msg, now_ns, wall_s):
        active = {tuple(s.goal_info.goal_id.uuid) for s in msg.status_list if s.status in (1, 2, 3)}
        self.action_status[topic] = (active, now_ns, wall_s)

    def intervention(self, now_ns, wall_s, own_goal=None):
        for topic, (active, _, _) in self.locks.items():
            if active:
                return f'mux_lock_active: {topic}'
        for topic, received in self.manual.items():
            if not 0 <= wall_s - received or wall_s - received <= 0.5:
                return f'manual_control_active: {topic}'
        for topic, (goals, _, _) in self.action_status.items():
            if goals - ({own_goal} if own_goal is not None else set()):
                return f'competing_navigation_goal: {topic}'
        return None

    def stop_reason(self, now_ns, wall_s):
        reason = self.intervention(now_ns, wall_s)
        # Action exclusivity is checked separately with the parking goal UUID.
        if reason and not reason.startswith('competing_navigation_goal'):
            return reason
        for topic in LOCK_TOPICS:
            item = self.locks.get(topic)
            if item is None or not fresh(item[1], item[2], now_ns, wall_s, self.limits.health_max_age_s):
                return f'stop_lock_unknown_or_stale: {topic}'
        for name, item in (('stop_loop', self.stop_status), ('platform_controller', self.controller)):
            if item is None or not fresh(item[1], item[2], now_ns, wall_s, self.limits.health_max_age_s):
                return f'{name}_unknown_or_stale'
            if not item[0]:
                return f'{name}_fault'
        return None

    def check(self, ctx, config, *, moving=False, own_goal=None):
        previous = self.last_moving_poll_s
        self.last_moving_poll_s = ctx.wall_s if moving else None
        if moving and previous is not None and not 0 <= ctx.wall_s-previous <= self.limits.latency_s:
            self.last_reason = 'protection_poll_deadline'
            return self.last_reason
        reason = self.intervention(ctx.now_ns, ctx.wall_s, own_goal)
        reason = reason or self.stop_reason(ctx.now_ns, ctx.wall_s)
        if reason:
            self.last_reason = reason
            return reason
        source = ctx.target_robot
        target = ctx.plan.target
        if (source is None or len(source) != 4 or source[3] != target.stamp_ns
                or not all(math.isfinite(v) for v in source)):
            self.last_reason = 'target_source_pose_unavailable'
            return self.last_reason
        displacement = math.hypot(ctx.robot[0] - source[0], ctx.robot[1] - source[1])
        if displacement > self.limits.max_unobserved_displacement_m:
            self.last_reason = 'target_unobserved_displacement_exceeded'
            return self.last_reason
        odom = ctx.odometry.twist.twist
        speeds = [(math.hypot(odom.linear.x, odom.linear.y), abs(odom.angular.z))]
        stamp = ctx.odometry.header.stamp
        age = max((ctx.now_ns - (stamp.sec*1_000_000_000+stamp.nanosec))*1e-9,
                  ctx.wall_s-ctx.odometry_received_s)
        if moving:
            for topic in COMMAND_TOPICS:
                item = self.commands.get(topic)
                if item is None or not fresh(item[1], item[2], ctx.now_ns, ctx.wall_s,
                                              self.limits.command_max_age_s):
                    self.last_reason = f'command_evidence_unknown_or_stale: {topic}'
                    return self.last_reason
                values, _, _, frame = item
                age = max(age,(ctx.now_ns-item[1])*1e-9,ctx.wall_s-item[2])
                if (not all(math.isfinite(v) for v in values)
                        or frame not in ('', ctx.base_frame)):
                    self.last_reason = f'command_evidence_invalid: {topic}'
                    return self.last_reason
                speeds.append((math.hypot(*values[:2]), abs(values[2])))
        v, w = max(s[0] for s in speeds), max(s[1] for s in speeds)
        r = max(math.hypot(*p) for p in config.footprint) + self.limits.footprint_padding_m
        # Triangle-inequality bound covers translation plus any yaw-swept corner
        # during command latency and a stop under the assumed minimum decel.
        latency = self.limits.latency_s + age
        travel = (v * latency + v*v / (2*self.limits.linear_deceleration)
                  + r * (w * latency + w*w / (2*self.limits.angular_deceleration)))
        old, new = ctx.snapshot.map_odom, ctx.map_odom
        dx, dy = target.x - old[0], target.y - old[1]
        ox = math.cos(old[2])*dx + math.sin(old[2])*dy
        oy = -math.sin(old[2])*dx + math.cos(old[2])*dy
        moved_x = new[0] + math.cos(new[2])*ox - math.sin(new[2])*oy
        moved_y = new[1] + math.sin(new[2])*ox + math.cos(new[2])*oy
        drift = math.hypot(moved_x - target.x, moved_y - target.y)
        polygon = rotated_footprint_vertices(config.footprint, *ctx.robot[:3])
        self.last_clearance = float(points_to_polygon_distance([target.x], [target.y], polygon)[0])
        self.last_required_clearance = ctx.plan.keepout_radius_m + travel + drift + self.limits.footprint_padding_m
        self.last_reason = ('predicted_g1_swept_stop_violation'
                            if self.last_clearance < self.last_required_clearance else 'conditional_sweep_clear')
        return None if self.last_reason == 'conditional_sweep_clear' else self.last_reason

    def summary(self):
        return dict(reason=self.last_reason, clearance_m=self.last_clearance,
                    required_clearance_m=self.last_required_clearance,
                    manual_exclusion_authorized=False, controller_stop_bound_qualified=False)


def keepout_mask(grid, target, radius):
    """Conservative cell-intersection disk; no simulator coordinates or obstacles."""
    if not math.isfinite(radius) or radius <= 0 or grid.width*grid.height > 2_000_000:
        raise ValueError('invalid keepout radius or mask budget exceeded')
    if (target.frame_id != grid.frame_id or target.stamp_ns <= 0
            or not all(math.isfinite(v) for v in (target.x,target.y))):
        raise ValueError('invalid keepout target frame, timestamp or coordinates')
    x, y = grid.map_to_local(target.x, target.y)
    if not 0 <= x < grid.width*grid.resolution or not 0 <= y < grid.height*grid.resolution:
        raise ValueError('target outside keepout mask')
    effective = radius + grid.resolution / math.sqrt(2)
    # Use the entire source-map envelope: Nav2's filter updateBounds uses mask
    # dimensions in master-grid indices, so a small remote tile can be skipped.
    corners = [(grid.origin_x + math.cos(grid.origin_yaw)*cx - math.sin(grid.origin_yaw)*cy,
                grid.origin_y + math.sin(grid.origin_yaw)*cx + math.cos(grid.origin_yaw)*cy)
               for cx in (0.,grid.width*grid.resolution)
               for cy in (0.,grid.height*grid.resolution)]
    origin_x, origin_y = min(p[0] for p in corners),min(p[1] for p in corners)
    width = math.ceil((max(p[0] for p in corners)-origin_x)/grid.resolution)
    height = math.ceil((max(p[1] for p in corners)-origin_y)/grid.resolution)
    if width*height > 2_000_000:
        raise ValueError('keepout envelope budget exceeded')
    rr, cc = np.mgrid[:height,:width]
    mx = origin_x + (cc+.5)*grid.resolution
    my = origin_y + (rr+.5)*grid.resolution
    # Nav2's mask reader assumes axis alignment. NO_INFORMATION outside the
    # disk is essential: zero can overwrite unknown master-costmap cells.
    data = np.where(np.hypot(mx-target.x,my-target.y)<=effective,100,-1).astype(np.int8)
    msg = OccupancyGrid()
    msg.header.frame_id = grid.frame_id
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(target.stamp_ns, 1_000_000_000)
    msg.info.resolution, msg.info.width, msg.info.height = grid.resolution, width, height
    msg.info.origin.position.x, msg.info.origin.position.y = origin_x, origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = data.ravel().tolist()
    return msg
