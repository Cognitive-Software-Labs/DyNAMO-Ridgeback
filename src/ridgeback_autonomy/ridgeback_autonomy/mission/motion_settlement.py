"""Fresh, continuous odometry evidence for a mission-owned stationary gate."""

from dataclasses import dataclass
import math


@dataclass
class MotionSettlement:
    linear_limit: float = 0.005
    angular_limit: float = 0.005
    stable_s: float = 2.0
    max_age_s: float = 0.5
    # A subscriber's sim-time clock can trail the odometry stamp by a clock
    # message interval; only stamps further ahead than this are invalid.
    clock_skew_s: float = 0.1
    reason: str = 'odometry_missing'

    def __post_init__(self):
        self.reset()

    def reset(self):
        self.start_ns = self.start_wall = None
        self.last_ns = self.last_wall = None
        self.reason = 'odometry_missing'

    def observe(self, velocities, stamp_ns, now_ns, wall_s, *, frame_valid=True):
        previous_ns, previous_wall = self.last_ns, self.last_wall
        reason = None
        if not frame_valid:
            reason = 'odometry_frame'
        elif not all(math.isfinite(v) for v in velocities):
            reason = 'odometry_nonfinite'
        elif stamp_ns <= 0 or (stamp_ns - now_ns) * 1e-9 > self.clock_skew_s:
            reason = 'odometry_timestamp'
        elif (now_ns - stamp_ns) * 1e-9 > self.max_age_s:
            reason = 'odometry_stale'
        elif previous_ns is not None and stamp_ns <= previous_ns:
            reason = 'odometry_not_advancing'
        elif math.hypot(*velocities[:3]) >= self.linear_limit or math.hypot(*velocities[3:]) >= self.angular_limit:
            reason = 'robot_moving'
        if reason:
            self.reset()
            self.reason = reason
            return
        if (previous_ns is None or (stamp_ns - previous_ns) * 1e-9 > self.max_age_s
                or wall_s - previous_wall > self.max_age_s):
            self.start_ns, self.start_wall = stamp_ns, wall_s
        self.last_ns, self.last_wall = stamp_ns, wall_s
        self.reason = 'collecting_stationary_interval'

    def ready(self, now_ns, wall_s):
        if not hasattr(self, 'last_ns') or self.last_ns is None:
            return False
        if ((self.last_ns - now_ns) * 1e-9 > self.clock_skew_s
                or (now_ns - self.last_ns) * 1e-9 > self.max_age_s
                or wall_s - self.last_wall > self.max_age_s):
            self.reset()
            self.reason = 'odometry_stale'
            return False
        ready = ((self.last_ns - self.start_ns) * 1e-9 >= self.stable_s
                 and wall_s - self.start_wall >= self.stable_s)
        if ready:
            self.reason = 'motion_settled'
        return ready
