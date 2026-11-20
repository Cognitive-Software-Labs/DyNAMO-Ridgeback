"""Fresh, continuous odometry evidence for a mission-owned stationary gate."""

from dataclasses import dataclass
import math


# An odometry stamp ahead of this node's clock is not one: after blocking path checks
# the node's clock callback trails the odometry, so such a sample only restarts the
# stationary interval, which must still complete within the settlement timeout. On
# 2026-11-05 and 2026-11-09 it discarded verified plans after long route checks.
FAULT_REASONS = frozenset({'odometry_frame', 'odometry_nonfinite',
                           'odometry_not_advancing', 'robot_moving'})


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
        # Evidence of motion or invalid odometry, as opposed to the stationary
        # interval merely restarting after delayed or late samples.
        self.faults = 0
        self.last_fault = None       # diagnostics of the latest fault; kept like `faults`
        self.restart_total = 0       # every restart ever, for rate-limited logging
        self.reset()

    def reset(self):
        """Start a new wait: no interval yet, and this wait's diagnostics cleared."""
        self._clear_interval()
        self.reason = 'odometry_missing'
        # Diagnostics only; nothing below decides readiness. Each restart of the
        # interval, by cause, and the odometry stamp minus this node's clock.
        self.restarts = {}
        self.last_restart = None
        self.skew_range_s = None

    def _clear_interval(self):
        self.start_ns = self.start_wall = None
        self.last_ns = self.last_wall = None

    def _record(self, reason, stamp_ns, now_ns, wall_s, **detail):
        record = {'reason': reason, 'fault': reason in FAULT_REASONS,
                  'odom_stamp_s': round(stamp_ns * 1e-9, 3),
                  'clock_s': round(now_ns * 1e-9, 3),
                  'stamp_minus_clock_s': round((stamp_ns - now_ns) * 1e-9, 3),
                  'wall_s': round(wall_s, 3)}
        record.update({k: round(v, 4) for k, v in detail.items()})
        self.restarts[reason] = self.restarts.get(reason, 0) + 1
        self.restart_total += 1
        self.last_restart = record
        if record['fault']:
            self.last_fault = record

    def observe(self, velocities, stamp_ns, now_ns, wall_s, *, frame_valid=True):
        previous_ns, previous_wall = self.last_ns, self.last_wall
        if math.isfinite(stamp_ns) and stamp_ns > 0:
            skew = round((stamp_ns - now_ns) * 1e-9, 3)
            low, high = self.skew_range_s or (skew, skew)
            self.skew_range_s = (min(low, skew), max(high, skew))
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
            if reason in FAULT_REASONS:
                self.faults += 1
            detail = {}
            if reason == 'robot_moving':
                detail = {'linear_mps': math.hypot(*velocities[:3]),
                          'angular_radps': math.hypot(*velocities[3:])}
            self._record(reason, stamp_ns, now_ns, wall_s, **detail)
            self._clear_interval()
            self.reason = reason
            return
        if previous_ns is None:
            self.start_ns, self.start_wall = stamp_ns, wall_s
        elif (stamp_ns - previous_ns) * 1e-9 > self.max_age_s:
            self._record('odometry_gap', stamp_ns, now_ns, wall_s,
                         gap_s=(stamp_ns - previous_ns) * 1e-9)
            self.start_ns, self.start_wall = stamp_ns, wall_s
        elif wall_s - previous_wall > self.max_age_s:
            self._record('callback_gap', stamp_ns, now_ns, wall_s, gap_s=wall_s - previous_wall)
            self.start_ns, self.start_wall = stamp_ns, wall_s
        self.last_ns, self.last_wall = stamp_ns, wall_s
        self.reason = 'collecting_stationary_interval'

    def ready(self, now_ns, wall_s):
        if not hasattr(self, 'last_ns') or self.last_ns is None:
            return False
        if ((self.last_ns - now_ns) * 1e-9 > self.clock_skew_s
                or (now_ns - self.last_ns) * 1e-9 > self.max_age_s
                or wall_s - self.last_wall > self.max_age_s):
            self._record('odometry_stale', self.last_ns, now_ns, wall_s,
                         wall_age_s=wall_s - self.last_wall)
            self._clear_interval()
            self.reason = 'odometry_stale'
            return False
        ready = ((self.last_ns - self.start_ns) * 1e-9 >= self.stable_s
                 and wall_s - self.start_wall >= self.stable_s)
        if ready:
            self.reason = 'motion_settled'
        return ready

    def diagnostics(self):
        """This wait's restarts for the mission state; diagnostics only."""
        return {'restarts': dict(self.restarts), 'last_restart': self.last_restart,
                'last_fault': self.last_fault,
                'stamp_minus_clock_range_s': None if self.skew_range_s is None
                else list(self.skew_range_s)}

    @staticmethod
    def describe(record):
        """One restart or fault as text: its cause, stamps and measured values."""
        if record is None:
            return 'none'
        extra = ''.join(f', {k} {record[k]}' for k in
                        ('linear_mps', 'angular_radps', 'gap_s', 'wall_age_s') if k in record)
        return (f"{record['reason']} (odom stamp {record['odom_stamp_s']:.3f} s, clock "
                f"{record['clock_s']:.3f} s, stamp-clock {record['stamp_minus_clock_s']:+.3f} s{extra})")

    def summary(self):
        """This wait's restarts in one line, or '' when there were none."""
        if not self.restarts:
            return ''
        counts = ', '.join(f'{k} {v}' for k, v in sorted(self.restarts.items()))
        skew = ''
        if self.skew_range_s is not None:
            skew = f'; stamp-clock {self.skew_range_s[0]:+.3f}..{self.skew_range_s[1]:+.3f} s'
        return f'last restart {self.describe(self.last_restart)}; restarts this wait: {counts}{skew}'

    def explain(self):
        """The current state, with this wait's restarts when there were any."""
        summary = self.summary()
        return f'{self.reason} ({summary})' if summary else self.reason
