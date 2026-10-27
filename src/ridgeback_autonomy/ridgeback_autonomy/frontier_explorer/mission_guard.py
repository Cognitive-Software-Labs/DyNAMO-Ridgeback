"""Latched safety and ownership interlock around the explorer's Nav2 client."""
import json
import math
import threading
import time
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ridgeback_interfaces.msg import LocalizationHealth
from ridgeback_localization.contracts import HEALTH_TOPIC


class MissionGuard:
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        for name, value in dict(localization_required=False, health_timeout=3.0,
                                localization_progress_timeout=15.0, cancellation_timeout=5.0).items():
            node.declare_parameter(name, value)
        self.required = bool(node.get_parameter('localization_required').value)
        self.health_timeout = float(node.get_parameter('health_timeout').value)
        self.progress_timeout = float(node.get_parameter('localization_progress_timeout').value)
        self.cancel_timeout = float(node.get_parameter('cancellation_timeout').value)
        if any(not math.isfinite(v) or v <= 0 for v in (self.health_timeout, self.progress_timeout, self.cancel_timeout)):
            raise ValueError('Supervision timeouts must be finite and positive')
        self.paused = self.required
        self.last_health = None
        self.session = None
        self.progress = {}
        self.health_state = 'startup'
        self.reason = 'Awaiting health and explicit resume' if self.required else 'Localization optional'
        self.handle = None
        self.goal_pending = False
        self.cancel_state = 'none'
        self.cancel_started = 0.0
        self.cancel_overdue = False
        self.pause_requested = False
        self.pause_started = 0.0
        self.subscription = node.create_subscription(LocalizationHealth, HEALTH_TOPIC, self.observe, 10)
        self.publisher = node.create_publisher(String, 'mission/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pause_service = node.create_service(Trigger, 'mission/pause', self.request_pause)
        self.resume_service = node.create_service(Trigger, 'mission/resume', self.resume)
        self.timer = node.create_timer(0.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe(self, msg):
        with self.lock:
            now = time.monotonic()
            if not msg.session_id or not msg.streams or len(set(msg.streams)) != len(msg.streams) or not (len(msg.streams) == len(msg.progress) == len(msg.source_stamps)):
                self.health_state = 'invalid'
                self.pause('Malformed localization health')
                return
            if self.session is not None and msg.session_id != self.session:
                self.pause('Localization process restarted')
                self.progress.clear()
            self.session = msg.session_id
            self.last_health = now
            self.health_state = msg.state
            names = set(msg.streams)
            if self.progress and names != set(self.progress):
                self.pause('Localization stream contract changed')
                self.progress.clear()
            for name, count, stamp in zip(msg.streams, msg.progress, msg.source_stamps):
                ns = stamp.sec * 10**9 + stamp.nanosec
                old = self.progress.get(name)
                if old is None:
                    self.progress[name] = (count, ns, now, False)
                elif count > old[0] and ns > old[1]:
                    self.progress[name] = (count, ns, now, True)
                elif count < old[0] or ns < old[1]:
                    self.pause('Localization progress reset')
                    self.progress[name] = (count, ns, now, False)
            if msg.state != 'processing': self.pause('Localization ' + msg.state + ': ' + msg.detail)

    def healthy(self):
        now = time.monotonic()
        return (self.last_health is not None and now - self.last_health <= self.health_timeout
                and self.health_state == 'processing' and bool(self.progress)
                and all(value[3] and now-value[2] <= self.progress_timeout for value in self.progress.values()))

    def allowed(self):
        return (not self.paused and not self.goal_pending
            and self.cancel_state == 'none'
            and (not self.required or self.healthy()))

    def handoff_safe(self):
        """Whether an explicit requester may take over Nav2 ownership."""
        with self.lock:
            return self._handoff_safe_locked()

    def _handoff_safe_locked(self):
        return (self.pause_requested and self.paused
            and not self.goal_pending and self.handle is None
            and self.cancel_state == 'none')

    def pause(self, reason):
        if not self.required: return
        self.paused = True
        self.reason = reason
        self.cancel()

    def cancel(self):
        with self.lock:
            self._cancel_locked()

    def _cancel_locked(self):
        if self.handle is None or self.cancel_state != 'none': return
        self.cancel_state = 'request_pending'
        self.cancel_started = time.monotonic()
        self.cancel_overdue = False
        try:
            future = self.handle.cancel_goal_async()
            handle = self.handle
            future.add_done_callback(lambda done: self.cancel_done(done, handle))
        except Exception as exc:
            self.cancel_state = 'failed'
            self.reason = 'Cancellation failed: ' + str(exc)

    def cancel_done(self, future, handle):
        with self.lock:
            if self.handle is not handle: return  # terminal result arrived first
            try:
                result = future.result()
                self.cancel_state = 'awaiting_result' if result.goals_canceling else 'rejected'
                if self.cancel_state == 'rejected':
                    self.reason = 'Cancellation rejected; awaiting terminal result'
            except Exception as exc:
                self.cancel_state = 'failed'
                self.reason = 'Cancellation failed: ' + str(exc)
            if self.cancel_state in ('failed', 'rejected'): self.paused = True
            self._publish_status_locked()

    def accepted(self, handle):
        with self.lock:
            self.goal_pending = False
            self.handle = handle
            if self.paused:
                self._cancel_locked()
            elif self.required and not self.healthy():
                self.pause('Localization unavailable when goal accepted')
            self._publish_status_locked()

    def finished(self, handle):
        with self.lock:
            if self.handle is handle:
                self.handle = None
                self.cancel_state = 'none'
                self.cancel_overdue = False
                self._publish_status_locked()

    def request_pause(self, request, response):
        """Latch an external pause and report success only after settlement.

        The service callback never waits for Nav2.  A false response means the
        request is latched but the explorer still owns an unresolved goal; the
        caller waits for ``handoff_safe`` on the transient-local status topic or
        repeats the idempotent request.  A true response is a terminal ownership
        acknowledgment, not merely acceptance of ``cancel_goal_async``.
        """
        del request
        with self.lock:
            new_request = not self.pause_requested
            if new_request:
                self.pause_started = time.monotonic()
            self.pause_requested = True
            self.paused = True
            if new_request:
                self.reason = 'External mission pause requested'
            self._cancel_locked()
            safe = self._handoff_safe_locked()
            response.success = safe
            response.message = (
                'Exploration safely paused; Nav2 ownership released'
                if safe else
                'Exploration pause latched; waiting for owned Nav2 goal to terminate'
            )
            self._publish_status_locked()
            return response

    def resume(self, request, response):
        with self.lock:
            if self.required and not self.healthy():
                response.success, response.message = False, 'Current advancing localization required'
            elif self.goal_pending or self.handle is not None or self.cancel_state != 'none':
                response.success, response.message = False, 'Previous goal must reach a terminal result'
            else:
                self.pause_requested = False
                self.pause_started = 0.0
                self.cancel_overdue = False
                self.paused = False
                self.reason = 'Explicitly resumed'
                response.success, response.message = True, self.reason
            self._publish_status_locked()
            return response

    def tick(self):
        with self.lock:
            if self.required and not self.healthy(): self.pause('Localization missing, unhealthy, or progress frozen')
            if self.cancel_state in ('request_pending', 'awaiting_result') and time.monotonic()-self.cancel_started > self.cancel_timeout:
                self.paused = True
                self.cancel_overdue = True
                self.reason = 'Cancellation overdue; awaiting terminal result'
            elif (self.pause_requested and self.goal_pending
                    and time.monotonic()-self.pause_started > self.cancel_timeout):
                self.paused = True
                self.cancel_overdue = True
                self.reason = 'Pause handoff overdue; goal acceptance unresolved'
            self._publish_status_locked()

    def _publish_status_locked(self):
        self.publisher.publish(String(data=json.dumps(dict(
            required=self.required,
            paused=self.paused,
            healthy=self.healthy(),
            reason=self.reason,
            pause_requested=self.pause_requested,
            handoff_safe=self._handoff_safe_locked(),
            cancellation=self.cancel_state,
            cancellation_overdue=self.cancel_overdue,
            goal_pending=self.goal_pending,
            goal_active=self.handle is not None,
        ))))
