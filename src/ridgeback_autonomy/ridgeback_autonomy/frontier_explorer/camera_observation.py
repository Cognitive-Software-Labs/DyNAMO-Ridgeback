"""Camera observation turns at frontier stops, for the parking mission's G1 search.

After a successful frontier goal the explorer stops, confirms the stop on odometry, and
turns in place with Nav2 Spin toward at most ``max_views`` directions its camera has not
usefully seen (``camera_coverage``), waiting at each for fresh detector frames so OWLv2's
vote can confirm G1. The explorer's MissionGuard owns each Spin goal exactly like a frontier
goal: the mission's pause on a G1 confirmation cancels the turn, and a turn is never sent
while exploration is paused. Spin itself checks the turn on the local costmap and its
command passes the velocity smoother and the collision monitor; before it is sent, the
swept footprint (basket included) must also clear the global costmap.

Nothing is retried: a turn refused, rejected, aborted, cancelled or overdue ends the
stop's observations, and a view without usable frames is recorded as not seen. The
observations share one time budget. They are off unless ``camera_observation_enabled``.
"""
import math
import os
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration as RclDuration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException

from ridgeback_interfaces.msg import TargetDetections
from ridgeback_localization.contracts import RAW_DETECTIONS_TOPIC
from ridgeback_autonomy.frontier_explorer.camera_coverage import (
    CameraCoverage, ViewModel, map_view, rotation_problem)
from ridgeback_autonomy.mission.motion_settlement import MotionSettlement
from ridgeback_autonomy.mission.parking_geometry import (
    load_nav2_footprint, normalize_angle, parking_footprint)

_TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def optical_axis_yaw(q):
    """Map heading of an optical frame's +z (viewing) axis."""
    fx = 2.0 * (q.x * q.z + q.w * q.y)
    fy = 2.0 * (q.y * q.z - q.w * q.x)
    return math.atan2(fy, fx)


class CameraObserver:
    PARAMETERS = dict(
        camera_observation_enabled=False,
        camera_observation_camera_frame='camera_0_color_optical_frame',
        camera_observation_frames_topic=RAW_DETECTIONS_TOPIC,
        camera_observation_map_topic='map',
        camera_observation_odometry_topic='platform/odom',
        camera_observation_odom_frame='odom',
        camera_observation_max_views=3,
        camera_observation_min_new_area_m2=1.0,
        camera_observation_budget_s=180.0,
        camera_observation_stable_s=0.5,
        camera_observation_settle_timeout_s=5.0,
        camera_observation_spin_time_allowance_s=15.0,
        camera_observation_dwell_frames=5,
        camera_observation_dwell_timeout_s=3.0,
        camera_observation_view_tolerance_rad=0.15,
        camera_observation_turn_margin_m=0.05,
        camera_observation_frame_max_turn_rate=0.35,
        camera_observation_min_range_m=1.3,
        camera_observation_max_range_m=3.5,
        camera_observation_half_fov_deg=30.0,
        camera_observation_repeat_radius_m=1.0,
        # Camera frontiers: once LiDAR frontiers are gone, viewpoints toward never-seen free
        # space, each reached by the explorer's usual goal and observed by the turns above.
        camera_frontier_max_goals=8,
        camera_frontier_budget_s=300.0,
        camera_frontier_min_cells=10,
        camera_frontier_standoffs_m=[2.0, 1.5, 1.0],
        camera_frontier_min_turn_budget_s=15.0,
    )

    def __init__(self, node):
        self.node = node
        for name, value in self.PARAMETERS.items():
            node.declare_parameter(name, value)
        node.declare_parameter('camera_observation_nav2_params_path', os.path.join(
            get_package_share_directory('ridgeback_autonomy'), 'config', 'nav2_params.yaml'))
        p = lambda n: node.get_parameter('camera_observation_' + n).value
        self.enabled = bool(p('enabled'))
        self.camera_frame = str(p('camera_frame'))
        self.odom_frame = str(p('odom_frame'))
        self.max_views = int(p('max_views'))
        self.budget_s = float(p('budget_s'))
        self.settle_timeout_s = float(p('settle_timeout_s'))
        self.spin_allowance_s = float(p('spin_time_allowance_s'))
        self.dwell_frames = int(p('dwell_frames'))
        self.dwell_timeout_s = float(p('dwell_timeout_s'))
        self.view_tolerance = float(p('view_tolerance_rad'))
        self.turn_margin_m = float(p('turn_margin_m'))
        self.frame_max_rate = float(p('frame_max_turn_rate'))
        self.repeat_radius_m = float(p('repeat_radius_m'))
        self.min_new_area_m2 = float(p('min_new_area_m2'))
        q = lambda n: node.get_parameter('camera_frontier_' + n).value
        self.frontier_max_goals = int(q('max_goals'))
        self.frontier_budget_s = float(q('budget_s'))
        self.frontier_min_cells = int(q('min_cells'))
        self.frontier_standoffs_m = [float(v) for v in q('standoffs_m')]
        self.frontier_min_turn_s = float(q('min_turn_budget_s'))
        self.frontier_tried = []       # world (x, y) of every viewpoint sent, never sent again
        self.frontier_started = None
        self.frontier_done = None      # why the camera frontiers ended, once they have
        if not (1 <= self.max_views <= 3 and self.budget_s >= 0 and self.dwell_frames >= 1
                and self.spin_allowance_s > 0 and self.dwell_timeout_s > 0):
            raise ValueError('camera observation limits out of range')
        self.coverage = CameraCoverage(ViewModel(
            half_fov_rad=math.radians(float(p('half_fov_deg'))),
            min_range_m=float(p('min_range_m')), max_range_m=float(p('max_range_m'))))
        self.motion = MotionSettlement(stable_s=float(p('stable_s')))
        self.round = None          # the current stop's observations
        self.spin_handle = None
        self.spent_s = 0.0
        self.attempts = []         # (x, y, heading) of views that were not seen, never retried
        self.history = []          # one summary per stop, for logs and tests
        self.map = None
        # Frames used for coverage, and those skipped (no map or pose at the stamp, turning).
        self.frames = {'marked': 0, 'no_pose': 0, 'turning': 0}
        self._coverage_lock = threading.Lock()
        self._camera_offset = None
        if not self.enabled:
            return
        self.footprint = parking_footprint(load_nav2_footprint(
            str(node.get_parameter('camera_observation_nav2_params_path').value)))
        group = MutuallyExclusiveCallbackGroup()
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, str(p('map_topic')), self.on_map, map_qos,
                                 callback_group=group)
        node.create_subscription(TargetDetections, str(p('frames_topic')), self.on_frame, 10,
                                 callback_group=group)
        node.create_subscription(Odometry, str(p('odometry_topic')), self.on_odometry, 10,
                                 callback_group=group)
        self.spin_client = ActionClient(node, Spin, 'spin')
        node.get_logger().info(
            f'Camera observation turns enabled: at most {self.max_views} per stop, '
            f'{self.budget_s:.0f} s budget')

    def frontier_problem(self, now):
        """Why no further camera-frontier viewpoint may be sent, or None."""
        if not self.enabled:
            return 'camera observation is off'
        if len(self.frontier_tried) >= self.frontier_max_goals:
            return f'all {self.frontier_max_goals} camera viewpoints used'
        if self.frontier_started is not None and now - self.frontier_started >= self.frontier_budget_s:
            return f'camera-frontier time budget of {self.frontier_budget_s:.0f} s spent'
        if self.budget_s - self.spent_s < self.frontier_min_turn_s:
            return 'observation-turn budget spent'
        return None

    def seen_on(self, grid):
        """A copy of the coverage aligned with ``grid``."""
        with self._coverage_lock:
            self.coverage._align(grid)
            return self.coverage.seen.copy()

    def viewpoint_gain(self, grid, x, y):
        """Unseen cells the best single heading would show from (x, y): the observation turns'
        own selection, so a viewpoint is only chosen where they would turn."""
        offset = self._offset()
        if offset is None:
            return 0
        with self._coverage_lock:
            views = self.coverage.select_views(
                grid, (x, y, 0.0), offset, max_views=1,
                min_new_cells=max(1, int(self.min_new_area_m2 / grid.resolution ** 2)))
        return views[0][2] if views else 0

    # Inputs.

    @property
    def active(self):
        return self.round is not None

    def on_map(self, msg):
        view = map_view(msg)
        if view is not None:
            self.map = view

    def on_odometry(self, msg):
        t = msg.twist.twist
        with self.node.mission_guard.lock:
            self.motion.observe((t.linear.x, t.linear.y, t.linear.z,
                                 t.angular.x, t.angular.y, t.angular.z),
                                Time.from_msg(msg.header.stamp).nanoseconds,
                                self.node.get_clock().now().nanoseconds, time.monotonic(),
                                frame_valid=(msg.child_frame_id == self.node.robot_base_frame
                                             and msg.header.frame_id == self.odom_frame))

    def camera_pose(self, stamp):
        """(x, y, viewing yaw) of the camera in the map at ``stamp``, or None."""
        try:
            t = self.node.tf_buffer.lookup_transform(
                self.node.current_costmap_frame(), self.camera_frame, stamp).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, optical_axis_yaw(t.rotation)

    def on_frame(self, msg):
        """One processed camera image (the detector publishes every frame, detected or not):
        mark what it saw, unless taken while turning fast; count it for a waiting view."""
        grid = self.map
        stamp = Time.from_msg(msg.header.stamp)
        pose = self.camera_pose(stamp)
        before = self.camera_pose(stamp - RclDuration(seconds=0.2))
        if grid is None or pose is None or before is None:
            self.frames['no_pose'] += 1
            return
        if abs(normalize_angle(pose[2] - before[2])) / 0.2 > self.frame_max_rate:
            self.frames['turning'] += 1
            return
        with self._coverage_lock:
            self.coverage.mark(grid, *pose)
            self.frames['marked'] += 1
        with self.node.mission_guard.lock:
            r = self.round
            if (r is not None and r['phase'] == 'dwelling' and stamp.nanoseconds >= r['dwell_from_ns']
                    and abs(normalize_angle(pose[2] - r['views'][r['index']][0])) <= self.view_tolerance):
                r['frames'] += 1

    # The stop's observations; every method below runs under the MissionGuard lock.

    def begin(self, *, focused):
        """After a successful frontier goal: start this stop's observations, or say why not."""
        if not self.enabled:
            return False
        if focused:
            return self._skip('exploration is focused around a localized G1')
        if self.spent_s >= self.budget_s:
            return self._skip(f'observation budget of {self.budget_s:.0f} s spent')
        now = time.monotonic()
        self.motion.reset()
        self.round = dict(phase='settling', started=now, deadline=now + self.settle_timeout_s,
                          views=[], index=0, seen=0, frames=0, notes=[], stop=None)
        return True

    def _skip(self, why):
        self.history.append({'outcome': 'skipped', 'reason': why})
        self.node.get_logger().info(f'Camera observation skipped: {why}')
        return False

    def poll(self):
        r = self.round
        if r is None:
            return
        now = time.monotonic()
        if r['phase'] == 'settling':
            if self.motion.ready(self.node.get_clock().now().nanoseconds, now):
                self._plan()
            elif now >= r['deadline']:
                self.finish(f'stop not confirmed: {self.motion.explain()}')
        elif r['phase'] == 'turning' and now >= r['deadline'] and not r.get('cancel_sent'):
            r['cancel_sent'] = True
            r['notes'].append('turn overdue; cancelling')
            self.node.mission_guard.cancel()
        elif r['phase'] == 'dwelling':
            if r['frames'] >= self.dwell_frames:
                r['seen'] += 1
                r['notes'].append(f'{math.degrees(r["views"][r["index"]][0]):+.0f} deg seen '
                                  f'({r["frames"]} frames)')
                self._next()
            elif now >= r['deadline']:
                heading = r['views'][r['index']][0]
                self.attempts.append((*r['stop'][:2], heading))
                r['notes'].append(f'{math.degrees(heading):+.0f} deg not seen: '
                                  f'{r["frames"]} usable frames')
                self._next()

    def _robot_pose(self):
        try:
            t = self.node.tf_buffer.lookup_transform(
                self.node.current_costmap_frame(), self.node.robot_base_frame, Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def _offset(self):
        if self._camera_offset is None:
            try:
                t = self.node.tf_buffer.lookup_transform(
                    self.node.robot_base_frame, self.camera_frame, Time()).transform
            except TransformException:
                return None
            self._camera_offset = (t.translation.x, t.translation.y)
        return self._camera_offset

    def _plan(self):
        r = self.round
        robot, grid, offset = self._robot_pose(), self.map, self._offset()
        if robot is None or grid is None or offset is None or self.node.current_costmap is None:
            self.finish('robot pose, map, camera transform or costmap unavailable')
            return
        r['stop'] = robot
        excluded = [h for x, y, h in self.attempts
                    if math.hypot(x - robot[0], y - robot[1]) <= self.repeat_radius_m]
        with self._coverage_lock:
            views = self.coverage.select_views(
                grid, robot, offset, max_views=self.max_views, excluded=excluded,
                min_new_cells=max(1, int(self.min_new_area_m2 / grid.resolution ** 2)))
        if not views:
            self.finish('no useful unseen direction')
            return
        r['views'] = views
        r['notes'].append('planned ' + ', '.join(
            f'{math.degrees(h):+.0f} deg ({g} cells)' for h, _, g in views))
        self._turn()

    def _turn(self):
        r = self.round
        heading, turn, _ = r['views'][r['index']]
        if self.spent_s + time.monotonic() - r['started'] >= self.budget_s:
            self.finish('observation budget spent')
            return
        robot = self._robot_pose()
        costmap = map_view(self.node.current_costmap)
        if robot is None or costmap is None:
            self.finish('robot pose or costmap unavailable before the turn')
            return
        # The planned signed turn, from where the robot now points.
        turn = turn + normalize_angle(self._planned_heading() - robot[2])
        problem = rotation_problem(costmap, self.footprint, robot, turn, margin_m=self.turn_margin_m)
        if problem is not None:
            self.attempts.append((robot[0], robot[1], heading))
            self.finish(f'turn to {math.degrees(heading):+.0f} deg not safe: {problem}')
            return
        guard = self.node.mission_guard
        if not guard.allowed() or guard.handle is not None:
            self.finish('exploration paused or a goal is still active')
            return
        if not self.spin_client.server_is_ready():
            self.finish('Nav2 spin server not ready')
            return
        goal = Spin.Goal()
        goal.target_yaw = float(turn)
        whole = int(self.spin_allowance_s)
        goal.time_allowance = Duration(sec=whole, nanosec=int((self.spin_allowance_s - whole) * 1e9))
        guard.goal_pending = True
        try:
            future = self.spin_client.send_goal_async(goal)
        except Exception as exc:
            guard.goal_pending = False
            self.finish(f'spin not sent: {exc}')
            return
        r.update(phase='turning', deadline=time.monotonic() + self.spin_allowance_s + 5.0,
                 cancel_sent=False)
        future.add_done_callback(self.on_spin_response)

    def _planned_heading(self):
        """Where the previous view left the camera: the stop's heading before the first."""
        r = self.round
        return r['stop'][2] if r['index'] == 0 else r['views'][r['index'] - 1][0]

    def on_spin_response(self, future):
        guard = self.node.mission_guard
        with guard.lock:
            guard.goal_pending = False
            guard.cancel_overdue = False
            try:
                handle = future.result()
            except Exception as exc:
                guard.pause('Spin acceptance failed: ' + str(exc))
                guard._publish_status_locked()
                self.finish(f'spin acceptance failed: {exc}')
                return
            if not handle.accepted:
                guard._publish_status_locked()
                self.finish('spin rejected by Nav2')
                return
            self.spin_handle = handle
            guard.accepted(handle)   # cancels at once if a pause arrived meanwhile
            handle.get_result_async().add_done_callback(
                lambda f, h=handle: self.on_spin_result(f, h))

    def on_spin_result(self, future, handle):
        guard = self.node.mission_guard
        with guard.lock:
            if handle is not self.spin_handle:
                return
            try:
                result = future.result()
                status = result.status
            except Exception as exc:
                guard.pause('Spin result unavailable: ' + str(exc))
                self.finish(f'spin result unavailable: {exc}')
                return
            if status not in _TERMINAL:
                guard.pause('Spin result is not terminal')
                self.finish('spin result not terminal')
                return
            guard.finished(handle)
            self.spin_handle = None
            r = self.round
            if r is None:
                return
            if status != GoalStatus.STATUS_SUCCEEDED:
                heading = r['views'][r['index']][0]
                self.attempts.append((*r['stop'][:2], heading))
                code = getattr(result.result, 'error_code', None)
                self.finish(f'turn to {math.degrees(heading):+.0f} deg ended: status {status}'
                            + (f', error {code}' if code else ''))
                return
            r.update(phase='dwelling', frames=0, deadline=time.monotonic() + self.dwell_timeout_s,
                     dwell_from_ns=self.node.get_clock().now().nanoseconds)

    def on_paused(self):
        """Exploration was paused (a G1 confirmation, or lost health): the guard has cancelled
        any turn; end the stop's observations once nothing is in flight."""
        if self.round is not None and self.spin_handle is None and not self.node.mission_guard.goal_pending:
            self.finish('exploration paused')

    def _next(self):
        r = self.round
        r['index'] += 1
        if r['index'] >= len(r['views']):
            self.finish('done')
        else:
            self._turn()

    def finish(self, why):
        r, self.round = self.round, None
        if r is None:
            return
        elapsed = time.monotonic() - r['started']
        self.spent_s += elapsed
        stop = r['stop']
        summary = {'outcome': why, 'stop': None if stop is None else tuple(round(v, 2) for v in stop),
                   'views_planned': len(r['views']), 'views_seen': r['seen'],
                   'duration_s': round(elapsed, 1), 'spent_s': round(self.spent_s, 1),
                   'notes': list(r['notes'])}
        self.history.append(summary)
        where = '' if stop is None else f' at ({stop[0]:.2f}, {stop[1]:.2f})'
        self.node.get_logger().info(
            f'Camera observation{where}: {why}; {r["seen"]}/{len(r["views"])} views seen in '
            f'{elapsed:.1f} s ({self.spent_s:.0f}/{self.budget_s:.0f} s budget); '
            + '; '.join(r['notes']) + f'; frames so far {self.frames}')
