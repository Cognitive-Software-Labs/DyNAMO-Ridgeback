#!/usr/bin/env python3
"""Follow the locked leader through Nav2; stop whenever identity is not LOCKED.

The ROS-free ``FollowPlanner`` turns the leader and robot positions into a
standoff goal. ``FollowControllerNode`` sends that goal to Nav2's
``navigate_to_pose`` with a goal-updating behaviour tree, refreshes it through
``goal_update`` while the lock holds, and cancels navigation as soon as the
leader state is not LOCKED or stops arriving. Nav2 supplies path planning,
obstacle clearance, speed limits and the collision monitor.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

Pose2D = Tuple[float, float, float]  # x, y, yaw in the global frame


@dataclass(frozen=True)
class FollowSettings:
    follow_distance: float = 1.2   # metres from robot centre to leader
    hold_tolerance: float = .15    # within follow_distance + this, turn in place only
    stale_timeout: float = .5      # seconds without a fresh LOCKED state before stopping
    # Aim at the leader's predicted position this far ahead (plus the state's
    # age). Without it the robot only matches the leader's speed and trails
    # by a constant gap.
    lead_time: float = 1.5
    max_speed: float = 2.          # leader speeds above this are clamped

    def __post_init__(self):
        if not (self.follow_distance > 0 and self.hold_tolerance >= 0 and self.stale_timeout > 0
                and self.lead_time >= 0 and self.max_speed > 0):
            raise ValueError('follow_distance, stale_timeout and max_speed must be positive; '
                             'hold_tolerance and lead_time non-negative')


class LeaderVelocity:
    """Smoothed leader velocity from successive LOCKED positions of one track."""

    def __init__(self, smoothing: float = .3, max_gap: float = .5) -> None:
        self.smoothing, self.max_gap = smoothing, max_gap
        self.reset()

    def reset(self) -> None:
        self.track_id = self.stamp = self.position = None
        self.velocity = (0., 0.)

    def update(self, track_id: int, stamp: float, position: Tuple[float, float]) -> Tuple[float, float]:
        if (track_id != self.track_id or self.stamp is None or stamp < self.stamp
                or stamp - self.stamp > self.max_gap):
            self.reset()
            self.track_id, self.stamp, self.position = track_id, stamp, position
            return self.velocity
        dt = stamp - self.stamp
        if dt <= 0:
            return self.velocity
        raw = ((position[0] - self.position[0]) / dt, (position[1] - self.position[1]) / dt)
        a = self.smoothing
        self.velocity = (a * raw[0] + (1 - a) * self.velocity[0], a * raw[1] + (1 - a) * self.velocity[1])
        self.stamp, self.position = stamp, position
        return self.velocity


@dataclass(frozen=True)
class FollowDecision:
    goal: Optional[Pose2D]   # None: stop (cancel navigation)
    mode: str                # following | holding | stopped
    reason: str


class FollowPlanner:
    def __init__(self, settings: FollowSettings = FollowSettings()) -> None:
        self.settings = settings

    def decide(self, locked: bool, leader: Optional[Tuple[float, float]], age: Optional[float],
               robot: Optional[Pose2D], velocity: Tuple[float, float] = (0., 0.)) -> FollowDecision:
        """Return the goal for this tick. Every uncertain input stops the robot."""
        if not locked:
            return FollowDecision(None, 'stopped', 'leader_not_locked')
        if leader is None or age is None or not math.isfinite(age) or age > self.settings.stale_timeout:
            return FollowDecision(None, 'stopped', 'leader_state_stale')
        if robot is None:
            return FollowDecision(None, 'stopped', 'no_robot_pose')
        speed = math.hypot(*velocity)
        if not math.isfinite(speed):
            velocity, speed = (0., 0.), 0.
        if speed > self.settings.max_speed:
            velocity = (velocity[0] * self.settings.max_speed / speed, velocity[1] * self.settings.max_speed / speed)
        horizon = age + self.settings.lead_time
        target = (leader[0] + velocity[0] * horizon, leader[1] + velocity[1] * horizon)
        dx, dy = target[0] - robot[0], target[1] - robot[1]
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance):
            return FollowDecision(None, 'stopped', 'invalid_position')
        # Face the leader so it stays in the front camera's view.
        yaw = math.atan2(dy, dx) if distance > 1e-6 else robot[2]
        if distance <= self.settings.follow_distance + self.settings.hold_tolerance:
            return FollowDecision((robot[0], robot[1], yaw), 'holding', 'within_follow_distance')
        scale = (distance - self.settings.follow_distance) / distance
        return FollowDecision((robot[0] + dx * scale, robot[1] + dy * scale, yaw), 'following', 'leader_locked')


def heading_rate(robot: Pose2D, leader: Tuple[float, float], gain: float, max_rate: float) -> float:
    """Turn rate that points the robot's front (and camera) at the leader."""
    error = math.atan2(leader[1] - robot[1], leader[0] - robot[0]) - robot[2]
    error = math.atan2(math.sin(error), math.cos(error))
    return max(-max_rate, min(max_rate, gain * error))


def yaw_to_quaternion(yaw: float):
    return 0., 0., math.sin(yaw / 2), math.cos(yaw / 2)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    import rclpy
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from leader_follow_interfaces.msg import LeaderState

    class FollowControllerNode(Node):
        def __init__(self):
            super().__init__('follow_controller')
            share = get_package_share_directory('ridgeback_leader_follow')
            defaults = dict(leader_topic='leader_follow/leader', global_frame='odom', base_frame='base_link',
                            follow_distance=1.2, hold_tolerance=.15, stale_timeout=.5, lead_time=1.5,
                            update_rate=5.,
                            behavior_tree=f'{share}/config/behavior_trees/follow_leader.xml',
                            navigate_action='navigate_to_pose', goal_update_topic='goal_update',
                            heading_gain=2., max_heading_rate=1.,
                            controller_cmd_topic='cmd_vel_mppi', output_cmd_topic='cmd_vel_nav')
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            p = lambda name: self.get_parameter(name).value
            self.global_frame, self.base_frame = p('global_frame'), p('base_frame')
            self.behavior_tree = p('behavior_tree')
            self.planner = FollowPlanner(FollowSettings(p('follow_distance'), p('hold_tolerance'),
                                                        p('stale_timeout'), p('lead_time')))
            self.velocity = LeaderVelocity()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.navigate = ActionClient(self, NavigateToPose, p('navigate_action'))
            self.goal_update = self.create_publisher(PoseStamped, p('goal_update_topic'), 10)
            self.status = self.create_publisher(String, 'leader_follow/follow_status', 10)
            self.leader = None
            self.goal_handle = None
            self.sending = False
            self.mode = None
            self.heading_gain, self.max_heading_rate = p('heading_gain'), p('max_heading_rate')
            self.stale_timeout = p('stale_timeout')
            # Nav2 plans translation; this node owns the turn rate so the
            # camera stays on the leader while the holonomic base sidesteps.
            self.cmd_out = self.create_publisher(TwistStamped, p('output_cmd_topic'), 10)
            self.create_subscription(TwistStamped, p('controller_cmd_topic'), self.on_controller_cmd, 10)
            self.create_subscription(LeaderState, p('leader_topic'), self.on_leader, 10)
            self.create_timer(1. / float(p('update_rate')), self.tick)

        def on_leader(self, msg):
            self.leader = msg
            if msg.state == LeaderState.LOCKED and msg.has_leader and msg.header.frame_id == self.global_frame:
                self.velocity.update(msg.track_id, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                     (msg.position.x, msg.position.y))
            else:
                self.velocity.reset()

        def on_controller_cmd(self, cmd):
            msg = self.leader
            if (self.mode in ('following', 'holding') and msg is not None and msg.state == LeaderState.LOCKED
                    and msg.has_leader and msg.header.frame_id == self.global_frame):
                age = self.get_clock().now().nanoseconds * 1e-9 - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                robot = self.robot_pose()
                if robot is not None and age <= self.stale_timeout:
                    cmd.twist.angular.z = heading_rate(robot, (msg.position.x, msg.position.y),
                                                       self.heading_gain, self.max_heading_rate)
            self.cmd_out.publish(cmd)

        def robot_pose(self) -> Optional[Pose2D]:
            try:
                transform = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
            except TransformException:
                return None
            t, q = transform.transform.translation, transform.transform.rotation
            return t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)

        def tick(self):
            msg = self.leader
            now = self.get_clock().now().nanoseconds * 1e-9
            locked = msg is not None and msg.state == LeaderState.LOCKED and msg.has_leader
            leader = age = None
            if locked:
                if msg.header.frame_id != self.global_frame:
                    locked = False
                    self.get_logger().warn(
                        f'leader frame {msg.header.frame_id!r} is not {self.global_frame!r}; stopping',
                        throttle_duration_sec=5.)
                else:
                    leader = (msg.position.x, msg.position.y)
                    age = now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            decision = self.planner.decide(locked, leader, age, self.robot_pose(),
                                           self.velocity.velocity if locked else (0., 0.))
            if decision.mode != self.mode:
                self.get_logger().info(f'{decision.mode}: {decision.reason}')
                self.mode = decision.mode
            self.status.publish(String(data=f'{decision.mode}:{decision.reason}'))
            if decision.goal is None:
                self.stop()
                return
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = decision.goal[0], decision.goal[1]
            q = yaw_to_quaternion(decision.goal[2])
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            if self.goal_handle is None and not self.sending:
                self.start(pose)
            else:
                self.goal_update.publish(pose)

        def start(self, pose):
            if not self.navigate.server_is_ready():
                self.get_logger().warn('navigate_to_pose is not ready', throttle_duration_sec=5.)
                return
            goal = NavigateToPose.Goal(pose=pose, behavior_tree=self.behavior_tree)
            self.sending = True
            self.navigate.send_goal_async(goal).add_done_callback(self.on_accepted)

        def on_accepted(self, future):
            self.sending = False
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('follow goal rejected')
                return
            self.goal_handle = handle
            if self.mode == 'stopped':
                self.stop()   # the lock dropped while the goal was in flight
                return
            handle.get_result_async().add_done_callback(lambda f, h=handle: self.on_result(h, f))

        def on_result(self, handle, future):
            if self.goal_handle is handle:
                self.goal_handle = None   # aborted or finished: the next tick re-sends
                status = future.result().status
                if status != GoalStatus.STATUS_CANCELED:
                    self.get_logger().info(f'navigation ended with status {status}; re-sending')

        def stop(self):
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None

    rclpy.init()
    node = FollowControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
