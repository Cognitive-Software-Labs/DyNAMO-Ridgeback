#!/usr/bin/env python3
"""Tell Nav2 where bystanders are heading, so it plans and paces around them.

The tracker publishes camera-confirmed people with velocities, and walkers
seen only by the lidar (``leader_follow/moving_people``: people approaching
from outside the camera view). This node excludes the leader, including any
lidar walker within ``leader_exclusion`` of the leader's last position, and,
for every other person:

* publishes a ``PointCloud2`` in the robot frame that marks the person's body
  now and along their constant-velocity prediction (``leader_follow/people_obstacles``).
  A dedicated Nav2 obstacle layer consumes it; a ring of points beyond the
  layer's marking range clears the previous prediction on every update;
* publishes ``nav2_msgs/SpeedLimit`` on ``speed_limit``: the speed along the
  robot's current direction that, rolled out with the platform's acceleration
  limit, keeps it clearest of everyone's predicted motion over ``horizon``.
  Nav2's controller caps its speed accordingly, so the robot brakes for people
  it is about to pass close to, but not for someone who will pass behind it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

Person2D = Tuple[float, float, float, float]  # x, y, vx, vy in the global frame


@dataclass(frozen=True)
class PredictionSettings:
    horizon: float = 3.          # seconds of motion considered for the speed limit
    mark_horizon: float = 1.     # seconds of predicted path marked in the costmap
    mark_step: float = .25       # seconds between marked predictions
    body_radius: float = .3      # metres marked around each predicted position
    comfort: float = 1.2         # centre distance at which no slowing is needed
    contact: float = .7          # centre distance treated as touching
    min_speed: float = .05       # slowest speed limit (m/s); Nav2 reads 0 as "no limit"
    max_speed: float = 1.3       # speed limit when nobody is predicted near (platform limit)
    acceleration: float = 1.     # platform acceleration limit used in the rollouts (m/s^2)
    speed_step: float = .05      # spacing of the candidate speeds
    time_step: float = .1        # rollout sampling (s)
    speed_preference: float = .1 # score a full stop pays over max_speed, to break near-ties
    clear_radius: float = 7.     # ring radius that clears the previous prediction
    clear_points: int = 900

    def __post_init__(self):
        if not (0 < self.contact < self.comfort and 0 < self.min_speed < self.max_speed
                and self.acceleration > 0 and self.speed_step > 0 and self.time_step > 0
                and self.speed_preference >= 0 and self.horizon > 0 and 0 <= self.mark_horizon and self.mark_step > 0
                and self.body_radius > 0 and self.clear_radius > 0 and self.clear_points >= 8):
            raise ValueError('invalid people prediction settings')


def braking_travel(speed: float, target: float, times, acceleration: float):
    """Distance covered, and speed, at ``times`` while changing from ``speed`` to ``target``."""
    import numpy as np
    change = abs(target - speed) / acceleration
    a = math.copysign(acceleration, target - speed)
    times = np.asarray(times, dtype=float)
    reached = speed * change + .5 * a * change * change
    changing = times < change
    return (np.where(changing, speed * times + .5 * a * times * times, reached + target * (times - change)),
            np.where(changing, speed + a * times, target))


def encounter_score(robot: Tuple[float, float], direction: Tuple[float, float], travel, speeds,
                    people: Sequence[Person2D], times, settings: PredictionSettings) -> float:
    """Score of one robot motion against everyone's predicted motion; lower is better.

    A motion that comes within ``contact`` of anyone scores above every motion
    that does not, plus how fast and how soon it touches them, so an
    unavoidable contact is met as slowly and as late as possible. Otherwise the
    score is the worst closeness (zero at ``comfort``, one at ``contact``)
    discounted by how far in the future it happens.
    """
    import numpy as np
    if not people:
        return 0.
    p = np.asarray(people, dtype=float)
    px, py = p[:, 0, None] + p[:, 2, None] * times, p[:, 1, None] + p[:, 3, None] * times
    distance = np.hypot(px - (robot[0] + direction[0] * travel), py - (robot[1] + direction[1] * travel))
    urgency = 1. - times / settings.horizon
    touching = distance < settings.contact
    if touching.any():
        impact = urgency * np.asarray(speeds) / settings.max_speed
        return 2. + float(impact[touching.any(axis=0)].max())
    closeness = np.clip((settings.comfort - distance) / (settings.comfort - settings.contact), 0., 1.)
    return float((closeness * urgency).max())


def speed_limit(robot: Tuple[float, float], robot_velocity: Tuple[float, float],
                people: Iterable[Person2D], settings: PredictionSettings, heading: float = 0.) -> float:
    """Speed cap that keeps the robot clearest of everyone's predicted motion.

    Each candidate speed along the robot's current direction of travel (its
    heading when nearly still) is rolled out over ``horizon`` with the
    platform's ``acceleration`` limit, against every person moving at constant
    velocity, and scored by ``encounter_score``; the cap is the candidate with
    the lowest score, with a small ``speed_preference`` for faster ones. Braking is therefore chosen only when it actually widens the
    gap: when a walker will pass behind, slowing would put the robot in their
    path, so the cap stays high.
    """
    import numpy as np
    people = list(people)
    if not people:
        return settings.max_speed
    speed = math.hypot(*robot_velocity)
    direction = ((robot_velocity[0] / speed, robot_velocity[1] / speed) if speed > .1
                 else (math.cos(heading), math.sin(heading)))
    times = np.arange(0., settings.horizon + 1e-9, settings.time_step)
    best = None
    count = int(round((settings.max_speed - settings.min_speed) / settings.speed_step))
    for k in range(count + 1):
        candidate = settings.max_speed - k * settings.speed_step
        travel, speeds = braking_travel(speed, candidate, times, settings.acceleration)
        score = (encounter_score(robot, direction, travel, speeds, people, times, settings)
                 + settings.speed_preference * (settings.max_speed - candidate) / settings.max_speed)
        if best is None or score < best[0] - 1e-9:
            best = (score, candidate)
    return best[1]


def obstacle_points(people: Iterable[Person2D], settings: PredictionSettings) -> List[Tuple[float, float]]:
    """Body discs at each person's current and predicted positions (global frame)."""
    points = []
    steps = int(round(settings.mark_horizon / settings.mark_step))
    for x, y, vx, vy in people:
        for k in range(steps + 1):
            t = k * settings.mark_step
            cx, cy = x + vx * t, y + vy * t
            points.append((cx, cy))
            for ring, count in ((settings.body_radius / 2, 6), (settings.body_radius, 12)):
                for i in range(count):
                    a = 2 * math.pi * i / count
                    points.append((cx + ring * math.cos(a), cy + ring * math.sin(a)))
    return points


def clearing_ring(settings: PredictionSettings) -> List[Tuple[float, float]]:
    """Robot-frame points beyond the marking range: their rays clear old predictions."""
    n = settings.clear_points
    return [(settings.clear_radius * math.cos(2 * math.pi * i / n),
             settings.clear_radius * math.sin(2 * math.pi * i / n)) for i in range(n)]


def bystanders(people: Sequence[Tuple[int, float, float, float, float]], leader_id: Optional[int]) -> List[Person2D]:
    return [(x, y, vx, vy) for track_id, x, y, vx, vy in people if track_id != leader_id]


def away_from(people: Iterable[Person2D], centre: Optional[Tuple[float, float]], radius: float) -> List[Person2D]:
    """People further than ``radius`` from ``centre`` (all of them without a centre)."""
    if centre is None:
        return list(people)
    return [p for p in people if math.hypot(p[0] - centre[0], p[1] - centre[1]) > radius]


def main():
    import struct

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import Buffer, TransformException, TransformListener

    from leader_follow_interfaces.msg import LeaderState, People

    class PeoplePredictorNode(Node):
        def __init__(self):
            super().__init__('people_predictor')
            defaults = dict(people_topic='leader_follow/people', leader_topic='leader_follow/leader',
                            odom_topic='platform/odom', base_frame='base_link',
                            obstacle_topic='leader_follow/people_obstacles', speed_limit_topic='speed_limit',
                            moving_topic='leader_follow/moving_people', moving_max_age=.25,
                            leader_exclusion=1., leader_memory=2.,
                            cloud_height=.5, horizon=3., mark_horizon=1., comfort=1.2, contact=.7,
                            min_speed=.05, max_speed=1.3, acceleration=1.)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            p = lambda name: self.get_parameter(name).value
            self.settings = PredictionSettings(horizon=p('horizon'), mark_horizon=p('mark_horizon'),
                                               comfort=p('comfort'), contact=p('contact'),
                                               min_speed=p('min_speed'), max_speed=p('max_speed'),
                                               acceleration=p('acceleration'))
            self.base_frame, self.height = p('base_frame'), float(p('cloud_height'))
            self.ring = clearing_ring(self.settings)
            self.leader_id = None
            self.leader_seen = None   # (stamp, x, y, vx, vy) of the leader's last track
            self.moving = None        # latest lidar-only walkers message
            self.moving_max_age = float(p('moving_max_age'))
            self.leader_exclusion = float(p('leader_exclusion'))
            self.leader_memory = float(p('leader_memory'))
            self.robot_velocity = (0., 0.)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.cloud = self.create_publisher(PointCloud2, p('obstacle_topic'), 10)
            self.limit = self.create_publisher(SpeedLimit, p('speed_limit_topic'), 10)
            self.create_subscription(LeaderState, p('leader_topic'), self.on_leader, 10)
            self.create_subscription(Odometry, p('odom_topic'), self.on_odom, 10)
            self.create_subscription(People, p('moving_topic'), self.on_moving, 10)
            self.create_subscription(People, p('people_topic'), self.on_people, 10)

        def on_leader(self, msg):
            self.leader_id = int(msg.track_id) if msg.has_leader else None

        def on_moving(self, msg):
            self.moving = msg

        def leader_centre(self, at):
            if self.leader_seen is None or not 0 <= at - self.leader_seen[0] <= self.leader_memory:
                return None
            seen, x, y, vx, vy = self.leader_seen
            return x + vx * (at - seen), y + vy * (at - seen)

        def on_odom(self, msg):
            # Body-frame twist; rotated into the global frame with the TF yaw below.
            self.robot_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        def on_people(self, msg):
            try:
                tf = self.tf_buffer.lookup_transform(msg.header.frame_id, self.base_frame, Time())
            except TransformException:
                return
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = self.robot_velocity
            velocity = (c * bx - s * by, s * bx + c * by)
            at = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for p in msg.people:
                if int(p.id) == self.leader_id:
                    self.leader_seen = (at, p.position.x, p.position.y, p.velocity.x, p.velocity.y)
            people = bystanders([(int(p.id), p.position.x, p.position.y, p.velocity.x, p.velocity.y)
                                 for p in msg.people], self.leader_id)
            moving = self.moving
            if (moving is not None and moving.header.frame_id == msg.header.frame_id
                    and abs(at - (moving.header.stamp.sec + moving.header.stamp.nanosec * 1e-9)) <= self.moving_max_age):
                # Duplicates of confirmed people are harmless here; the leader is not.
                people += away_from([(m.position.x, m.position.y, m.velocity.x, m.velocity.y) for m in moving.people],
                                    self.leader_centre(at), self.leader_exclusion)
            limit = speed_limit((t.x, t.y), velocity, people, self.settings, heading=yaw)
            self.limit.publish(SpeedLimit(header=msg.header, percentage=False, speed_limit=float(limit)))
            # Points to the robot frame: the obstacle layer raytraces from the cloud's origin.
            local = []
            for gx, gy in obstacle_points(people, self.settings):
                dx, dy = gx - t.x, gy - t.y
                local.append((c * dx + s * dy, -s * dx + c * dy))
            cloud = PointCloud2()
            cloud.header.frame_id = self.base_frame
            cloud.header.stamp = self.get_clock().now().to_msg()
            cloud.height, cloud.is_bigendian, cloud.is_dense = 1, False, True
            cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate('xyz')]
            cloud.point_step = 12
            points = local + self.ring
            cloud.width = len(points)
            cloud.row_step = 12 * len(points)
            cloud.data = b''.join(struct.pack('<fff', x, y, self.height) for x, y in points)
            self.cloud.publish(cloud)

    rclpy.init()
    node = PeoplePredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
