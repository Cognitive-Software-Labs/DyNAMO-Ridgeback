#!/usr/bin/env python3
"""Turn tracked people into a leader state with the lock-on state machine.

Subscribes to ``People`` and publishes one ``LeaderState`` per message. The
leader is acquired as the nearest person in front of the robot and confirmed by
appearance (lock_on.py). A person without an appearance embedding cannot be
confirmed, so until the tracker supplies embeddings the lock never reaches
LOCKED: the node fails closed rather than following an unverified track.
"""
import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from leader_follow_interfaces.msg import LeaderState, People
from ridgeback_leader_follow.acquisition import nearest_in_front
from ridgeback_leader_follow.lock_on import LeaderLock
from ridgeback_leader_follow.tracks import Track

STATE_CODES = {
    "SEARCHING": LeaderState.SEARCHING,
    "LOCKED": LeaderState.LOCKED,
    "UNCERTAIN": LeaderState.UNCERTAIN,
    "LOST": LeaderState.LOST,
}

Planar = Tuple[float, float, float]  # x, y, yaw of a frame change


def planar_transform(transform) -> Planar:
    """Reduce a geometry_msgs/Transform to its ground-plane translation and yaw."""
    q = transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return transform.translation.x, transform.translation.y, yaw


def apply_planar(x: float, y: float, frame: Planar) -> Tuple[float, float]:
    tx, ty, yaw = frame
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def person_tracks(msg: People):
    return [Track(int(p.id), float(p.confidence), np.asarray(p.appearance, dtype=np.float32)) for p in msg.people]


class LeaderIdentity:
    """ROS-free core: People + (people frame -> base frame) in, LeaderState out."""

    def __init__(self, lock: LeaderLock, max_range: float, half_angle: float) -> None:
        self.lock = lock
        self.max_range = max_range
        self.half_angle = half_angle

    def update(self, msg: People, to_base: Optional[Planar]) -> LeaderState:
        tracks = person_tracks(msg)
        positions: Dict[int, object] = {int(p.id): p.position for p in msg.people}

        def propose(candidates):
            if to_base is None:
                return None, "no_base_transform"
            placed = [(t, *apply_planar(positions[t.track_id].x, positions[t.track_id].y, to_base)) for t in candidates]
            return nearest_in_front(placed, self.max_range, self.half_angle)

        update = self.lock.update(tracks, acquire=propose)
        out = LeaderState()
        out.header = msg.header
        out.state = STATE_CODES[self.lock.state]
        out.reason = self.lock.reason
        if update.selected is not None:
            out.has_leader = True
            out.track_id = update.selected.track_id
            out.position = positions[update.selected.track_id]
        return out


class LeaderIdentityNode(Node):
    def __init__(self, **kwargs):
        super().__init__("leader_identity", **kwargs)
        defaults = dict(people_topic="leader_follow/people", leader_topic="leader_follow/leader",
                        base_frame="base_link", acquire_range=2.0, acquire_half_angle_deg=30.0,
                        acquire_interval=8, acquire_frames=3, lost_frames=8, match_threshold=0.72)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.base_frame = p("base_frame")
        lock = LeaderLock(acquire_interval=p("acquire_interval"), acquire_frames=p("acquire_frames"),
                          lost_frames=p("lost_frames"), match_threshold=p("match_threshold"))
        self.identity = LeaderIdentity(lock, p("acquire_range"), math.radians(p("acquire_half_angle_deg")))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(LeaderState, p("leader_topic"), 10)
        self.create_subscription(People, p("people_topic"), self.on_people, 10)

    def to_base(self, msg: People) -> Optional[Planar]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp), Duration(seconds=0.1))
        except TransformException as error:
            self.get_logger().warn(f"no {msg.header.frame_id} -> {self.base_frame} transform: {error}",
                                   throttle_duration_sec=5.0)
            return None
        return planar_transform(transform.transform)

    def on_people(self, msg: People) -> None:
        self.publisher.publish(self.identity.update(msg, self.to_base(msg)))


def main():
    rclpy.init()
    node = LeaderIdentityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
