#!/usr/bin/env python3
"""Turn tracked people into a leader state with the lock-on state machine.

Subscribes to ``People`` and publishes one ``LeaderState`` per message. The
leader is acquired as the nearest person in front of the robot and confirmed by
appearance (lock_on.py). A person without an appearance embedding cannot be
confirmed, so until the tracker supplies embeddings the lock never reaches
LOCKED: the node fails closed rather than following an unverified track.
"""
import math
from collections import deque
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from leader_follow_interfaces.msg import LeaderState, People
from ridgeback_leader_follow.acquisition import nearest_in_front
from ridgeback_leader_follow.lock_on import LeaderLock, cosine_similarity
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

    def __init__(self, lock: LeaderLock, max_range: float, half_angle: float,
                 acquire_distance_tolerance: float = 0., acquire_lateral_tolerance: float = 0.,
                 continuity_min_confidence: float = .5) -> None:
        self.lock = lock
        # Track continuity ignores observations below this confidence, and
        # nearby people at or above it count as competing hypotheses.
        self.continuity_min_confidence = float(continuity_min_confidence)
        self.max_range = max_range
        self.half_angle = half_angle
        self.acquire_distance_tolerance=acquire_distance_tolerance
        self.acquire_lateral_tolerance=acquire_lateral_tolerance
        self.previous = {}
        self.previous_stamp = None
        self.previous_frame = None
        self.last_verified = None
        self.last_verified_stamp = None
        self.last_verified_frame = None
        self.nonleader_ids = set()
        self.nonleader_features = {}

    def update(self, msg: People, to_base: Optional[Planar]) -> LeaderState:
        if self.lock.state == "SEARCHING":
            self.nonleader_ids.clear()
            self.nonleader_features.clear()
        tracks = person_tracks(msg)
        positions: Dict[int, object] = {int(p.id): p.position for p in msg.people}

        def propose(candidates):
            if to_base is None:
                return None, "no_base_transform"
            placed = [(t, *apply_planar(positions[t.track_id].x, positions[t.track_id].y, to_base)) for t in candidates]
            return nearest_in_front(placed, self.max_range, self.half_angle,
                                    self.acquire_distance_tolerance,self.acquire_lateral_tolerance)

        current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        dt = current_stamp-self.previous_stamp if self.previous_stamp is not None else None
        if dt is not None and dt < 0:
            self.lock.reset()
            self.previous = {}
            self.last_verified = None
            self.nonleader_ids.clear()
            self.nonleader_features.clear()
        observations = {int(p.id): p for p in msg.people}
        def continuity(track):
            previous = self.previous.get(track.track_id)
            reference_stamp = self.previous_stamp
            reference_frame = self.previous_frame
            if self.lock.state != "LOCKED" or previous is None:
                previous = self.last_verified
                reference_stamp = self.last_verified_stamp
                reference_frame = self.last_verified_frame
            current = observations[track.track_id]
            elapsed = current_stamp-reference_stamp if reference_stamp is not None else None
            if (previous is None or int(previous.id) != track.track_id or elapsed is None
                    or not 0 < elapsed <= .75 or reference_frame != msg.header.frame_id
                    or not math.isfinite(track.score) or track.score < self.continuity_min_confidence):
                return False
            predicted = (previous.position.x + previous.velocity.x*elapsed,
                         previous.position.y + previous.velocity.y*elapsed)
            error = math.hypot(current.position.x-predicted[0], current.position.y-predicted[1])
            if not math.isfinite(error) or error > .75:
                return False
            for other_id, other in observations.items():
                if other_id == track.track_id or other.confidence < self.continuity_min_confidence:
                    continue
                competing = math.hypot(other.position.x-predicted[0],other.position.y-predicted[1])
                if math.isfinite(competing) and competing <= error+.1:
                    return False
            return True
        def motion_candidates(candidates):
            if (self.last_verified is None or self.last_verified_stamp is None
                    or self.last_verified_frame != msg.header.frame_id):
                return []
            elapsed = current_stamp-self.last_verified_stamp
            if not 0 < elapsed <= 3.:
                return []
            predicted = (self.last_verified.position.x+self.last_verified.velocity.x*elapsed,
                         self.last_verified.position.y+self.last_verified.velocity.y*elapsed)
            gated = []
            for candidate in candidates:
                if candidate.track_id in self.nonleader_ids:
                    positive=self.lock.similarity(candidate.feature)
                    negatives=self.nonleader_features.get(candidate.track_id,[])
                    previous_similarity=max((cosine_similarity(candidate.feature,feature) for feature in negatives),default=1.)
                    separation=(positive-previous_similarity) if positive is not None else -1.
                    if (positive is None or positive < self.lock.match_threshold
                            or separation <= max(1e-7,self.lock.recovery_margin)):
                        continue
                point = observations[candidate.track_id].position
                distance = math.hypot(point.x-predicted[0],point.y-predicted[1])
                if math.isfinite(distance) and distance <= .75:
                    gated.append((distance,candidate))
            if not gated:
                return []
            gated.sort(key=lambda pair:pair[0])
            return [candidate for distance,candidate in gated if distance <= gated[0][0]+.1]
        def confirm(candidates):
            plausible=motion_candidates(candidates)
            if len(plausible) != 1:
                return None, "ambiguous_or_missing_motion_recovery"
            return plausible[0], "motion_consistent_candidate"
        update = self.lock.update(tracks, acquire=propose, confirm=confirm, continuity=continuity,
                                  recovery_filter=motion_candidates)
        if update.valid(self.lock.state):
            self.nonleader_ids.discard(update.selected.track_id)
            self.nonleader_features.pop(update.selected.track_id,None)
            self.nonleader_ids.update(track_id for track_id in observations
                                      if track_id != update.selected.track_id)
            for track in tracks:
                if track.track_id != update.selected.track_id and np.asarray(track.feature).size:
                    history=self.nonleader_features.setdefault(track.track_id,[])
                    history.append(np.asarray(track.feature,dtype=np.float32).copy())
                    self.nonleader_features[track.track_id]=history[-8:]
            self.last_verified = observations[update.selected.track_id]
            self.last_verified_stamp = current_stamp
            self.last_verified_frame = msg.header.frame_id
        self.previous = observations
        self.previous_stamp = current_stamp
        self.previous_frame = msg.header.frame_id
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
                        acquire_interval=8, acquire_frames=3, lost_frames=8, match_threshold=0.72, recovery_margin=0.05, tf_wait_timeout=0.25,
                        acquire_distance_tolerance=0.0, acquire_lateral_tolerance=0.0,
                        continuity_min_confidence=0.5)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.base_frame = p("base_frame")
        lock = LeaderLock(acquire_interval=p("acquire_interval"), acquire_frames=p("acquire_frames"),
                          lost_frames=p("lost_frames"), match_threshold=p("match_threshold"),
                          recovery_margin=p("recovery_margin"))
        self.identity = LeaderIdentity(lock, p("acquire_range"), math.radians(p("acquire_half_angle_deg")),
                                       p("acquire_distance_tolerance"),p("acquire_lateral_tolerance"),
                                       p("continuity_min_confidence"))
        self.tf_wait_timeout = float(p("tf_wait_timeout"))
        self.pending = deque(maxlen=100)
        self.last_received_stamp = -math.inf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(LeaderState, p("leader_topic"), 10)
        self.create_subscription(People, p("people_topic"), self.on_people, 10)
        self.create_timer(.02, self.process_pending)

    def to_base(self, msg: People) -> Optional[Planar]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp), Duration(seconds=0.0))
        except TransformException as error:
            self.get_logger().warn(f"no {msg.header.frame_id} -> {self.base_frame} transform: {error}",
                                   throttle_duration_sec=5.0)
            return None
        return planar_transform(transform.transform)

    def on_people(self, msg: People) -> None:
        at=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if at < self.last_received_stamp:
            self.pending.clear()
        self.last_received_stamp=at
        self.pending.append(msg)

    def process_pending(self) -> None:
        now=self.get_clock().now().nanoseconds*1e-9
        while self.pending:
            msg=self.pending[0]
            transform=self.to_base(msg)
            at=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if transform is None and now-at <= self.tf_wait_timeout:
                return  # Let queued TF callbacks run; never block inside this callback.
            self.pending.popleft()
            if transform is None:
                empty=People();empty.header=msg.header
                result=self.identity.update(empty,None)
                result.reason="no_base_transform"
            else:
                result=self.identity.update(msg,transform)
            self.publisher.publish(result)


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
