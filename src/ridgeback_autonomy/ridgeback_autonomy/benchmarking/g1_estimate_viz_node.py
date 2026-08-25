#!/usr/bin/env python3

"""Where each estimator thinks the G1 is, as RViz rings, plus a HUD readout.

Two renderings of the same measurements. The rings put every estimator's answer
on the floor plan at once, so disagreement is spatial and immediate; the HUD
panel prints the same numbers against the benchmark's ground truth, because a
ring 60 mm off and a ring 2 m off look alike once they are small.

The estimator set is driven from ``benchmarking/estimators.py`` rather than
listed here, so a new estimator appears in both renderings by registering there.
"""

from __future__ import annotations

import html
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_LABELS,
    ESTIMATOR_POSITION_ATTRS,
    GROUND_TRUTH_TOPIC,
    PUBLIC_ESTIMATOR_ORDER,
)
from ridgeback_autonomy.msg import G1Measurements


# Seconds before a marker auto-expires if no new detection arrives.
# Set to 1.5x the detector's max publish interval (detector runs at ≤5 Hz → 0.2s per frame).
MARKER_LIFETIME_SEC = 1.5

# Ring radius and center dot size in metres.
RING_RADIUS_M = 0.15
RING_POINTS = 32
DOT_RADIUS_M = 0.04

# Z height above ground plane so markers sit on top of the costmap.
MARKER_Z_M = 0.05

# Matches the overlay's gate: the truth line disappears between trials rather
# than sitting next to a target that has already been teleported away.
TRUTH_MAX_AGE_S = 3.0

CAMERA_MEASUREMENTS_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENTS_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
HUD_DISTANCES_TOPIC = 'hud/g1_distances'

# (r, g, b, a) per estimator. The first five keep the colours they have always
# had; the mask paths take hues that stay apart from them and from each other,
# since in a clean scene all eight rings land within centimetres.
ESTIMATOR_COLOURS = {
    'rgb':                      (1.0, 0.0, 0.0, 1.0),
    'sensor_depth':             (1.0, 0.5, 0.0, 1.0),
    'depth_anything':           (0.6, 0.0, 1.0, 1.0),
    'pointcloud':               (0.0, 0.4, 1.0, 1.0),
    'lidar':                    (0.0, 0.9, 0.0, 1.0),
    'projective_ranging':       (0.0, 0.9, 0.9, 1.0),
    'euclidean_reconstruction': (1.0, 0.9, 0.1, 1.0),
    'polar_profiling':          (1.0, 0.2, 0.7, 1.0),
}

# Stable marker-id base per estimator so DELETEALL is not needed — we just overwrite.
_ESTIMATOR_ID_BASE = {name: i * 100 for i, name in enumerate(PUBLIC_ESTIMATOR_ORDER)}


# The HUD draws on a half-opaque black panel, where a ring colour's own
# luminance decides whether its line is readable. Red (0.21) and the purple of
# depth_anything (0.20) sit well under this; green and yellow are far above it.
HUD_MIN_LUMINANCE = 0.40
HUD_HEADER_COLOUR = (1.0, 1.0, 1.0)

# Rec. 709 luma weights -- the eye's actual sensitivity, so a colour is judged
# by how bright it looks rather than by its largest channel.
_LUMA_WEIGHTS = (0.2126, 0.7152, 0.0722)


def hud_text_colour(estimator: str) -> tuple[float, float, float]:
    """A ring's colour, lightened only as far as legibility needs.

    Blending toward white raises luminance while holding the hue, so a dark ring
    colour stays recognisable as the same estimator instead of being swapped for
    a brighter unrelated one. Luminance is linear in the channels, so the exact
    blend that reaches the floor is closed-form -- no iterating.
    """

    red, green, blue = ESTIMATOR_COLOURS[estimator][:3]
    luminance = sum(w * c for w, c in zip(_LUMA_WEIGHTS, (red, green, blue)))
    if luminance >= HUD_MIN_LUMINANCE:
        return red, green, blue
    blend = (HUD_MIN_LUMINANCE - luminance) / (1.0 - luminance)
    return tuple(channel + (1.0 - channel) * blend for channel in (red, green, blue))


def _hud_line(body: str, colour: tuple[float, float, float]) -> str:
    """One coloured HUD row.

    Escape before padding, never after: turning spaces into ``&nbsp;`` first
    would leave the escaper rewriting its own ampersands into ``&amp;nbsp;``.
    The padding is needed at all because rich text collapses runs of spaces,
    which is what keeps the columns lined up.
    """

    red, green, blue = (int(round(channel * 255)) for channel in colour)
    escaped = html.escape(body, quote=False).replace(' ', '&nbsp;')
    return f'<span style="color: rgb({red}, {green}, {blue})">{escaped}</span>'


def _ring_points(cx: float, cy: float, z: float, r: float, n: int) -> list:
    pts = []
    for i in range(n + 1):
        angle = 2.0 * math.pi * i / n
        p = Point()
        p.x = cx + r * math.cos(angle)
        p.y = cy + r * math.sin(angle)
        p.z = z
        pts.append(p)
    return pts


def _color_msg(r: float, g: float, b: float, a: float):
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


def estimator_reading(msg, estimator: str, index: int) -> tuple[float | None, float | None, float | None]:
    """``(forward, lateral, distance)`` for one estimator on one detection.

    Every measurement topic carries the same message type, so an estimator that
    this producer does not compute simply has an empty or NaN slot. Reading them
    all and dropping the blanks avoids hard-coding which node owns which field.
    """

    position_attrs = ESTIMATOR_POSITION_ATTRS.get(estimator)
    forward = lateral = None
    if position_attrs is not None:
        forward_attr, lateral_attr = position_attrs
        forward = _get(getattr(msg, forward_attr, None), index)
        lateral = _get(getattr(msg, lateral_attr, None), index)
    distance = _get(getattr(msg, ESTIMATOR_FIELD_KEYS[estimator], None), index)
    return forward, lateral, distance


class G1EstimateVizNode(Node):
    def __init__(self) -> None:
        super().__init__('g1_estimate_viz_node')

        namespace_name = self.get_namespace().strip('/')
        default_base_frame = (
            f'{namespace_name}/robot/base_link' if namespace_name else 'robot/base_link'
        )

        self.declare_parameter('base_frame', default_base_frame)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('marker_lifetime_sec', MARKER_LIFETIME_SEC)
        self.declare_parameter('ground_truth_topic', GROUND_TRUTH_TOPIC)
        self.declare_parameter('hud_distances_topic', HUD_DISTANCES_TOPIC)

        self.base_frame = self.get_parameter('base_frame').value or default_base_frame
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_lifetime = self.get_parameter('marker_lifetime_sec').value

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._lock = threading.Lock()
        self._latest: dict[str, G1Measurements | None] = {
            'camera': None, 'lidar': None, 'mask': None,
        }
        self._latest_truth: PointStamped | None = None
        self._latest_truth_monotonic = 0.0

        for key, topic in (
            ('camera', CAMERA_MEASUREMENTS_TOPIC),
            ('lidar', LIDAR_MEASUREMENTS_TOPIC),
            ('mask', MASK_MEASUREMENTS_TOPIC),
        ):
            self.create_subscription(
                G1Measurements, topic,
                lambda msg, key=key: self._measurement_cb(key, msg), 10)

        self.create_subscription(
            PointStamped, str(self.get_parameter('ground_truth_topic').value),
            self._truth_cb, 10)

        self._pub = self.create_publisher(MarkerArray, 'visualization/g1/estimates', 10)
        # The HUD aggregator owns the container style; this node contributes one
        # labelled section and nothing else.
        self._hud_pub = self.create_publisher(
            OverlayText, str(self.get_parameter('hud_distances_topic').value), 10)

    def _measurement_cb(self, key: str, msg: G1Measurements) -> None:
        with self._lock:
            self._latest[key] = msg
        self._publish()

    def _truth_cb(self, msg: PointStamped) -> None:
        with self._lock:
            self._latest_truth = msg
            self._latest_truth_monotonic = time.monotonic()

    def _current_truth(self) -> float | None:
        """Benchmark truth distance while it is live, else ``None``.

        Packed by the runner as x=lateral, y=forward, z=distance.
        """

        with self._lock:
            msg, seen_at = self._latest_truth, self._latest_truth_monotonic
        if msg is None or time.monotonic() - seen_at > TRUTH_MAX_AGE_S:
            return None
        return float(msg.point.z)

    def _fresh_messages(self) -> list[G1Measurements]:
        """Cached messages still within the marker lifetime, oldest cache dropped."""

        now = self.get_clock().now()
        stale_threshold = rclpy.duration.Duration(seconds=self.marker_lifetime)
        with self._lock:
            cached = dict(self._latest)

        fresh = []
        for msg in cached.values():
            if msg is None:
                continue
            if now - rclpy.time.Time.from_msg(msg.header.stamp) > stale_threshold:
                continue
            fresh.append(msg)
        return fresh

    def _publish(self) -> None:
        fresh = self._fresh_messages()
        if not fresh:
            return

        self._publish_hud(fresh)

        # Use Time(0) to get the latest available TF — avoids sim-time buffer
        # mismatches since the robot is stationary during benchmarks.
        robot_x, robot_y, robot_yaw, actual_frame = self._robot_pose_in_world(
            rclpy.time.Time()
        )
        if robot_x is None:
            return

        markers = []
        marker_time = self.get_clock().now().to_msg()
        for msg in fresh:
            if not msg.detected:
                continue
            for index in range(msg.count):
                for estimator in PUBLIC_ESTIMATOR_ORDER:
                    forward, lateral, distance = estimator_reading(msg, estimator, index)
                    self._add_estimator_markers(
                        markers, estimator, index, forward, lateral,
                        robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                        distance_m=distance,
                    )

        if not markers:
            return

        ma = MarkerArray()
        ma.markers = markers
        self._pub.publish(ma)

    def _publish_hud(self, fresh: list[G1Measurements]) -> None:
        """One HUD section: every estimator's distance against the truth.

        Rendered by RViz as text, so it stays legible at any window size — unlike
        numbers burned into the perception overlay bitmap, which shrink with it.

        Each line is coloured from ``ESTIMATOR_COLOURS``, the same table that
        colours that estimator's ring, so a reading and its ring cannot drift
        apart. That markup makes the overlay rich text, hence ``&nbsp;`` padding
        and ``<br/>`` breaks -- see ``hud_node``'s ``rich_text``.
        """

        truth = self._current_truth()
        header = 'G1 DISTANCES' + (f'   truth {truth:.3f} m' if truth is not None else '')
        lines = [_hud_line(header, HUD_HEADER_COLOUR)]

        readings: dict[str, float] = {}
        for msg in fresh:
            if not msg.detected or msg.count < 1:
                continue
            for estimator in PUBLIC_ESTIMATOR_ORDER:
                # First detection only: the HUD is a scalar readout, and the
                # rings already carry the per-instance picture.
                _, _, distance = estimator_reading(msg, estimator, 0)
                if distance is not None:
                    readings[estimator] = distance

        for estimator in PUBLIC_ESTIMATOR_ORDER:
            label = ESTIMATOR_LABELS[estimator]
            distance = readings.get(estimator)
            if distance is None:
                body = f'{label:<24}      --    miss'
            else:
                error = '' if truth is None else f'  {distance - truth:+.3f}'
                body = f'{label:<24} {distance:7.3f}{error}'
            lines.append(_hud_line(body, hud_text_colour(estimator)))

        text = OverlayText()
        text.text = '<br/>'.join(lines)
        self._hud_pub.publish(text)

    def _add_estimator_markers(
        self,
        markers: list,
        estimator: str,
        detection_index: int,
        forward_m: float | None,
        lateral_m: float | None,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        stamp,
        frame_id: str,
        distance_m: float | None = None,
    ) -> None:
        # For depth-only estimators (sensor_depth, depth_anything) we only have a scalar
        # distance along the camera boresight — treat lateral as 0.
        if forward_m is None and distance_m is not None:
            forward_m = distance_m
            lateral_m = 0.0
        if forward_m is None or lateral_m is None:
            return

        # Rotate base-frame (forward, lateral) into world frame.
        wx = robot_x + math.cos(robot_yaw) * forward_m - math.sin(robot_yaw) * lateral_m
        wy = robot_y + math.sin(robot_yaw) * forward_m + math.cos(robot_yaw) * lateral_m

        colour = ESTIMATOR_COLOURS[estimator]
        id_base = _ESTIMATOR_ID_BASE[estimator] + detection_index * 2
        lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()

        # Ring marker (LINE_STRIP circle).
        ring = Marker()
        ring.header.frame_id = frame_id
        ring.header.stamp = stamp
        ring.ns = f'g1_estimates/{estimator}'
        ring.id = id_base
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.06   # line width
        ring.color = _color_msg(*colour)
        ring.lifetime = lifetime
        ring.points = _ring_points(wx, wy, MARKER_Z_M, RING_RADIUS_M, RING_POINTS)
        markers.append(ring)

        # Centre dot (SPHERE).
        dot = Marker()
        dot.header.frame_id = frame_id
        dot.header.stamp = stamp
        dot.ns = f'g1_estimates/{estimator}'
        dot.id = id_base + 1
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x = wx
        dot.pose.position.y = wy
        dot.pose.position.z = MARKER_Z_M
        dot.pose.orientation.w = 1.0
        dot.scale.x = DOT_RADIUS_M * 2
        dot.scale.y = DOT_RADIUS_M * 2
        dot.scale.z = DOT_RADIUS_M * 2
        dot.color = _color_msg(*colour)
        dot.lifetime = lifetime
        markers.append(dot)

    def _robot_pose_in_world(self, stamp: rclpy.time.Time) -> tuple[float | None, float | None, float | None, str | None]:
        base_frames = [self.base_frame]
        if self.base_frame != 'base_link':
            base_frames.append('base_link')
        for world_frame in (self.world_frame, 'odom'):
            for base_frame in base_frames:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        world_frame, base_frame, stamp,
                        timeout=rclpy.duration.Duration(seconds=0.2),
                    )
                    tx = tf.transform.translation.x
                    ty = tf.transform.translation.y
                    q = tf.transform.rotation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                    if world_frame != self.world_frame:
                        self.get_logger().warn(
                            f'"{self.world_frame}" frame unavailable; '
                            f'publishing estimate markers in "{world_frame}" frame.',
                            throttle_duration_sec=10.0,
                        )
                    return tx, ty, yaw, world_frame
                except TransformException:
                    continue
        return None, None, None, None


def _get(arr, i: int) -> float | None:
    if arr is not None and len(arr) > i and arr[i] == arr[i]:  # NaN check
        return float(arr[i])
    return None


def main() -> None:
    rclpy.init()
    node = G1EstimateVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
