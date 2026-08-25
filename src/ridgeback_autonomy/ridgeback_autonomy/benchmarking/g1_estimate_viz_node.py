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
    display_bearing,
    nearest_instance_index,
    truth_reading,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.geometry import remove_vehicle_front_offset


# Seconds before a marker auto-expires if no new detection arrives.
# Set to 1.5x the detector's max publish interval (detector runs at ≤5 Hz → 0.2s per frame).
MARKER_LIFETIME_SEC = 1.5

# Ring radius and center dot size in metres.
RING_RADIUS_M = 0.15
RING_POINTS = 32
DOT_RADIUS_M = 0.04

# Ring line width. The thin one marks a ring whose *direction* is not its own
# estimator's answer: the depth-only rows publish a distance and no position, so
# their bearing is borrowed (see ``_add_estimator_markers``). Their radius is
# still their own number -- only the direction is second-hand, and the weight
# difference is what keeps that distinction readable on the floor plan.
RING_LINE_WIDTH_M = 0.06
BORROWED_BEARING_LINE_WIDTH_M = 0.03

# Z height above ground plane so markers sit on top of the costmap.
MARKER_Z_M = 0.05

CAMERA_MEASUREMENTS_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENTS_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
HUD_DISTANCES_TOPIC = 'hud/g1_distances'

# (r, g, b, a) per estimator, split by colour temperature: the five legacy rows
# are warm (rose through chartreuse), the three mask rows cold (cyan through
# violet). Nothing on the HUD or in RViz names the two families, so the
# temperature is the only thing that groups them -- which is why no hue crosses
# over, however much room that would buy inside a family. Within a family the
# hues are still spread as far as five (or three) allow, since in a clean scene
# all eight rings land within centimetres of each other.
ESTIMATOR_COLOURS = {
    'rgb':                      (1.0, 0.0, 0.0, 1.0),
    'sensor_depth':             (1.0, 0.45, 0.0, 1.0),
    'depth_anything':           (1.0, 0.1, 0.55, 1.0),
    'pointcloud':               (1.0, 0.85, 0.0, 1.0),
    'lidar':                    (0.85, 1.0, 0.15, 1.0),
    'projective_ranging':       (0.0, 0.85, 0.85, 1.0),
    'euclidean_reconstruction': (0.2, 0.55, 1.0, 1.0),
    'polar_profiling':          (0.7, 0.4, 1.0, 1.0),
}

# Stable marker-id base per estimator so DELETEALL is not needed — we just
# overwrite. Only the nearest instance is drawn, so an estimator needs a fixed
# pair of ids (ring, dot) rather than a pair per detection: an id that moved
# with the detection index would strand the previous instance's ring on screen
# for a full lifetime whenever the nearest one changes.
_ESTIMATOR_ID_BASE = {name: i * 100 for i, name in enumerate(PUBLIC_ESTIMATOR_ORDER)}


# The HUD draws on a half-opaque black panel, where a ring colour's own
# luminance decides whether its line is readable. Red (0.21) and the rose of
# depth_anything (0.32) sit under this; every cold row clears it unlifted (0.51
# at the darkest), so the lift never touches the mask family.
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


def hud_truth_header(truth) -> str:
    """The HUD's first line, naming the trial its truth belongs to.

    The trial id is what makes a mismatch legible. A bare number cannot be
    checked against the scene on screen: a truth left over from an earlier
    trial reads as the estimators disagreeing with the target rather than as
    the wrong target, which is exactly how it went unnoticed.
    """

    if truth is None:
        return 'G1 DISTANCES'
    header = f'G1 DISTANCES   truth {truth.distance_m:.3f} m'
    return f'{header}  [{truth.trial_id}]' if truth.trial_id else header


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


def merged_distance_reader(fresh: list):
    """``(estimator, index) -> distance`` across every producer in one frame.

    Each topic fills only the estimators its node computes, so a reading has to
    be looked up in all of them. The indices line up because all three producers
    number their detections from the same detector batch.
    """

    def read_distance(estimator: str, index: int) -> float | None:
        for msg in fresh:
            if not msg.detected or index >= msg.count:
                continue
            _, _, distance = estimator_reading(msg, estimator, index)
            if distance is not None:
                return distance
        return None

    return read_distance


def merged_position_reader(fresh: list):
    """``(estimator, index) -> (forward, lateral) | None`` across every producer.

    The positional counterpart to ``merged_distance_reader``, feeding
    ``display_bearing`` so the depth-only rows can be pointed somewhere. Same
    reason for scanning every message: an estimator's fields are filled by
    exactly one producer and are empty in the other two.
    """

    def read_position(estimator: str, index: int):
        for msg in fresh:
            if not msg.detected or index >= msg.count:
                continue
            forward, lateral, _ = estimator_reading(msg, estimator, index)
            if forward is not None and lateral is not None:
                return forward, lateral
        return None

    return read_position


def world_marker_point(
    forward_m: float,
    lateral_m: float,
    origin_x: float,
    origin_y: float,
    yaw_rad: float,
) -> tuple[float, float]:
    """A base-frame ``(forward, lateral)`` placed in the world.

    ``origin_x`` / ``origin_y`` / ``yaw_rad`` are the base pose TF reports, so
    the measurement passed in must already be referenced to the base origin --
    callers undo the front offset first. Module-level and pure so the placement
    can be asserted against a known world point without standing up a node,
    which is what let the offset bug survive.
    """

    world_x = origin_x + math.cos(yaw_rad) * forward_m - math.sin(yaw_rad) * lateral_m
    world_y = origin_y + math.sin(yaw_rad) * forward_m + math.cos(yaw_rad) * lateral_m
    return world_x, world_y


def nearest_detection_index(fresh: list) -> int:
    """The detection every surface in this node speaks for.

    Falls back to 0 when no estimator placed anything: the frame then renders as
    an all-miss HUD with no rings, which is what it did before there was a
    nearest-instance rule at all.
    """

    count = max((msg.count for msg in fresh if msg.detected), default=0)
    nearest = nearest_instance_index(count, merged_distance_reader(fresh))
    return 0 if nearest is None else nearest


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

        for key, topic in (
            ('camera', CAMERA_MEASUREMENTS_TOPIC),
            ('lidar', LIDAR_MEASUREMENTS_TOPIC),
            ('mask', MASK_MEASUREMENTS_TOPIC),
        ):
            self.create_subscription(
                G1Measurements, topic,
                lambda msg, key=key: self._measurement_cb(key, msg), 10)

        # Depth 1: the truth line is a latest-value-wins signal, so a deeper
        # queue only buys lag. This node's callbacks share one executor thread
        # with three measurement topics that build markers and hit TF, and a
        # backlog here pins a previous trial's truth under the current scene.
        self.create_subscription(
            PointStamped, str(self.get_parameter('ground_truth_topic').value),
            self._truth_cb, 1)

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

    def _current_truth(self):
        """The live benchmark truth as a ``TruthReading``, else ``None``.

        Expiry is judged on the message stamp against this node's clock, so a
        message that waited in the queue is old data rather than fresh -- see
        ``truth_reading``.
        """

        with self._lock:
            msg = self._latest_truth
        return truth_reading(msg, self.get_clock().now().nanoseconds)

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

        # Rings, HUD and the mask node's rays all show a single instance, so the
        # index is resolved once per frame and shared by both surfaces here.
        nearest = nearest_detection_index(fresh)
        self._publish_hud(fresh, nearest)

        # Use Time(0) to get the latest available TF — avoids sim-time buffer
        # mismatches since the robot is stationary during benchmarks.
        robot_x, robot_y, robot_yaw, actual_frame = self._robot_pose_in_world(
            rclpy.time.Time()
        )
        if robot_x is None:
            return

        # Resolved once per frame rather than per estimator: every depth-only
        # ring on this detection must point the same way, or two rows that
        # measured the same robot would appear to disagree about where it is.
        bearing = display_bearing(merged_position_reader(fresh), nearest)

        markers = []
        marker_time = self.get_clock().now().to_msg()
        for msg in fresh:
            if not msg.detected or nearest >= msg.count:
                continue
            for estimator in PUBLIC_ESTIMATOR_ORDER:
                forward, lateral, distance = estimator_reading(msg, estimator, nearest)
                self._add_estimator_markers(
                    markers, estimator, forward, lateral,
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                    distance_m=distance, bearing_rad=bearing,
                )

        if not markers:
            return

        ma = MarkerArray()
        ma.markers = markers
        self._pub.publish(ma)

    def _publish_hud(self, fresh: list[G1Measurements], nearest: int) -> None:
        """One HUD section: every estimator's distance against the truth.

        Rendered by RViz as text, so it stays legible at any window size — unlike
        numbers burned into the perception overlay bitmap, which shrink with it.

        Each line is coloured from ``ESTIMATOR_COLOURS``, the same table that
        colours that estimator's ring, so a reading and its ring cannot drift
        apart. That markup makes the overlay rich text, hence ``&nbsp;`` padding
        and ``<br/>`` breaks -- see ``hud_node``'s ``rich_text``.
        """

        truth = self._current_truth()
        lines = [_hud_line(hud_truth_header(truth), HUD_HEADER_COLOUR)]

        readings: dict[str, float] = {}
        for msg in fresh:
            if not msg.detected or nearest >= msg.count:
                continue
            for estimator in PUBLIC_ESTIMATOR_ORDER:
                # The HUD is a scalar readout, so it quotes the same nearest
                # instance the rings are drawn on rather than a second one.
                _, _, distance = estimator_reading(msg, estimator, nearest)
                if distance is not None:
                    readings[estimator] = distance

        for estimator in PUBLIC_ESTIMATOR_ORDER:
            label = ESTIMATOR_LABELS[estimator]
            distance = readings.get(estimator)
            if distance is None:
                body = f'{label:<24}      --    miss'
            else:
                error = '' if truth is None else f'  {distance - truth.distance_m:+.3f}'
                body = f'{label:<24} {distance:7.3f}{error}'
            lines.append(_hud_line(body, hud_text_colour(estimator)))

        text = OverlayText()
        text.text = '<br/>'.join(lines)
        self._hud_pub.publish(text)

    def _add_estimator_markers(
        self,
        markers: list,
        estimator: str,
        forward_m: float | None,
        lateral_m: float | None,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        stamp,
        frame_id: str,
        distance_m: float | None = None,
        bearing_rad: float | None = None,
    ) -> None:
        # The depth-only rows (sensor_depth, depth_anything) publish a planar
        # distance and no position, so a direction has to come from somewhere.
        # Borrowing the bearing another row measured on this same detection puts
        # the ring on the right robot; the old fallback of pointing it down the
        # boresight was wrong by the whole lateral component, which for an
        # off-axis target is metres. The radius stays the row's own number, so
        # the ring still disagrees with its neighbours exactly as much as the
        # HUD says it does. No bearing at all (nothing else placed this
        # detection) keeps the boresight guess rather than dropping the ring.
        borrowed_bearing = ESTIMATOR_POSITION_ATTRS.get(estimator) is None
        if forward_m is None and distance_m is not None:
            if bearing_rad is None:
                forward_m, lateral_m = distance_m, 0.0
            else:
                forward_m = distance_m * math.cos(bearing_rad)
                lateral_m = distance_m * math.sin(bearing_rad)
        if forward_m is None or lateral_m is None:
            return

        # Every estimator reports off the robot FRONT, but TF hands back the base
        # origin, so the offset goes back on before the measurement is rotated
        # out. Without it the ring lands a fixed 0.25 m nearer the robot than the
        # distance it is drawing -- small enough to read as sensor error rather
        # than as a fault in the plotting.
        lateral_m, forward_m = remove_vehicle_front_offset(lateral_m, forward_m)
        wx, wy = world_marker_point(forward_m, lateral_m, robot_x, robot_y, robot_yaw)

        colour = ESTIMATOR_COLOURS[estimator]
        id_base = _ESTIMATOR_ID_BASE[estimator]
        lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()

        # Ring marker (LINE_STRIP circle).
        ring = Marker()
        ring.header.frame_id = frame_id
        ring.header.stamp = stamp
        ring.ns = f'g1_estimates/{estimator}'
        ring.id = id_base
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = (
            BORROWED_BEARING_LINE_WIDTH_M if borrowed_bearing else RING_LINE_WIDTH_M
        )
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
