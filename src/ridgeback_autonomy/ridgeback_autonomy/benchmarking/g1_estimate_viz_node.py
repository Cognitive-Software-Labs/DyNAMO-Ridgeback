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
    nearest_instance_index,
    parse_estimators,
    truth_reading,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.vehicle_frame import remove_vehicle_front_offset


# How long silence is tolerated, in the two places that have to agree on it: a
# marker carries this as its RViz ``lifetime``, so a ring vanishes this long
# after the message that drew it, and the same budget decides whether a cached
# message still counts as a running producer. Set by how long a ring may linger
# before it reads as a live claim about a robot that is no longer being
# measured -- not by the detector's rate, which the comment here used to derive
# it from with a formula that yielded 0.3 s and never matched the value.
MARKER_LIFETIME_SEC = 1.5

# How old an observation may be and still describe the scene on screen. A
# separate question from the budget above, and the reason the mask rows used to
# blink: a mask measurement is stamped with the DETECTION instant and only
# reaches this node after inference, segmentation and a depth lookup, so it
# arrives already a second or more old. That is the pipeline's latency, not
# staleness, and judging it against the silence budget threw away readings from
# an estimator that was working. Same order as ``TRUTH_MAX_AGE_S`` because it
# answers the same question -- could this still be the trial on screen.
MAX_OBSERVATION_AGE_S = 3.0

# How often the HUD section is rebuilt. It needs a clock of its own rather than
# riding the measurement callbacks, which cannot fire when the thing to be shown
# is that nothing is arriving: expiring rows, expiring the truth line and
# ticking the age column all have to happen with no traffic at all. Matches
# ``hud_node``'s own sampler rate, above which the extra renders are discarded.
HUD_PUBLISH_RATE_HZ = 5.0

# Ring radius and center dot size in metres.
RING_RADIUS_M = 0.15
RING_POINTS = 32
DOT_RADIUS_M = 0.04

RING_LINE_WIDTH_M = 0.06

# Z height above ground plane so markers sit on top of the costmap.
MARKER_Z_M = 0.05

POINTCLOUD_MEASUREMENTS_TOPIC = 'measurements/g1/pointcloud'
MASK_MEASUREMENTS_TOPIC = 'measurements/g1/mask'
HUD_DISTANCES_TOPIC = 'hud/g1_distances'

# (r, g, b, a) per estimator, split by colour temperature: the pointcloud row is
# warm, the three mask rows cold (cyan through violet). Nothing on the HUD or in
# RViz names the two families, so the temperature is the only thing that groups
# them -- which is why no hue crosses over, however much room that would buy
# inside a family. Within the mask family the hues are still spread as far as
# three allow, since in a clean scene all four rings land within centimetres of
# each other.
ESTIMATOR_COLOURS = {
    'pointcloud':               (1.0, 0.85, 0.0, 1.0),
    'projective_ranging':       (0.0, 0.85, 0.85, 1.0),
    'euclidean_reconstruction': (0.2, 0.55, 1.0, 1.0),
    'polar_profiling':          (0.7, 0.4, 1.0, 1.0),
}

# Stable marker-id base per estimator so DELETEALL is not needed — we just
# overwrite. Only the nearest instance is drawn, so an estimator needs a fixed
# pair of ids (ring, dot) rather than a pair per detection: an id that moved
# with the detection index would strand the previous instance's ring on screen
# for a full lifetime whenever the nearest one changes. It enumerates the whole
# registry for the same reason and not the run's selected set — ids that shifted
# with the selection would let a leftover marker collide with a different
# estimator's ring.
_ESTIMATOR_ID_BASE = {name: i * 100 for i, name in enumerate(PUBLIC_ESTIMATOR_ORDER)}


# The HUD draws on a half-opaque black panel, where a ring colour's own
# luminance decides whether its line is readable. Every currently registered
# colour clears this unlifted (0.51 at the darkest), so the lift is a floor for
# a colour added later rather than something any row hits today.
HUD_MIN_LUMINANCE = 0.40
HUD_HEADER_COLOUR = (1.0, 1.0, 1.0)

# The one luminance every aged row is driven to, up or down. A dim multiplier
# would not do: a colour sitting at the legibility floor above would be scaled
# down, clamped back to that floor, and handed straight back its fresh
# appearance. Driving every aged row to a single figure also keeps the
# fresh/aged step the same size between two adjacent rows of different hue,
# which is what makes it readable as a state rather than as one row happening
# to be darker.
#
# It sits below the documented legibility floor deliberately -- an aged row must
# read as secondary -- so this value is settled on screen, not from the number.
HUD_AGED_LUMINANCE = 0.28

# Rec. 709 luma weights -- the eye's actual sensitivity, so a colour is judged
# by how bright it looks rather than by its largest channel.
_LUMA_WEIGHTS = (0.2126, 0.7152, 0.0722)


def _luminance(colour: tuple[float, float, float]) -> float:
    return sum(weight * channel for weight, channel in zip(_LUMA_WEIGHTS, colour))


def colour_at_luminance(
    colour: tuple[float, float, float], target: float
) -> tuple[float, float, float]:
    """``colour`` moved to ``target`` luminance, as close to its hue as it can be.

    Two rungs, because the two directions have different best answers. Going up,
    there is nowhere to go but toward white, and luminance is linear in the
    channels so the exact blend that lands on the target is closed-form -- hue
    survives, saturation necessarily does not. Going down, scaling all three
    channels by one factor holds their ratios exactly, so hue *and* saturation
    come through untouched.

    Hue fidelity is what ties a row to its estimator, and it matters more the
    dimmer the row: an aged row has no ring on the floor plan to be matched
    against, so its colour is the only thing left identifying it.
    """

    luminance = _luminance(colour)
    if luminance == target:
        return colour
    if luminance > target:
        scale = target / luminance
        return tuple(channel * scale for channel in colour)
    blend = (target - luminance) / (1.0 - luminance)
    return tuple(channel + (1.0 - channel) * blend for channel in colour)


def hud_text_colour(estimator: str, aged: bool = False) -> tuple[float, float, float]:
    """A ring's colour as HUD text, at the luminance that row's state calls for.

    A fresh row is lightened only as far as legibility needs and otherwise left
    exactly alone, so the text and the ring it belongs to do not drift apart for
    no reason. An aged row is driven to ``HUD_AGED_LUMINANCE`` whichever side it
    started on -- see that constant for why it is a target rather than a dimming.
    """

    colour = ESTIMATOR_COLOURS[estimator][:3]
    if aged:
        return colour_at_luminance(colour, HUD_AGED_LUMINANCE)
    return colour_at_luminance(colour, max(_luminance(colour), HUD_MIN_LUMINANCE))


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


def merged_distance_reader(batch: list):
    """``(estimator, index) -> distance`` across every producer in one frame.

    Each topic fills only the estimators its node computes, so a reading has to
    be looked up in all of them. The indices line up because both producers
    number their detections from the same detector batch -- which is a premise,
    not a guarantee, and the reason ``partition_measurements`` hands this only
    the messages that share a stamp.
    """

    def read_distance(estimator: str, index: int) -> float | None:
        for msg in batch:
            if not msg.detected or index >= msg.count:
                continue
            _, _, distance = estimator_reading(msg, estimator, index)
            if distance is not None:
                return distance
        return None

    return read_distance


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


def nearest_detection_index(batch: list) -> int:
    """The detection every surface in this node speaks for.

    Falls back to 0 when no estimator placed anything: the frame then renders as
    an all-miss HUD with no rings, which is what it did before there was a
    nearest-instance rule at all.
    """

    count = max((msg.count for msg in batch if msg.detected), default=0)
    nearest = nearest_instance_index(count, merged_distance_reader(batch))
    return 0 if nearest is None else nearest


def stamp_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def partition_measurements(
    cached,
    now_nanoseconds: int,
    liveness_budget_s: float,
    max_observation_age_s: float,
) -> tuple[list, list]:
    """Cached messages split into ``(same_batch, aged)``, the rest discarded.

    Two gates, because liveness and validity are different questions and the
    single stamp gate this replaces was answering neither for the mask rows:

    * **liveness**, on the RECEIPT time -- has this producer gone quiet? A mask
      reading genuinely *is* a second or more old by the time it arrives, and
      charging its own pipeline latency against a silence budget is what made
      the mask rows blink at the detector's rate. Receipt time is the right
      clock here precisely because these rows are not trial-keyed; the truth
      line is, which is why ``truth_reading`` keeps its stamp-only rule -- no
      receipt gate can tell trial N-1's truth from trial N's.
    * **validity**, on the STAMP -- can this observation still describe the
      scene? This is what retains the cross-trial protection.

    Survivors then split on whether they carry the newest stamp present.
    ``merged_distance_reader`` and everything downstream of it assume the
    indices in one message name the same detections as the indices in another,
    which holds only within a detector batch: an older mask message's index 0
    may be a different robot. So the batch drives the rings and the ranking, and
    the aged remainder -- returned as ``(message, age_seconds)`` -- is only ever
    allowed to print numbers.
    """

    live = []
    for entry in cached:
        if entry is None:
            continue
        msg, receipt_nanoseconds = entry
        if now_nanoseconds - receipt_nanoseconds > liveness_budget_s * 1_000_000_000:
            continue
        stamp = stamp_nanoseconds(msg.header.stamp)
        if now_nanoseconds - stamp > max_observation_age_s * 1_000_000_000:
            continue
        live.append((msg, stamp))

    if not live:
        return [], []

    newest = max(stamp for _, stamp in live)
    same_batch = [msg for msg, stamp in live if stamp == newest]
    aged = [
        (msg, (now_nanoseconds - stamp) / 1_000_000_000)
        for msg, stamp in live if stamp != newest
    ]
    return same_batch, aged


def _reading_body(label: str, distance_m: float, truth) -> str:
    error = '' if truth is None else f'  {distance_m - truth.distance_m:+.3f}'
    return f'{label:<24} {distance_m:7.3f}{error}'


def hud_rows(
    same_batch: list,
    aged: list,
    nearest: int,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> list[str]:
    """One coloured row per estimator this run selected, in three states.

    A fresh reading renders as it always has. An aged one renders its value and
    its error alongside an age column, dimmed to ``HUD_AGED_LUMINANCE`` -- the
    number is real and the panel says how old it is, rather than the row being
    dropped and repainted as ``-- miss`` by whichever producer answered in the
    meantime. Only a row nothing reported is a miss, which is what that word was
    supposed to mean -- and which is why ``estimators`` is the run's set rather
    than the registry: a row for a path the run never launched can only ever
    print that same word, about an estimator that was never asked.

    Defaults to every registered estimator, so a caller with no run
    configuration to hand (the exploration entrypoint, and the tests) keeps the
    full panel. ``parse_estimators`` returns its subset already in
    ``PUBLIC_ESTIMATOR_ORDER`` sequence, so the row order is the canonical one
    however the launch argument happened to name them.

    An aged message is ranked against itself rather than against ``nearest``:
    it comes from a different detector batch, so the batch's index would be a
    guess about which robot it names, and the message's own three estimators
    rank exactly the instance its producer was speaking for.
    """

    fresh_readings: dict[str, float] = {}
    for msg in same_batch:
        if not msg.detected or nearest >= msg.count:
            continue
        for estimator in estimators:
            # The HUD is a scalar readout, so it quotes the same nearest
            # instance the rings are drawn on rather than a second one.
            _, _, distance = estimator_reading(msg, estimator, nearest)
            if distance is not None:
                fresh_readings[estimator] = distance

    aged_readings: dict[str, tuple[float, float]] = {}
    for msg, age_s in aged:
        index = nearest_detection_index([msg])
        if not msg.detected or index >= msg.count:
            continue
        for estimator in estimators:
            if estimator in fresh_readings:
                continue
            _, _, distance = estimator_reading(msg, estimator, index)
            if distance is not None:
                aged_readings[estimator] = (distance, age_s)

    rows = []
    for estimator in estimators:
        label = ESTIMATOR_LABELS[estimator]
        if estimator in fresh_readings:
            body = _reading_body(label, fresh_readings[estimator], truth)
            colour = hud_text_colour(estimator)
        elif estimator in aged_readings:
            distance, age_s = aged_readings[estimator]
            body = f'{_reading_body(label, distance, truth)}   {age_s:.1f}s'
            colour = hud_text_colour(estimator, aged=True)
        else:
            body = f'{label:<24}      --    miss'
            colour = hud_text_colour(estimator)
        rows.append(_hud_line(body, colour))
    return rows


def hud_section_text(
    same_batch: list,
    aged: list,
    nearest: int,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> str:
    """The whole HUD section, or ``''`` when there is nothing to say.

    Rendered by RViz as text, so it stays legible at any window size -- unlike
    numbers burned into the perception overlay bitmap, which shrink with it.

    Each line is coloured from ``ESTIMATOR_COLOURS``, the same table that colours
    that estimator's ring, so a reading and its ring cannot drift apart. That
    markup makes the overlay rich text, hence ``&nbsp;`` padding and ``<br/>``
    breaks -- see ``hud_node``'s ``rich_text``.

    The empty string is load-bearing: ``hud_node`` drops falsy panels, so it
    collapses the section instead of holding the last text it was ever sent. A
    panel of numbers left standing under a floor plan whose rings have all
    correctly expired reads as a broken ring layer rather than a dead detector.
    """

    if not same_batch and not aged:
        return ''
    lines = [_hud_line(hud_truth_header(truth), HUD_HEADER_COLOUR)]
    lines.extend(hud_rows(same_batch, aged, nearest, truth, estimators))
    return '<br/>'.join(lines)


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
        self.declare_parameter('max_observation_age_sec', MAX_OBSERVATION_AGE_S)
        self.declare_parameter('hud_publish_rate_hz', HUD_PUBLISH_RATE_HZ)
        self.declare_parameter('ground_truth_topic', GROUND_TRUTH_TOPIC)
        self.declare_parameter('hud_distances_topic', HUD_DISTANCES_TOPIC)
        # Which estimators this run selected, comma-separated, as the launch
        # file resolved it. "all" (the default) is every registered row, which
        # is what the exploration entrypoint gets -- it has no such argument.
        self.declare_parameter('estimators', 'all')

        self.base_frame = self.get_parameter('base_frame').value or default_base_frame
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_lifetime = self.get_parameter('marker_lifetime_sec').value
        self.max_observation_age = float(
            self.get_parameter('max_observation_age_sec').value)
        self.estimators = parse_estimators(str(self.get_parameter('estimators').value))

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._lock = threading.Lock()
        # Each slot holds ``(message, receipt_nanoseconds)``: the stamp answers
        # "how old is this observation", the receipt answers "is this producer
        # still running", and for the mask topic those two are a second or more
        # apart. See ``partition_measurements``.
        self._latest: dict[str, tuple[G1Measurements, int] | None] = {
            'pointcloud': None, 'mask': None,
        }
        self._latest_truth: PointStamped | None = None

        for key, topic in (
            ('pointcloud', POINTCLOUD_MEASUREMENTS_TOPIC),
            ('mask', MASK_MEASUREMENTS_TOPIC),
        ):
            self.create_subscription(
                G1Measurements, topic,
                lambda msg, key=key: self._measurement_cb(key, msg), 10)

        # Depth 1: the truth line is a latest-value-wins signal, so a deeper
        # queue only buys lag. This node's callbacks share one executor thread
        # with the measurement topics that build markers and hit TF, and a
        # backlog here pins a previous trial's truth under the current scene.
        self.create_subscription(
            PointStamped, str(self.get_parameter('ground_truth_topic').value),
            self._truth_cb, 1)

        self._pub = self.create_publisher(MarkerArray, 'visualization/g1/estimates', 10)
        # The HUD aggregator owns the container style; this node contributes one
        # labelled section and nothing else.
        self._hud_pub = self.create_publisher(
            OverlayText, str(self.get_parameter('hud_distances_topic').value), 10)

        # The HUD renders on a clock, the markers stay event-driven. Markers
        # carry a ``lifetime``, so republishing unchanged data against a fresh
        # stamp would renew that lifetime forever and no ring could ever expire;
        # the HUD has the opposite problem and cannot expire anything without a
        # tick of its own. ``create_timer`` runs on the node clock, so under
        # ``use_sim_time`` a paused sim freezes the panel and the ages together
        # rather than ticking ages forward against a world that is not moving.
        hud_rate = float(self.get_parameter('hud_publish_rate_hz').value)
        self.create_timer(
            1.0 / (hud_rate if hud_rate > 0.0 else HUD_PUBLISH_RATE_HZ),
            self._render_hud)

    def _measurement_cb(self, key: str, msg: G1Measurements) -> None:
        with self._lock:
            self._latest[key] = (msg, self.get_clock().now().nanoseconds)
        self._publish_markers()

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

    def _partitioned_messages(self) -> tuple[list, list]:
        """The cache split into the current batch and the aged remainder."""

        now_nanoseconds = self.get_clock().now().nanoseconds
        with self._lock:
            cached = list(self._latest.values())
        return partition_measurements(
            cached, now_nanoseconds, self.marker_lifetime, self.max_observation_age)

    def _publish_markers(self) -> None:
        """Rings and dots for the current detector batch.

        Aged messages are excluded on purpose. A stale number is a claim about
        the past that the HUD can label as such; a stale ring is a claim about
        where a robot is *now*, with nothing on the floor plan able to say
        otherwise, so it is simply not drawn.

        Only the run's estimators are drawn, the same set the panel lists, so
        the two surfaces agree by construction rather than by both happening to
        enumerate the registry. Producers are normally already filtered -- the
        mask node resolves its own ``enabled_estimators`` -- but a field filled
        for an estimator this run did not select would otherwise still get a
        ring with no row beside it to name it.
        """

        same_batch, _ = self._partitioned_messages()
        if not same_batch:
            return

        # Rings, HUD and the mask node's rays all show a single instance, so the
        # index is resolved once per frame and shared by both surfaces here.
        nearest = nearest_detection_index(same_batch)

        # Use Time(0) to get the latest available TF — avoids sim-time buffer
        # mismatches since the robot is stationary during benchmarks.
        robot_x, robot_y, robot_yaw, actual_frame = self._robot_pose_in_world(
            rclpy.time.Time()
        )
        if robot_x is None:
            return

        markers = []
        marker_time = self.get_clock().now().to_msg()
        for msg in same_batch:
            if not msg.detected or nearest >= msg.count:
                continue
            for estimator in self.estimators:
                forward, lateral, _ = estimator_reading(msg, estimator, nearest)
                self._add_estimator_markers(
                    markers, estimator, forward, lateral,
                    robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                )

        if not markers:
            return

        ma = MarkerArray()
        ma.markers = markers
        self._pub.publish(ma)

    def _render_hud(self) -> None:
        """Publish the HUD section, unconditionally, on every tick.

        Unconditionally is the point. Returning early on an empty cache is what
        left the panel latched: ``hud_node`` only ever overwrites its cached
        text, so a section that stops publishing keeps showing its last numbers
        for as long as RViz is open. Publishing ``''`` collapses it instead.

        It is also the only thing that re-evaluates the truth gate. ``truth_reading``
        expires on ``TRUTH_MAX_AGE_S`` so the line disappears between trials, but
        reachable only from a measurement callback it was a gate with nothing to
        pull it -- when detections stopped it never fired again, and the previous
        trial's truth sat under a target that had already been teleported away.
        """

        same_batch, aged = self._partitioned_messages()
        nearest = nearest_detection_index(same_batch) if same_batch else 0
        text = OverlayText()
        text.text = hud_section_text(
            same_batch, aged, nearest, self._current_truth(), self.estimators)
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
    ) -> None:
        # Every registered estimator places its own detection
        # (``ESTIMATOR_POSITION_ATTRS`` covers ``PUBLIC_ESTIMATOR_ORDER``), so a
        # row with no position here simply had nothing to report this frame.
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
        ring.scale.x = RING_LINE_WIDTH_M
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
                    # Zero timeout, and it has to stay zero. The TF listener is
                    # constructed with ``spin_thread=False``, so it shares this
                    # node's single-threaded executor: the /tf callback that
                    # would satisfy a wait is queued BEHIND the callback doing
                    # the waiting, and can never run until it returns. A
                    # non-zero timeout therefore cannot succeed -- it only burns
                    # the whole budget, every time, and this runs on every
                    # measurement callback. At 0.2 s against a ``world_frame``
                    # the run does not have (the benchmark has no "map"; it
                    # falls through to "odom") that cost two stalls per message
                    # and starved the node to a sixth of its rate, which the HUD
                    # showed as permanent "-- miss" rows: the measurement slots
                    # aged out of the liveness gate while the executor blocked.
                    # ``stamp`` is Time() -- latest available -- so the buffer
                    # either holds the transform now or does not.
                    tf = self.tf_buffer.lookup_transform(
                        world_frame, base_frame, stamp,
                        timeout=rclpy.duration.Duration(seconds=0.0),
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
