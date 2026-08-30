#!/usr/bin/env python3

"""Where each estimator thinks the G1 is, as RViz rings, plus a HUD readout.

Two renderings of the same measurements. The rings put every estimator's answer
on the floor plan at once, so disagreement is spatial and immediate; the HUD
panel prints the same numbers against the benchmark's ground truth, because a
ring 60 mm off and a ring 2 m off look alike once they are small.

The estimator set is driven from ``perception/estimators.py`` rather than
listed here, so a new estimator appears in both renderings by registering there.
"""

from __future__ import annotations

from dataclasses import dataclass
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

from ridgeback_autonomy.perception.estimators import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_LABELS,
    ESTIMATOR_POSITION_ATTRS,
    ESTIMATOR_SHORT_LABELS,
    PUBLIC_ESTIMATOR_ORDER,
    nearest_instance_index,
    parse_estimators,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.vehicle_frame import remove_vehicle_front_offset
from ridgeback_autonomy.perception.ground_truth import GROUND_TRUTH_TOPIC, truth_reading


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

# Two renderings of the same readings, for two panel shapes. ``rows`` is the
# benchmark's: one labelled row per estimator, with the truth and error columns
# that only a benchmark has a truth source for. ``wide`` transposes it into
# estimator columns for exploration, which has no truth at all, so the vertical
# space a row layout spends on labels buys nothing there.
HUD_LAYOUT_ROWS = 'rows'
HUD_LAYOUT_WIDE = 'wide'
HUD_LAYOUTS = (HUD_LAYOUT_ROWS, HUD_LAYOUT_WIDE)

# Columns per wide-layout cell, right-aligned. Wide enough for the widest header
# ('Project', 7) and the widest reading ('12.345', 6) with a gap between
# neighbours; four of them make a ~40-column panel, which is what
# ``overlay_width`` has to cover or the last column is silently clipped off.
HUD_WIDE_CELL_COLUMNS = 10

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

    Both halves are ``(message, age_seconds)`` entries. A same-batch message has
    an age too -- its stamp is the detection instant, so the figure is the
    pipeline latency behind that row -- and the wide HUD layout prints it beside
    every reading, not only the ones that fell out of the current batch.

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
    the aged remainder is only ever allowed to print numbers.
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
    dated = [
        (msg, (now_nanoseconds - stamp) / 1_000_000_000, stamp)
        for msg, stamp in live
    ]
    same_batch = [(msg, age_s) for msg, age_s, stamp in dated if stamp == newest]
    aged = [(msg, age_s) for msg, age_s, stamp in dated if stamp != newest]
    return same_batch, aged


def batch_messages(entries: list) -> list:
    """The messages out of ``(message, age_seconds)`` entries.

    ``nearest_detection_index`` and the marker loop rank and draw messages, not
    ages, so they take the plain list; the ages exist only for the HUD.
    """

    return [msg for msg, _ in entries]


def _reading_body(label: str, distance_m: float, truth) -> str:
    error = '' if truth is None else f'  {distance_m - truth.distance_m:+.3f}'
    return f'{label:<24} {distance_m:7.3f}{error}'


@dataclass(frozen=True)
class EstimatorReading:
    """One estimator's answer for the instance every surface speaks for.

    Carries what the panel prints *and* what the ring is drawn from, because
    those must be the same answer. A column and its ring are two renderings of
    one claim; computing them from separate reads of the cache is what let a
    path's number and its ring show different states, with nothing on screen to
    say which one was current.

    ``stamp`` is the source message's, so the ring can be placed against the
    robot pose from that instant rather than the latest one.
    """

    distance_m: float
    age_s: float
    aged: bool
    forward_m: float | None
    lateral_m: float | None
    stamp: object


def collect_readings(
    same_batch: list,
    aged: list,
    nearest: int,
    estimators: tuple[str, ...],
) -> dict[str, 'EstimatorReading']:
    """``estimator -> EstimatorReading`` for everything this frame can show.

    The single snapshot every surface renders. An estimator absent from the map
    is a miss: the panel prints ``--`` for it and the marker layer deletes its
    ring, on the same tick, off this one dict. That is what stops a path's text
    and its ring from showing different states -- they are two renderings of one
    entry, not two independent reads of the cache.

    A fresh reading wins over an aged one for the same estimator. An aged
    message is ranked against itself rather than against ``nearest``: it comes
    from a different detector batch, so the batch's index would be a guess about
    which robot it names, and the message's own estimators rank exactly the
    instance its producer was speaking for.
    """

    readings: dict[str, EstimatorReading] = {}
    for msg, age_s in same_batch:
        if not msg.detected or nearest >= msg.count:
            continue
        for estimator in estimators:
            # Every surface is a readout for one instance, so they all quote the
            # same nearest one rather than each picking their own.
            forward, lateral, distance = estimator_reading(msg, estimator, nearest)
            if distance is not None:
                readings[estimator] = EstimatorReading(
                    distance_m=distance, age_s=age_s, aged=False,
                    forward_m=forward, lateral_m=lateral, stamp=msg.header.stamp)

    for msg, age_s in aged:
        index = nearest_detection_index([msg])
        if not msg.detected or index >= msg.count:
            continue
        for estimator in estimators:
            if estimator in readings:
                continue
            forward, lateral, distance = estimator_reading(msg, estimator, index)
            if distance is not None:
                readings[estimator] = EstimatorReading(
                    distance_m=distance, age_s=age_s, aged=True,
                    forward_m=forward, lateral_m=lateral, stamp=msg.header.stamp)

    return readings


def hud_rows(
    same_batch: list,
    aged: list,
    nearest: int,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> list[str]:
    """``hud_rows_from_readings`` for callers holding messages, not a snapshot."""

    return hud_rows_from_readings(
        collect_readings(same_batch, aged, nearest, estimators), truth, estimators)


def hud_rows_from_readings(
    readings: dict,
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

    A fresh reading's own age is discarded here: this layout spends its width on
    the truth and error columns, and an age column on every row would push them
    off a panel that already clips. The wide layout prints it.
    """

    rows = []
    for estimator in estimators:
        label = ESTIMATOR_LABELS[estimator]
        reading = readings.get(estimator)
        if reading is None:
            body = f'{label:<24}      --    miss'
            colour = hud_text_colour(estimator)
        elif reading.aged:
            body = (f'{_reading_body(label, reading.distance_m, truth)}'
                    f'   {reading.age_s:.1f}s')
            colour = hud_text_colour(estimator, aged=True)
        else:
            body = _reading_body(label, reading.distance_m, truth)
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
    """``hud_section_from_readings`` for callers holding messages, not a snapshot."""

    return hud_section_from_readings(
        collect_readings(same_batch, aged, nearest, estimators),
        truth, estimators, bool(same_batch or aged))


def hud_section_from_readings(
    readings: dict,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
    has_messages: bool = True,
) -> str:
    """The whole HUD section, or ``''`` when there is nothing to say.

    Rendered by RViz as text, so it stays legible at any window size -- unlike
    numbers burned into the perception overlay bitmap, which shrink with it.

    Each line is coloured from ``ESTIMATOR_COLOURS``, the same table that colours
    that estimator's ring, so a reading and its ring cannot drift apart. That
    markup makes the overlay rich text, hence ``&nbsp;`` padding and ``<br/>``
    breaks -- see ``hud_node``'s ``rich_text``.

    ``has_messages`` is what decides the empty string, not an empty ``readings``:
    a live producer whose every field is NaN has to print ``-- miss`` rows, which
    is the one place that word is honest. Only a cache with nothing live in it
    collapses the panel.

    The empty string is load-bearing: ``hud_node`` drops falsy panels, so it
    collapses the section instead of holding the last text it was ever sent. A
    panel of numbers left standing under a floor plan whose rings have all
    correctly expired reads as a broken ring layer rather than a dead detector.
    """

    if not has_messages:
        return ''
    lines = [_hud_line(hud_truth_header(truth), HUD_HEADER_COLOUR)]
    lines.extend(hud_rows_from_readings(readings, truth, estimators))
    return '<br/>'.join(lines)


def parse_hud_layout(raw_layout: str | None) -> str:
    """Validate the ``hud_layout`` parameter (``rows`` | ``wide``).

    Raising beats falling back to the default: a typo would otherwise ship the
    benchmark's row layout into a panel sized for columns, which looks like a
    clipping fault rather than a misspelled parameter.
    """

    layout = (raw_layout or '').strip()
    if not layout:
        return HUD_LAYOUT_ROWS
    if layout not in HUD_LAYOUTS:
        supported = ', '.join(HUD_LAYOUTS)
        raise ValueError(f'Unsupported hud_layout "{layout}". Expected one of: {supported}')
    return layout


def _hud_wide_cell(body: str, colour: tuple[float, float, float]) -> str:
    """One right-aligned fixed-width cell.

    Padded before ``_hud_line`` sees it, so the escape-then-pad rule there still
    holds -- the spaces added here are turned into ``&nbsp;`` by the same pass
    that escapes the body, rather than by a second one that would rewrite its
    own ampersands.
    """

    return _hud_line(f'{body:>{HUD_WIDE_CELL_COLUMNS}}', colour)


def hud_wide_section_text(
    same_batch: list,
    aged: list,
    nearest: int,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> str:
    """``hud_wide_text`` for callers holding messages, not a snapshot."""

    return hud_wide_text(
        collect_readings(same_batch, aged, nearest, estimators),
        estimators, bool(same_batch or aged))


def hud_wide_text(
    readings: dict,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
    has_messages: bool = True,
) -> str:
    """The same readings as three lines of columns, or ``''`` when there are none.

    Estimator names across the top, distances under them, ages under those --
    here with the polar row a batch behind the other three::

             Cloud   Project    Euclid     Polar
             3.310     3.262     3.255     3.180
              0.1s      0.1s      0.1s      1.3s

    For exploration, which has no truth source: there is no truth line and no
    error column, so the row layout's width would go on labels alone. A column
    is coloured per cell rather than per line so each keeps its own ring's hue
    from ``ESTIMATOR_COLOURS``, and dims to ``HUD_AGED_LUMINANCE`` as a whole
    when that estimator's reading is aged -- the header dimming with the number
    is what marks the column rather than the row.

    A column nothing reported prints ``--`` over a blank age cell. The
    ``''``-when-empty contract is ``hud_section_text``'s, for the same reason:
    ``hud_node`` drops falsy panels, and a latched panel of numbers under rings
    that have correctly expired reads as a broken ring layer.
    """

    if not has_messages:
        return ''

    headers, distances, ages = [], [], []
    for estimator in estimators:
        reading = readings.get(estimator)
        colour = hud_text_colour(
            estimator, aged=reading is not None and reading.aged)
        headers.append(_hud_wide_cell(ESTIMATOR_SHORT_LABELS[estimator], colour))
        if reading is None:
            distances.append(_hud_wide_cell('--', colour))
            ages.append(_hud_wide_cell('', colour))
            continue
        distances.append(_hud_wide_cell(f'{reading.distance_m:.3f}', colour))
        ages.append(_hud_wide_cell(f'{reading.age_s:.1f}s', colour))

    return '<br/>'.join(''.join(cells) for cells in (headers, distances, ages))


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
        # file resolved it. "all" (the default) is every registered row.
        self.declare_parameter('estimators', 'all')
        # Which HUD shape to publish. The default is the benchmark's row layout,
        # the one surface with a truth source to put in it; exploration asks for
        # "wide" and gets columns with no truth and no error.
        self.declare_parameter('hud_layout', HUD_LAYOUT_ROWS)

        self.base_frame = self.get_parameter('base_frame').value or default_base_frame
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_lifetime = self.get_parameter('marker_lifetime_sec').value
        self.max_observation_age = float(
            self.get_parameter('max_observation_age_sec').value)
        self.estimators = parse_estimators(str(self.get_parameter('estimators').value))
        self.hud_layout = parse_hud_layout(str(self.get_parameter('hud_layout').value))

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

        # Both surfaces render on this one clock, off one snapshot, so a path's
        # ring and its number always describe the same instant. The markers used
        # to be event-driven instead, on the grounds that republishing them
        # against a fresh stamp would renew their ``lifetime`` forever and no
        # ring could expire -- true, and now answered by deleting a ring
        # explicitly when its estimator has no reading rather than waiting it
        # out. The panel has always needed a tick of its own: it cannot expire
        # anything from a callback that stops firing. ``create_timer`` runs on
        # the node clock, so under ``use_sim_time`` a paused sim freezes the
        # panel, the ages and the rings together rather than ticking them
        # forward against a world that is not moving.
        hud_rate = float(self.get_parameter('hud_publish_rate_hz').value)
        self.create_timer(
            1.0 / (hud_rate if hud_rate > 0.0 else HUD_PUBLISH_RATE_HZ),
            self._render)

    def _measurement_cb(self, key: str, msg: G1Measurements) -> None:
        # Cache only. Rendering is the timer's job, for both surfaces at once --
        # drawing markers from here and the panel from the timer is what let a
        # path's ring and its number describe different instants.
        with self._lock:
            self._latest[key] = (msg, self.get_clock().now().nanoseconds)

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

    def _publish_markers(self, readings: dict) -> None:
        """Rings and dots for one snapshot, each placed at its own stamp.

        Takes the snapshot rather than re-reading the cache, because this and the
        HUD are two renderings of one claim. Reading the cache twice let a path's
        number and its ring describe different instants -- the panel could print
        a distance for a row whose ring had already gone, or hold a ring under a
        row that read ``--`` -- with nothing on screen saying which was current.

        Aged messages are drawn, which they were not before. Excluding them did
        not cost an occasional ring, it cost the mask rows *every* ring: a mask
        measurement carries the detection instant but only arrives after
        inference, segmentation and the depth lookup, so by the time it lands the
        pointcloud producer has published several newer stamps and it is never
        the newest. Measured in exploration, 19 of 19 detected mask messages
        landed in the aged remainder and none in the batch.

        What made a stale ring dishonest was not its age but its placement:
        drawing a 1.2 s old measurement against the robot's pose *now* moves the
        ring by however far the robot has driven and swings it by however far it
        has turned. So the pose is looked up at the reading's own stamp instead.
        The measurement was true relative to where the robot stood then, and
        markers are published in the world frame, so the ring lands on the point
        the sensor actually saw. A lookup that falls outside the TF buffer drops
        that one ring rather than the whole frame.

        An estimator with no reading is DELETED rather than left to expire.
        ``lifetime`` alone cannot keep the two surfaces in step: it starts at the
        last publish, so a ring outlived its own column by up to a full lifetime
        after a producer went quiet. The delete makes the ring vanish on the same
        tick the column turns to ``--``; the lifetime stays as the backstop for
        this node dying outright, which no delete could cover.
        """

        markers = []
        marker_time = self.get_clock().now().to_msg()
        for estimator in self.estimators:
            reading = readings.get(estimator)
            placed = False
            if reading is not None:
                robot_x, robot_y, robot_yaw, actual_frame = self._robot_pose_in_world(
                    rclpy.time.Time.from_msg(reading.stamp)
                )
                if robot_x is not None:
                    placed = self._add_estimator_markers(
                        markers, estimator, reading.forward_m, reading.lateral_m,
                        robot_x, robot_y, robot_yaw, marker_time, actual_frame,
                    )
            if not placed:
                self._add_delete_markers(markers, estimator)

        ma = MarkerArray()
        ma.markers = markers
        self._pub.publish(ma)

    def _add_delete_markers(self, markers: list, estimator: str) -> None:
        """Remove this estimator's ring and dot now, without waiting them out."""

        id_base = _ESTIMATOR_ID_BASE[estimator]
        for offset in (0, 1):
            marker = Marker()
            marker.ns = f'g1_estimates/{estimator}'
            marker.id = id_base + offset
            marker.action = Marker.DELETE
            markers.append(marker)

    def _render(self) -> None:
        """One tick: read the cache once, draw both surfaces from that snapshot.

        Both renderings happen here, off one ``collect_readings`` result, so a
        path's number and its ring cannot describe different instants. They used
        to be driven separately -- rings on every measurement callback, the panel
        on this timer -- and each re-read the cache, so the two could disagree by
        a tick at best and by a whole marker lifetime at worst.

        Publishing unconditionally is the point. Returning early on an empty
        cache is what left the panel latched: ``hud_node`` only ever overwrites
        its cached text, so a section that stops publishing keeps showing its
        last numbers for as long as RViz is open. Publishing ``''`` collapses it,
        and the marker side deletes every ring in the same breath.

        This is also the only thing that re-evaluates the truth gate.
        ``truth_reading`` expires on ``TRUTH_MAX_AGE_S`` so the line disappears
        between trials, but reachable only from a measurement callback it was a
        gate with nothing to pull it -- when detections stopped it never fired
        again, and the previous trial's truth sat under a target that had
        already been teleported away.
        """

        same_batch, aged = self._partitioned_messages()
        nearest = (
            nearest_detection_index(batch_messages(same_batch)) if same_batch else 0
        )
        readings = collect_readings(same_batch, aged, nearest, self.estimators)

        text = OverlayText()
        if self.hud_layout == HUD_LAYOUT_WIDE:
            text.text = hud_wide_text(readings, self.estimators)
        else:
            text.text = hud_section_from_readings(
                readings, self._current_truth(), self.estimators)
        self._hud_pub.publish(text)

        self._publish_markers(readings)

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
    ) -> bool:
        """Append this estimator's ring and dot; ``False`` if it placed nothing.

        The caller needs the answer, not just the side effect: a row that
        reported a distance but no position gets no ring, and its previous ring
        has to be deleted rather than left standing.
        """

        # Every registered estimator places its own detection
        # (``ESTIMATOR_POSITION_ATTRS`` covers ``PUBLIC_ESTIMATOR_ORDER``), so a
        # row with no position here simply had nothing to report this frame.
        if forward_m is None or lateral_m is None:
            return False

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
        return True

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
                    # ``stamp`` is a measurement's own stamp, always in the
                    # past, so the transform is either already in the buffer or
                    # older than its cache -- waiting could not help either way.
                    # It must not go back to Time() (latest available): that
                    # places a reading from 1.2 s ago against the pose now,
                    # which is only harmless while the robot is stationary.
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
