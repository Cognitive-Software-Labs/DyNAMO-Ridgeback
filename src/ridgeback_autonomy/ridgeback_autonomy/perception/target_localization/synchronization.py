"""Exact-stamp buffers and diagnostics for target-localization inputs."""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
import time

import numpy as np
from sensor_msgs.msg import Image

from ridgeback_autonomy.common.stamps import stamp_key
from ridgeback_autonomy.perception.target_localization.core.timing import TimingStats

# ~0.5 s of color frames at 30 fps -- comfortably above the detector's
# publish latency, so the exact-stamp lookup only misses when the pipeline is
# genuinely stalled. The margin widens, never narrows, as the detector rate
# rises: a faster detector reaches each frame sooner after it is buffered.
COLOR_BUFFER_DEPTH_DEFAULT = 15

# Depth input and scans are buffered across detector latency plus jitter. Depth
# shares the color stamp and is exact-matched; the scan free-runs at ~40 Hz and
# is matched to the nearest stamp inside the tolerance.
DEPTH_MATCH_BUFFER_DEPTH = 15
SCAN_MATCH_BUFFER_DEPTH = 20
SCAN_MATCH_TOLERANCE_S_DEFAULT = 0.05
DEPTH_MATCH_RECORD_LIMIT = 256
TIMING_STAGE_NAMES = (
    'slimsam_load',
    'rgb_prepare',
    'mask_region_prepare',
    'slimsam',
    'stereo_depth',
    'depth_anything',
    'estimator_reduction',
    'cuda_sync',
)


@dataclass(frozen=True)
class PreparedColorFrame:
    """The exact color message and RGB array prepared for one batch only."""

    message: Image
    rgb: np.ndarray


@dataclass
class DepthMissRecord:
    """One bounded stamp-only record for a failed exact lookup."""

    target_ns: int
    lookup_monotonic_ns: int
    placement: str
    nearest_delta_ns: int | None
    arrival_monotonic_ns: int | None = None


class StampedMessageBuffer:
    """Stamp-keyed rolling buffer of recent messages, matched by header stamp.

    Used for the color frame (the silhouette gate's prompt image and the
    monocular depth source's input), the raw depth stream, and the scan.
    Color and depth share the exact color stamp, so they match with
    ``lookup`` (no tolerance); the scan free-runs, so it matches with
    ``lookup_nearest`` within a tolerance window. A miss means the message aged
    out (or never arrived); the caller skips that source's paths rather than
    pairing whatever arrived most recently, which would smear distance under
    motion or mislabel the silhouette benchmark row.
    """

    def __init__(self, depth: int) -> None:
        self.depth = int(depth)
        self._msgs: OrderedDict[tuple[int, int], object] = OrderedDict()

    def __len__(self) -> int:
        return len(self._msgs)

    def store(self, msg) -> None:
        key = stamp_key(msg.header.stamp)
        self._msgs[key] = msg
        self._msgs.move_to_end(key)
        while len(self._msgs) > self.depth:
            self._msgs.popitem(last=False)

    def lookup(self, stamp):
        """Exact stamp match, or ``None``."""

        return self._msgs.get(stamp_key(stamp))

    def lookup_nearest(self, stamp, tolerance_s: float):
        """Buffered message closest to ``stamp`` within ``tolerance_s``, or ``None``."""

        sec, nanosec = stamp_key(stamp)
        target_ns = sec * 1_000_000_000 + nanosec
        tol_ns = int(tolerance_s * 1_000_000_000)
        best = None
        best_delta = None
        for (key_sec, key_nanosec), msg in self._msgs.items():
            delta = abs(key_sec * 1_000_000_000 + key_nanosec - target_ns)
            if delta <= tol_ns and (best_delta is None or delta < best_delta):
                best_delta = delta
                best = msg
        return best

    def stamps_ns(self) -> list[int]:
        """Buffered stamps as nanoseconds, oldest first. Diagnostics only."""

        return [sec * 1_000_000_000 + nanosec for sec, nanosec in self._msgs]


class DepthMatchDiagnostics:
    """Counts why the depth input lookup hits or misses, for one run.

    Answers three competing explanations for a ``NO_DEPTH_FRAME`` with one
    log line, without changing any matching behaviour:

    - **reception loss** -- ``depth`` received well below ``color``. The frame
      was published but this process never got it.
    - **stamp mismatch** -- both streams received at the same rate, the target
      stamp sits inside the buffered span, and the nearest buffered stamp is a
      near-constant offset away. The streams are not stamped alike.
    - **lag** -- the target is *newer* than everything buffered, so the depth
      frame simply had not arrived yet when the detection was processed.
    """

    def __init__(self) -> None:
        self.color_rx = 0
        self.depth_rx = 0
        self.hits = 0
        self.misses = 0
        self.miss_target_newer = 0
        self.miss_target_older = 0
        self.miss_target_inside = 0
        self.miss_nearest_delta_ms: deque[float] = deque(
            maxlen=DEPTH_MATCH_RECORD_LIMIT)
        self.empty_buffer_misses = 0
        self.depth_duplicates = 0
        self.depth_out_of_order = 0
        self.last_depth_stamp_ns: int | None = None
        self.last_depth_arrival_monotonic_ns: int | None = None
        self.max_depth_callback_gap_ns = 0
        self.late_arrivals = 0
        self.late_arrival_delay_ns: deque[int] = deque(
            maxlen=DEPTH_MATCH_RECORD_LIMIT)
        self.miss_records: OrderedDict[int, DepthMissRecord] = OrderedDict()
        self.evicted_unresolved_records = 0
        self.detections_rx = 0
        self.detection_slot_replacements = 0
        self.last_detection_arrival_monotonic_ns: int | None = None
        self.detection_interarrival = TimingStats()
        self.detection_queue_age = TimingStats()
        self.worker_processing = TimingStats()
        self.publication_age = TimingStats()
        self.completed_batches = 0
        self.failed_batches = 0
        self.batch_outcomes: deque[tuple[int, str]] = deque(
            maxlen=DEPTH_MATCH_RECORD_LIMIT)
        self.failed_batch_stamps_ns: deque[int] = deque(
            maxlen=DEPTH_MATCH_RECORD_LIMIT)
        self.stage_timing = {
            name: TimingStats() for name in TIMING_STAGE_NAMES
        }
        self.lock_wait: dict[str, TimingStats] = {}
        self.lock_hold: dict[str, TimingStats] = {}

    def record_depth_arrival(
        self,
        stamp_ns: int,
        *,
        now_ns: int | None = None,
    ) -> None:
        """Account for one depth callback and resolve any earlier miss."""

        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        self.depth_rx += 1
        if self.last_depth_stamp_ns is not None:
            if stamp_ns == self.last_depth_stamp_ns:
                self.depth_duplicates += 1
            elif stamp_ns < self.last_depth_stamp_ns:
                self.depth_out_of_order += 1
        if self.last_depth_arrival_monotonic_ns is not None:
            self.max_depth_callback_gap_ns = max(
                self.max_depth_callback_gap_ns,
                now_ns - self.last_depth_arrival_monotonic_ns,
            )
        self.last_depth_stamp_ns = stamp_ns
        self.last_depth_arrival_monotonic_ns = now_ns

        record = self.miss_records.get(stamp_ns)
        if record is not None and record.arrival_monotonic_ns is None:
            record.arrival_monotonic_ns = now_ns
            self.late_arrivals += 1
            self.late_arrival_delay_ns.append(
                max(0, now_ns - record.lookup_monotonic_ns))

    def record_detection_arrival(
        self,
        *,
        replaced_pending: bool,
        now_ns: int | None = None,
    ) -> None:
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        self.detections_rx += 1
        if replaced_pending:
            self.detection_slot_replacements += 1
        if self.last_detection_arrival_monotonic_ns is not None:
            self.detection_interarrival.record(
                now_ns - self.last_detection_arrival_monotonic_ns)
        self.last_detection_arrival_monotonic_ns = now_ns

    def record_detection_dequeue(self, age_ns: int) -> None:
        self.detection_queue_age.record(age_ns)

    def record_worker_processing(self, elapsed_ns: int) -> None:
        self.worker_processing.record(elapsed_ns)

    def record_stage(self, name: str, elapsed_ns: int) -> None:
        if name not in self.stage_timing:
            raise ValueError(f'Unknown timing stage "{name}".')
        self.stage_timing[name].record(elapsed_ns)

    def record_batch_completed(self, stamp_ns: int, publication_age_ns: int) -> None:
        self.completed_batches += 1
        self.batch_outcomes.append((int(stamp_ns), 'ok'))
        self.publication_age.record(publication_age_ns)

    def record_batch_failed(self, stamp_ns: int) -> None:
        self.failed_batches += 1
        stamp_ns = int(stamp_ns)
        self.batch_outcomes.append((stamp_ns, 'failed'))
        self.failed_batch_stamps_ns.append(stamp_ns)

    def record_lock_timing(self, owner: str, wait_ns: int, hold_ns: int) -> None:
        self.lock_wait.setdefault(owner, TimingStats()).record(wait_ns)
        self.lock_hold.setdefault(owner, TimingStats()).record(hold_ns)

    def record_lookup(
        self,
        hit: bool,
        target_ns: int,
        buffered_ns: list[int],
        *,
        now_ns: int | None = None,
    ) -> None:
        if hit:
            self.hits += 1
            return
        self.misses += 1
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        placement = 'empty_buffer'
        nearest_ns = None
        if not buffered_ns:
            self.empty_buffer_misses += 1
        else:
            if target_ns > max(buffered_ns):
                self.miss_target_newer += 1
                placement = 'target_newer'
            elif target_ns < min(buffered_ns):
                self.miss_target_older += 1
                placement = 'target_older'
            else:
                self.miss_target_inside += 1
                placement = 'target_inside'
            nearest_ns = min(abs(stamp - target_ns) for stamp in buffered_ns)
            self.miss_nearest_delta_ms.append(nearest_ns / 1e6)

        if target_ns not in self.miss_records:
            self.miss_records[target_ns] = DepthMissRecord(
                target_ns=target_ns,
                lookup_monotonic_ns=now_ns,
                placement=placement,
                nearest_delta_ns=nearest_ns,
            )
            while len(self.miss_records) > DEPTH_MATCH_RECORD_LIMIT:
                _, evicted = self.miss_records.popitem(last=False)
                if evicted.arrival_monotonic_ns is None:
                    self.evicted_unresolved_records += 1

    @staticmethod
    def _spread_ms(values_ns) -> str:
        if not values_ns:
            return 'n/a'
        ordered = sorted(values_ns)
        median = ordered[len(ordered) // 2]
        return (
            f'min={ordered[0] / 1e6:.3f} med={median / 1e6:.3f} '
            f'max={ordered[-1] / 1e6:.3f}')

    def summary(self) -> str:
        lookups = self.hits + self.misses
        ratio = (self.depth_rx / self.color_rx) if self.color_rx else float('nan')
        hit_rate = (self.hits / lookups) if lookups else float('nan')
        lines = [
            f'depth-match: rx color={self.color_rx} depth={self.depth_rx} '
            f'(depth/color={ratio:.2f}) | lookups={lookups} hit={self.hits} '
            f'miss={self.misses} (hit_rate={hit_rate:.2f})',
        ]
        if self.misses:
            deltas = sorted(self.miss_nearest_delta_ms)
            if deltas:
                median = deltas[len(deltas) // 2]
                spread = (
                    f'min={deltas[0]:.1f} med={median:.1f} max={deltas[-1]:.1f}')
            else:
                spread = 'n/a'
            lines.append(
                f'  miss placement: target_newer={self.miss_target_newer} '
                f'target_inside={self.miss_target_inside} '
                f'target_older={self.miss_target_older} '
                f'empty_buffer={self.empty_buffer_misses}')
            lines.append(f'  nearest buffered stamp delta (ms): {spread}')
            unresolved = [
                record for record in self.miss_records.values()
                if record.arrival_monotonic_ns is None
            ]
            sample = ','.join(
                str(record.target_ns) for record in unresolved[-8:]) or 'none'
            lines.append(
                f'  later arrival: resolved={self.late_arrivals} '
                f'unresolved_bounded={len(unresolved)} '
                f'evicted_unresolved={self.evicted_unresolved_records} '
                f'delay_ms={self._spread_ms(self.late_arrival_delay_ns)}')
            lines.append(f'  unresolved target stamps ns (latest 8): {sample}')
        lines.append(
            f'  depth callbacks: duplicate={self.depth_duplicates} '
            f'out_of_order={self.depth_out_of_order} '
            f'max_gap_ms={self.max_depth_callback_gap_ns / 1e6:.3f}')
        lines.append(
            f'  detections: rx={self.detections_rx} '
            f'pending_replaced={self.detection_slot_replacements} '
            f'interarrival_ms={self.detection_interarrival.format_ms()} '
            f'dequeue_age_ms={self.detection_queue_age.format_ms()} '
            f'worker_ms={self.worker_processing.format_ms()}')
        finished = self.completed_batches + self.failed_batches
        completion_rate = (
            self.completed_batches / finished if finished else float('nan'))
        lines.append(
            f'  batches: completed={self.completed_batches} failed={self.failed_batches} '
            f'completion_rate={completion_rate:.3f} '
            f'publication_age_ms={self.publication_age.format_ms()}')
        if self.batch_outcomes:
            outcome_sample = ','.join(
                f'{stamp_ns}:{outcome}'
                for stamp_ns, outcome in list(self.batch_outcomes)[-8:])
            lines.append(f'  batch outcomes (latest 8): {outcome_sample}')
        if self.failed_batch_stamps_ns:
            failed_sample = ','.join(
                str(stamp_ns) for stamp_ns in list(self.failed_batch_stamps_ns)[-8:])
            lines.append(f'  failed target stamps ns (latest 8): {failed_sample}')
        for name in TIMING_STAGE_NAMES:
            stats = self.stage_timing[name]
            if stats.count:
                lines.append(f'  stage {name}_ms={stats.format_ms()}')
        for owner in sorted(self.lock_wait):
            lines.append(
                f'  lock {owner}: wait_ms={self.lock_wait[owner].format_ms()} '
                f'hold_ms={self.lock_hold[owner].format_ms()}')
        return '\n'.join(lines)
