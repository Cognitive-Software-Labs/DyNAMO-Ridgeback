"""Exact-stamp buffers and diagnostics for target-localization inputs."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import Image


# ~0.5 s of color frames at 30 fps -- comfortably above the detector latency
# (~200 ms at 5 FPS), so the exact-stamp lookup only misses when the pipeline
# is genuinely stalled.
COLOR_BUFFER_DEPTH_DEFAULT = 15

# Depth input and scans are buffered across detector latency plus jitter. Depth
# shares the color stamp and is exact-matched; the scan free-runs at ~40 Hz and
# is matched to the nearest stamp inside the tolerance.
DEPTH_MATCH_BUFFER_DEPTH = 15
SCAN_MATCH_BUFFER_DEPTH = 20
SCAN_MATCH_TOLERANCE_S_DEFAULT = 0.05


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


@dataclass(frozen=True)
class PreparedColorFrame:
    """The exact color message and RGB array prepared for one batch only."""

    message: Image
    rgb: np.ndarray


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
        self.miss_nearest_delta_ms: list[float] = []
        self.empty_buffer_misses = 0

    def record_lookup(self, hit: bool, target_ns: int, buffered_ns: list[int]) -> None:
        if hit:
            self.hits += 1
            return
        self.misses += 1
        if not buffered_ns:
            self.empty_buffer_misses += 1
            return
        if target_ns > max(buffered_ns):
            self.miss_target_newer += 1
        elif target_ns < min(buffered_ns):
            self.miss_target_older += 1
        else:
            self.miss_target_inside += 1
        nearest = min(abs(stamp - target_ns) for stamp in buffered_ns)
        self.miss_nearest_delta_ms.append(nearest / 1e6)

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
        return '\n'.join(lines)
