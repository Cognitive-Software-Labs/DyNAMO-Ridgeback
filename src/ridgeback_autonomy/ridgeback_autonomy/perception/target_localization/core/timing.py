"""Constant-space cold/warm timing summaries, shared by the perception nodes.

The detector and the mask node each run one model-bearing worker thread whose
per-stage cost is only interesting as a distribution, and neither can afford to
retain every sample for the length of a run. This owns the one bounded
accumulator both use: the first few calls (lazy model load, allocator setup,
kernel warm-up) stay visible as cold values and are kept out of the warm
percentiles, which are taken over a rolling window of recent samples.

ROS-free and node-free, so a diagnostic's arithmetic can be asserted without
standing up a node.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

TIMING_SAMPLE_LIMIT = 512
TIMING_COLD_SAMPLE_COUNT = 5


@dataclass
class TimingStats:
    """Constant-space cold/warm timing summary in nanoseconds."""

    count: int = 0
    total_ns: int = 0
    max_ns: int = 0
    first_ns: int | None = None
    cold_samples_ns: list[int] = field(default_factory=list)
    samples_ns: deque[int] = field(
        default_factory=lambda: deque(maxlen=TIMING_SAMPLE_LIMIT))

    def record(self, elapsed_ns: int) -> None:
        elapsed_ns = max(0, int(elapsed_ns))
        if self.first_ns is None:
            self.first_ns = elapsed_ns
        if len(self.cold_samples_ns) < TIMING_COLD_SAMPLE_COUNT:
            self.cold_samples_ns.append(elapsed_ns)
        self.count += 1
        self.total_ns += elapsed_ns
        self.max_ns = max(self.max_ns, elapsed_ns)
        self.samples_ns.append(elapsed_ns)

    @staticmethod
    def _percentile(values: list[int], fraction: float) -> int:
        """Nearest-rank percentile for a non-empty sample."""

        ordered = sorted(values)
        index = max(0, math.ceil(fraction * len(ordered)) - 1)
        return ordered[index]

    def warm_samples_ns(self) -> list[int]:
        """The retained samples with the earliest calls excluded.

        Early calls may include lazy model loading, allocator setup, and kernel
        warm-up. The first five stay visible as cold values but are kept out of
        every warm figure. Once the bounded deque rolls over, every retained
        sample at or beyond index five is warm.
        """

        retained = list(self.samples_ns)
        oldest_retained_index = self.count - len(retained)
        cold_values_still_retained = max(
            0, TIMING_COLD_SAMPLE_COUNT - oldest_retained_index)
        return retained[cold_values_still_retained:]

    def warm_percentile_ns(self, fraction: float) -> int | None:
        """One warm percentile, or ``None`` before any warm sample exists.

        Prefer this over the warm mean for any figure quoted as a headline: a
        mean over intervals is dominated by the rare long gap (a paused stream,
        a scene change between benchmark trials) rather than by the cadence the
        reader is being told about.
        """

        warm = self.warm_samples_ns()
        return self._percentile(warm, fraction) if warm else None

    def format_ms(self) -> str:
        if not self.count:
            return 'n/a'
        warm = self.warm_samples_ns()
        if warm:
            warm_mean_ms = sum(warm) / len(warm) / 1e6
            warm_text = (
                f'warm_n={len(warm)} warm_mean={warm_mean_ms:.3f} '
                f'warm_p50={self._percentile(warm, 0.50) / 1e6:.3f} '
                f'warm_p95={self._percentile(warm, 0.95) / 1e6:.3f} '
                f'warm_p99={self._percentile(warm, 0.99) / 1e6:.3f}')
        else:
            warm_text = 'warm_n=0 warm_mean=n/a warm_p50=n/a warm_p95=n/a warm_p99=n/a'
        cold_text = ','.join(
            f'{value / 1e6:.3f}' for value in self.cold_samples_ns)
        return (
            f'count={self.count} first={self.first_ns / 1e6:.3f} '
            f'cold_ms=[{cold_text}] {warm_text} max={self.max_ns / 1e6:.3f}')
