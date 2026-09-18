"""Progress-based health, independent of ROS and inference frameworks."""
from dataclasses import dataclass
import math


@dataclass
class StreamProgress:
    stamp: int = -1
    changed: float = 0.0
    count: int = 0


class PipelineHealth:
    def __init__(self, inputs, outputs, *, now, startup_timeout=120.0,
                 input_timeout=3.0, progress_timeout=15.0, source_age=15.0):
        self.inputs = tuple(inputs)
        self.outputs = tuple(outputs)
        self.streams = {name: StreamProgress() for name in (*inputs, *outputs)}
        self.started = now
        self.startup_timeout = startup_timeout
        self.input_timeout = input_timeout
        self.progress_timeout = progress_timeout
        self.source_age = source_age
        if any(not math.isfinite(x) or x <= 0 for x in (
                startup_timeout, input_timeout, progress_timeout, source_age)):
            raise ValueError('Health timeouts must be finite and positive')
        self.failures = {}
        self.ever_ready = False

    def observe(self, name, stamp, now):
        stream = self.streams[name]
        if stamp > stream.stamp:
            stream.stamp, stream.changed = stamp, now
            stream.count += 1

    def state(self, now, ros_now_ns):
        if self.failures:
            return 'failure', '; '.join(f'{k}: {v}' for k, v in sorted(self.failures.items()))
        missing = [name for name, value in self.streams.items() if not value.count]
        if missing and not self.ever_ready and now - self.started < self.startup_timeout:
            return 'startup', 'Waiting for ' + ', '.join(missing)
        for name in self.inputs:
            s = self.streams[name]
            if not s.count or now - s.changed > self.input_timeout or not 0 <= (ros_now_ns - s.stamp) / 1e9 <= self.source_age:
                return 'stale_input', name
        for name in self.outputs:
            s = self.streams[name]
            if not s.count or now - s.changed > self.progress_timeout or not 0 <= (ros_now_ns - s.stamp) / 1e9 <= self.source_age:
                return 'stalled', name
        self.ever_ready = True
        return 'processing', 'Current input and output stamps advancing'
