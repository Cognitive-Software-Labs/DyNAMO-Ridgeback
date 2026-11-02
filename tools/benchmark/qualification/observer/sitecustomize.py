"""Observe capture windows without changing their contents or stopping rules."""
import builtins
import functools
import json
import os
import time

_original = builtins.__build_class__


def _build(function, name, *args, **kwargs):
    cls = _original(function, name, *args, **kwargs)
    if name != 'TargetDistanceBenchmarkRunner':
        return cls
    method = cls.__dict__.get('capture_measurement_window')
    if method is None:
        return cls

    @functools.wraps(method)
    def capture(self, duration_sec):
        started = time.monotonic()
        result = method(self, duration_sec)
        record = {
            'run_label': self.run_label,
            'trial_id': self.active_trial_id,
            'requested_sec': duration_sec,
            'wall_sec': time.monotonic() - started,
            'total_events': result.get('total_events'),
            'usable_events': len(result.get('usable_events', ())),
        }
        path = os.environ.get('QUALIFICATION_CAPTURE_TIMINGS')
        if path:
            with open(path, 'a', encoding='utf-8') as stream:
                stream.write(json.dumps(record) + '\n')
        return result

    cls.capture_measurement_window = capture
    return cls


if os.environ.get('QUALIFICATION_CAPTURE_TIMINGS'):
    builtins.__build_class__ = _build
