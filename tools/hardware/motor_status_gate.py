"""Read-only Puma status predicate for bounded physical movement trials."""

import math


EXPECTED_WHEELS = frozenset({
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint',
})


def summarize(message):
    drivers = list(message.drivers)
    names = [driver.device_name for driver in drivers]
    return {
        'complete': len(drivers) == 4 and frozenset(names) == EXPECTED_WHEELS
                    and len(set(names)) == 4,
        'drivers': [{
            'name': driver.device_name,
            'mode': int(driver.mode),
            'fault': int(driver.fault),
            'bus_voltage': float(driver.bus_voltage),
        } for driver in drivers],
    }


def ready(summary):
    return summary['complete'] and all(
        driver['mode'] == 2 and driver['fault'] == 0
        and math.isfinite(driver['bus_voltage']) and driver['bus_voltage'] > 10
        for driver in summary['drivers'])


def stable_after_release(first_ready_at, ready_samples, now, status_at, release_at):
    return (first_ready_at is not None and first_ready_at > release_at
            and ready_samples >= 2 and now-first_ready_at >= 1.0
            and now-status_at <= 1.0)
