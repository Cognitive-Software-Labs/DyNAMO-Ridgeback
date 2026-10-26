import pytest

from ridgeback_autonomy.perception.core.time_sync import select_closest_timestamped


def test_closest_timestamped_selects_sensor_frame_nearest_detection() -> None:
    samples = [(9.7, 'old'), (10.02, 'matched'), (10.5, 'latest')]

    result = select_closest_timestamped(samples, 10.0, 0.10)

    assert result is not None
    sample, dt_s = result
    assert sample == 'matched'
    assert dt_s == pytest.approx(0.02)


def test_closest_timestamped_rejects_stale_sensor_frames() -> None:
    samples = [(9.7, 'old'), (10.5, 'future')]

    assert select_closest_timestamped(samples, 10.0, 0.10) is None
