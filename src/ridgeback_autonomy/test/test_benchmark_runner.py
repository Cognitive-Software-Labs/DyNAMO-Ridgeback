from __future__ import annotations

import pytest

from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
    G1DistanceBenchmarkRunner,
)


def test_benchmark_runner_on_measurement_uses_new_message_fields() -> None:
    runner = object.__new__(G1DistanceBenchmarkRunner)
    runner.latest_measurement_msg = None
    runner.capture_samples = []
    runner.capture_active = True
    runner.capture_last_measurement_snapshot = None

    msg = G1Measurements()
    msg.detected = True
    msg.count = 1
    msg.bbox_xyxy = [1.0, 2.0, 3.0, 4.0]
    msg.rgb_distance_m = [4.5]
    msg.sensor_depth_distance_m = [4.0]
    msg.mono_depth_distance_m = [float('nan')]
    msg.lidar_distance_m = [3.9]
    msg.pointcloud_distance_m = [0.0]

    G1DistanceBenchmarkRunner.on_measurement(runner, msg)

    assert runner.latest_measurement_msg is msg
    snapshot = runner.capture_last_measurement_snapshot
    assert snapshot['detected'] is True
    assert snapshot['count'] == 1
    assert snapshot['bboxes'] == [[1, 2, 3, 4]]
    assert snapshot['rgb_distance_m'] == pytest.approx(4.5, rel=1e-6)
    assert snapshot['sensor_depth_distance_m'] == pytest.approx(4.0, rel=1e-6)
    assert snapshot['mono_depth_distance_m'] is None
    assert snapshot['lidar_distance_m'] == pytest.approx(3.9, rel=1e-6)
    assert snapshot['pointcloud_distance_m'] is None

    assert len(runner.capture_samples) == 1
    sample = runner.capture_samples[0]
    assert sample['count'] == 1
    assert sample['detected'] is True
    assert sample['rgb_distance_m'] == pytest.approx(4.5, rel=1e-6)
    assert sample['sensor_depth_distance_m'] == pytest.approx(4.0, rel=1e-6)
    assert sample['mono_depth_distance_m'] is None
    assert sample['lidar_distance_m'] == pytest.approx(3.9, rel=1e-6)
    assert sample['pointcloud_distance_m'] is None
