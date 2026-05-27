from __future__ import annotations

import importlib


def test_packaged_modules_import() -> None:
    module_names = [
        'ridgeback_autonomy.common.camera_config',
        'ridgeback_autonomy.common.messages',
        'ridgeback_autonomy.common.models',
        'ridgeback_autonomy.common.tf_utils',
        'ridgeback_autonomy.benchmarking.alignment',
        'ridgeback_autonomy.benchmarking.estimators',
        'ridgeback_autonomy.perception.core.detection',
        'ridgeback_autonomy.perception.core.geometry',
        'ridgeback_autonomy.perception.core.image_utils',
        'ridgeback_autonomy.perception.core.rendering',
        'ridgeback_autonomy.benchmarking.metrics',
        'ridgeback_autonomy.benchmarking.reduction',
        'ridgeback_autonomy.benchmarking.rendering',
        'ridgeback_autonomy.benchmarking.summary',
        'ridgeback_autonomy.perception.g1_detector_node',
        'ridgeback_autonomy.perception.g1_camera_measurement_node',
        'ridgeback_autonomy.perception.g1_lidar_measurement_node',
        'ridgeback_autonomy.perception.g1_overlay_node',
        'ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node',
    ]

    for module_name in module_names:
        assert importlib.import_module(module_name) is not None
