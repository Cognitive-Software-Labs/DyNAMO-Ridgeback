from pathlib import Path

import pytest
import yaml


def test_realsense_stream_profiles_are_left_to_clearpath_defaults() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load(
        (repo_root / 'clearpath' / 'robot.yaml').read_text(encoding='utf-8')
    )
    parameters = config['sensors']['camera'][0]['ros_parameters']['intel_realsense']

    assert parameters['enable_color'] is True
    assert parameters['enable_depth'] is True
    assert parameters['align_depth.enable'] is True
    assert parameters['enable_sync'] is True
    assert not any(key.endswith('profile') for key in parameters)


def test_clearpath_parser_retains_alignment_and_sync_parameters() -> None:
    clearpath_config = pytest.importorskip('clearpath_config.sensors.types.cameras')
    repo_root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load(
        (repo_root / 'clearpath' / 'robot.yaml').read_text(encoding='utf-8')
    )
    source = config['sensors']['camera'][0]['ros_parameters']

    generated = clearpath_config.IntelRealsense(ros_parameters=source).ros_parameters
    parameters = generated['intel_realsense']
    assert parameters['align_depth.enable'] is True
    assert parameters['enable_sync'] is True
