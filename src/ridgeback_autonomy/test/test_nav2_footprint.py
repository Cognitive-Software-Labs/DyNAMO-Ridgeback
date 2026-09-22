from pathlib import Path

import pytest
import yaml


PARAMS = Path(__file__).resolve().parents[1] / 'config/nav2_params.yaml'


def test_local_and_global_costmaps_share_pinned_footprint_clearance():
    params = yaml.safe_load(PARAMS.read_text())
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']

    assert yaml.safe_load(local['footprint']) == yaml.safe_load(global_['footprint'])
    assert local['footprint_padding'] == pytest.approx(0.01)
    assert global_['footprint_padding'] == pytest.approx(0.01)
