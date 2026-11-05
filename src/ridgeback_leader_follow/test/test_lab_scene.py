"""Asset integrity and continuous demonstration paths, independent of Kit."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from ridgeback_leader_follow.lab_scene import demo_scenario
from ridgeback_leader_follow.crowd_runner import main


def test_demo_paths_keep_humans_apart_and_never_teleport():
    scenario = demo_scenario(2)
    assert scenario.duration == 60
    a, b = scenario.people
    for t in np.arange(0., 60., .025):
        ax, ay, _ = a.sample(float(t))
        bx, by, _ = b.sample(float(t))
        assert np.hypot(ax-bx, ay-by) > .7
    for person in scenario.people:
        assert person.sample(0)[:2] == person.sample(30)[:2] == person.sample(60)[:2]
        assert person.distance(30) > 0
    with pytest.raises(ValueError):
        demo_scenario(121)


def test_lab_profile_rejects_unsupported_backend_before_launch():
    with pytest.raises(SystemExit):
        main(['--backend', 'gz', '--scene-profile', 'lab'])


def test_pedestrian_bake_has_valid_materials_and_receipted_files():
    directory = Path(__file__).resolve().parents[1] / 'sim/assets/maya'
    receipt = json.loads((directory / 'conversion.json').read_text())
    for name, digest in receipt['sha256'].items():
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == digest
    data = np.load(directory / 'walk.npz')
    assert data['points'].shape == (48, receipt['vertices'], 3)
    assert np.isfinite(data['points']).all()
    assert data['faces'].min() >= 0
    assert data['faces'].max() < receipt['vertices']
    assert data['uv'].shape == (receipt['faces']*3, 2)
    assert data['materials'].shape == (receipt['faces'],)
    assert data['points'][:, :, 2].min() == pytest.approx(0)
    assert data['points'][:, :, 2].max() == pytest.approx(1.72)
