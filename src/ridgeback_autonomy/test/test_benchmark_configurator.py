"""Focused contract and safety tests for the local benchmark configurator."""

from __future__ import annotations

import http.client
import json
from pathlib import Path
import threading

import pytest

from ridgeback_autonomy.benchmarking.configurator import (
    ConfiguratorError,
    capabilities,
    import_job,
    make_server,
    render_job,
    validate_job,
)
from ridgeback_autonomy.benchmarking.replay import ReplayDatasetWriter
from ridgeback_autonomy.benchmarking.replay_jobs import parse_job


def _dataset(tmp_path: Path) -> Path:
    root = tmp_path / 'replay'
    writer = ReplayDatasetWriter(root, {'trials_included': 1})
    writer.write_trial(
        {'trial_id': 'trial', 'scene_id': 'scene', 'ground_truth': []},
        [{'stamp_ns': 1, 'detected': False, 'count': 0, 'image_width': 1,
          'image_height': 1, 'detections': []}],
    )
    writer.finalize(trials_included=1, trials_skipped=0)
    return root


def _job(dataset: Path) -> dict:
    return {
        'question': 'measurement', 'profile': 'measurement',
        'inputs': {'dataset': str(dataset)}, 'repeats': 1, 'workers': 1,
        'variants': [
            {'name': 'baseline', 'arguments': {}},
            {'name': 'wide_band', 'arguments': {'isolation_2d_band_m': 0.5}},
        ], 'baseline': 'baseline',
    }


def test_capabilities_expose_all_four_canonical_profiles():
    data = capabilities()
    assert [item['id'] for item in data['questions']] == [
        'tune-measurement', 'compare-mask-outputs', 'try-mask-model', 'measure-live-system']
    assert data['profiles']['measurement']['available'] is True
    assert data['profiles']['mask-output']['available'] is True
    assert data['profiles']['mask-model']['available'] is True


def test_rendered_measurement_job_is_accepted_by_the_canonical_job_parser(tmp_path):
    rendered = render_job(_job(_dataset(tmp_path)))
    assert rendered['valid'] is True
    assert 'target_replay_benchmark' in rendered['command']
    assert parse_job(__import__('yaml').safe_load(rendered['yaml'])).profile == 'measurement'


def test_import_export_round_trip_is_stable(tmp_path):
    first = render_job(_job(_dataset(tmp_path)))
    imported = import_job(first['yaml'])
    second = render_job(imported['job'])
    assert second['yaml'] == first['yaml']


def test_mask_model_is_enabled_and_reports_its_required_sensor_input():
    result = validate_job({'profile': 'mask-model', 'variants': [{'name': 'baseline'}]})
    assert result['valid'] is False
    assert result['issues'][0]['field'] == 'inputs.sensor_capture'


def test_canonical_job_import_round_trips(tmp_path):
    rendered = render_job(_job(_dataset(tmp_path)))
    imported = import_job(rendered['yaml'])
    assert imported['job']['profile'] == 'measurement'
    assert imported['valid'] is True


def test_server_requires_token_and_serves_assets_through_a_symlink(tmp_path, monkeypatch):
    import ridgeback_autonomy.benchmarking.configurator as configurator

    assets_link = tmp_path / 'installed-assets'
    assets_link.symlink_to(
        Path(configurator.__file__).with_name('configurator_assets'), target_is_directory=True)
    monkeypatch.setattr(configurator, '_assets_directory', lambda: assets_link)
    server = make_server(token='test-token', workspace_root=tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address[:2]
        connection = http.client.HTTPConnection(host, port)
        connection.request('GET', '/api/capabilities')
        assert connection.getresponse().status == 403
        connection.close()

        connection = http.client.HTTPConnection(host, port)
        connection.request('GET', '/')
        response = connection.getresponse()
        assert response.status == 200
        assert response.getheader('Content-Type') == 'text/html; charset=utf-8'
        assert b'Configure a<br><em>reproducible</em> benchmark.' in response.read()
        connection.close()

        connection = http.client.HTTPConnection(host, port)
        connection.request('GET', '/api/capabilities', headers={'X-Configurator-Token': 'test-token'})
        response = connection.getresponse()
        assert response.status == 200
        assert json.loads(response.read())['profiles']['live-system']['available'] is True
        connection.close()
        assert list(tmp_path.iterdir()) == [assets_link]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
