"""Compute-process environment: cached models by default, explicit opt-in to downloads."""
from ridgeback_localization import environment


def test_models_load_from_the_cache_by_default(monkeypatch):
    monkeypatch.delenv('RIDGEBACK_MODEL_DOWNLOADS', raising=False)
    monkeypatch.delenv('HF_HUB_OFFLINE', raising=False)
    monkeypatch.delenv('RIDGEBACK_PERCEPTION_VENV', raising=False)
    env = environment.compute_environment()
    assert env['HF_HUB_OFFLINE'] == '1' and env['TRANSFORMERS_OFFLINE'] == '1'


def test_downloads_can_be_enabled_for_a_first_run(monkeypatch):
    monkeypatch.setenv('RIDGEBACK_MODEL_DOWNLOADS', '1')
    monkeypatch.delenv('HF_HUB_OFFLINE', raising=False)
    assert 'HF_HUB_OFFLINE' not in environment.compute_environment()


def test_an_explicit_caller_setting_wins(monkeypatch):
    monkeypatch.delenv('RIDGEBACK_MODEL_DOWNLOADS', raising=False)
    monkeypatch.setenv('HF_HUB_OFFLINE', '0')
    assert 'HF_HUB_OFFLINE' not in environment.compute_environment()


def test_the_venv_is_still_applied(monkeypatch, tmp_path):
    (tmp_path / 'bin').mkdir()
    (tmp_path / 'bin/python3').write_text('')
    monkeypatch.setenv('RIDGEBACK_PERCEPTION_VENV', str(tmp_path))
    env = environment.compute_environment()
    assert env['VIRTUAL_ENV'] == str(tmp_path.resolve()) and env['HF_HUB_OFFLINE'] == '1'
