"""Apply optional inference Python environment only to compute processes.

Compute processes load models from the local Hugging Face cache by default.
Without this, a robot whose Wi-Fi drops cannot start perception at all: the
loaders retry huggingface.co for every file even when every model is cached
(r100_0160's Thor, 2026-10-16). Set RIDGEBACK_MODEL_DOWNLOADS=1 for a first
run that must download models; an explicit HF_HUB_OFFLINE from the caller wins.
"""
import os
from pathlib import Path
from ridgeback_common.paths import workspace_root


def model_cache_environment() -> dict[str, str]:
    if os.environ.get('RIDGEBACK_MODEL_DOWNLOADS') == '1' or 'HF_HUB_OFFLINE' in os.environ:
        return {}
    return {'HF_HUB_OFFLINE': '1', 'TRANSFORMERS_OFFLINE': '1'}


def compute_environment() -> dict[str, str]:
    return {**model_cache_environment(), **_venv_environment()}


def _venv_environment() -> dict[str, str]:
    selected = os.environ.get('RIDGEBACK_PERCEPTION_VENV')
    if selected:
        venv = Path(selected).expanduser().resolve()
        if not (venv / 'bin/python3').is_file():
            raise ValueError(f'Invalid RIDGEBACK_PERCEPTION_VENV: {venv}')
    else:
        try:
            venv = workspace_root() / 'perception_venv'
        except RuntimeError:
            return {}
        if not (venv / 'bin/python3').is_file():
            return {}
    return {'VIRTUAL_ENV': str(venv), 'PATH': str(venv / 'bin') + os.pathsep + os.environ.get('PATH', '')}
