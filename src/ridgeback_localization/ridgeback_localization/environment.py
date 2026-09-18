"""Apply optional inference Python environment only to compute processes."""
import os
from pathlib import Path
from ridgeback_common.paths import workspace_root


def compute_environment() -> dict[str, str]:
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
