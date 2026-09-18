"""Locate checkout resources without assuming a colcon install directory depth."""
import os
from pathlib import Path


def workspace_root(hint: str = '') -> Path:
    explicit = os.environ.get('RIDGEBACK_WORKSPACE')
    if explicit:
        return Path(explicit).expanduser().resolve()
    for origin in (Path(hint), Path(__file__).resolve(), Path.cwd()):
        for candidate in (origin, *origin.parents):
            if (candidate / 'clearpath/robot.yaml').is_file() and (candidate / 'src').is_dir():
                return candidate
    raise RuntimeError('Set RIDGEBACK_WORKSPACE to the repository checkout for workspace resources.')
