"""Import the runner's modules the way the runner does: from ``sim/isaac``."""

from pathlib import Path
import sys


RUNNER_DIR = Path(__file__).resolve().parents[1] / 'sim' / 'isaac'
sys.path.insert(0, str(RUNNER_DIR))
