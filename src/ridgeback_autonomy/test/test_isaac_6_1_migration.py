from __future__ import annotations

from pathlib import Path
import subprocess


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def test_isaac_requirement_is_pinned_to_6_1_ga() -> None:
    requirement = (_repo_root() / 'requirements-isaac.txt').read_text(encoding='utf-8')

    assert 'isaacsim[all,extscache]==6.1.0.0' in requirement
    assert 'isaacsim[all,extscache]==6.0.1' not in requirement


def test_installer_exposes_side_by_side_candidate_path() -> None:
    result = subprocess.run(
        ['bash', 'tools/install_isaac_venv.sh', '--help'],
        cwd=_repo_root(),
        check=True,
        capture_output=True,
        text=True,
    )

    assert '--venv <path>' in result.stdout
    assert '--warmup' in result.stdout


def test_smoke_gate_requires_exact_6_1_version_and_clock_delivery() -> None:
    smoke = (_repo_root() / 'tools/isaac/smoke_test.py').read_text(encoding='utf-8')

    assert 'default="6.1.0.0"' in smoke
    assert 'installed_version != args.expected_version' in smoke
    assert 'no /clock messages received over system ROS 2 DDS' in smoke
