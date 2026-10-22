"""tools/check_dependencies against a throwaway workspace of fixture checkouts.

The checker resolves its manifests and patches from ``git rev-parse
--show-toplevel``, so each test copies it into a temporary root repository
whose ``src/`` holds small nested repositories at known commits.
"""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
CHECKER = REPO_ROOT / 'tools/check_dependencies'


def _git(cwd: Path, *args: str) -> str:
    return subprocess.run(
        ['git', '-c', 'user.name=fixture', '-c', 'user.email=fixture@example.invalid',
         '-c', 'commit.gpgsign=false', *args],
        cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def _package(root: Path, name: str) -> None:
    (root / name).mkdir(parents=True)
    (root / name / 'package.xml').write_text(f'<package><name>{name}</name></package>\n')
    (root / name / 'source.txt').write_text('upstream\n')


def _nested_repository(path: Path, packages: tuple[str, ...], ignored: tuple[str, ...] = ()):
    path.mkdir(parents=True)
    _git(path, 'init', '-q')
    for package in packages:
        _package(path, package)
    for package in ignored:
        (path / package / 'COLCON_IGNORE').write_text('')
    _git(path, 'add', '-A')
    _git(path, 'commit', '-q', '-m', 'upstream')
    return _git(path, 'rev-parse', 'HEAD')


@pytest.fixture
def workspace(tmp_path):
    """A root repository with one partial overlay and one plain dependency."""
    root = tmp_path / 'workspace'
    (root / 'tools').mkdir(parents=True)
    _git(root, 'init', '-q')
    overlay = _nested_repository(
        root / 'src/overlay', ('pkg_kept', 'pkg_other', 'pkg_upstream_ignored'),
        ignored=('pkg_upstream_ignored',))
    plain = _nested_repository(root / 'src/plain', ('pkg_plain',))

    # The recorded patch, generated the way the runbook prescribes.
    source = root / 'src/overlay/pkg_kept/source.txt'
    source.write_text('patched\n')
    (root / 'patches').mkdir()
    (root / 'patches/overlay.patch').write_text(
        _git(root / 'src/overlay', 'diff', '--no-ext-diff', 'HEAD') + '\n')
    _git(root / 'src/overlay', 'checkout', '-q', '--', '.')

    (root / 'dependencies').mkdir()
    (root / 'dependencies/core.repos').write_text(
        f'repositories:\n  src/plain:\n    type: git\n    url: x\n    version: {plain}\n')
    (root / 'dependencies/gz.repos').write_text(
        f'repositories:\n  src/overlay:\n    type: git\n    url: x\n    version: {overlay}\n')

    text = CHECKER.read_text(encoding='utf-8')
    for old, new in (
        ('declare -A BUILD_ONLY=()', 'declare -A BUILD_ONLY=([src/overlay]="pkg_kept")'),
        ('declare -A PATCHES=(', 'declare -A PATCHES=(\n    [src/overlay]="patches/overlay.patch"'),
    ):
        assert text.count(old) == 1
        text = text.replace(old, new)
    (root / 'tools/check_dependencies').write_text(text)
    (root / 'tools/check_dependencies').chmod(0o755)
    return root


def _check(root: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ['bash', str(root / 'tools/check_dependencies'), *args],
        cwd=root, capture_output=True, text=True)


def test_missing_markers_fail_read_only_and_apply_creates_exactly_them(workspace) -> None:
    first = _check(workspace, '--profile', 'all')
    assert first.returncode == 1
    assert 'src/overlay is missing 1 COLCON_IGNORE marker(s); rerun with --apply' in first.stderr

    applied = _check(workspace, '--apply', '--profile', 'all')
    assert applied.returncode == 0, applied.stderr
    assert 'MARKED: 1 package(s) of src/overlay skipped by colcon' in applied.stdout
    assert 'APPLIED: patches/overlay.patch to src/overlay' in applied.stdout
    overlay = workspace / 'src/overlay'
    assert (overlay / 'pkg_other/COLCON_IGNORE').read_bytes() == b''
    assert not (overlay / 'pkg_kept/COLCON_IGNORE').exists()

    rerun = _check(workspace, '--profile', 'all')
    assert rerun.returncode == 0, rerun.stderr
    assert 'MARKED' not in rerun.stdout
    assert 'OK: src/overlay' in rerun.stdout and '(patches/overlay.patch applied)' in rerun.stdout


def test_a_stray_untracked_file_in_an_overlay_fails(workspace) -> None:
    assert _check(workspace, '--apply', '--profile', 'all').returncode == 0
    (workspace / 'src/overlay/pkg_kept/notes.txt').write_text('local\n')

    for args in (('--profile', 'all'), ('--apply', '--profile', 'all')):
        result = _check(workspace, *args)
        assert result.returncode == 1
        assert 'src/overlay has untracked files: pkg_kept/notes.txt' in result.stderr


@pytest.mark.parametrize('marker', ['COLCON_IGNORE', 'AMENT_IGNORE'])
def test_a_kept_package_that_is_marked_ignored_fails(workspace, marker) -> None:
    assert _check(workspace, '--apply', '--profile', 'all').returncode == 0
    (workspace / f'src/overlay/pkg_kept/{marker}').write_text('')

    result = _check(workspace, '--apply', '--profile', 'all')
    assert result.returncode == 1
    assert f'src/overlay/pkg_kept must be built but contains {marker}' in result.stderr


def test_a_kept_package_that_does_not_exist_fails(workspace) -> None:
    checker = workspace / 'tools/check_dependencies'
    checker.write_text(checker.read_text().replace(
        '[src/overlay]="pkg_kept"', '[src/overlay]="pkg_kept pkg_absent"'))

    result = _check(workspace, '--apply', '--profile', 'gz')
    assert result.returncode == 1
    assert 'src/overlay has no package pkg_absent to build' in result.stderr


def test_dependencies_without_an_overlay_entry_keep_the_strict_untracked_rule(
        workspace) -> None:
    (workspace / 'src/plain/pkg_plain/COLCON_IGNORE').write_text('')

    result = _check(workspace, '--profile', 'core')
    assert result.returncode == 1
    assert 'ERROR: src/plain has untracked files: pkg_plain/COLCON_IGNORE' in result.stderr
