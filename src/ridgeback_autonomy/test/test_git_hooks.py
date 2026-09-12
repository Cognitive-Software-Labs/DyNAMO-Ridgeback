from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[3]
GRAPH_PATHS = (
    'graphify-out/GRAPH_REPORT.md',
    'graphify-out/graph.html',
    'graphify-out/graph.json',
)


def _git(repo: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ('git', *args), cwd=repo, check=True, text=True,
        capture_output=True,
    )


def _make_repo(tmp_path: Path, rebuild_exit: int = 0) -> Path:
    repo = tmp_path / 'repo'
    (repo / 'tools/hooks').mkdir(parents=True)
    (repo / 'graphify-out').mkdir()
    shutil.copy(REPO_ROOT / 'tools/hooks/pre-commit', repo / 'tools/hooks/pre-commit')
    rebuild = repo / 'tools/rebuild_graphify'
    rebuild.write_text(
        '#!/bin/bash\n'
        f'exit_code={rebuild_exit}\n'
        'if [ "$exit_code" -ne 0 ]; then exit "$exit_code"; fi\n'
        'printf rebuilt > graphify-out/GRAPH_REPORT.md\n'
        'printf rebuilt > graphify-out/graph.html\n'
        'printf rebuilt > graphify-out/graph.json\n',
        encoding='utf-8',
    )
    rebuild.chmod(0o755)
    _git(repo, 'init', '-q')
    _git(repo, 'config', 'user.email', 'test@example.com')
    _git(repo, 'config', 'user.name', 'Test User')
    (repo / 'module.py').write_text('value = 1\n', encoding='utf-8')
    for graph_path in GRAPH_PATHS:
        (repo / graph_path).write_text('old\n', encoding='utf-8')
    _git(repo, 'add', '.')
    _git(repo, '-c', 'core.hooksPath=/dev/null', 'commit', '-qm', 'initial')
    return repo


def test_pre_commit_rebuilds_and_stages_graph_for_staged_code(tmp_path: Path) -> None:
    repo = _make_repo(tmp_path)
    (repo / 'module.py').write_text('value = 2\n', encoding='utf-8')
    _git(repo, 'add', 'module.py')

    subprocess.run(
        ('bash', 'tools/hooks/pre-commit'), cwd=repo, check=True,
        text=True, capture_output=True,
    )

    staged = set(_git(repo, 'diff', '--cached', '--name-only').stdout.splitlines())
    assert staged == {'module.py', *GRAPH_PATHS}


def test_pre_commit_propagates_graph_rebuild_failure(tmp_path: Path) -> None:
    repo = _make_repo(tmp_path, rebuild_exit=7)
    (repo / 'module.py').write_text('value = 2\n', encoding='utf-8')
    _git(repo, 'add', 'module.py')

    result = subprocess.run(
        ('bash', 'tools/hooks/pre-commit'), cwd=repo, check=False,
        text=True, capture_output=True,
    )

    assert result.returncode == 7
    assert _git(repo, 'diff', '--cached', '--name-only').stdout.splitlines() == ['module.py']
