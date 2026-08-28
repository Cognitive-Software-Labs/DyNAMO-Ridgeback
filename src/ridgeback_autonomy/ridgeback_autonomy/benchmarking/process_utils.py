"""Small subprocess/provenance helpers shared by benchmark executables."""

from __future__ import annotations

import json
import subprocess
from typing import Any


GIT_TIMEOUT_SEC = 5.0


def extract_json_payload(text: str) -> dict[str, Any]:
    """Extract the first JSON object from command output with leading noise."""

    decoder = json.JSONDecoder()
    search_from = 0
    while True:
        start = text.find('{', search_from)
        if start == -1:
            raise RuntimeError(f'Failed to parse Gazebo JSON payload: {text.strip()}')
        try:
            payload, _ = decoder.raw_decode(text[start:])
        except json.JSONDecodeError:
            search_from = start + 1
            continue
        if isinstance(payload, dict):
            return payload
        search_from = start + 1


def try_command(
    command: list[str],
    *,
    timeout_sec: float,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    """Run a command and turn timeouts into the benchmark's RuntimeError form."""

    try:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            env=env,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(
            f'Command timed out after {timeout_sec:.1f}s: {" ".join(command)}'
        ) from exc


def run_command(
    command: list[str],
    *,
    timeout_sec: float,
    description: str,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    """Run a command and require a successful exit."""

    result = try_command(command, timeout_sec=timeout_sec, env=env)
    if result.returncode != 0:
        raise RuntimeError(
            f'Failed to {description}: exit={result.returncode} '
            f'stdout="{result.stdout.strip()}" stderr="{result.stderr.strip()}"'
        )
    return result


def git_provenance(repo_dir: str) -> dict[str, Any]:
    """Best-effort commit, branch, and dirty count for a repository."""

    def capture(args: list[str]) -> str | None:
        try:
            result = subprocess.run(
                ['git', '-C', repo_dir] + args,
                check=False,
                capture_output=True,
                text=True,
                timeout=GIT_TIMEOUT_SEC,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        return result.stdout.strip() if result.returncode == 0 else None

    status = capture(['status', '--porcelain'])
    return {
        'commit': capture(['rev-parse', '--short', 'HEAD']),
        'branch': capture(['rev-parse', '--abbrev-ref', 'HEAD']),
        'dirty_count': (
            None if status is None
            else len([line for line in status.splitlines() if line.strip()])
        ),
    }


def format_commit(provenance: dict[str, Any]) -> str:
    """One line naming the code state that produced a result."""

    commit = provenance.get('commit')
    if commit is None:
        return 'unknown'
    dirty_count = provenance.get('dirty_count')
    if dirty_count is None:
        return f'{commit} (dirty state unknown)'
    if dirty_count:
        return (f'{commit} + {dirty_count} uncommitted file(s) — NOT reproducible '
                f'from this commit alone')
    return f'{commit} (clean)'
