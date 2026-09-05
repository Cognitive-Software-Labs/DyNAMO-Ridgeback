"""Small subprocess/provenance helpers shared by benchmark executables."""

from __future__ import annotations

import json
import subprocess
from typing import Any


GIT_TIMEOUT_SEC = 5.0
GLXINFO_TIMEOUT_SEC = 10.0
# Mesa's CPU rasterizers. A remote display session (xrdp, VNC, Xvfb) has no
# GPU attached to its X server and resolves GL to one of these.
SOFTWARE_GL_RENDERERS = ('llvmpipe', 'softpipe', 'swrast', 'lavapipe')


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


def parse_gl_renderer(text: str) -> dict[str, Any]:
    """Pull vendor/renderer out of ``glxinfo`` output and classify it.

    Split from the subprocess call so the classification can be asserted
    without a GL stack present.
    """

    vendor = renderer = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('OpenGL vendor string:'):
            vendor = stripped.split(':', 1)[1].strip()
        elif stripped.startswith('OpenGL renderer string:'):
            renderer = stripped.split(':', 1)[1].strip()
    software = None
    if renderer is not None:
        lowered = renderer.lower()
        software = any(name in lowered for name in SOFTWARE_GL_RENDERERS)
    return {'vendor': vendor, 'renderer': renderer, 'software': software}


def gl_renderer_provenance() -> dict[str, Any]:
    """Best-effort GL renderer, and whether it is a CPU rasterizer.

    This belongs beside the commit in a benchmark's provenance because it
    silently decides whether the result is usable. A remote display session
    resolves GL to Mesa software, and the simulator then rasterizes every
    camera and depth frame on the CPU: those sensors collapse to a fraction of
    their configured rate while physics, sim time and therefore **RTF stay
    normal**. Nothing else in the recorded metrics reveals it, so a run can
    look healthy and still carry cadence and coverage numbers that are wrong.
    """

    try:
        result = subprocess.run(
            ['glxinfo', '-B'],
            check=False,
            capture_output=True,
            text=True,
            timeout=GLXINFO_TIMEOUT_SEC,
        )
    except (OSError, subprocess.SubprocessError):
        # glxinfo absent (mesa-utils not installed) or DISPLAY unreachable.
        # Unknown is reported as unknown; it is not evidence of either state.
        return {'vendor': None, 'renderer': None, 'software': None}
    if result.returncode != 0:
        return {'vendor': None, 'renderer': None, 'software': None}
    return parse_gl_renderer(result.stdout)


def software_gl_warning(gl: dict[str, Any]) -> str | None:
    """The operator-facing warning for a software-rasterized run, or ``None``."""

    if not gl.get('software'):
        return None
    renderer = gl.get('renderer') or 'unknown'
    return (
        f'GL renders in software ("{renderer}"). Gazebo will rasterize every '
        'camera and depth frame on the CPU, so those sensors can run at a '
        'fraction of their configured rate. Sim time and RTF stay normal, so '
        'nothing else in this run will reveal it, and any cadence or coverage '
        'number it produces may be invalid. Fix with '
        '"export __GLX_VENDOR_LIBRARY_NAME=nvidia" before launching, then '
        'confirm with "glxinfo -B | grep renderer".'
    )


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
