"""Node entry points must survive the shutdown rclpy's own SIGINT handler performs.

rclpy's default handler shuts the default context down before ``spin`` returns,
so an unconditional ``rclpy.shutdown()`` in ``finally`` raises
``rcl_shutdown already called`` and turns a clean Ctrl-C into exit 1.
"""

from __future__ import annotations

import ast
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SOURCE_ROOT = REPO_ROOT / 'src'

STARTUP_SEC = 3.0
EXIT_TIMEOUT_SEC = 10.0

ENTRY_POINTS = {
    'velocity_overlay': 'ridgeback_autonomy.diagnostics.velocity_overlay_node',
    'coverage_overlay': 'ridgeback_autonomy.diagnostics.coverage_overlay_node',
    'localization_overlay': 'ridgeback_autonomy.diagnostics.localization_overlay_node',
    'hud': 'ridgeback_autonomy.diagnostics.hud_node',
    'localization_health': 'ridgeback_localization.health_node',
    'target_overlay': 'ridgeback_localization.overlay_node',
    'target_visualization': 'ridgeback_localization.visualization_node',
    'frontier_explorer': 'ridgeback_autonomy.frontier_explorer.frontier_explorer_node',
}


def _is_rclpy_call(node: ast.AST, name: str) -> bool:
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == name
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == 'rclpy'
    )


def _unguarded_shutdowns(path: Path) -> list[int]:
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    parents: dict[ast.AST, ast.AST] = {}
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            parents[child] = parent

    def guarded(node: ast.AST) -> bool:
        child = node
        parent = parents.get(child)
        while parent is not None:
            if (isinstance(parent, ast.If) and _is_rclpy_call(parent.test, 'ok')
                    and child in parent.body):
                return True
            child, parent = parent, parents.get(parent)
        return False

    return [
        node.lineno for node in ast.walk(tree)
        if _is_rclpy_call(node, 'shutdown') and not guarded(node)
    ]


def test_every_rclpy_shutdown_tolerates_an_already_shut_context() -> None:
    offenders = []
    for package in sorted(SOURCE_ROOT.glob('ridgeback_*')):
        for path in sorted(package.rglob('*.py')):
            relative = path.relative_to(SOURCE_ROOT)
            if 'test' in relative.parts:
                continue
            offenders.extend(f'{relative}:{line}' for line in _unguarded_shutdowns(path))
    assert offenders == [], (
        'call rclpy.try_shutdown() or guard rclpy.shutdown() with rclpy.ok(): '
        + ', '.join(offenders))


@pytest.fixture(scope='module')
def interrupted_entry_points(tmp_path_factory):
    """Start every entry point together, SIGINT each once, collect exit and output."""
    root = tmp_path_factory.mktemp('node_shutdown')
    running = {}
    for offset, (name, module) in enumerate(ENTRY_POINTS.items()):
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(180 + offset)
        env['ROS_LOG_DIR'] = str(root / f'{name}_ros_logs')
        log = (root / f'{name}.log').open('wb')
        process = subprocess.Popen(
            [sys.executable, '-c', f'from {module} import main; main()'],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        running[name] = (process, log)

    time.sleep(STARTUP_SEC)
    alive = {name: process.poll() is None for name, (process, _) in running.items()}
    for name, (process, _) in running.items():
        if alive[name]:
            os.kill(process.pid, signal.SIGINT)

    results = {}
    for name, (process, log) in running.items():
        try:
            returncode = process.wait(timeout=EXIT_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            returncode = process.wait()
        log.close()
        output = (root / f'{name}.log').read_text(encoding='utf-8', errors='replace')
        results[name] = (alive[name], returncode, output)
    return results


@pytest.mark.parametrize('name', sorted(ENTRY_POINTS))
def test_one_sigint_exits_cleanly(interrupted_entry_points, name: str) -> None:
    alive, returncode, output = interrupted_entry_points[name]
    assert alive, f'{name} exited before the signal:\n{output}'
    assert returncode == 0, output
    assert 'Traceback' not in output, output
