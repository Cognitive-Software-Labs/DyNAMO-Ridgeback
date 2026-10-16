"""cleanup.sh ownership, on harmless sentinel processes and a dry run.

Each sentinel is a ``sleep`` whose command line imitates one case. The dry run
never kills, so this is safe on any host, including the robot.
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def checkout(tmp_path):
    root = tmp_path / 'checkout'
    root.mkdir()
    shutil.copy(REPO_ROOT / 'cleanup.sh', root / 'cleanup.sh')
    return root


@pytest.fixture
def sentinels():
    started = []

    def start(argv0, ament_prefix_path=None):
        env = {key: value for key, value in os.environ.items() if key != 'AMENT_PREFIX_PATH'}
        if ament_prefix_path is not None:
            env['AMENT_PREFIX_PATH'] = ament_prefix_path
        process = subprocess.Popen(['bash', '-c', 'exec -a "$0" sleep 120', argv0], env=env)
        started.append(process)
        return process.pid

    yield start
    for process in started:
        process.kill()
        process.wait()


def dry_run_targets(script, parent_argv0=None):
    """PIDs a dry run would kill, optionally run two levels below a renamed shell.

    Two levels, so the renamed process is a grandparent: $PPID alone would
    already protect a direct parent.
    """

    env = dict(os.environ, CLEANUP_DRY_RUN='1')
    time.sleep(0.3)  # let every sentinel finish its exec
    if parent_argv0 is None:
        argv = ['bash', str(script)]
    else:
        inner = f'bash -c "bash {script}; true"; true'
        argv = ['bash', '-c', f'exec -a "$0" bash -c \'{inner}\'', parent_argv0]
    output = subprocess.run(argv, env=env, capture_output=True, text=True, check=True).stdout
    assert 'Dry run: nothing was killed' in output
    targets = {int(m) for m in re.findall(r'^would kill (\d+):', output, re.M)}
    targets |= {int(line.split()[1]) for line in output.splitlines()
                if re.match(r'^\S+\s+\d+\s', line)}
    return targets, output


def test_stale_nodes_of_this_checkout_are_targeted(checkout, sentinels):
    own = f'{checkout}/install/ridgeback_autonomy:/opt/ros/jazzy'
    stale = sentinels(f'{checkout}/install/ridgeback_autonomy/lib/ridgeback_autonomy/stale_node', own)
    bare_rviz = sentinels('/opt/ros/jazzy/lib/rviz2/rviz2', '/opt/ros/jazzy')
    targets, output = dry_run_targets(checkout / 'cleanup.sh')
    assert {stale, bare_rviz} <= targets, output


def test_other_checkouts_configurators_and_the_gui_are_preserved(checkout, sentinels, tmp_path):
    other = tmp_path / 'other'
    own = f'{checkout}/install/ridgeback_autonomy:/opt/ros/jazzy'
    preserved = {
        # The old lib/ridgeback_autonomy/ catch-all reached into other worktrees.
        sentinels(f'{other}/install/ridgeback_autonomy/lib/ridgeback_autonomy/node',
                  f'{other}/install/ridgeback_autonomy:/opt/ros/jazzy'),
        # A robot-style service built from its own overlay.
        sentinels('/opt/ros/jazzy/lib/robot_localization/ekf_node',
                  '/opt/vendor/install/pkg:/opt/ros/jazzy'),
        # The page and a replay it started, whether or not they are ancestors.
        sentinels(f'{checkout}/install/ridgeback_autonomy/lib/ridgeback_autonomy/'
                  'target_benchmark_configurator', own),
        sentinels('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 run ridgeback_autonomy '
                  'target_benchmark_configurator', own),
        sentinels(f'/usr/bin/python3 {checkout}/install/ridgeback_autonomy/lib/ridgeback_autonomy/'
                  'target_replay_benchmark --job j.yaml', own),
        sentinels('gz sim gui', own),
        # `explore` names a node executable, not every path containing it.
        sentinels('less notes/explore.md', '/opt/ros/jazzy'),
    }
    targets, output = dry_run_targets(checkout / 'cleanup.sh')
    assert not preserved & targets, output


def test_the_calling_chain_is_never_targeted(checkout, sentinels):
    """A configurator-started sweep must not kill the configurator above it."""

    own = f'{checkout}/install/ridgeback_autonomy:/opt/ros/jazzy'
    sibling = sentinels(f'{checkout}/install/ridgeback_autonomy/lib/ridgeback_autonomy/stale_node', own)
    parent = f'{checkout}/install/ridgeback_autonomy/lib/ridgeback_autonomy/target_benchmark_sweep'
    targets, output = dry_run_targets(checkout / 'cleanup.sh', parent_argv0=parent)
    assert sibling in targets, output
    assert parent not in output, output


def test_a_live_run_refuses_while_the_platform_service_is_active(checkout, tmp_path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    # systemctl reports "active"; pgrep and ps find nothing, so even a broken
    # guard could not reach a real process from this test.
    for name, body in (('systemctl', 'exit 0'), ('pgrep', 'exit 1'), ('ps', 'exit 0')):
        (fake_bin / name).write_text(f'#!/bin/sh\n{body}\n')
        (fake_bin / name).chmod(0o755)
    env = dict(os.environ, PATH=f'{fake_bin}:{os.environ["PATH"]}')
    env.pop('CLEANUP_DRY_RUN', None)
    result = subprocess.run(['bash', str(checkout / 'cleanup.sh')], env=env,
                            capture_output=True, text=True)
    assert result.returncode == 1
    assert 'clearpath-platform.service is active' in result.stderr
