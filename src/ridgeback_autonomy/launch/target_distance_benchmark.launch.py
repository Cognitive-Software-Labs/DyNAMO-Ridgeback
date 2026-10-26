"""Public single-run entrypoint with ordered configuration/environment shutdown."""

import json
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ridgeback_autonomy.benchmarking.target_benchmark_sweep import (
    PROCESS_GROUP_STOP_BUDGET_SEC,
    PROCESS_POLL_SEC,
    _launch_command,
)


CONFIG_LAUNCH = 'target_benchmark_config.launch.py'
ENVIRONMENT_LAUNCH = 'target_benchmark_env.launch.py'


def layer_arguments():
    """Read declarations without evaluating their substitution-backed defaults."""
    launch_dir = os.path.join(get_package_share_directory('ridgeback_autonomy'), 'launch')
    return {
        filename: PythonLaunchDescriptionSource(os.path.join(launch_dir, filename))
        .get_launch_description(LaunchContext()).get_launch_arguments()
        for filename in (CONFIG_LAUNCH, ENVIRONMENT_LAUNCH)
    }


def build_supervisor(context, *, layers):
    commands = {
        filename: _launch_command(filename, {
            argument.name: LaunchConfiguration(argument.name).perform(context)
            for argument in arguments
        })
        for filename, arguments in layers.items()
    }
    supervisor = ExecuteProcess(
        cmd=[sys.executable, '-m', 'ridgeback_autonomy.benchmarking.single_run',
             '--environment-command', json.dumps(commands[ENVIRONMENT_LAUNCH]),
             '--configuration-command', json.dumps(commands[CONFIG_LAUNCH])],
        name='benchmark_supervisor', output='screen',
        # Both groups may consume the full shared shutdown budget. The outer
        # launch must keep the supervisor alive throughout their ordered stop.
        sigterm_timeout=str(2 * PROCESS_GROUP_STOP_BUDGET_SEC + PROCESS_POLL_SEC),
        sigkill_timeout=str(PROCESS_GROUP_STOP_BUDGET_SEC),
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=supervisor, on_exit=supervisor_exited)),
        supervisor,
    ]


def supervisor_exited(event, _context):
    """Propagate lifecycle failures through the public ros2 launch caller."""
    if event.returncode != 0:
        raise RuntimeError(f'Benchmark supervisor exited with code {event.returncode}')


def generate_launch_description():
    layers = layer_arguments()
    merged = {}
    for arguments in layers.values():
        for argument in arguments:
            merged.setdefault(argument.name, argument)
    return LaunchDescription([
        *merged.values(),
        OpaqueFunction(function=build_supervisor, kwargs={'layers': layers}),
    ])
