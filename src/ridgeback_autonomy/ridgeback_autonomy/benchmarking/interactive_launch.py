"""Run a package's launch file the way ``ros2 launch`` does from a terminal.

Usage: ``python3 -m ridgeback_autonomy.benchmarking.interactive_launch
<package> <launch file> [name:=value ...]``

``ros2 launch`` decides how to stop its children from whether stdin is a
terminal. From a terminal, Ctrl-C has already reached every process in the
foreground group, so launch forwards nothing. Without one -- a supervisor's
``stdin=DEVNULL`` -- it sends each child its own SIGINT. The benchmark sweep
signals the whole process group itself, exactly as a terminal does, so under
plain ``ros2 launch`` every child received SIGINT twice; the second one lands
in cleanup and kills processes mid-teardown. This entry point runs the same
launch service in the terminal mode, so each child receives one SIGINT, from
its owner. SIGTERM and SIGKILL escalation by launch itself is unchanged.
"""

from __future__ import annotations

import sys

from launch import EventHandler, LaunchDescription, LaunchService
from launch.actions import EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api.api import parse_launch_arguments


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__.split('\n\n')[1], file=sys.stderr)
        return 2
    package, file_name, *arguments = argv
    path = get_share_file_path_from_package(package_name=package, file_name=file_name)
    failed_children = []
    escalations = []

    def record_exit(event, context):
        # Launch itself can return zero after a child crashes. Preserve the
        # intentional interrupt status of our runner, but surface actual child
        # failures even when the benchmark already wrote a valid run.json.
        expected = event.returncode == 0 or (
            context.is_shutdown and event.returncode in {-2, 130})
        if not expected:
            failed_children.append((event.process_name, event.returncode))
            if not context.is_shutdown:
                return [EmitEvent(event=Shutdown(reason=f'{event.process_name} failed'))]

    def record_escalation(context):
        escalations.append(context.locals.event.signal_name)

    service = LaunchService(argv=arguments, noninteractive=False)
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=record_exit)),
        RegisterEventHandler(EventHandler(
            matcher=lambda event: isinstance(event, SignalProcess)
            and event.signal_name in {'SIGTERM', 'SIGKILL'},
            entities=[OpaqueFunction(function=record_escalation)],
        )),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(path),
            launch_arguments=parse_launch_arguments(arguments),
        ),
    ]))
    result = service.run()
    for name, returncode in failed_children:
        print(f'[benchmark-launch] {name} failed with exit {returncode}.',
              file=sys.stderr, flush=True)
    for signame in escalations:
        print(f'[benchmark-launch] Child shutdown required {signame}.',
              file=sys.stderr, flush=True)
    return result or int(bool(failed_children or escalations))


if __name__ == '__main__':
    sys.exit(main())
