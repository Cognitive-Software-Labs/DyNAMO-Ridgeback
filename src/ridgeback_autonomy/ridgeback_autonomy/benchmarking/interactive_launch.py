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

from ros2launch.api import get_share_file_path_from_package, launch_a_launch_file


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__.split('\n\n')[1], file=sys.stderr)
        return 2
    package, file_name, *arguments = argv
    path = get_share_file_path_from_package(package_name=package, file_name=file_name)
    return launch_a_launch_file(
        launch_file_path=path,
        launch_file_arguments=arguments,
        noninteractive=False,
    )


if __name__ == '__main__':
    sys.exit(main())
