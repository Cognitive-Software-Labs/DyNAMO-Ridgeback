"""Own a single benchmark's configuration and persistent environment sessions.

The configuration must finish its scene rollback while the simulator still
answers. Keeping these launches in separate sessions makes that ordering hold
for terminal interrupts as well as normal runner completion.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from ridgeback_common.ros_signals import shielded
from ridgeback_autonomy.benchmarking.target_benchmark_sweep import stop_process_group


POLL_INTERVAL_SEC = 0.05


def run_commands(
    environment_command: Sequence[str],
    configuration_command: Sequence[str],
    *,
    environment: Mapping[str, str] | None = None,
    report_path: Path | None = None,
) -> int:
    """Run both layers; stop the complete configuration group before the environment.

    A configuration that remains alive after runner completion keeps inspection
    available, as requested by ``shutdown_on_complete=false``. No run document
    can substitute for the configuration process's exit status.
    """

    stop_requested = False
    lifecycle = {'started_monotonic': time.monotonic(), 'groups': {}}

    def save_report():
        if report_path is not None:
            temporary = report_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(lifecycle, indent=2) + '\n')
            temporary.replace(report_path)

    def interrupt(_signum, _frame):
        # Recording rather than raising cannot split Popen from assignment of
        # its owned child, or interrupt another non-atomic lifecycle operation.
        nonlocal stop_requested
        stop_requested = True

    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in previous:
        signal.signal(signum, interrupt)
    persistent = configuration = None
    result = 1
    interrupted = False
    try:
        persistent = subprocess.Popen(
            list(environment_command), env=environment, start_new_session=True)
        lifecycle['environment_pid'] = persistent.pid
        save_report()
        if not stop_requested:
            configuration = subprocess.Popen(
                list(configuration_command), env=environment, start_new_session=True)
            lifecycle['configuration_pid'] = configuration.pid
            save_report()
        while True:
            if stop_requested:
                interrupted = True
                result = 0
                break
            if persistent.poll() is not None:
                print('[benchmark] Environment exited before configuration shutdown.', flush=True)
                result = 1
                break
            status = configuration.poll()
            if status is not None:
                result = 0 if status == 0 else 1
                break
            time.sleep(POLL_INTERVAL_SEC)
    except KeyboardInterrupt:
        interrupted = True
        result = 0
    except Exception as exc:
        print(f'[benchmark] Cannot run benchmark: {exc}', flush=True)
        result = 1
    finally:
        try:
            with shielded():
                for process, label in ((configuration, 'benchmark configuration'),
                                       (persistent, 'benchmark environment')):
                    try:
                        stopping = time.monotonic()
                        report = stop_process_group(process, label)
                        lifecycle['groups'][label] = {
                            'stop_started_monotonic': stopping,
                            'stop_finished_monotonic': time.monotonic(),
                            'report': report,
                        }
                        if report is not None:
                            expected = {0, 130, -signal.SIGINT} if (
                                process is persistent or interrupted) else {0}
                            if (report['escalated'] or not report['group_empty']
                                    or report['returncode'] not in expected):
                                print(f'[benchmark] Unsuccessful {label} shutdown: {report}', flush=True)
                                result = 1
                    except Exception as exc:
                        lifecycle['groups'][label] = {'exception': f'{type(exc).__name__}: {exc}'}
                        print(f'[benchmark] Failed to stop {label}: {exc}', flush=True)
                        result = 1
        finally:
            lifecycle.update(returncode=result, operator_interrupted=interrupted,
                             finished_monotonic=time.monotonic())
            try:
                save_report()
            except Exception as exc:
                print(f'[benchmark] Cannot save lifecycle report: {exc}', flush=True)
                result = 1
            for signum, handler in previous.items():
                signal.signal(signum, handler)
    return result


def _command(value: str) -> list[str]:
    command = json.loads(value)
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise argparse.ArgumentTypeError('command must be a nonempty JSON array of strings')
    return command


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment-command', type=_command, required=True)
    parser.add_argument('--configuration-command', type=_command, required=True)
    parser.add_argument('--report', type=Path, help='Optional owned-process lifecycle evidence file')
    args = parser.parse_args(argv)
    status = run_commands(args.environment_command, args.configuration_command,
                          environment=os.environ, report_path=args.report)
    with shielded(final=True):
        return status


if __name__ == '__main__':
    raise SystemExit(main())
