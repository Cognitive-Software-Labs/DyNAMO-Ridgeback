"""Spawn, track, and cancel the benchmark runs the configurator starts.

A run outlives the configurator by design. The browser tab is a viewer, the
server is restartable, and a replay can take longer than either, so every fact a
later reader needs is written into the run directory instead of held here. This
object owns only the single-flight lock, the handles of children it spawned
itself, and the threads escalating a cancellation.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import secrets
import shlex
import signal
import subprocess
import threading
import time

import yaml

from ridgeback_autonomy.benchmarking.configurator_errors import ConfiguratorError
from ridgeback_autonomy.benchmarking.paths import (
    JOB_ROOT_RELATIVE,
    RUN_ROOT_RELATIVE,
    anchored_path,
    write_json_atomic,
)
from ridgeback_autonomy.benchmarking.replay_jobs import command_argv
from ridgeback_autonomy.benchmarking.replay_profiles import PROFILE_LIVE_SYSTEM


ACTIVE_STATES = frozenset({'running', 'cancelling'})
# SIGINT first, and never a bare SIGKILL: the sweep supervisor's KeyboardInterrupt
# handler is the only path that stops the video recorder gently enough to leave a
# readable index, and that marks the sweep resumable rather than abandoned.
CANCEL_ESCALATION = ((signal.SIGINT, 30.0), (signal.SIGTERM, 10.0), (signal.SIGKILL, 0.0))
LIVENESS_POLL_SEC = 0.2
LOG_TAIL_BYTES = 64 * 1024
LOG_TAIL_LINES = 200


def _timestamp() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


def _wall_timestamp() -> str:
    return time.strftime('%Y-%m-%d %H:%M:%S %z')


def _process_argv(pid: int) -> list[str] | None:
    """The live argv of a PID, or None when nothing readable is there.

    A reaped child leaves no entry and an unreaped one leaves an empty cmdline,
    so both resolve to a value that cannot match a recorded argv.
    """

    try:
        raw = Path(f'/proc/{pid}/cmdline').read_bytes()
    except OSError:
        return None
    parts = raw.split(b'\0')
    while parts and parts[-1] == b'':
        parts.pop()
    return [part.decode('utf-8', 'replace') for part in parts]


def is_alive(record: dict) -> bool:
    """Whether the recorded process is still the one this run started.

    ``os.kill(pid, 0)`` is not enough across a configurator restart, because the
    kernel reuses PIDs and an unrelated process would then be reported as the
    operator's run.
    """

    pid = record.get('pid')
    if not isinstance(pid, int):
        return False
    return _process_argv(pid) == list(record.get('spawn_argv') or ())


def group_alive(pgid: object) -> bool:
    """Whether anything is still running in the run's process group.

    Cancellation has to watch the group rather than the wrapper shell. The
    wrapper dies on the first SIGINT, while the benchmark it started may ignore
    that signal or take minutes to shut down; watching only the wrapper would
    declare the run over and leave the real process orphaned unescalated.
    """

    if not isinstance(pgid, int):
        return False
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # The group exists but is no longer ours to signal.
        return True
    return True


def write_job_file(workspace_root: Path, job, stamp: str | None = None) -> Path:
    """Write the canonical job YAML and return the path a command can name.

    The browser downloads to a directory it cannot disclose, and every relative
    path inside a job resolves against the job file's own parent, so a rendered
    command can only be correct if the server owns the write.
    """

    directory = workspace_root / JOB_ROOT_RELATIVE
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f'benchmark-{job.profile}-{stamp or _timestamp()}.yaml'
    path.write_text(
        yaml.safe_dump(job.document, sort_keys=False, allow_unicode=True),
        encoding='utf-8')
    return path


class RunSupervisor:
    """One GUI-owned benchmark run at a time, recorded on disk."""

    def __init__(self, workspace_root: Path, *, escalation=CANCEL_ESCALATION):
        self._root = Path(workspace_root)
        self._runs = self._root / RUN_ROOT_RELATIVE
        self._escalation = tuple(escalation)
        self._lock = threading.RLock()
        self._children: dict[str, subprocess.Popen] = {}
        self._cancellers: dict[str, threading.Thread] = {}

    # -- reading ---------------------------------------------------------

    def _directory(self, run_id: str) -> Path:
        return self._runs / run_id

    def _load(self, run_id: str) -> dict:
        try:
            return json.loads((self._directory(run_id) / 'run.json').read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            raise ConfiguratorError(
                'run_id', 'unknown_run', f'No such benchmark run: {run_id}') from exc

    def _returncode(self, run_id: str) -> int | None:
        try:
            text = (self._directory(run_id) / 'returncode').read_text(encoding='utf-8')
        except OSError:
            return None
        try:
            # The wrapper shell creates the file before it writes, so a partial
            # read here means the child is still exiting, not that it failed.
            return int(text.strip())
        except ValueError:
            return None

    def _settle(self, record: dict, returncode: int | None, state: str) -> dict:
        settled = {
            **record,
            'state': state,
            'returncode': returncode,
            'finished': record.get('finished') or _wall_timestamp(),
        }
        write_json_atomic(self._directory(record['run_id']) / 'run.json', settled)
        self._children.pop(record['run_id'], None)
        return settled

    def _resolve(self, record: dict) -> dict:
        """Reconcile a record that still claims to be active with the system."""

        if record.get('state') not in ACTIVE_STATES:
            return record
        child = self._children.get(record['run_id'])
        if child is not None:
            # Reap, so an exited child of this process stops occupying a PID
            # that liveness would otherwise still find in /proc.
            child.poll()
        returncode = self._returncode(record['run_id'])
        cancelled = bool(record.get('cancel_requested'))
        if returncode is not None:
            if cancelled and returncode != 0:
                return self._settle(record, returncode, 'cancelled')
            return self._settle(record, returncode, 'succeeded' if returncode == 0 else 'failed')
        if is_alive(record):
            return record
        if cancelled:
            if group_alive(record.get('pgid')):
                # The wrapper shell died on the first signal, but the benchmark
                # it started is still tearing down. Not over yet.
                return record
            # A signalled wrapper shell dies before it can record the code.
            return self._settle(record, None, 'cancelled')
        # The process is gone and left no exit code, which happens when the
        # machine or the shell went down under it. The output directory is the
        # only remaining evidence, so say so rather than guess an outcome.
        return self._settle(record, None, 'unknown')

    def records(self) -> list[dict]:
        if not self._runs.is_dir():
            return []
        with self._lock:
            resolved = []
            for directory in sorted(self._runs.iterdir(), reverse=True):
                try:
                    record = json.loads((directory / 'run.json').read_text(encoding='utf-8'))
                except (OSError, ValueError):
                    continue
                resolved.append(self._resolve(record))
            return resolved

    def active(self) -> dict | None:
        return next((record for record in self.records() if record['state'] in ACTIVE_STATES), None)

    def detail(self, run_id: str, *, lines: int = LOG_TAIL_LINES) -> dict:
        with self._lock:
            record = self._resolve(self._load(run_id))
        return {**record, 'log': self._log_tail(run_id, lines)}

    def _log_tail(self, run_id: str, lines: int) -> str:
        try:
            with open(self._directory(run_id) / 'run.log', 'rb') as handle:
                handle.seek(0, os.SEEK_END)
                handle.seek(max(0, handle.tell() - LOG_TAIL_BYTES))
                text = handle.read().decode('utf-8', 'replace')
        except OSError:
            return ''
        return '\n'.join(text.splitlines()[-lines:])

    # -- starting --------------------------------------------------------

    def resolved_output(self, job) -> Path:
        """The directory the runner will write, refusing what it would refuse.

        The runner rejects a pre-existing output directory after it has already
        started, so the same rejection happens here to reach the operator as a
        field error instead of a subprocess that dies two seconds later.
        """

        if not job.output_dir:
            raise ConfiguratorError(
                'output_dir', 'missing_required_field',
                'An output directory is required to start a run.')
        output = Path(anchored_path(str(self._root), job.output_dir))
        if output.exists():
            raise ConfiguratorError(
                'output_dir', 'output_exists',
                f'The runner refuses an output directory that already exists: {output}')
        return output

    @staticmethod
    def ensure_startable(profile: str) -> None:
        """Refuse the profiles this page cannot own, before anything is written.

        A live sweep needs a GPU-backed X session and runs ``cleanup.sh`` before
        Gazebo starts, and that catch-all currently matches the configurator's
        own command line -- so a sweep launched from here would kill this page
        mid-click.
        """

        if profile == PROFILE_LIVE_SYSTEM:
            raise ConfiguratorError(
                'profile', 'live_run_unsupported',
                'A live sweep still starts from a terminal: it needs a GPU-backed '
                'X session, and its preflight cleanup would kill this page.')

    def start(self, job) -> dict:
        self.ensure_startable(job.profile)
        with self._lock:
            running = self.active()
            if running is not None:
                raise ConfiguratorError(
                    'run', 'run_in_progress',
                    f'Run {running["run_id"]} ({running["profile"]}) is still '
                    f'{running["state"]}. Cancel it before starting another; two '
                    'replays would contend for the worker counts you chose.')
            output = self.resolved_output(job)
            run_id = f'{_timestamp()}_{secrets.token_hex(3)}'
            directory = self._directory(run_id)
            directory.mkdir(parents=True)
            job_path = write_job_file(self._root, job, run_id)
            argv = command_argv(str(job_path), job)
            # Without `exec`, so the wrapper survives the command and lands the
            # exit code on disk. Nothing can waitpid for this child once the
            # configurator is gone, so the file is the only durable record.
            spawn_argv = [
                'bash', '-c',
                f'{shlex.join(argv)}; echo $? > {shlex.quote(str(directory / "returncode"))}']
            with open(directory / 'run.log', 'wb', buffering=0) as log:
                child = subprocess.Popen(
                    spawn_argv,
                    cwd=str(self._root),
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    # Its own process group, so the run survives the GUI exiting
                    # and one signal reaches everything the run started.
                    start_new_session=True,
                    # The configurator is itself launched through `ros2 run`, so
                    # it already holds the sourced overlay the child needs.
                    env=os.environ,
                )
            record = {
                'run_id': run_id,
                'profile': job.profile,
                'question': job.question,
                'job_path': str(job_path),
                'output_dir': str(output),
                'argv': argv,
                'command': shlex.join(argv),
                'spawn_argv': spawn_argv,
                'pid': child.pid,
                'pgid': child.pid,
                'started': _wall_timestamp(),
                'state': 'running',
                'returncode': None,
                'finished': None,
            }
            write_json_atomic(directory / 'run.json', record)
            self._children[run_id] = child
            return record

    # -- cancelling ------------------------------------------------------

    def cancel(self, run_id: str) -> dict:
        with self._lock:
            record = self._resolve(self._load(run_id))
            if record['state'] not in ACTIVE_STATES:
                raise ConfiguratorError(
                    'run_id', 'not_running',
                    f'Run {run_id} already finished ({record["state"]}).')
            if record.get('cancel_requested'):
                return record
            record = {**record, 'state': 'cancelling', 'cancel_requested': _wall_timestamp()}
            write_json_atomic(self._directory(run_id) / 'run.json', record)
            canceller = threading.Thread(
                target=self._escalate, args=(run_id,), name=f'cancel-{run_id}', daemon=True)
            self._cancellers[run_id] = canceller
        canceller.start()
        return record

    def _escalate(self, run_id: str) -> None:
        record = self._load(run_id)
        pgid = record.get('pgid')
        for number, grace in self._escalation:
            if not group_alive(pgid):
                break
            try:
                os.killpg(pgid, number)
            except OSError:
                break
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline and group_alive(pgid):
                time.sleep(LIVENESS_POLL_SEC)
        with self._lock:
            self._resolve(self._load(run_id))

    def await_cancellation(self, run_id: str, *, timeout: float | None = None) -> None:
        """Join the escalation thread. Tests need this; the browser polls."""

        canceller = self._cancellers.get(run_id)
        if canceller is not None:
            canceller.join(timeout)
