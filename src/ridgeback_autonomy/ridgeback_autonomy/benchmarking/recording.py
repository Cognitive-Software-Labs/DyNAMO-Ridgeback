"""Screen-record the RViz window for the length of a benchmark run.

A run is ~90 trials over ~40 minutes and the interesting moments -- an estimator
latching onto an occluder, a mask collapsing -- are gone by the time the CSVs are
read. The collage PNGs freeze one representative frame per trial; this keeps the
motion around it.

Captures a single window by id rather than a screen region, so the recording is
unaffected by what else is on the desktop or where the window sits. That relies
on a compositing X server, which redirects each window to its own offscreen
pixmap; without one, an occluded window records whatever is drawn over it.

Recording is strictly best-effort. Every failure path here -- no ffmpeg, no
xdotool, no window, a non-zero exit -- logs and returns, because a benchmark run
costs 40 minutes and a screen recorder is not a reason to lose one.

Pure module (no ROS): the caller supplies a logger.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time


# Enough to follow what happened without producing a file nobody will keep.
DEFAULT_FPS = 10
# Hard ceiling so a recorder orphaned by a killed launch cannot run forever.
DEFAULT_MAX_SECONDS = 4 * 60 * 60

# Grace periods for the three-stage stop. ffmpeg finalizes the container on a
# clean exit, so each stage gets time to do that before the next escalates.
QUIT_GRACE_S = 10.0
SIGINT_GRACE_S = 5.0
SIGTERM_GRACE_S = 5.0

WINDOW_POLL_INTERVAL_S = 1.0


def _noop_log(_message: str) -> None:
    pass


def build_ffmpeg_command(
    window_id: int,
    output_path: str,
    *,
    display: str,
    fps: int = DEFAULT_FPS,
    max_seconds: int = DEFAULT_MAX_SECONDS,
) -> list[str]:
    """The x11grab invocation, as a list so nothing goes through a shell."""

    return [
        'ffmpeg', '-y', '-loglevel', 'warning',
        '-f', 'x11grab',
        '-framerate', str(int(fps)),
        '-draw_mouse', '0',
        '-window_id', str(int(window_id)),
        '-i', display,
        '-t', str(int(max_seconds)),
        # yuv420p requires even dimensions and a window can be any size; without
        # this ffmpeg exits with "width not divisible by 2" and records nothing.
        '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2',
        '-c:v', 'libx264',
        '-preset', 'veryfast',
        '-crf', '28',
        '-pix_fmt', 'yuv420p',
        # Moves the index to the front on close, so the file seeks properly.
        '-movflags', '+faststart',
        output_path,
    ]


def _window_area(window_id: int) -> int:
    """Pixel area of a window, or 0 if it cannot be measured."""

    try:
        result = subprocess.run(
            ['xdotool', 'getwindowgeometry', str(window_id)],
            capture_output=True, text=True, timeout=5.0, check=False)
    except (OSError, subprocess.SubprocessError):
        return 0
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line.startswith('Geometry:'):
            continue
        try:
            width, height = line.split(':', 1)[1].strip().split('x')
            return int(width) * int(height)
        except ValueError:
            return 0
    return 0


def find_window_id(
    class_hint: str,
    *,
    timeout_s: float = 0.0,
    log=_noop_log,
) -> int | None:
    """The largest visible window whose class matches ``class_hint``.

    Size is the tiebreak because a class search also turns up tiny helper
    windows an application owns -- picking the first match can return a 10x10
    icon window instead of the interface.
    """

    if shutil.which('xdotool') is None:
        log('xdotool not found; cannot locate the window to record.')
        return None

    deadline = time.monotonic() + max(0.0, timeout_s)
    while True:
        try:
            result = subprocess.run(
                ['xdotool', 'search', '--onlyvisible', '--class', class_hint],
                capture_output=True, text=True, timeout=10.0, check=False)
        except (OSError, subprocess.SubprocessError) as exc:
            log(f'Window search failed: {exc}')
            return None

        candidates = [int(line) for line in result.stdout.split() if line.isdigit()]
        if candidates:
            best = max(candidates, key=_window_area)
            if _window_area(best) > 0:
                return best
        if time.monotonic() >= deadline:
            log(f'No visible "{class_hint}" window found; not recording.')
            return None
        time.sleep(WINDOW_POLL_INTERVAL_S)


class ScreenRecorder:
    """One ffmpeg process, started and stopped around a run."""

    def __init__(
        self,
        *,
        fps: int = DEFAULT_FPS,
        max_seconds: int = DEFAULT_MAX_SECONDS,
        log=_noop_log,
    ) -> None:
        self.fps = fps
        self.max_seconds = max_seconds
        self.log = log
        self.process: subprocess.Popen | None = None
        self.output_path: str | None = None

    @property
    def active(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self, window_id: int, output_path: str) -> bool:
        if self.active:
            return True
        if shutil.which('ffmpeg') is None:
            self.log('ffmpeg not found; this run will not be recorded.')
            return False

        display = os.environ.get('DISPLAY')
        if not display:
            self.log('DISPLAY is unset; this run will not be recorded.')
            return False

        os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
        command = build_ffmpeg_command(
            window_id, output_path,
            display=display, fps=self.fps, max_seconds=self.max_seconds)
        try:
            # stdin stays open: the clean way to stop ffmpeg is to send it "q".
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.log(f'Could not start the screen recorder: {exc}')
            self.process = None
            return False

        self.output_path = output_path
        self.log(f'Recording RViz window {window_id} to {output_path}')
        return True

    def stop(self) -> bool:
        """Stop and finalize. ``True`` if a playable file was written.

        Escalates rather than killing: ffmpeg writes the container index when it
        exits, so a process killed outright leaves an unplayable file. "q" on
        stdin is the documented clean stop, then signals, and SIGKILL only once
        the file is already forfeit.
        """

        process, self.process = self.process, None
        if process is None:
            return False

        if process.poll() is None:
            try:
                if process.stdin is not None:
                    process.stdin.write(b'q')
                    process.stdin.flush()
            except (OSError, ValueError):
                pass

            for grace, escalate in (
                (QUIT_GRACE_S, None),
                (SIGINT_GRACE_S, signal.SIGINT),
                (SIGTERM_GRACE_S, signal.SIGTERM),
            ):
                if escalate is not None:
                    self.log(f'Recorder did not stop; sending {escalate.name}.')
                    try:
                        process.send_signal(escalate)
                    except OSError:
                        break
                try:
                    process.wait(timeout=grace)
                    break
                except subprocess.TimeoutExpired:
                    continue
            else:
                self.log('Recorder unresponsive; killing it. The video may be truncated.')
                process.kill()
                process.wait()

        try:
            if process.stdin is not None:
                process.stdin.close()
        except (OSError, ValueError):
            pass

        path = self.output_path
        if path and os.path.exists(path) and os.path.getsize(path) > 0:
            self.log(f'Recording written to {path}')
            return True
        self.log('Recorder produced no output file.')
        return False
