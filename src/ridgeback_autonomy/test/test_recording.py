from __future__ import annotations

import signal
import subprocess

import pytest

from ridgeback_autonomy.benchmarking import recording
from ridgeback_autonomy.benchmarking.recording import (
    ScreenRecorder,
    build_ffmpeg_command,
    find_window_id,
)


def test_command_captures_one_window_not_a_screen_region() -> None:
    # Region capture would record whatever sits at those coordinates; by id the
    # recording follows the window.
    command = build_ffmpeg_command(1234, '/tmp/out.mp4', display=':0')

    assert '-window_id' in command
    assert command[command.index('-window_id') + 1] == '1234'
    assert command[command.index('-i') + 1] == ':0'


def test_command_pads_to_even_dimensions() -> None:
    # yuv420p rejects odd dimensions and a window can be any size, so without the
    # pad filter ffmpeg exits immediately and the run records nothing.
    command = build_ffmpeg_command(1, '/tmp/out.mp4', display=':0')

    assert command[command.index('-vf') + 1] == 'pad=ceil(iw/2)*2:ceil(ih/2)*2'
    assert command[command.index('-pix_fmt') + 1] == 'yuv420p'


def test_command_has_a_hard_duration_cap() -> None:
    command = build_ffmpeg_command(1, '/tmp/out.mp4', display=':0', max_seconds=42)

    assert command[command.index('-t') + 1] == '42'


def test_command_is_argv_not_a_shell_string() -> None:
    command = build_ffmpeg_command(1, '/tmp/o u t.mp4', display=':0')

    assert isinstance(command, list)
    assert command[-1] == '/tmp/o u t.mp4'


class FakeProcess:
    """A Popen stand-in that stays alive until a chosen stop stage."""

    def __init__(self, stops_after: str | None = 'quit') -> None:
        self.stops_after = stops_after
        self.signals: list[int] = []
        self.killed = False
        self.stdin = None
        self._stopped = False

    def poll(self):
        return 0 if self._stopped else None

    def wait(self, timeout=None):
        if self._stopped:
            return 0
        raise subprocess.TimeoutExpired('ffmpeg', timeout)

    def send_signal(self, sig) -> None:
        self.signals.append(sig)
        if self.stops_after == 'sigint' and sig == signal.SIGINT:
            self._stopped = True
        if self.stops_after == 'sigterm' and sig == signal.SIGTERM:
            self._stopped = True

    def kill(self) -> None:
        self.killed = True
        self._stopped = True


def make_recorder(tmp_path, process, monkeypatch) -> ScreenRecorder:
    monkeypatch.setattr(recording, 'QUIT_GRACE_S', 0.01)
    monkeypatch.setattr(recording, 'SIGINT_GRACE_S', 0.01)
    monkeypatch.setattr(recording, 'SIGTERM_GRACE_S', 0.01)
    recorder = ScreenRecorder()
    recorder.process = process
    output = tmp_path / 'run.mp4'
    output.write_bytes(b'video')
    recorder.output_path = str(output)
    return recorder


def test_stop_escalates_instead_of_killing_outright(tmp_path, monkeypatch) -> None:
    # ffmpeg writes the container index on exit, so killing first leaves an
    # unplayable file. SIGKILL is a last resort, not the first move.
    process = FakeProcess(stops_after='sigterm')
    recorder = make_recorder(tmp_path, process, monkeypatch)

    assert recorder.stop() is True
    assert process.signals == [signal.SIGINT, signal.SIGTERM]
    assert process.killed is False


def test_stop_kills_only_an_unresponsive_recorder(tmp_path, monkeypatch) -> None:
    process = FakeProcess(stops_after=None)
    recorder = make_recorder(tmp_path, process, monkeypatch)

    recorder.stop()

    assert process.signals == [signal.SIGINT, signal.SIGTERM]
    assert process.killed is True


def test_stop_reports_failure_when_no_file_was_written(tmp_path, monkeypatch) -> None:
    process = FakeProcess(stops_after='sigint')
    recorder = make_recorder(tmp_path, process, monkeypatch)
    recorder.output_path = str(tmp_path / 'never_created.mp4')

    assert recorder.stop() is False


def test_stop_without_a_running_recorder_is_harmless() -> None:
    assert ScreenRecorder().stop() is False


def test_missing_ffmpeg_does_not_raise(monkeypatch) -> None:
    # A missing tool must cost a warning, never a 40-minute run.
    monkeypatch.setattr(recording.shutil, 'which', lambda _name: None)
    messages: list[str] = []

    assert ScreenRecorder(log=messages.append).start(1, '/tmp/out.mp4') is False
    assert any('ffmpeg' in message for message in messages)


def test_missing_display_does_not_raise(monkeypatch) -> None:
    monkeypatch.setattr(recording.shutil, 'which', lambda _name: '/usr/bin/ffmpeg')
    monkeypatch.delenv('DISPLAY', raising=False)
    messages: list[str] = []

    assert ScreenRecorder(log=messages.append).start(1, '/tmp/out.mp4') is False
    assert any('DISPLAY' in message for message in messages)


def test_window_search_prefers_the_largest_match(monkeypatch) -> None:
    # A bare class search also returns tiny helper windows an app owns, so the
    # first match can be a 10x10 icon rather than the interface.
    monkeypatch.setattr(recording.shutil, 'which', lambda _name: '/usr/bin/xdotool')
    monkeypatch.setattr(
        recording.subprocess, 'run',
        lambda command, **kwargs: subprocess.CompletedProcess(
            command, 0, stdout='11\n22\n', stderr=''))
    monkeypatch.setattr(
        recording, '_window_area', lambda window_id: {11: 100, 22: 1_000_000}[window_id])

    assert find_window_id('rviz') == 22


def test_missing_xdotool_returns_none(monkeypatch) -> None:
    monkeypatch.setattr(recording.shutil, 'which', lambda _name: None)
    messages: list[str] = []

    assert find_window_id('rviz', log=messages.append) is None
    assert any('xdotool' in message for message in messages)


def test_no_matching_window_returns_none(monkeypatch) -> None:
    monkeypatch.setattr(recording.shutil, 'which', lambda _name: '/usr/bin/xdotool')
    monkeypatch.setattr(
        recording.subprocess, 'run',
        lambda command, **kwargs: subprocess.CompletedProcess(command, 0, stdout='', stderr=''))
    messages: list[str] = []

    assert find_window_id('rviz', log=messages.append) is None
    assert any('not recording' in message for message in messages)
