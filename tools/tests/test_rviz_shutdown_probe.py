"""Regression gates for readiness and bounded, child-only shutdown decisions."""

import importlib.util
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace


SPEC = importlib.util.spec_from_file_location(
    "rviz_shutdown_probe", Path(__file__).parents[1] / "diagnostics/rviz_shutdown_probe.py"
)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class Child:
    def __init__(self, timeout=False, returncode=None):
        self.timeout = timeout
        self.returncode = returncode
        self.signals = []
        self.kills = 0

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.signals.append(value)

    def wait(self, timeout):
        if self.timeout:
            self.timeout = False
            raise subprocess.TimeoutExpired("rviz2", timeout)
        self.returncode = -signal.SIGKILL if self.kills else 0
        return self.returncode

    def kill(self):
        self.kills += 1


def test_startup_exit_zero_never_counts_as_clean_shutdown():
    assert probe.shutdown_outcome(False, 0, False) == "startup_failed"


def test_shutdown_sends_one_graceful_signal_and_only_escalates_after_timeout():
    child = Child(timeout=True)
    result = probe.stop_child(child, 0.1)
    assert child.signals == [signal.SIGINT]
    assert child.kills == 1
    assert result == {"signal": "SIGINT", "escalated": True, "exit_code": -signal.SIGKILL}
    assert probe.shutdown_outcome(True, result["exit_code"], result["escalated"]) == "shutdown_timeout"


def test_already_exited_child_receives_no_signal():
    child = Child(returncode=-signal.SIGSEGV)
    assert probe.stop_child(child, 1) == {"signal": None, "escalated": False, "exit_code": -signal.SIGSEGV}
    assert child.signals == []
    assert child.kills == 0


def test_readiness_requires_render_graph_and_configured_subscribers():
    status = {"rviz_node_seen": True, "subscriptions": {"/tf": 1, probe.IMAGE_TOPIC: 1}}
    assert probe.readiness(status, "OpenGl version: 4.5", "image", probe.IMAGE_TOPIC)
    assert not probe.readiness(status, "", "image", probe.IMAGE_TOPIC)
    assert not probe.readiness({}, "OpenGl version: 4.5", "empty", probe.IMAGE_TOPIC)
    status["subscriptions"][probe.IMAGE_TOPIC] = 0
    assert not probe.readiness(status, "OpenGl version: 4.5", "image", probe.IMAGE_TOPIC)
    assert probe.readiness(status, "OpenGl version: 4.5", "tf", probe.IMAGE_TOPIC)


def test_missing_config_records_blocked_run_without_spawning(tmp_path):
    args = SimpleNamespace(benchmark_config=tmp_path / "missing.rviz")
    result = probe.run_one(args, tmp_path, {}, "/unused/rviz2", "benchmark", 1)
    assert result["outcome"] == "blocked"
    assert not result["ready"]
    assert "FileNotFoundError" in result["error"]
    assert (tmp_path / "benchmark-01/summary.json").is_file()


def test_simulated_namespace_requires_matching_tf_and_clock_subscribers():
    status = {"rviz_node_seen": True, "subscriptions": {probe.IMAGE_TOPIC: 1, "/tf": 1}}
    assert not probe.readiness(status, "OpenGl version: 4.5", "benchmark", probe.IMAGE_TOPIC,
                               "/r100_0001/tf", True)
    status["subscriptions"]["/r100_0001/tf"] = 1
    assert not probe.readiness(status, "OpenGl version: 4.5", "benchmark", probe.IMAGE_TOPIC,
                               "/r100_0001/tf", True)
    status["subscriptions"]["/clock"] = 1
    assert probe.readiness(status, "OpenGl version: 4.5", "benchmark", probe.IMAGE_TOPIC,
                           "/r100_0001/tf", True)


def test_launch_parity_scopes_sim_time_without_changing_hidden_nodes(tmp_path, monkeypatch):
    args = probe.parser().parse_args([
        "--namespace", "r100_0001", "--node-name", "rviz2", "--sim-time",
        "--sim-time-scope", "node", "--no-clock",
    ])
    spawned = []

    def prevent_start(command, **kwargs):
        spawned.append(command)
        raise RuntimeError("No processes are started by this test")

    monkeypatch.setattr(probe.subprocess, "Popen", prevent_start)
    result = probe.run_one(args, tmp_path, {}, "/unused/rviz2", "image", 1)
    command = result["command"]
    assert "__node:=rviz2" in command
    assert "__ns:=/r100_0001" in command
    assert "/tf:=tf" in command
    assert "/tf_static:=tf_static" in command
    assert "-p" not in command
    parameters = Path(command[command.index("--params-file") + 1])
    assert parameters.read_text() == "/r100_0001/rviz2:\n  ros__parameters:\n    use_sim_time: true\n"
    assert spawned[0][-2:] == ["/r100_0001/tf", "true-no-clock"]
