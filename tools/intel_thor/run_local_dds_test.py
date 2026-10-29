#!/usr/bin/env python3
"""Guarded Intel DDS test. Run with sudo and --operator-confirmed.

This controller owns only its reader/capture subprocess groups, one DDS XML
change, and a temporary camera ExecStart override. It never edits robot.yaml.
Run evidence includes CURRENT_CONFIG.json and an explicit recovery command.
--verify-current only observes the installed profile; it never reloads services.
"""

from __future__ import annotations

import argparse
import difflib
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import pwd
import re
import shlex
import shutil
import signal
import socket
import stat
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from analyze_image_timing import analyze_reader


ROOT = Path(__file__).resolve().parents[2]
PROBE = Path(__file__).with_name("local_sensor_probe.py")
DDS = Path("/etc/clearpath/cyclonedds.xml")
ROBOT = Path("/etc/clearpath/robot.yaml")
START = Path("/usr/local/sbin/realsense-camera-start")
MERGER = Path("/opt/mybotshop/install/ros2_laser_scan_merger/share/ros2_laser_scan_merger")
OVERRIDE = Path("/etc/systemd/system/realsense-camera.service.d/95-dynamo-dds-experiment.conf")
LEGACY = Path("/etc/clearpath/dynamo-rmw-backup")
UNITS = tuple(name + ".service" for name in (
    "clearpath-platform", "clearpath-sensors", "battery-sysfs-bridge",
    "clearpath-scan-merger", "clearpath-joy-x-combiner", "realsense-camera",
    "depth-to-mono8", "ridgeback-camera-mjpeg", "mbs-webserver",
    "camera-web-ready", "ridgeback-startup-recover", "robot-slam", "robot-map-navi",
))
PORTS = ("eno1", "enp2s0")
ERROR = re.compile(r"(?:\b(?:error|fatal)\b|read.*timeout|timed? out|reconnect|device.*disconn|hardware notification.*error|crash|segmentation fault)", re.I)


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def spdp_only(data: bytes) -> bytes:
    """One textual insertion preserves every original byte, including comments."""
    root = ET.fromstring(data)
    if any(node.tag.rsplit("}", 1)[-1] == "AllowMulticast" for node in root.iter()):
        raise ValueError("AllowMulticast already exists; this experiment requires an unrestricted A")
    # Do not interpret text in XML comments as an element boundary.
    visible = re.sub(rb"<!--.*?-->", lambda m: b" " * len(m.group()), data, flags=re.S)
    general = list(re.finditer(rb"<General\s*>", visible))
    if len(general) != 1:
        raise ValueError("Expected exactly one unprefixed General element")
    position = general[0].end()
    newline = b"\r\n" if b"\r\n" in data else b"\n"
    result = data[:position] + newline + b"      <AllowMulticast>spdp</AllowMulticast>" + data[position:]
    ET.fromstring(result)
    return result


def tx_violation(increments, previous, threshold=20.0):
    """Two consecutive one-second samples above threshold on the same port."""
    return any(value > threshold and previous.get(port, 0) > threshold
               for port, value in increments.items())


def needs_sensor_recovery(stage):
    """Camera-rate failure alone is not a reason to restart sensor drivers."""
    for trial in [stage.get('baseline', {}), *stage.get('trials', [])]:
        guard = trial.get('scan_guard', trial.get('scans', {}))
        if any(item.get('reason') in ('scan_missing', 'scan_gap', 'empty_scan')
               for item in guard.get('failures', [])):
            return True
        reason = str(trial.get('failure', '')).lower()
        if 'clearpath-sensors.service' in reason or 'hokuyo' in reason:
            return True
    return False


def scan_window(events_path, start, end, baseline=None):
    """Evaluate scan and traffic evidence for an explicit reader capture window."""
    spec = importlib.util.spec_from_file_location("dds_metrics", PROBE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    metrics = {name: module.StreamMetrics(start) for name in ("front", "rear", "merged")}
    with Path(events_path).open() as stream:
        events = (json.loads(line) for line in stream)
        return _scan_events_window(events, start, end, baseline, metrics)


def _scan_events_window(events, start, end, baseline, metrics):
    totals = {}
    previous_sample = None
    for event in events:
        when = event.get("monotonic_s", 0)
        if event.get("event") == "scan":
            received = event["receive_monotonic_s"]
            name = event["stream"]
            if start <= received <= end:
                metrics[name].record(event["stamp_ns"], received, not event.get("rays"))
        elif event.get("event") == "startup":
            previous_sample = when
        elif event.get("event") in ("host_sample", "host_sample_final"):
            if previous_sample is not None:
                weight = max(0, min(when, end) - max(previous_sample, start))
                for port, sample in event.get("interfaces", {}).items():
                    item = totals.setdefault(port, {"duration_s": 0., "tx_mbit": 0., "rx_mbit": 0.})
                    item["duration_s"] += weight
                    for kind in ("rx", "tx"):
                        item[kind + "_mbit"] += weight * sample[kind + "_mbps"]
            previous_sample = when
    baseline = baseline or {}
    streams = {name: metric.summary(end, .9 * baseline.get("streams", {}).get("merged", {}).get("rate_hz", 0)
                                   if name == "merged" else 38, .5 if name == "merged" else .25)
               for name, metric in metrics.items()}
    interfaces = {}
    for port, values in totals.items():
        duration = values["duration_s"]
        item = {**values, "mean_tx_mbps": values["tx_mbit"] / duration if duration else 0,
                "mean_rx_mbps": values["rx_mbit"] / duration if duration else 0}
        if port in PORTS and port in baseline.get("interfaces", {}):
            item["incremental_tx_mbps"] = item["mean_tx_mbps"] - baseline["interfaces"][port]["mean_tx_mbps"]
            item["traffic_passed"] = item["incremental_tx_mbps"] <= 5
        interfaces[port] = item
    return {"streams": streams, "interfaces": interfaces, "window_start_monotonic_s": start,
            "window_end_monotonic_s": end, "duration_s": end - start,
            "passed": end > start and all(item["passed"] for item in streams.values())
            and all(item.get("traffic_passed") is not False for item in interfaces.values())
            and all(port in interfaces for port in PORTS),
            "limits": ["One-second network intervals intersecting window edges are prorated"]}


def atomic_restore(path: Path, original: bytes, mode: int, uid: int, gid: int):
    """Restore exactly the file this controller changed, preserving its metadata."""
    temporary = path.with_name(path.name + ".dds-experiment-tmp")
    with temporary.open("wb") as stream:
        stream.write(original)
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(temporary, mode)
    os.chown(temporary, uid, gid)
    os.replace(temporary, path)


def stop_group(process):
    """Only signal a group created by this controller; never search by name."""
    if process.poll() is not None:
        return
    try:
        if os.getpgid(process.pid) != process.pid:
            raise RuntimeError("Refusing to signal an unowned process group")
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def process_tree(main_pid):
    records = {}
    for directory in Path("/proc").glob("[0-9]*"):
        try:
            content = (directory / "stat").read_text()
            fields = content[content.rindex(")") + 2:].split()
            records[int(directory.name)] = (int(fields[1]), int(fields[19]))
        except (OSError, ValueError, IndexError):
            continue
    wanted = {main_pid} if main_pid else set()
    while True:
        expanded = wanted | {pid for pid, (parent, _) in records.items() if parent in wanted}
        if expanded == wanted:
            break
        wanted = expanded
    return {str(pid): records[pid][1] for pid in sorted(wanted) if pid in records}


def persistent_ros_tree(main_pid):
    """Watch the service main process, launchers, and ROS workers, not helpers.

    Diagnostics legitimately spawn short-lived commands such as ip/iw/sysctl.
    Their exit is not a sensor or service restart. Launched ROS workers have
    --ros-args; the launch parent is retained too. Root service workers without
    ROS arguments are covered by the MainPID/systemd restart checks.
    """
    selected = {}
    for pid, started in process_tree(main_pid).items():
        try:
            args = (Path('/proc') / pid / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        except OSError:
            continue
        if int(pid) == main_pid or '--ros-args' in args or ('launch' in args and any(Path(arg).name == 'ros2' for arg in args)):
            selected[pid] = started
    return selected


def d455_serial():
    devices = []
    for device in Path("/sys/bus/usb/devices").glob("*"):
        try:
            product = (device / "product").read_text().strip()
            if "455" in product and "RealSense" in product:
                devices.append({"path": str(device), "product": product,
                                "serial": (device / "serial").read_text().strip(),
                                "speed": (device / "speed").read_text().strip()})
        except OSError:
            continue
    if len(devices) != 1 or not re.fullmatch(r"[0-9]+", devices[0]["serial"]):
        raise RuntimeError(f"Expected one D455 with a numeric USB serial; found {devices}")
    return devices[0]


def aligned_script(original: str, serial: str) -> str:
    """Preserve original startup settings; change only the requested arguments."""
    if "exec ros2 launch realsense2_camera rs_launch.py" not in original:
        raise ValueError("Camera startup does not have the expected single rs_launch command")
    if len(re.findall(r"\bexec\s+ros2\s+launch\b", original)) != 1:
        raise ValueError("Ambiguous camera launch command")
    if not re.fullmatch(r"[0-9]+", serial):
        raise ValueError("Invalid serial")
    text = original
    for name, value in (("rgb_camera.color_profile", "640x480x30"),
                        ("depth_module.depth_profile", "640x480x30"),
                        ("enable_infra2", "false"), ("align_depth.enable", "true"),
                        # Double quotes preserve the inner quotes through bash;
                        # launch's YAML coercion then retains a string serial.
                        ("serial_no", "\"'" + serial + "'\"")):
        pattern = re.compile(re.escape(name) + r":=(?:'[^']*'|\"[^\"]*\"|[^\s\\]+)")
        if pattern.search(text):
            text = pattern.sub(lambda _: name + ":=" + value, text)
        else:
            text = text.rstrip() + " \\\n  " + name + ":=" + value + "\n"
    return text


class Controller:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.log = self.directory / "commands.jsonl"
        self.processes = []
        self.active = []
        self.original_active = {}
        self.original_dds = None
        self.dds_meta = None
        self.dds_changed = False
        self.camera_changed = False
        self.retain = False
        self.screening = False
        self.focused_native = False
        self.verify_current = False
        self.retain_passing_native = False
        self.profile = None
        self.results = {"A": {}, "B": {}, "aligned": {}, "soak": {}}
        self.failures = []
        self.cameras = []
        self.tree = {}
        self.units_state = {}
        self.start_time = time.time()
        self.journal_cursor = None
        self.baseline_tx = {port: 0.0 for port in PORTS}
        self.baseline_merged = None
        self.original_override = None
        self.override_meta = None
        self.stage = "preparation"
        self.scan = None
        self.lock = None
        self.prepared = False
        self.directory.mkdir(parents=True, exist_ok=True)
        os.chmod(self.directory, 0o755)

    def say(self, text):
        print(time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), text, flush=True)

    def mode(self):
        return ('verify-current' if self.verify_current else 'focused-native' if self.focused_native
                else 'screening' if self.screening else 'qualification')

    def command(self, argv, *, check=True, timeout=45, output=None):
        if self.verify_current and argv and argv[0] == 'systemctl' and argv[1] not in ('show', 'cat'):
            raise RuntimeError('Current-profile verification prohibits service mutations: ' + shlex.join(argv))
        if argv and argv[0] == 'git':
            # Preserve the repository user's ownership boundary; root's git
            # otherwise rejects this user-owned checkout as dubious ownership.
            argv = ['runuser', '-u', 'robot', '--', *argv]
        with self.log.open("a") as stream:
            stream.write(json.dumps({"time": time.time(), "argv": argv}) + "\n")
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        if output:
            Path(output).write_text(result.stdout + result.stderr)
        if check and result.returncode:
            raise RuntimeError(f"Command failed: {shlex.join(argv)}: {result.stderr[-2000:]}")
        return result.stdout

    def launch(self, argv, log):
        handle = Path(log).open("w")
        with self.log.open("a") as stream:
            stream.write(json.dumps({"time": time.time(), "argv": argv, "log": str(log)}) + "\n")
        try:
            process = subprocess.Popen(argv, stdout=handle, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        finally:
            handle.close()
        self.processes.append(process)
        self.state()
        self.state()
        return process

    def ros(self, arguments):
        # Clean environment prevents an inherited overlay or discovery-server setting.
        robot = pwd.getpwnam("robot")
        prefix = ["runuser", "-u", "robot", "--", "env", "-i", "HOME=" + robot.pw_dir,
                  "USER=robot", "PATH=/usr/local/bin:/usr/bin:/bin",
                  "ROS_DOMAIN_ID=0", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
                  "CYCLONEDDS_URI=" + self.profile.as_uri(),
                  "ROS_LOG_DIR=" + str(self.directory / "ros-logs")]
        return prefix + ["bash", "-c", 'source /opt/ros/jazzy/setup.bash; exec "$@"',
                         "dds-experiment", *arguments]

    def state(self):
        dump(self.directory / "CURRENT_CONFIG.json", {
            "stage": self.stage, "installed_dds": str(DDS),
            "original_dds": str(self.directory / "before" / "cyclonedds.xml"),
            "dds_changed": self.dds_changed, "camera_changed": self.camera_changed,
            "retain_B": self.retain, "original_active": self.original_active,
            "mode": self.mode(),
            "current_profile_only": self.verify_current,
            "comparison_performed": bool(self.results['A'].get('baseline') and self.results['B'].get('baseline')),
            "installed_profile_sha256": (hashlib.sha256(self.original_dds).hexdigest()
                                          if self.original_dds is not None else None),
            "reader_profile": str(self.profile) if self.profile else None,
            "retain_passing_native_requested": self.retain_passing_native,
            "prepared": self.prepared,
            "override": str(OVERRIDE), "started": self.start_time,
            "owned_process_groups": [{"pid": p.pid, "start_ticks": process_tree(p.pid).get(str(p.pid))}
                                      for p in self.processes if p.poll() is None],
            "recovery_command": ["sudo", "python3", str(Path(__file__).resolve()),
                                  "--operator-confirmed", "--run-dir", str(self.directory), "--recover", "--detach"],
        })

    def unit_info(self, unit):
        content = self.command(["systemctl", "show", unit,
                                "-p", "ActiveState", "-p", "MainPID", "-p", "NRestarts",
                                "-p", "ControlGroup", "-p", "FragmentPath", "-p", "DropInPaths"])
        return dict(line.split("=", 1) for line in content.splitlines() if "=" in line)

    def process_snapshot(self, label):
        records = {}
        for unit in UNITS:
            info = self.unit_info(unit)
            tree = process_tree(int(info.get("MainPID", "0")))
            members = []
            for pid, started in tree.items():
                try:
                    path = Path("/proc") / pid
                    env = dict(item.split("=", 1) for item in (path / "environ").read_bytes().decode(errors="replace").split("\0") if "=" in item)
                    members.append({"pid": int(pid), "start_ticks": started,
                                    "cmdline": (path / "cmdline").read_bytes().decode(errors="replace").replace("\0", " "),
                                    "environment": {key: value for key, value in env.items() if key.startswith(("ROS", "RMW", "CYCLONE", "FAST", "AMENT", "COLCON", "LD_LIBRARY", "PYTHONPATH", "MMP"))},
                                    "middleware_libraries": [line for line in (path / "maps").read_text().splitlines() if re.search(r"(?:librmw_|libddsc|libfastdds|libfastrtps)", line)]})
                except OSError as error:
                    members.append({"pid": int(pid), "error": str(error)})
            records[unit] = {"systemd": info, "processes": members}
        dump(self.directory / label / "processes.json", records)
        return records

    def reset_guard(self):
        self.units_state = {unit: self.unit_info(unit) for unit in self.active}
        self.tree = {unit: persistent_ros_tree(int(info.get("MainPID", "0"))) for unit, info in self.units_state.items()}
        raw = self.command(["journalctl", "-n", "0", "--show-cursor", "--no-pager"], check=False)
        match = re.search(r"-- cursor: (\S+)", raw)
        if not match:
            raise RuntimeError("Could not establish a journal cursor for the driver guard")
        self.journal_cursor = match.group(1)

    def guard_services(self):
        for unit in self.active:
            info = self.unit_info(unit)
            prior = self.units_state[unit]
            if any(info.get(key) != prior.get(key) for key in ("MainPID", "NRestarts", "ActiveState")):
                return f"Unexpected service restart/state change: {unit}"
            tree = persistent_ros_tree(int(info.get("MainPID", "0")))
            # Losing/replacing any initial descendant catches launch respawns as well.
            if any(tree.get(pid) != started for pid, started in self.tree[unit].items()):
                lost = {pid: started for pid, started in self.tree[unit].items() if tree.get(pid) != started}
                return f"Unexpected ROS worker exit/restart: {unit}; missing identities={lost}"
            self.tree[unit].update(tree)
        raw = self.command(["journalctl", "--after-cursor", self.journal_cursor,
                            "--show-cursor", "-o", "json", "--no-pager"], check=False)
        for line in raw.splitlines():
            if line.startswith("-- cursor:"):
                self.journal_cursor = line.split(":", 1)[1].strip()
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            unit = record.get("_SYSTEMD_UNIT", "")
            if unit not in ("clearpath-sensors.service", "realsense-camera.service", "clearpath-scan-merger.service"):
                continue
            message = record.get("MESSAGE", "")
            if not isinstance(message, str):
                continue
            with (self.directory / "journal-events.jsonl").open("a") as stream:
                stream.write(json.dumps(record) + "\n")
            if "type hash" in message.lower() or "type_hash" in message.lower():
                continue
            if ERROR.search(message):
                return f"Driver journal error in {unit}: {message}"
        return None

    def validate(self):
        if os.geteuid() != 0:
            raise RuntimeError("Run this controller with sudo; packet headers, journals, and service reloads need root")
        if socket.gethostname() != "cpr-r100-0160":
            raise RuntimeError("This controller is restricted to Intel cpr-r100-0160")
        import yaml
        robot = yaml.safe_load(ROBOT.read_text())
        if robot.get("serial_number") != "r100-0160" or robot["system"]["ros2"]["namespace"] != "r100_0160":
            raise RuntimeError("Robot identity/namespace mismatch")
        if robot["system"]["ros2"]["middleware"]["implementation"] != "rmw_cyclonedds_cpp":
            raise RuntimeError("Installed robot must already select CycloneDDS for A")
        ip = json.loads(self.command(["ip", "-j", "address", "show", "br0"]))
        if not any(item.get("local") == "192.168.131.1" for dev in ip for item in dev.get("addr_info", [])):
            raise RuntimeError("Intel br0 lacks 192.168.131.1")
        for port in PORTS:
            if not (Path("/sys/class/net") / port).exists():
                raise RuntimeError(f"Missing required physical port {port}")
        if not PROBE.is_file():
            raise RuntimeError("Measurement harness is missing")
        for name in ("tcpdump", "systemctl", "journalctl", "runuser", "ss"):
            if shutil.which(name) is None:
                raise RuntimeError(f"Missing required executable {name}")
        launch = (MERGER / "launch/merge_ridgeback.launch.py").read_text()
        if '"rmw_fastrtps_cpp"' not in launch:
            raise RuntimeError("Scan-merger Fast DDS override was not found; inspect before testing")
        if self.verify_current:
            ET.fromstring(DDS.read_bytes())
        else:
            spdp_only(DDS.read_bytes())
        if DDS.is_symlink():
            raise RuntimeError("Installed DDS file is a symlink; inspect its ownership before this experiment")

    def snapshot(self):
        before = self.directory / "before"
        before.mkdir(exist_ok=True)
        self.original_dds = DDS.read_bytes()
        metadata = DDS.stat()
        self.dds_meta = (stat.S_IMODE(metadata.st_mode), metadata.st_uid, metadata.st_gid)
        (before / "cyclonedds.xml").write_bytes(self.original_dds)
        dump(before / "dds-metadata.json", self.dds_meta)
        for path in (ROBOT, START, Path("/etc/clearpath/setup.bash"),
                     MERGER / "launch/merge_ridgeback.launch.py", MERGER / "config/params_ridgeback.yaml"):
            target = before / "files" / str(path).lstrip("/")
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
        self.original_active = {unit: self.unit_info(unit)["ActiveState"] for unit in UNITS}
        self.active = [unit for unit, status in self.original_active.items() if status == "active"]
        for required in ("clearpath-sensors.service", "clearpath-scan-merger.service", "realsense-camera.service", "depth-to-mono8.service", "ridgeback-camera-mjpeg.service"):
            if required not in self.active:
                raise RuntimeError(f"Required sensor/vendor consumer is not active: {required}")
        dump(before / "active-services.json", self.original_active)
        for unit in UNITS:
            self.command(["systemctl", "cat", unit], check=False, output=before / (unit + ".txt"))
            for base in (Path("/etc/systemd/system"), Path("/usr/lib/systemd/system")):
                for source in [base / unit, *(base / (unit + ".d")).glob("*")]:
                    if source.is_file():
                        target = before / "files" / str(source).lstrip("/")
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(source, target)
        if OVERRIDE.is_symlink():
            raise RuntimeError("Experiment override path is already a symlink; refusing")
        if OVERRIDE.exists():
            self.original_override = OVERRIDE.read_bytes()
            metadata = OVERRIDE.stat()
            self.override_meta = (stat.S_IMODE(metadata.st_mode), metadata.st_uid, metadata.st_gid)
            (before / "camera-override.conf").write_bytes(self.original_override)
            dump(before / "camera-override-metadata.json", self.override_meta)
        else:
            (before / "camera-override.absent").touch()
        dump(before / "usb-camera.json", d455_serial())
        legacy = {str(path): digest(path) for path in LEGACY.rglob("*") if path.is_file()}
        dump(before / "legacy-backup-hashes.json", legacy)
        self.process_snapshot("before")
        self.command(["dpkg-query", "-W"], output=before / "packages.txt")
        self.command(["git", "-C", str(ROOT), "status", "--porcelain=v1"], output=before / "git-status.txt")
        self.command(["git", "-C", str(ROOT), "rev-parse", "HEAD"], output=before / "git-revision.txt")
        self.command(["git", "-C", str(ROOT), "diff", "--binary", "HEAD"], output=before / "dirty.patch")
        names = self.command(["git", "-C", str(ROOT), "ls-files", "--others", "--exclude-standard"]).splitlines()
        for name in names:
            source = ROOT / name
            if source.is_file() and not name.startswith("artifacts/"):
                target = before / "untracked" / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
        self.command(["journalctl", "--since", "-15min", "--no-pager"], check=False, output=before / "journal.txt")
        if self.verify_current:
            dump(before / 'preserved-file-metadata.json', {
                str(path.relative_to(before / 'files')): {
                    'metadata': [stat.S_IMODE(source.stat().st_mode), source.stat().st_uid, source.stat().st_gid],
                    'symlink_target': os.readlink(source) if source.is_symlink() else None}
                for path in (before / 'files').rglob('*') if path.is_file()
                for source in [Path('/') / path.relative_to(before / 'files')]})
        self.profile = self.directory / ("reader-current.xml" if self.verify_current else "reader-A.xml")
        self.profile.write_bytes(self.original_dds)
        os.chmod(self.profile, 0o644)
        self.state()
        account = pwd.getpwnam("robot")
        for path in (self.directory / "ros-logs", self.directory / "measurements"):
            path.mkdir(exist_ok=True)
            os.chown(path, account.pw_uid, account.pw_gid)
        self.prepared = True
        self.state()
        self.say("Snapshots saved; installed XML and legacy backup remain unchanged" +
                 (" for current-profile verification" if self.verify_current else " for A"))

    def network(self):
        return {path.name: {kind: int((path / "statistics" / (kind + "_bytes")).read_text())
                            for kind in ("rx", "tx")}
                for path in Path("/sys/class/net").iterdir() if path.name != "lo"}

    def evidence(self, directory):
        directory.mkdir(parents=True, exist_ok=True)
        self.command(["ss", "-tanp"], check=False, output=directory / "tcp-sessions.txt")
        self.command(["ps", "-eo", "pid,ppid,lstart,pcpu,pmem,args"], check=False, output=directory / "process-state.txt")
        self.command(["journalctl", "--since", "@" + str(int(self.start_time)), "--no-pager"], check=False, output=directory / "journal.txt")
        self.process_snapshot(str(directory.relative_to(self.directory) / "failure-processes"))

    def stop_cameras(self):
        for process in self.cameras:
            stop_group(process)
        self.cameras.clear()

    def restart_active(self):
        for unit in self.active:
            self.command(["systemctl", "restart", unit], timeout=120)
        self.ready()

    def ready(self):
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            if all(self.unit_info(unit).get("ActiveState") == "active" for unit in self.active):
                # systemd "active" does not guarantee a ros2 launch child loaded DDS.
                required = ("clearpath-sensors.service", "realsense-camera.service", "clearpath-scan-merger.service")
                healthy = True
                for unit in required:
                    if unit not in self.active:
                        continue
                    members = process_tree(int(self.unit_info(unit).get("MainPID", "0")))
                    actual = []
                    for pid in members:
                        if pid == self.unit_info(unit).get('MainPID'):
                            continue
                        try:
                            maps = (Path("/proc") / pid / "maps").read_text()
                            if re.search(r"librmw_(?:cyclonedds_cpp|fastrtps_cpp)\.so", maps):
                                actual.append(pid)
                        except OSError:
                            continue
                    count = 2 if unit in ("clearpath-sensors.service", "clearpath-scan-merger.service") else 1
                    healthy = healthy and len(actual) >= count
                if healthy:
                    self.reset_guard()
                    return
            time.sleep(1)
        raise RuntimeError("Previously active services did not become active after commanded restart")

    def verify_profile(self, label):
        records = self.process_snapshot(label)
        for unit in self.active:
            processes = records[unit]["processes"]
            relevant = [p for p in processes if any(re.search(r"librmw_(?:cyclonedds_cpp|fastrtps_cpp)\.so", line)
                                                   for line in p.get("middleware_libraries", []))]
            if not relevant:
                # The webserver can spawn its ROS worker shortly after the main process.
                if unit in ("clearpath-sensors.service", "realsense-camera.service", "clearpath-scan-merger.service"):
                    raise RuntimeError(f"No loaded DDS process found for {unit}")
                continue
            for process in relevant:
                env = process.get("environment", {})
                main_pid = str(records[unit].get('systemd', {}).get('MainPID', ''))
                # ros2 launch itself can initialize CycloneDDS for launch_ros
                # introspection. Only the merger/converter workers override RMW.
                worker = unit == 'clearpath-scan-merger.service' and str(process['pid']) != main_pid
                expected = 'rmw_fastrtps_cpp' if worker else 'rmw_cyclonedds_cpp'
                if env.get("RMW_IMPLEMENTATION") != expected:
                    raise RuntimeError(f"Unexpected RMW in {unit} pid {process['pid']}: {env}")
                if expected == "rmw_cyclonedds_cpp" and env.get("CYCLONEDDS_URI") != DDS.as_uri():
                    raise RuntimeError(f"Installed profile not selected in {unit} pid {process['pid']}")
                if env.get("ROS_DOMAIN_ID", "0") != "0":
                    raise RuntimeError(f"Unexpected domain in {unit}")

    def recover_sensors(self, reason, directory):
        self.stop_cameras()
        self.evidence(directory)
        if self.scan:
            stop_group(self.scan)
            self.scan = None
        # Only sensor drivers are recovered when scans are stuck, never the whole robot.
        self.say("Preserved failure evidence before sensor-only recovery: " + reason)
        self.command(["systemctl", "restart", "clearpath-sensors.service"], timeout=120)
        self.ready()

    def graph(self, label):
        directory = self.directory / label
        directory.mkdir(parents=True, exist_ok=True)
        # Discovery/introspection only: no Image subscription and no CLI daemon.
        script = '''import json,time,rclpy
from rclpy.node import Node
rclpy.init()
n=Node("dynamo_dds_graph_snapshot", namespace="r100_0160")
deadline=time.monotonic()+5
while time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.1)
def endpoint(e):
 return {"node_name":e.node_name,"node_namespace":e.node_namespace,"topic_type":e.topic_type,"gid":list(e.endpoint_gid),"qos":{"reliability":str(e.qos_profile.reliability),"durability":str(e.qos_profile.durability),"history":str(e.qos_profile.history),"depth":e.qos_profile.depth}}
graph={topic:{"types":types,"publishers":[endpoint(e) for e in n.get_publishers_info_by_topic(topic)],"subscriptions":[endpoint(e) for e in n.get_subscriptions_info_by_topic(topic)]} for topic,types in n.get_topic_names_and_types()}
print(json.dumps(graph,indent=2))
n.destroy_node();rclpy.shutdown()
'''
        output = self.command(self.ros(["python3", "-c", script]), timeout=20,
                              output=directory / "topics.json")
        # Reject absent required topics before subscribing. QoS is saved for review.
        try:
            graph = json.loads(output)
        except ValueError as error:
            raise RuntimeError("Graph introspection failed") from error
        for suffix in ("sensors/lidar2d_0/scan", "sensors/lidar2d_1/scan", "sensors/scan",
                       "camera/color/image_raw", "camera/depth/image_rect_raw"):
            topic = "/r100_0160/" + suffix
            if not graph.get(topic, {}).get("publishers"):
                raise RuntimeError("Expected sensor publisher missing: " + topic)
        dump(directory / "topics.json", graph)
        return graph

    def marker(self, path, reason):
        try:
            with path.open("x") as stream:
                json.dump({"kind": "controller_guard", "detail": reason, "time": time.time()}, stream)
        except FileExistsError:
            pass

    def trial(self, label, topics=None, readers=1, warmup=10, duration=60,
              short=False, baseline=None):
        """Finite concurrent probes share a capture window; scan guards start first."""
        directory = self.directory / "measurements" / label
        directory.mkdir(parents=True, exist_ok=False)
        account = pwd.getpwnam("robot")
        os.chown(directory, account.pw_uid, account.pw_gid)
        stop = directory / "STOP.json"
        scan_dir = directory / "scans"
        scan_args = ["python3", str(PROBE), "--mode", "scans", "--output-dir", str(scan_dir),
                     "--duration", "0", "--warmup", str(warmup + (3 if topics else 0)),
                     "--discovery-grace", "3", "--stop-file", str(stop), "--name", "dds_scan_guard"]
        if baseline:
            scan_args += ["--baseline", str(baseline)]
        captures = []
        for port in PORTS:
            # Header-only, bounded rotating pcaps: at most 20MB per physical port.
            captures.append(self.launch(["tcpdump", "-i", port, "-n", "-U", "-s", "192", "-B", "4096",
                                         "-Z", "root", "-C", "10", "-W", "2", "-w", str(directory / (port + ".pcap")),
                                         "ip or ip6"], directory / (port + "-tcpdump.log")))
        self.scan = self.launch(self.ros(scan_args), directory / "scans.log")
        cameras = []
        started = time.monotonic()
        next_guard = started
        camera_at = started + 3
        previous = self.network()
        sample_at = started
        prior_excess = {}
        samples = []
        failure = None
        try:
            while True:
                now = time.monotonic()
                if topics and not cameras and now >= camera_at:
                    if stop.exists():
                        failure = "Scan watchdog failed before camera startup"
                        break
                    for number in range(readers):
                        camera_args = ["python3", str(PROBE), "--mode", "camera", "--topics", topics,
                                       "--output-dir", str(directory / f"reader_{number + 1}"),
                                       "--duration", str(duration), "--warmup", str(warmup),
                                       "--stop-file", str(stop), "--name", f"dds_local_reader_{number + 1}"]
                        process = self.launch(self.ros(camera_args), directory / f"reader_{number + 1}.log")
                        cameras.append(process)
                        self.cameras.append(process)
                if stop.exists():
                    failure = stop.read_text()
                if now >= next_guard and not failure:
                    failure = self.guard_services()
                    next_guard = now + 1
                if now - sample_at >= 1:
                    counters = self.network()
                    elapsed = now - sample_at
                    rates = {port: (counters[port]["tx"] - previous[port]["tx"]) * 8 / elapsed / 1e6 for port in PORTS}
                    excess = {port: value - self.baseline_tx[port] for port, value in rates.items()}
                    samples.append({"time_monotonic": now, "rates_mbps": rates, "incremental_mbps": excess,
                                    "controller_cpu_s": time.process_time()})
                    if baseline and tx_violation(excess, prior_excess):
                        failure = "Physical-port incremental TX exceeded 20Mbit/s in two consecutive seconds"
                    prior_excess = excess
                    previous, sample_at = counters, now
                if failure:
                    self.marker(stop, failure)
                    self.stop_cameras()
                    # Scans continue while sessions, process identity and journals are preserved.
                    self.evidence(directory)
                    break
                if any(process.poll() is not None for process in captures):
                    failure = "Packet capture exited unexpectedly"
                    self.marker(stop, failure)
                    continue
                if self.scan.poll() is not None:
                    failure = "Continuous scan guard exited unexpectedly"
                    self.marker(stop, failure)
                    continue
                done = bool(topics and cameras and all(p.poll() is not None for p in cameras))
                if not topics and (scan_dir / "events.jsonl").exists():
                    # Scan baseline's actual startup time includes runuser/imports.
                    with (scan_dir / "events.jsonl").open() as stream:
                        startup = next((json.loads(line) for line in stream if '"event": "startup"' in line), None)
                    done = bool(startup and now >= startup["measurement_start_monotonic_s"] + duration)
                if done:
                    failure = self.guard_services()
                    if failure:
                        self.marker(stop, failure)
                        continue
                    break
                expected = warmup + duration + (3 if topics else 0)
                if now - started > expected + 15:
                    failure = "Probe completion exceeded expected capture window"
                    self.marker(stop, failure)
                    continue
                time.sleep(.1)
        finally:
            self.stop_cameras()
            if self.scan:
                stop_group(self.scan)
                self.scan = None
            for process in captures:
                stop_group(process)
        dump(directory / "controller-network.json", samples)
        summary = {"label": label, "topics": topics, "readers": readers,
                   "warmup_s": warmup, "capture_s": duration, "short_safety_trial": short,
                   "failure": failure, "scans": {}, "cameras": [], "packet_evidence": {}}
        if (scan_dir / "summary.json").exists():
            summary["scans"] = json.loads((scan_dir / "summary.json").read_text())
        for number in range(readers if topics else 0):
            path = directory / f"reader_{number + 1}" / "summary.json"
            if path.exists():
                camera = json.loads(path.read_text())
                timing = analyze_reader(path.parent)
                dump(path.parent / 'image-timing.json', timing)
                camera['image_timing'] = timing
                if short:
                    seen_after_grace = set()
                    for event_line in (path.parent / "events.jsonl").read_text().splitlines():
                        event = json.loads(event_line)
                        if event.get("event") == "image" and event.get("receive_monotonic_s", 0) >= camera["start_monotonic_s"] + 3 and not event.get("empty"):
                            seen_after_grace.add(event["stream"])
                    camera["received_after_grace"] = sorted(seen_after_grace)
                summary["cameras"].append(camera)
            else:
                failure = failure or f"Missing reader_{number + 1} summary"
        self.packet_evidence(directory, summary)
        scan = summary["scans"]
        baseline_data = json.loads(Path(baseline).read_text()) if baseline else None
        summary["scan_guard"] = scan
        summary["scan_windows"] = []
        if scan and not failure:
            windows = [(camera["measurement_start_monotonic_s"], camera["end_monotonic_s"])
                       for camera in summary["cameras"]]
            if not topics:
                windows = [(scan["measurement_start_monotonic_s"], scan["measurement_start_monotonic_s"] + duration)]
            for start, end in windows:
                summary["scan_windows"].append(scan_window(scan_dir / "events.jsonl", start, end, baseline_data))
            if summary["scan_windows"]:
                summary["scans"] = summary["scan_windows"][0]
        measured_scan = summary["scans"]
        if not scan:
            failure = failure or "Missing scan summary"
        elif scan.get("failures") or stop.exists():
            failure = failure or "Scan/safety watchdog captured failure"
        elif short:
            # Five seconds checks readiness/safety; full rates use warmed captures.
            if not all(value.get("received", 0) for value in scan.get("streams", {}).values()):
                failure = "Short safety trial did not receive every scan stream"
        elif not summary["scan_windows"] or not all(window["passed"] for window in summary["scan_windows"]):
            failure = "Scan acceptance failed"
        for camera in summary["cameras"]:
            if camera.get("failures"):
                failure = failure or "Camera reader captured failure"
            elif short:
                if not all(value.get("received", 0) and not value.get("empty") for value in camera.get("streams", {}).values()):
                    failure = failure or "Short safety trial did not receive every requested image stream"
                if set(camera.get("received_after_grace", [])) != set(topics.split(",")):
                    failure = failure or "Short safety trial did not receive every requested image stream after discovery grace"
            elif not camera.get("passed"):
                failure = failure or "Camera rate/gap/exact-match acceptance failed"
        if baseline and not short:
            for port in PORTS:
                value = measured_scan.get("interfaces", {}).get(port, {}).get("mean_tx_mbps")
                if value is None or value - self.baseline_tx[port] > 5:
                    failure = failure or "Mean physical-port incremental TX exceeded 5Mbit/s"
        summary["failure"] = failure
        summary["passed"] = not failure
        dump(directory / "summary.json", summary)
        # This explicit-window summary is the next trial's baseline input.
        dump(directory / "scan-window-summary.json", measured_scan)
        self.say(f"{label}: {'PASS' if not failure else 'FAIL'}" + (" — " + failure if failure else ""))
        return summary, directory / "scan-window-summary.json"

    def packet_evidence(self, directory, summary):
        for port in PORTS:
            counts = {"multicast_packets": 0, "unicast_packets": 0}
            destinations = {}
            for number, path in enumerate(sorted(directory.glob(port + ".pcap*"))):
                output = directory / f"{port}-headers-{number}.txt"
                text = self.command(["tcpdump", "-n", "-r", str(path)], check=False, timeout=45, output=output)
                for line in text.splitlines():
                    match = re.search(r" > ([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)(?:\.\d+)?:", line)
                    if match:
                        dest = match.group(1)
                        multicast = 224 <= int(dest.split(".")[0]) <= 239
                        counts["multicast_packets" if multicast else "unicast_packets"] += 1
                        destinations[dest] = destinations.get(dest, 0) + 1
            summary["packet_evidence"][port] = {**counts, "destination_packets": destinations,
                "capture": "192-byte headers, rotating 2x10MB bounded capture; packet counts are sampled evidence, not throughput"}

    def baseline(self, stage):
        self.say(stage + ": 60-second scan-only baseline")
        summary, path = self.trial(stage + "/baseline", warmup=3, duration=60)
        if summary["passed"]:
            self.baseline_tx = {port: summary["scans"]["interfaces"][port]["mean_tx_mbps"] for port in PORTS}
            self.baseline_merged = summary["scans"]["streams"]["merged"]["rate_hz"]
            # Quiet baseline excludes large camera streams already leaving Intel.
            if any(value > 20 for value in self.baseline_tx.values()):
                summary["passed"] = False
                summary["failure"] = "Quiet baseline physical TX exceeds 20Mbit/s; remote camera streaming cannot be excluded"
                dump(path.parent / "summary.json", summary)
        return summary, path

    def native_stage(self, stage):
        self.stage = stage
        self.state()
        self.ready()
        self.graph(stage + "-graph")
        self.verify_profile(stage + "-effective")
        baseline, path = self.baseline(stage)
        result = {"baseline": baseline, "trials": [], "passed": False, "complete": False}
        self.results[stage] = result
        if not baseline["passed"]:
            self.failures.append(stage + " baseline: " + str(baseline["failure"]))
            self.report()
            return False
        for topics, readers in (("color", 1), ("depth", 1), ("color,depth", 1), ("color,depth", 2)):
            name = topics.replace(",", "_") + "_" + str(readers)
            short_trial, _ = self.trial(stage + "/" + name + "/short", topics, readers,
                                        warmup=0, duration=5, short=True, baseline=path)
            result["trials"].append(short_trial)
            if not short_trial["passed"]:
                self.failures.append(stage + ": " + str(short_trial["failure"]))
                self.report()
                return False
            if self.screening:
                self.report()
                continue
            for repeat in range(3):
                trial, _ = self.trial(stage + "/" + name + f"/repeat_{repeat + 1}",
                                      topics, readers, baseline=path)
                result["trials"].append(trial)
                self.report()
                if not trial["passed"]:
                    self.failures.append(stage + ": " + str(trial["failure"]))
                    self.report()
                    return False
        if self.screening:
            result.update(screening_passed=True, screening_complete=True)
        else:
            result.update(passed=True, complete=True)
        self.report()
        return True

    def install_B(self):
        if self.verify_current:
            raise RuntimeError('Current-profile verification cannot install a DDS candidate')
        if DDS.read_bytes() != self.original_dds:
            raise RuntimeError("Installed DDS configuration changed during A; refusing to overwrite it")
        candidate = spdp_only(self.original_dds)
        self.profile = self.directory / "reader-B.xml"
        self.profile.write_bytes(candidate)
        os.chmod(self.profile, 0o644)
        (self.directory / "dds-B.diff").write_text("".join(difflib.unified_diff(
            self.original_dds.decode().splitlines(keepends=True), candidate.decode().splitlines(keepends=True),
            fromfile=str(DDS) + " before", tofile=str(DDS) + " B")))
        # Publish recovery intent before the first mutation so crash recovery is safe.
        self.dds_changed = True
        self.stage = "B-installing"
        self.state()
        atomic_restore(DDS, candidate, *self.dds_meta)
        self.say("Installed discovery-only multicast; explicitly reloading originally active DDS services")
        self.restart_active()

    def install_aligned(self):
        if self.verify_current:
            raise RuntimeError('Current-profile verification cannot change the camera profile')
        if START.read_bytes() != (self.directory / 'before/files' / str(START).lstrip('/')).read_bytes():
            raise RuntimeError("Camera startup changed during the comparison; refusing a different cold-start recipe")
        device = d455_serial()
        dump(self.directory / "aligned-usb-camera.json", device)
        script = aligned_script(START.read_text(), device["serial"])
        path = self.directory / "camera-aligned-start"
        path.write_text(script)
        os.chmod(path, 0o755)
        self.camera_changed = True
        self.stage = "aligned-installing"
        self.state()
        OVERRIDE.parent.mkdir(exist_ok=True)
        content = "[Service]\nExecStart=\nExecStart=" + str(path) + "\n"
        atomic_restore(OVERRIDE, content.encode(), 0o644, 0, 0)
        self.command(["systemctl", "daemon-reload"])
        self.say("Cold-starting one serial-pinned D455: aligned VGA30, infrared 2 disabled")
        self.command(["systemctl", "restart", "realsense-camera.service"], timeout=120)
        self.ready()
        self.graph("aligned-graph")

    def aligned_and_soak(self):
        self.install_aligned()
        self.stage = "aligned"
        self.state()
        baseline, path = self.baseline("aligned")
        result = {"baseline": baseline, "trials": [], "passed": False, "complete": False}
        self.results["aligned"] = result
        if not baseline["passed"]:
            self.failures.append("Aligned baseline: " + str(baseline["failure"]))
            return False
        short_trial, _ = self.trial("aligned/short", "color,aligned", warmup=0,
                                    duration=5, short=True, baseline=path)
        result["trials"].append(short_trial)
        if not short_trial["passed"]:
            self.failures.append("Aligned short: " + str(short_trial["failure"]))
            return False
        for repeat in range(3):
            trial, _ = self.trial(f"aligned/repeat_{repeat + 1}", "color,aligned", baseline=path)
            result["trials"].append(trial)
            self.report()
            if not trial["passed"]:
                self.failures.append("Aligned: " + str(trial["failure"]))
                return False
        result.update(passed=True, complete=True)
        self.stage = "soak"
        self.state()
        self.say("30-minute soak: vendor consumers and two independent color/aligned-depth reader pairs")
        trial, _ = self.trial("soak", "color,aligned", readers=2, duration=1800, baseline=path)
        self.results["soak"] = {"trials": [trial], "passed": trial["passed"],
                                "complete": trial["passed"] and trial["scans"].get("streams", {}).get("front", {}).get("duration_s", 0) >= 1799.9}
        if not trial["passed"]:
            self.failures.append("Soak: " + str(trial["failure"]))
        self.report()
        return self.results["soak"]["passed"] and self.results["soak"]["complete"]

    def execute(self):
        self.validate()
        self.snapshot()
        if self.verify_current:
            self.stage = 'current-native'
            self.state()
            self.ready()
            self.verify_current_files()
            self.graph('current-native-graph')
            self.verify_profile('current-native-effective')
            baseline, path = self.baseline('current-native')
            result = {'baseline': baseline, 'trials': [], 'passed': False, 'complete': False}
            self.results['current-native'] = result
            if baseline['passed']:
                self.say('Current profile: two independent native color/depth reader pairs, 10-second warm-up and 60-second capture')
                trial, _ = self.trial('current-native/color_depth_2/capture', 'color,depth', readers=2,
                                      warmup=10, duration=60, baseline=path)
                result['trials'].append(trial)
                result.update(passed=trial['passed'], complete=trial['passed'])
                if not trial['passed']:
                    self.failures.append('Current profile: ' + str(trial['failure']))
            else:
                self.failures.append('Current profile baseline: ' + str(baseline['failure']))
            self.verify_current_files()
            self.say('Current-profile capture finished; installed settings remain unchanged. No A/B comparison, alignment or soak was performed.')
            self.report()
            return
        if self.focused_native:
            self.install_B()
            self.stage = 'B-focused'
            self.state()
            self.graph('B-focused-graph')
            self.verify_profile('B-focused-effective')
            baseline, path = self.baseline('B-focused')
            result = {'baseline': baseline, 'trials': [], 'passed': False, 'complete': False}
            self.results['B-focused'] = result
            if baseline['passed']:
                self.say('B-focused: two independent native color/depth reader pairs, 10-second warm-up and 60-second capture')
                trial, _ = self.trial('B-focused/color_depth_2/capture', 'color,depth', readers=2,
                                      warmup=10, duration=60, baseline=path)
                result['trials'].append(trial)
                result.update(passed=trial['passed'], complete=trial['passed'])
                if not trial['passed']:
                    self.failures.append('B-focused: ' + str(trial['failure']))
            else:
                self.failures.append('B-focused baseline: ' + str(baseline['failure']))
            self.retain = bool(self.retain_passing_native and result['passed'] and not self.failures)
            self.say('Focused capture finished; ' + ('keeping the passing discovery-only multicast setting.'
                     if self.retain else 'restoring original settings.') +
                     ' Full matrix, alignment and soak remain untested.')
            self.report()
            return
        passed_a = self.native_stage("A")
        if 'Quiet baseline' in str(self.results['A'].get('baseline', {}).get('failure')):
            raise RuntimeError('Local-only comparison is inconclusive: exclude remote camera streaming before rerunning')
        if not passed_a and needs_sensor_recovery(self.results['A']):
            self.recover_sensors("A failed; preparing controlled B", self.directory / "measurements" / "A" / "baseline")
        self.install_B()
        passed_b = self.native_stage("B")
        if not passed_b:
            self.failures.append("B failed; restoring exact original DDS XML")
            atomic_restore(DDS, self.original_dds, *self.dds_meta)
            self.dds_changed = False
            self.profile = self.directory / "reader-A.xml"
            self.stage = "A-restored-after-B"
            self.state()
            self.restart_active()
        if self.screening:
            self.say('Short A/B diagnostic finished; restoring original settings. Sustained, aligned and soak tests remain untested.')
            self.retain = False
        elif passed_b or passed_a:
            final_pass = self.aligned_and_soak()
            self.retain = bool(passed_b and final_pass)
        self.report()

    def report(self):
        report = {"results": self.results, "failures": self.failures,
                  "mode": self.mode(),
                  "current_profile_only": self.verify_current,
                  "comparison_performed": bool(self.results['A'].get('baseline') and self.results['B'].get('baseline')),
                  "installed_profile_sha256": (hashlib.sha256(self.original_dds).hexdigest()
                                                if self.original_dds is not None else None),
                  "retain_passing_native_requested": self.retain_passing_native,
                  "retained_B": self.retain, "current_stage": self.stage,
                  "observed": [item for item in self.failures],
                  "not_reproduced": (["Historical camera/subscriber failure during the complete native A matrix"]
                                     if self.results["A"].get("passed") else []),
                  "still_untested": [name for name, value in self.results.items() if not value.get("complete")],
                  "limits": (["Only the current installed profile was verified; no A/B comparison or new B retention"]
                             if self.verify_current else ["Multicast causality remains a hypothesis"]) +
                            ["No intentional LiDAR interruption or multi-hour recovery qualification", "Thor, clocks, HD, TF and navigation excluded"]}
        dump(self.directory / "report.json", report)
        lines = ["# Intel DDS controlled local-reader test", "", "| Stage | Complete | Passed | Captures |", "|---|---|---|---|"]
        for name, value in self.results.items():
            lines.append(f"| {name} | {value.get('complete', False)} | {value.get('passed', False)} | {len(value.get('trials', []))} |")
        if self.screening:
            lines += ['', 'Short diagnostic only: five-second camera trials check receipt and immediate stop conditions. They do not qualify sustained rates, alignment, or soak behavior. The original DDS configuration is restored even if screening passes.']
            for name in ('A', 'B'):
                lines.append(f"{name} short matrix passed: {self.results[name].get('screening_passed', False)}")
        if self.focused_native:
            lines += ['', 'Focused B native capture only: two independent color/depth reader pairs, ten seconds of warm-up and sixty seconds measured. Camera rate/gap and scan/traffic acceptance apply. The full A/B matrix, alignment and soak remain untested. ' +
                      ('Explicit retention requested: keep B only after capture and final verification pass; failure restores the original settings.'
                       if self.retain_passing_native else 'Original settings are always restored.')]
        if self.verify_current:
            lines += ['', 'Current installed profile only: quiet scan baseline followed by two independent native color/depth reader pairs, ten seconds of warm-up and sixty seconds measured. Installed DDS bytes, metadata and service activity are preserved; no DDS installation, service reload, A/B comparison or new B retention is performed. Alignment and soak remain untested.']
        lines += ['', '| Trial / reader | Images Hz | Image max receive / stamp gap (ms) | Exact match color / aligned (%) | Front / rear / merged Hz | Port TX increase eno1 / enp2s0 (Mbit/s) | Multicast packets eno1 / enp2s0 | Result |',
                  '|---|---|---|---|---|---|---|---|']
        def number(value, scale=1):
            return '—' if value is None else f'{value * scale:.2f}'
        for stage, result in self.results.items():
            for trial in ([result['baseline']] if 'baseline' in result else []) + result.get('trials', []):
                scans = trial.get('scans', {})
                scan_rates = ' / '.join(number(scans.get('streams', {}).get(name, {}).get('rate_hz')) for name in ('front', 'rear', 'merged'))
                ports = ' / '.join(number(scans.get('interfaces', {}).get(port, {}).get('incremental_tx_mbps')) for port in PORTS)
                multicast = ' / '.join(str(trial.get('packet_evidence', {}).get(port, {}).get('multicast_packets', '—')) for port in PORTS)
                for index, camera in enumerate(trial.get('cameras') or [{}]):
                    images = ', '.join(name + '=' + number(stream.get('rate_hz')) for name, stream in camera.get('streams', {}).items()) or '—'
                    gaps = ', '.join(name + '=' + number(stream.get('max_receive_gap_s'), 1000) + '/' + number(stream.get('max_stamp_gap_s'), 1000) for name, stream in camera.get('streams', {}).items()) or '—'
                    match = camera.get('exact_matches') or {}
                    matches = ' / '.join(number(match.get(name), 100) for name in ('color_ratio', 'aligned_ratio'))
                    label = trial.get('label', stage) + (f' / {index + 1}' if camera else '')
                    outcome = 'PASS' if trial.get('passed') else 'FAIL'
                    if trial.get('short_safety_trial'):
                        outcome += ' safety-only'
                    lines.append(f'| {label} | {images} | {gaps} | {matches} | {scan_rates} | {ports} | {multicast} | {outcome} |')
        lines += ["", "Camera acceptance covers measured windows after warm-up. Scan safety covers subscriber startup through the last reader exit. Observer callback overhead includes warm-up. Short trials check safety and receipt, not camera-rate acceptance.",
                  '', 'Packet counts are bounded IPv4 header samples, including discovery and unrelated traffic; they are not camera-specific throughput or proof of causation. Full per-reader scan windows, metadata, and CPU timing remain in trial summaries.', "", "Observed failures:"]
        lines += ["- " + failure for failure in self.failures] or ["- None captured."]
        lines += ["", "Not reproduced: " + "; ".join(report["not_reproduced"] or
                  ["No historical comparison performed." if self.verify_current else "No complete passing A matrix yet."]),
                  "", "Still untested: " + ", ".join(report["still_untested"] or ["None of the requested stages."]),
                  "", ("Installed profile unchanged; no new B retention." if self.verify_current else "B retained: " + str(self.retain)), "", *report["limits"]]
        (self.directory / "report.md").write_text("\n".join(lines) + "\n")

    def verify_current_files(self):
        """Detect external changes without rewriting any installed file."""
        if DDS.read_bytes() != self.original_dds:
            raise RuntimeError('Installed DDS changed during current-profile verification')
        metadata = DDS.stat()
        if (stat.S_IMODE(metadata.st_mode), metadata.st_uid, metadata.st_gid) != self.dds_meta:
            raise RuntimeError('Installed DDS metadata changed during current-profile verification')
        if self.profile.read_bytes() != self.original_dds:
            raise RuntimeError('Current reader profile differs from the installed snapshot')
        before = self.directory / 'before'
        saved_metadata = json.loads((before / 'preserved-file-metadata.json').read_text())
        for name, expected in saved_metadata.items():
            source, saved = Path('/') / name, before / 'files' / name
            metadata = source.stat()
            actual_meta = [stat.S_IMODE(metadata.st_mode), metadata.st_uid, metadata.st_gid]
            link = os.readlink(source) if source.is_symlink() else None
            if (link != expected['symlink_target'] or source.read_bytes() != saved.read_bytes() or
                    actual_meta != expected['metadata']):
                raise RuntimeError('Preserved deployment file changed: ' + str(source))
        if (OVERRIDE.exists() != (self.original_override is not None) or
                (self.original_override is not None and OVERRIDE.read_bytes() != self.original_override)):
            raise RuntimeError('Camera override changed during current-profile verification')
        legacy = {str(path): digest(path) for path in LEGACY.rglob('*') if path.is_file()}
        if legacy != json.loads((before / 'legacy-backup-hashes.json').read_text()):
            raise RuntimeError('Legacy backup changed during current-profile verification')

    def restore(self):
        errors = []
        def attempt(label, action):
            try:
                action()
            except Exception as error:
                errors.append(f'{label}: {type(error).__name__}: {error}')

        attempt('Stopping camera readers', self.stop_cameras)
        for process in self.processes:
            attempt('Stopping owned subprocess', lambda process=process: stop_group(process))
        if self.verify_current and (self.camera_changed or self.dds_changed or self.retain):
            errors.append('Current-profile verification unexpectedly requested configuration mutation')
        if self.camera_changed and not self.verify_current:
            if self.original_override is None:
                OVERRIDE.unlink(missing_ok=True)
            else:
                atomic_restore(OVERRIDE, self.original_override, *self.override_meta)
            attempt('Reloading original camera unit', lambda: self.command(["systemctl", "daemon-reload"]))
            if self.original_active.get("realsense-camera.service") == "active":
                attempt('Restarting original camera', lambda: self.command(["systemctl", "restart", "realsense-camera.service"], timeout=120))
            self.camera_changed = False
        if self.dds_changed and not self.retain and not self.verify_current:
            atomic_restore(DDS, self.original_dds, *self.dds_meta)
            self.profile = self.directory / 'reader-A.xml'
            for unit in self.active:
                attempt('Reloading ' + unit, lambda unit=unit: self.command(['systemctl', 'restart', unit], timeout=120))
            if not errors:
                self.dds_changed = False
        # Restore only activity differences in our explicit unit set; no global stops.
        for unit, status in self.original_active.items():
            current = self.unit_info(unit).get("ActiveState")
            if self.verify_current:
                if current != status:
                    errors.append('Service activity changed during current-profile verification: ' + unit)
            elif status == "active" and current != "active":
                attempt('Restoring activity ' + unit, lambda unit=unit: self.command(["systemctl", "start", unit], timeout=120))
            elif status != "active" and current == "active":
                attempt('Restoring inactivity ' + unit, lambda unit=unit: self.command(["systemctl", "stop", unit], timeout=120))
        if self.original_dds is not None and self.original_active and not errors:
            try:
                if self.verify_current:
                    self.verify_current_files()
                    failure = self.guard_services()
                    if failure:
                        raise RuntimeError(failure)
                expected_dds = spdp_only(self.original_dds) if self.retain else self.original_dds
                if DDS.read_bytes() != expected_dds:
                    raise RuntimeError('Installed DDS differs from the selected final configuration')
                if not self.verify_current:
                    self.ready()
                self.verify_profile('restored-effective')
                final, _ = self.trial('restore-verification', warmup=3, duration=10)
                self.results['restoration'] = {'complete': True, 'passed': final['passed'], 'trials': [final]}
                if not final['passed']:
                    raise RuntimeError('Three-scan restoration verification failed: ' + str(final['failure']))
                if self.verify_current:
                    self.verify_current_files()
                    if any(self.unit_info(unit).get('ActiveState') != status
                           for unit, status in self.original_active.items()):
                        raise RuntimeError('Service activity changed during final current-profile verification')
                before = self.directory / 'before'
                for source in (ROBOT, START, MERGER / 'launch/merge_ridgeback.launch.py', MERGER / 'config/params_ridgeback.yaml'):
                    saved = before / 'files' / str(source).lstrip('/')
                    if digest(source) != digest(saved):
                        raise RuntimeError('Preserved deployment file changed: ' + str(source))
                legacy = {str(path): digest(path) for path in LEGACY.rglob('*') if path.is_file()}
                if legacy != json.loads((before / 'legacy-backup-hashes.json').read_text()):
                    raise RuntimeError('Legacy backup changed during experiment')
            except Exception as error:
                errors.append(str(error))
        if errors and self.retain and not self.verify_current:
            # A failed restoration is not permission to retain the candidate.
            self.retain = False
            self.dds_changed = True
            self.state()
            atomic_restore(DDS, self.original_dds, *self.dds_meta)
            self.profile = self.directory / 'reader-A.xml'
            for unit in self.active:
                attempt('Reverting retained candidate ' + unit, lambda unit=unit: self.command(['systemctl', 'restart', unit], timeout=120))
        if errors and self.verify_current:
            attempt('Preserving final verification evidence', lambda: self.evidence(self.directory / 'verification-failure'))
        self.failures.extend('Cleanup: ' + item for item in errors)
        self.stage = ('verification-failed' if errors else 'current-profile-verified'
                      if self.results.get('current-native', {}).get('passed') else 'current-profile-unchanged') if self.verify_current else (
            'recovery-required' if errors else 'candidate-retained' if self.retain else 'restored')
        self.state()
        self.report()
        return not errors


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed", action="store_true", required=True,
                        help="Robot stationary, operator present, e-stop available")
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--detach", action="store_true",
                        help="Launch in an independent systemd service; survives vendor VNC restarts")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--recover", action="store_true", help="Recover this run's exact staged files/activity")
    mode.add_argument('--verify-current', action='store_true',
                      help='Verify the unchanged installed DDS profile with two warmed native reader pairs; never install DDS or reload services')
    mode.add_argument("--screening", action="store_true",
                        help="Only 60-second A/B baselines and five-second reader trials; always restore original DDS")
    mode.add_argument('--focused-native', action='store_true',
                      help='Only B scan baseline and one warmed 60-second two-pair native capture; restores DDS unless explicitly retained')
    parser.add_argument('--retain-passing-native', action='store_true',
                        help='Explicitly keep B after a passing focused native capture and final verification; full qualification remains untested')
    args = parser.parse_args()
    if args.retain_passing_native and (not args.focused_native or args.recover):
        parser.error('--retain-passing-native requires --focused-native and cannot accompany --recover')
    return args


def recover(controller):
    """Recover only the exact staged files and owned process groups of this run."""
    if socket.gethostname() != 'cpr-r100-0160':
        raise RuntimeError('Recovery is restricted to Intel cpr-r100-0160')
    saved = json.loads((controller.directory / 'CURRENT_CONFIG.json').read_text())
    controller.screening = saved.get('mode') == 'screening'
    controller.focused_native = saved.get('mode') == 'focused-native'
    controller.verify_current = saved.get('mode') == 'verify-current'
    controller.retain_passing_native = saved.get('retain_passing_native_requested', False)
    before = controller.directory / 'before'
    controller.original_dds = (before / 'cyclonedds.xml').read_bytes()
    controller.dds_meta = tuple(json.loads((before / 'dds-metadata.json').read_text()))
    controller.original_active = json.loads((before / 'active-services.json').read_text())
    controller.active = [unit for unit, status in controller.original_active.items() if status == 'active']
    controller.dds_changed = saved['dds_changed']
    controller.camera_changed = saved['camera_changed']
    if controller.verify_current and (controller.dds_changed or controller.camera_changed or saved.get('retain_B', False)):
        raise RuntimeError('Current-profile recovery has unexpected mutation flags; inspect before recovering')
    controller.prepared = saved.get('prepared', True)
    controller.profile = controller.directory / ('reader-current.xml' if controller.verify_current else 'reader-A.xml')
    controller.start_time = saved['started']
    report = controller.directory / 'report.json'
    if report.exists():
        prior = json.loads(report.read_text())
        controller.results = prior.get('results', controller.results)
        controller.failures = prior.get('failures', [])
    if controller.dds_changed and DDS.read_bytes() not in (controller.original_dds, spdp_only(controller.original_dds)):
        raise RuntimeError('DDS file differs from this experiment; inspect it before recovering')
    if (before / 'camera-override.conf').exists():
        controller.original_override = (before / 'camera-override.conf').read_bytes()
        controller.override_meta = tuple(json.loads((before / 'camera-override-metadata.json').read_text()))
    if controller.camera_changed and OVERRIDE.exists():
        expected = ('[Service]\nExecStart=\nExecStart=' + str(controller.directory / 'camera-aligned-start') + '\n').encode()
        if OVERRIDE.read_bytes() not in (expected, controller.original_override):
            raise RuntimeError('Camera override differs from this experiment; inspect it before recovering')
    for record in saved.get('owned_process_groups', []):
        pid = record['pid']
        if process_tree(pid).get(str(pid)) != record['start_ticks']:
            continue
        if os.getpgid(pid) != pid:
            raise RuntimeError('Saved process is no longer its group leader; refusing to signal it')
        os.killpg(pid, signal.SIGTERM)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and process_tree(pid).get(str(pid)) == record['start_ticks']:
            time.sleep(.1)
        if process_tree(pid).get(str(pid)) == record['start_ticks']:
            os.killpg(pid, signal.SIGKILL)
    # Recovery is an explicit exact-original restoration, even for a completed B.
    controller.retain = False
    controller.dds_changed = controller.dds_changed or saved.get('retain_B', False)
    if controller.verify_current:
        # A verification never owned an installed-file change to undo.
        controller.ready()
    # Keep evidence from any prior restoration attempt rather than overwriting it.
    verification = controller.directory / 'measurements/restore-verification'
    if verification.exists():
        verification.rename(verification.with_name('restore-verification-before-recovery-' + str(time.time_ns())))
    return controller.restore()


def ensure_independent_controller(cgroup_text=None):
    """A service reload must never terminate the controller's own desktop/session."""
    if cgroup_text is None:
        cgroup_text = Path('/proc/self/cgroup').read_text()
    for line in cgroup_text.splitlines():
        path = line.split(':', 2)[-1]
        for component in path.split('/'):
            if component in UNITS:
                raise RuntimeError(
                    f'This terminal belongs to {component}; restarting it kills the '
                    'desktop and experiment controller. Run in an independent '
                    'systemd service or SSH session. No configuration was changed.')


def detached_command(directory, recovery=False, screening=False, focused_native=False, retain_passing_native=False,
                     verify_current=False):
    token = hashlib.sha256(str(directory).encode()).hexdigest()[:12]
    unit = 'dynamo-dds-' + token + ('-recovery' if recovery else '')
    command = ['systemd-run', '--unit=' + unit, '--collect',
               '--property=WorkingDirectory=' + str(ROOT),
               '--property=TimeoutStopSec=600', '--property=KillMode=mixed',
               '--', '/usr/bin/python3', str(Path(__file__).resolve()),
               '--operator-confirmed', '--run-dir', str(directory)]
    if recovery:
        command.append('--recover')
    if screening:
        command.append('--screening')
    if focused_native:
        command.append('--focused-native')
    if retain_passing_native:
        command.append('--retain-passing-native')
    if verify_current:
        command.append('--verify-current')
    return unit + '.service', command


def vendor_camera_retired():
    """True when dynamo-camera.service owns the D455 instead of the vendor unit.

    This runner restarts realsense-camera.service and rewrites its start script
    to toggle alignment; under the camera service neither exists in that form.
    """

    def active(unit):
        return subprocess.run(['systemctl', 'is-active', '--quiet', unit], check=False).returncode == 0

    return active('dynamo-camera.service') or not active('realsense-camera.service')


def main():
    args = parse_args()
    if not args.recover and vendor_camera_retired():
        print('dynamo-camera.service owns the camera; this runner only drives the retired vendor '
              'realsense-camera unit. Use tools/camera_contract_check (see "Local reader checks" in '
              'docs/physical/robot_local_deployment.md). --recover still restores old runs.', file=sys.stderr)
        return 2
    if os.geteuid() != 0:
        print('Run with sudo: packet capture and service reloads require root.', file=sys.stderr)
        return 2
    directory = args.run_dir.resolve()
    evidence_root = (ROOT / 'artifacts/hardware').resolve()
    if not directory.is_relative_to(evidence_root) or directory == evidence_root:
        print('Choose a run directory below this checkout\'s artifacts/hardware/.', file=sys.stderr)
        return 2
    if not args.recover and directory.exists() and any(directory.iterdir()):
        print('Choose an empty/new run directory; existing evidence will not be overwritten.', file=sys.stderr)
        return 2
    if args.detach:
        unit, command = detached_command(directory, args.recover, args.screening, args.focused_native,
                                         args.retain_passing_native, args.verify_current)
        result = subprocess.run(command, check=False)
        if result.returncode == 0:
            print('Independent controller: ' + unit, flush=True)
            print('Controller continues through service reloads; VNC is preserved when its independent services are installed.', flush=True)
            print('Follow: journalctl -fu ' + unit, flush=True)
            print('Stop and restore: sudo systemctl kill --kill-whom=main --signal=SIGINT ' + unit, flush=True)
            print('Report: ' + str(directory / 'report.md'), flush=True)
        return result.returncode
    try:
        ensure_independent_controller()
    except (RuntimeError, OSError) as error:
        print(str(error), file=sys.stderr)
        return 2
    lock = open('/run/lock/dynamo-local-dds-test.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Another local DDS experiment or recovery is running.', file=sys.stderr)
        lock.close()
        return 2
    controller = Controller(directory)
    controller.screening = args.screening
    controller.focused_native = args.focused_native
    controller.verify_current = args.verify_current
    controller.retain_passing_native = args.retain_passing_native
    def interrupted(signum, frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    successful = False
    cleanup_ok = False
    try:
        if args.recover:
            cleanup_ok = recover(controller)
            successful = cleanup_ok
        else:
            controller.execute()
            successful = (bool(controller.results.get('current-native', {}).get('passed')) if controller.verify_current else
                          bool(controller.results.get('B-focused', {}).get('passed')) if controller.focused_native else
                          bool(controller.results['B'].get('screening_passed')) if controller.screening else
                          bool(controller.results['aligned'].get('passed') and controller.results['soak'].get('complete')))
    except (Exception, KeyboardInterrupt) as error:
        controller.retain = False
        controller.failures.append(f'{type(error).__name__}: {error}')
        (directory / 'exception.txt').write_text(traceback.format_exc())
        controller.say('Experiment stopped: ' + str(error))
    finally:
        # Once interrupted, let exact-file restoration finish without a second
        # Ctrl-C abandoning the robot halfway through a service reload.
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(signum, signal.SIG_IGN)
        if not args.recover and controller.prepared:
            try:
                cleanup_ok = controller.restore()
            except Exception as error:
                controller.retain = False
                controller.failures.append('Cleanup incomplete: ' + str(error))
                controller.stage = 'recovery-required'
                controller.state()
                controller.report()
                controller.say('Cleanup incomplete. Use the recovery command in CURRENT_CONFIG.json.')
        else:
            controller.report()
        files = {}
        for path in directory.rglob('*'):
            if path.is_file() and path.name != 'SHA256SUMS':
                files[str(path.relative_to(directory))] = digest(path)
        (directory / 'SHA256SUMS').write_text(''.join(f'{value}  {name}\n' for name, value in sorted(files.items())))
        controller.say('Report: ' + str(directory / 'report.md'))
        lock.close()
    return 0 if successful and cleanup_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
