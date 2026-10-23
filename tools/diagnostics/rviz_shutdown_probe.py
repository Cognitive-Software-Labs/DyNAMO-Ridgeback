#!/usr/bin/env python3
"""Bounded RViz process-lifetime probe; software rendering is not GPU qualification.

Source ROS and the desired workspace first. Example:
  python3 tools/diagnostics/rviz_shutdown_probe.py --configs empty tf image

Only direct children created by this probe receive signals. The helper publisher
observes the ROS graph without a ROS CLI daemon. Run on a host that permits DDS;
a sandbox denying UDP or display access cannot establish RViz behavior.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone


ROOT = Path(__file__).resolve().parents[2]
IMAGE_TOPIC = "/rviz_shutdown_probe/image"


def write_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stop_child(child, timeout, first_signal=signal.SIGINT):
    """One graceful signal, then one SIGKILL only if the deadline expires."""
    result = {"signal": None, "escalated": False}
    if child.poll() is None:
        result["signal"] = signal.Signals(first_signal).name
        child.send_signal(first_signal)
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            result["escalated"] = True
            child.kill()
            child.wait(timeout=5)
    result["exit_code"] = child.returncode
    return result


def shutdown_outcome(ready, exit_code, escalated):
    if not ready:
        return "startup_failed"
    if escalated:
        return "shutdown_timeout"
    return "clean" if exit_code == 0 else "shutdown_failed"


def config_data(kind):
    displays = []
    if kind in ("tf", "image"):
        displays.append({"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": True, "Value": True})
    if kind == "image":
        displays.append({
            "Class": "rviz_default_plugins/Image", "Name": "Synthetic image",
            "Enabled": True, "Value": True, "Queue Size": 10,
            "Topic": {"Value": IMAGE_TOPIC, "Reliability Policy": "Best Effort",
                      "Durability Policy": "Volatile", "History Policy": "Keep Last", "Depth": 5},
        })
    return {"Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"}],
            "Visualization Manager": {"Class": "", "Enabled": True, "Name": "root",
                                      "Displays": displays, "Global Options": {"Fixed Frame": "map"},
                                      "Tools": [], "Value": True},
            "Window Geometry": {"Width": 800, "Height": 600}}


def publisher(status_path, rviz_name, image_topic, fixed_frame, tf_topic, sim_time):
    # Import ROS only inside the child, so its DDS teardown cannot contaminate
    # the controlling process or change RViz's process lifetime.
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("rviz_shutdown_probe_publisher")
    image_pub = node.create_publisher(Image, image_topic, qos_profile_sensor_data)
    tf_pub = node.create_publisher(TFMessage, tf_topic, 10)
    clock_pub = node.create_publisher(Clock, "/clock", 10) if sim_time == "true" else None
    count = 0
    try:
        while rclpy.ok():
            now = node.get_clock().now().to_msg()
            if clock_pub:
                clock_pub.publish(Clock(clock=now))
            image = Image()
            image.header.stamp = now
            image.header.frame_id = "probe_camera"
            image.height, image.width, image.encoding, image.step = 48, 64, "rgb8", 64 * 3
            image.data = bytes([count % 256, 120, 200]) * (48 * 64)
            image_pub.publish(image)
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = fixed_frame
            transform.child_frame_id = "probe_camera"
            transform.transform.rotation.w = 1.0
            tf_pub.publish(TFMessage(transforms=[transform]))
            rclpy.spin_once(node, timeout_sec=0.05)
            count += 1
            if count % 5 == 0:
                names = [name for name, _ in node.get_node_names_and_namespaces()]
                observed_topics = [image_topic, tf_topic] + (["/clock"] if sim_time != "false" else [])
                endpoints = {topic: [info.node_name for info in node.get_subscriptions_info_by_topic(topic)]
                             for topic in observed_topics}
                # RViz creates auxiliary listener nodes with generated names.
                # This domain contains only our publisher and RViz; preserve the
                # endpoint names instead of assuming the main node owns TF.
                topics = {topic: len(names) for topic, names in endpoints.items()}
                write_json(status_path, {"rviz_node_seen": rviz_name in names,
                                         "node_names": names, "subscription_nodes": endpoints,
                                         "subscriptions": topics, "published_images": count,
                                         "published_clock": count if clock_pub else 0,
                                         "monotonic": time.monotonic()})
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def start_xvfb(output, env):
    executable = shutil.which("Xvfb")
    if not executable:
        raise RuntimeError("Xvfb unavailable; use --display existing with an accessible display")
    reader, writer = os.pipe()
    log = (output / "xvfb.log").open("w")
    try:
        child = subprocess.Popen([executable, "-displayfd", str(writer), "-screen", "0", "1024x768x24", "-nolisten", "tcp"],
                                 pass_fds=(writer,), stdout=log, stderr=subprocess.STDOUT, env=env)
    finally:
        os.close(writer)
        log.close()
    try:
        if not select.select([reader], [], [], 10)[0]:
            raise RuntimeError("Xvfb did not allocate a display within 10 seconds")
        display = os.read(reader, 100).decode().strip()
        if not display.isdigit() or child.poll() is not None:
            raise RuntimeError("Xvfb startup failed; inspect xvfb.log")
    except BaseException:
        stop_child(child, 3, signal.SIGTERM)
        raise
    finally:
        os.close(reader)
    return child, ":" + display


def readiness(status, log, kind, image_topic, tf_topic="/tf", sim_time=False):
    """Require graph discovery and a working render system, plus subscriptions."""
    if not status.get("rviz_node_seen") or "OpenGl version:" not in log:
        return False
    subscriptions = status.get("subscriptions", {})
    if kind in ("tf", "image", "benchmark") and not subscriptions.get(tf_topic):
        return False
    if sim_time and not subscriptions.get("/clock"):
        return False
    return kind not in ("image", "benchmark") or bool(subscriptions.get(image_topic))


def run_one(args, output, env, executable, kind, repetition):
    directory = output / f"{kind}-{repetition:02d}"
    directory.mkdir()
    config = directory / "input.rviz"
    image_topic = IMAGE_TOPIC
    fixed_frame = "map"
    try:
        if kind == "benchmark":
            import yaml
            data = yaml.safe_load(args.benchmark_config.read_text())
            config.write_bytes(args.benchmark_config.read_bytes())
            images = [display for display in data["Visualization Manager"]["Displays"]
                      if display.get("Class") == "rviz_default_plugins/Image" and display.get("Enabled", True)]
            if len(images) != 1:
                raise ValueError("benchmark config must contain exactly one enabled Image display")
            image_topic = images[0]["Topic"]["Value"]
            fixed_frame = data["Visualization Manager"]["Global Options"]["Fixed Frame"]
        else:
            config.write_text(json.dumps(config_data(kind), indent=2) + "\n")
    except Exception as error:
        result = {"config": kind, "repetition": repetition, "ready": False,
                  "outcome": "blocked", "error": f"{type(error).__name__}: {error}"}
        write_json(directory / "summary.json", result)
        return result
    run_env = env.copy()
    # Keep caller overrides; otherwise each run owns a separate ROS log tree.
    run_env.setdefault("ROS_LOG_DIR", str(directory / "ros"))
    rviz_name = args.node_name or f"rviz_shutdown_probe_{kind}_{repetition}"
    command = [executable, "-d", str(config), "--ros-args", "-r", f"__node:={rviz_name}"]
    tf_topic = "/tf"
    if args.namespace:
        namespace = "/" + args.namespace.strip("/")
        command += ["-r", f"__ns:={namespace}", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]
        tf_topic = namespace + "/tf"
    if args.sim_time and args.sim_time_scope == "node":
        full_name = "/" + "/".join(part for part in (args.namespace.strip("/"), rviz_name) if part)
        parameter_file = directory / "parameters.yaml"
        parameter_file.write_text(f"{full_name}:\n  ros__parameters:\n    use_sim_time: true\n")
        command += ["--params-file", str(parameter_file)]
    else:
        command += ["-p", f"use_sim_time:={'true' if args.sim_time else 'false'}"]
    result = {"config": kind, "repetition": repetition, "command": command,
              "config_sha256": sha256(config), "ros_log_dir": run_env["ROS_LOG_DIR"],
              "ready": False, "outcome": "startup_failed", "stop_order": args.stop_order}
    if args.sim_time and args.sim_time_scope == "node":
        result["parameters_sha256"] = sha256(parameter_file)
    helper = rviz = None
    start = time.monotonic()
    try:
        with (directory / "publisher.log").open("w") as helper_log, (directory / "rviz.log").open("w") as rviz_log:
            helper = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--_publisher",
                                       str(directory / "publisher-status.json"), rviz_name, image_topic, fixed_frame,
                                       tf_topic, ("true-no-clock" if args.no_clock else "true") if args.sim_time else "false"],
                                      env=run_env, stdout=helper_log, stderr=subprocess.STDOUT)
            rviz = subprocess.Popen(command, env=run_env, stdout=rviz_log, stderr=subprocess.STDOUT)
            result["rviz_pid"] = rviz.pid
            deadline = start + args.startup_timeout
            while time.monotonic() < deadline:
                if rviz.poll() is not None or helper.poll() is not None:
                    break
                status_path = directory / "publisher-status.json"
                status = json.loads(status_path.read_text()) if status_path.exists() else {}
                if readiness(status, (directory / "rviz.log").read_text(errors="replace"), kind, image_topic,
                             tf_topic, args.sim_time):
                    result["ready"] = True
                    result["startup_seconds"] = time.monotonic() - start
                    result["ready_evidence"] = status
                    break
                time.sleep(0.1)
            if result["ready"]:
                maps = Path(f"/proc/{rviz.pid}/maps")
                maps_text = maps.read_text()
                (directory / "loaded-libraries.txt").write_text(maps_text)
                result["loaded_library_hashes"] = {}
                for line in maps_text.splitlines():
                    fields = line.split(maxsplit=5)
                    path = fields[-1] if len(fields) == 6 else ""
                    if path.startswith("/") and any(name in path for name in ("librmw_", "libddsc", "librviz", "libOgre")):
                        if path not in result["loaded_library_hashes"] and Path(path).is_file():
                            result["loaded_library_hashes"][path] = sha256(path)
                deadline = time.monotonic() + args.hold_seconds
                while time.monotonic() < deadline and rviz.poll() is None and helper.poll() is None:
                    time.sleep(0.1)
            result["exited_before_signal"] = rviz.poll() is not None
            result["publisher_alive_before_signal"] = helper.poll() is None
            if args.stop_order == "publisher-first":
                result["publisher_cleanup"] = stop_child(helper, 3)
            result["publisher_running_at_rviz_signal"] = helper.poll() is None
            result["exited_before_signal"] = rviz.poll() is not None
            shutdown_start = time.monotonic()
            result.update(stop_child(rviz, args.shutdown_timeout, getattr(signal, args.signal)))
            result["shutdown_seconds"] = time.monotonic() - shutdown_start
            result["outcome"] = shutdown_outcome(result["ready"], result["exit_code"], result["escalated"])
            if result["exited_before_signal"] and result["ready"]:
                result["outcome"] = "runtime_failed"
            if not result["publisher_alive_before_signal"]:
                result["outcome"] = "publisher_failed"
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        result["outcome"] = "probe_failed"
    finally:
        if rviz and rviz.poll() is None:
            result["rviz_cleanup"] = stop_child(rviz, 3)
        if rviz:
            result.setdefault("exit_code", rviz.poll())
        if helper and "publisher_cleanup" not in result:
            result["publisher_cleanup"] = stop_child(helper, 3)
        result["total_seconds"] = time.monotonic() - start
        write_json(directory / "summary.json", result)
    return result


def parser():
    result = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    result.add_argument("--configs", nargs="+", choices=("empty", "tf", "image", "benchmark"), default=["empty", "tf", "image"])
    result.add_argument("--repetitions", type=int, default=2)
    result.add_argument("--domain", type=int, default=207)
    result.add_argument("--rmw", choices=("rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"), default="rmw_cyclonedds_cpp")
    result.add_argument("--display", choices=("existing", "software-xvfb"), default="software-xvfb")
    result.add_argument("--signal", choices=("SIGINT", "SIGTERM"), default="SIGINT")
    result.add_argument("--startup-timeout", type=float, default=20)
    result.add_argument("--shutdown-timeout", type=float, default=10)
    result.add_argument("--hold-seconds", type=float, default=2)
    result.add_argument("--sim-time", action="store_true", help="publish /clock and enable RViz use_sim_time")
    result.add_argument("--sim-time-scope", choices=("global", "node"), default="global",
                        help="scope use_sim_time to all nodes or only the named RViz node, matching launch_ros")
    result.add_argument("--no-clock", action="store_true", help="enable simulated time without publishing /clock")
    result.add_argument("--node-name", help="explicit RViz node name, e.g. rviz2 for launch parity")
    result.add_argument("--namespace", default="", help="RViz namespace; also remap TF to relative topics")
    result.add_argument("--stop-order", choices=("rviz-first", "publisher-first"), default="rviz-first")
    result.add_argument("--output", type=Path)
    result.add_argument("--benchmark-config", type=Path, default=ROOT / "src/ridgeback_autonomy/sim/rviz/benchmark.rviz")
    return result


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "--_publisher":
        return publisher(*argv[1:])
    args = parser().parse_args(argv)
    if not 0 <= args.domain <= 232 or args.domain == 42:
        parser().error("choose an isolated domain from 0..232 other than the simulation default 42")
    if not 1 <= args.repetitions <= 100 or min(args.startup_timeout, args.shutdown_timeout, args.hold_seconds) <= 0:
        parser().error("repetitions must be 1..100 and deadlines/hold must be positive")
    if args.no_clock and not args.sim_time:
        parser().error("--no-clock requires --sim-time")
    output = (args.output or ROOT / "artifacts/benchmarks" / datetime.now().strftime("%Y%m%d_%H%M%S_rviz_shutdown_probe")).resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {"schema_version": 1, "probe_sha256": sha256(__file__),
               "started_utc": datetime.now(timezone.utc).isoformat(), "outcome": "blocked", "runs": [],
               "scope": "process lifetime only; software Mesa/Xvfb results do not qualify GPU or simulator workloads",
               "domain": args.domain, "rmw": args.rmw, "display_mode": args.display,
               "arguments": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}}
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), RMW_IMPLEMENTATION=args.rmw)
    # Do not inherit an interface-specific CycloneDDS config or discovery peers.
    for key in ("CYCLONEDDS_URI", "ROS_DISCOVERY_SERVER", "ROS_STATIC_PEERS"):
        if key in env:
            summary.setdefault("removed_environment", {})[key] = env.pop(key)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    xvfb = None
    try:
        executable = shutil.which("rviz2", path=env.get("PATH"))
        if not executable or not env.get("AMENT_PREFIX_PATH"):
            raise RuntimeError("source ROS and the desired workspace before running this probe")
        summary["executable"] = str(Path(executable).resolve())
        summary["executable_sha256"] = sha256(executable)
        summary["environment"] = {key: env.get(key) for key in ("AMENT_PREFIX_PATH", "LD_LIBRARY_PATH", "ROS_DISTRO", "ROS_LOG_DIR")}
        versions = subprocess.run(["dpkg-query", "-W", "-f=${Package} ${Version}\\n", "ros-jazzy-rviz*", "ros-jazzy-rmw*", "ros-jazzy-cyclonedds*"],
                                  capture_output=True, text=True, timeout=10)
        (output / "package-versions.txt").write_text(versions.stdout + versions.stderr)
        if args.display == "software-xvfb":
            env.update(LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe", QT_X11_NO_MITSHM="1")
            # Xvfb allocates its own display; inherited seat credentials are unnecessary.
            env.pop("XAUTHORITY", None)
            xvfb, env["DISPLAY"] = start_xvfb(output, env)
        elif not env.get("DISPLAY"):
            raise RuntimeError("DISPLAY unset; choose software-xvfb or provide an accessible display")
        summary["display"] = env["DISPLAY"]
        for kind in args.configs:
            for repetition in range(1, args.repetitions + 1):
                result = run_one(args, output, env, executable, kind, repetition)
                summary["runs"].append(result)
                write_json(output / "summary.json", summary)
                print(f"{kind}-{repetition:02d}: {result['outcome']} exit={result.get('exit_code')} ready={result['ready']}", flush=True)
        summary["outcome"] = "passed" if all(run["outcome"] == "clean" for run in summary["runs"]) else "failed"
    except Exception as error:
        summary["error"] = f"{type(error).__name__}: {error}"
    finally:
        if xvfb:
            summary["xvfb_cleanup"] = stop_child(xvfb, 3, signal.SIGTERM)
        write_json(output / "summary.json", summary)
    print(f"{summary['outcome']}: {output}")
    return 0 if summary["outcome"] == "passed" else 2 if summary["outcome"] == "blocked" else 1


if __name__ == "__main__":
    raise SystemExit(main())
