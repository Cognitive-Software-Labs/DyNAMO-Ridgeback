#!/usr/bin/env python3
"""Isaac Sim 6.1 install smoke test: exact version, boot, bridge, and /clock.

Proves in one shot that the pip install works, headless Vulkan rendering
initializes on this box (no X needed), the ROS 2 bridge extension loads,
and its publishers reach the system DDS (CycloneDDS by default).

Run from the workspace root with ROS sourced, so the bridge picks up the
system RMW instead of its bundled libraries:

    source /opt/ros/jazzy/setup.bash
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file://$PWD/src/ridgeback_autonomy/config/cyclonedds.xml \
    isaac_venv/bin/python3 tools/isaac/smoke_test.py

While it runs, `ros2 topic hz /clock` in a plain terminal (same RMW env)
must show ~60 Hz ticks. Exit code 0 = all checks passed.
"""
import argparse
from importlib.metadata import version
import os
import sys
import time


parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument(
    "--expected-version",
    default="6.1.0.0",
    help="exact isaacsim package version (default: %(default)s)",
)
args, _kit_args = parser.parse_known_args()
# Keep Isaac Kit's argument parser from seeing smoke-test-only options.
sys.argv = [sys.argv[0], *_kit_args]

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp  # noqa: E402  (must precede omni imports)

app = SimulationApp({"headless": True})

import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

failures = []

installed_version = version("isaacsim")
if installed_version != args.expected_version:
    failures.append(
        f"isaacsim version {installed_version}, expected {args.expected_version}")
else:
    print(f"version gate OK: isaacsim {installed_version}", flush=True)

if not enable_extension("isaacsim.ros2.bridge"):
    failures.append("could not enable isaacsim.ros2.bridge")

# The bridge registers its OmniGraph nodes on load; assert the ones the
# runner depends on exist in this build (guards 6.x renames).
required_nodes = [
    "isaacsim.ros2.bridge.ROS2PublishClock",
    "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
    "isaacsim.ros2.bridge.ROS2CameraHelper",
]
registered = set(og.get_registered_nodes())
for node in required_nodes:
    if node not in registered:
        failures.append(f"OmniGraph node not registered: {node}")

clock_messages = []
clock_node = None
owns_rclpy_context = False
try:
    import rclpy  # noqa: E402
    from rosgraph_msgs.msg import Clock  # noqa: E402

    if not rclpy.ok():
        rclpy.init()
        owns_rclpy_context = True
    clock_node = rclpy.create_node("isaac_smoke_clock_probe")
    clock_node.create_subscription(
        Clock,
        "/clock",
        lambda msg: clock_messages.append(
            msg.clock.sec + msg.clock.nanosec * 1e-9),
        10,
    )
except Exception as exc:
    failures.append(f"could not create /clock subscriber: {exc!r}")

if not failures:
    og.Controller.edit(
        {"graph_path": "/SmokeGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("tick.outputs:time", "clock.inputs:timeStamp"),
            ],
        },
    )
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(300):  # ~5 s of /clock at 60 Hz
        app.update()
        rclpy.spin_once(clock_node, timeout_sec=0.0)
    timeline.stop()

    deadline = time.monotonic() + 2.0
    while not clock_messages and time.monotonic() < deadline:
        rclpy.spin_once(clock_node, timeout_sec=0.05)
    if not clock_messages:
        failures.append("no /clock messages received over system ROS 2 DDS")
    elif any(b < a for a, b in zip(clock_messages, clock_messages[1:])):
        failures.append("/clock moved backwards")
    else:
        print(
            f"/clock transport gate OK: received {len(clock_messages)} monotonic ticks",
            flush=True,
        )

# Fixed-step gate: the deterministic sim mode (isaac_runner --sim-mode
# deterministic) relies on omni.kit.loop manual mode advancing app.update()
# by an EXACT sim-dt headless — it lives below the isaacsim.core.api
# SimulationContext that segfaults headless in 6.0.1. Prove it holds so
# contention-immune A/B benchmarks are trustworthy on this box.
try:
    from omni.kit.loop import _loop as omni_loop
    _mm_loop = omni_loop.acquire_loop_interface()
    _mm_dt = 1.0 / 40.0
    _mm_loop.set_manual_step_size(_mm_dt)
    _mm_loop.set_manual_mode(True, name="main")   # matches the runner
    _mm_tl = omni.timeline.get_timeline_interface()
    _mm_tl.set_end_time(1.0e9)
    _mm_tl.set_looping(False)
    _mm_tl.set_time_codes_per_second(40.0)
    _mm_tl.play()
    for _ in range(20):          # warmup / physics init
        app.update()
    _mm_t0 = _mm_tl.get_current_time()
    for _ in range(40):
        app.update()
    _mm_adv = _mm_tl.get_current_time() - _mm_t0
    _mm_tl.stop()
    _mm_loop.set_manual_mode(False)
    if abs(_mm_adv - 40 * _mm_dt) > 1e-6:
        failures.append(
            f"manual-mode fixed-step broken: 40 frames advanced {_mm_adv:.6f}s, "
            f"expected {40 * _mm_dt:.6f}s (deterministic mode would drift)")
    else:
        print(f"fixed-step gate OK: 40 frames advanced exactly {_mm_adv:.4f}s",
              flush=True)
except Exception as e:
    failures.append(f"omni.kit.loop manual mode unavailable: {e!r}")

if clock_node is not None:
    clock_node.destroy_node()
if owns_rclpy_context:
    rclpy.shutdown()

# Verdict before app.close(): Kit teardown can swallow buffered stdout.
if failures:
    print("SMOKE TEST FAILED:", *failures, sep="\n  - ", flush=True)
    app.close()
    sys.exit(1)
print("SMOKE TEST OK: headless boot, ros2 bridge, /clock ~5 s, fixed-step gate",
      flush=True)
app.close()
