#!/usr/bin/env python3
"""Isaac Sim 6.0 install smoke test: headless boot + ROS 2 bridge + /clock.

Proves in one shot that the pip install works, headless Vulkan rendering
initializes on this box (no X needed), the ROS 2 bridge extension loads,
and its publishers reach the system DDS (CycloneDDS by default).

Run from the workspace root with ROS sourced, so the bridge picks up the
system RMW instead of its bundled libraries:

    source /opt/ros/jazzy/setup.bash
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file://$PWD/cyclonedds.xml \
    isaac_venv/bin/python3 tools/isaac/smoke_test.py

While it runs, `ros2 topic hz /clock` in a plain terminal (same RMW env)
must show ~60 Hz ticks. Exit code 0 = all checks passed.
"""
import os
import sys

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp  # noqa: E402  (must precede omni imports)

app = SimulationApp({"headless": True})

import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

failures = []

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
    timeline.stop()

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
    _mm_loop.set_manual_mode(True)
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

# Verdict before app.close(): Kit teardown can swallow buffered stdout.
if failures:
    print("SMOKE TEST FAILED:", *failures, sep="\n  - ", flush=True)
    app.close()
    sys.exit(1)
print("SMOKE TEST OK: headless boot, ros2 bridge, /clock ~5 s, fixed-step gate",
      flush=True)
app.close()
