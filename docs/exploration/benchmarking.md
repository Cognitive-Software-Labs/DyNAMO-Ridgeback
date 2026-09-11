# Exploration benchmark runbook

This runbook owns instrumented exploration benchmark runs. The probe and
camera-less robot profile remain under `tools/benchmark/`; the `sim-runner`
project helper (`.claude/agents/sim-runner.md`) automates the same recipe.

## Canonical run

```bash
# 1. Clean slate (always)
bash cleanup.sh

# 2. Stage the camera-less profile OUTSIDE the repo — the clearpath generator
#    writes urdf/srdf/platform artifacts into the setup_path directory
mkdir -p /tmp/bench-clearpath
cp tools/benchmark/robot_no_camera.yaml /tmp/bench-clearpath/robot.yaml

# 3. Launch through the measured NVIDIA GLX path. This retains Gazebo's GUI;
#    omit exploration_rviz only when the probe is the sole output you need.
setsid nohup tools/gpu-run bash start_exploration.sh warehouse \
    target_localization_enabled:=false exploration_rviz:=false \
    setup_path:=/tmp/bench-clearpath/ \
    >/dev/null 2>&1 &

# 4. Attach the probe once the explorer node is up
source install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/benchmark/explore_probe.py <tag> [max_wall_seconds]
```

For an additional server-only mode on SSH/non-seat sessions, replace the
launch command with:

```bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
setsid nohup bash start_exploration.sh warehouse \
    target_localization_enabled:=false exploration_rviz:=false \
    headless_rendering:=true setup_path:=/tmp/bench-clearpath/ \
    >/dev/null 2>&1 &
```

`headless_rendering` defaults to `false`; it is an alternative server-only EGL
path, not a replacement for `tools/gpu-run`.

## explore_probe.py

Subscribes to `navigate_to_pose` action status, `explore/frontiers` markers,
`explore/status`, and `hud/coverage`. Prints a checkpoint line every 15 s and
writes `<tag>_events.csv` + `<tag>_summary.json` (goal counts, succeeded /
aborted, preempt-vs-genuine abort split, frontier blacklist counts, coverage
peak, quit time) into the working directory.

Caveats:
- the preempt-vs-genuine abort classification uses a ±1.5 s accept window;
  cross-check against the explorer log's failure/blacklist lines
- `explore/status` is a `std_msgs/String` (`exploration_started` /
  `exploration_complete`); the probe exits ~10 s after completion

## Reading results

Record `uptime` load with every run: simulator health depends on total host
load, so an uncontrolled co-tenant workload can invalidate a navigation
comparison. Launch logs land in `artifacts/exploration/<run>/console.log`.
Current failure signatures and recovery steps live in
[troubleshooting](../troubleshooting.md).
