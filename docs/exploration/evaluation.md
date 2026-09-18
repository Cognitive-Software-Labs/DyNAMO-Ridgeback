# Exploration evaluation

This runbook measures frontier exploration: coverage, navigation outcomes, and
completion time. Target-distance estimation has its own
[benchmarking workflow](../target_distance_benchmarking/overview.md). The probe and
camera-less robot profile remain under `tools/benchmark/`; the `sim-runner`
project helper (`.claude/agents/sim-runner.md`) automates the same recipe.

The [Isaac exploration qualification backlog](../BACKLOG.md#isaac-exploration-recertification--p5)
owns baseline status and acceptance criteria. Dated sensor-attachment findings
are linked under archived evidence below.

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

# 4. Attach the probe once the explorer node is up (separate shell)
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

## Isaac comparison run

Isaac and Gazebo deliberately publish the same ROS topic and TF contract but
use different renderers and sensor models. Keep Isaac measurements isolated
from the canonical Gazebo run:

```bash
export ROS_DOMAIN_ID=77
bash start_exploration.sh warehouse_full sim:=isaac \
    sim_mode:=deterministic camera:=false \
    target_localization_enabled:=false exploration_rviz:=false \
    odom_noise:=0.0 setup_path:=/tmp/bench-clearpath/

# Attach the probe from a second shell with the same domain.
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
python3 tools/benchmark/explore_probe.py <tag> [max_wall_seconds]
```

Every backend-specific flag is load-bearing: deterministic mode prevents
render-wall contention from changing simulated lidar timing; disabling the
camera keeps the GPU budget focused on navigation; zero odometry noise isolates
scan geometry; and the dedicated domain prevents co-tenant stacks from
cross-publishing the same namespaced topics. Measure sensor rates in simulation
time, not wall time.

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

## Archived evidence

- [Isaac port history](../../archive/engineering/port-history.md#lidars-detached-from-the-articulation-2026-09-11)
