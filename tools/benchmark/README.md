# Benchmark harness

Instrumented exploration benchmark runs, promoted from the 2026-07-10 tuning
sessions. The `sim-runner` project subagent (`.claude/agents/sim-runner.md`)
automates this recipe.

## Canonical run

```bash
# 1. Clean slate (always)
bash cleanup.sh

# 2. Stage the camera-less profile OUTSIDE the repo — the clearpath generator
#    writes urdf/srdf/platform artifacts into the setup_path directory
mkdir -p /tmp/bench-clearpath
cp tools/benchmark/robot_no_camera.yaml /tmp/bench-clearpath/robot.yaml

# 3. Launch (NVIDIA EGL headless; headless_rendering also implies gz -s server-only)
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
setsid nohup bash start_exploration.sh warehouse explore_lite \
    g1_perception_enabled:=false exploration_rviz:=false \
    headless_rendering:=true setup_path:=/tmp/bench-clearpath/ \
    >/dev/null 2>&1 &

# 4. Attach the probe once the explorer node is up
source install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
       CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
python3 tools/benchmark/explore_probe.py <tag> [max_wall_seconds]
```

## explore_probe.py

Subscribes to `navigate_to_pose` action status, `explore/frontiers` markers,
`explore/status`, and `hud/coverage`. Prints a checkpoint line every 15 s and
writes `<tag>_events.csv` + `<tag>_summary.json` (goal counts, succeeded /
aborted, preempt-vs-genuine abort split, frontier blacklist counts, coverage
peak, quit time) into the working directory.

Caveats:
- the preempt-vs-genuine classification uses a ±1.5 s accept window; with the
  patched explore_lite (retries re-send within that window) trust the explore
  log's `attempt N/M` lines over the probe's split
- frontier avail/black counts only work with explore_lite's marker scheme;
  for `explorer:=custom` use goal stats + `explore/status`

## Reading results

Record `uptime` load with every run — sim health tracks total box load
(see ISSUES.md and `~/workstation.md`; co-tenant training/build storms cap
coverage regardless of nav config). Launch logs land in
`logs/ridgeback_<timestamp>_<world>_<explorer>.log`; triage recipes live in
ISSUES.md ("Exploration Quits Early", "Phantom Coverage", "RTF Collapse").
