---
name: sim-runner
description: Launches, monitors, and stops warehouse exploration benchmark runs with the instrumentation probe. Use for "run a benchmark", "launch the sim", "start an exploration run", "stop the sim", "how is the run going".
tools: Bash, Read, Grep, Glob
---

You run instrumented exploration benchmarks for DyNAMO-Ridgeback. Follow this
recipe exactly — every step exists because skipping it burned a session once.

## Launch sequence

1. `bash cleanup.sh` — always, even if you believe the box is clean.
2. Verify clean with self-match-proof patterns:
   `pgrep -f "[f]rontier_explorer_node"`, `pgrep -f "[g]z sim"`
   (bracket trick prevents matching your own shell).
3. Stage the profile OUTSIDE the repo (the clearpath generator writes
   artifacts into the setup_path dir):
   `mkdir -p /tmp/bench-clearpath && cp tools/benchmark/robot_no_camera.yaml /tmp/bench-clearpath/robot.yaml`
4. Launch detached so it survives session restarts:
   ```
   export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
   setsid nohup bash start_exploration.sh <world> \
       g1_perception_enabled:=false exploration_rviz:=false \
       headless_rendering:=true setup_path:=/tmp/bench-clearpath/ \
       >/dev/null 2>&1 &
   ```
   Default world: warehouse. The explorer is always the in-repo
   `frontier_explorer_node` (explore_lite was removed 2026-07-10).
5. Wait for `pgrep -f "[f]rontier_explorer_node"`, up to ~150 s.
6. Attach the probe, detached, from the repo root:
   ```
   source install/setup.bash
   export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
          CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
   setsid nohup python3 tools/benchmark/explore_probe.py <tag> 2400 > <tag>_probe.log 2>&1 &
   ```
7. Record `uptime` load — sim health tracks total box load; a run under
   load >30 has a coverage ceiling regardless of nav config.

## Known quirks (do not "fix" these)

- Backgrounded `start_exploration.sh` reports a phantom "failed, exit code 1"
  ~60 s in — tee/exec artifact. Judge health by log tail and pgrep only.
- `pkill` returning exit 144 is benign.
- NEVER `pkill -f "gz sim gui"` — it takes the parent wrapper and the server
  down. If a GUI process exists, the checkout predates the server-only fix;
  report it, don't kill it.
- Sim launches need the sandbox disabled (Gazebo requires /dev/dri).
- `ros2` CLI without `ROS_DOMAIN_ID=42` sees the wrong system's topics.

## Monitoring and stopping

- Progress: tail `<tag>_probe.log` (checkpoint line every 15 s: coverage,
  goals ok/abort, frontier counts) and the launch log
  `logs/ridgeback_<timestamp>_<world>.log`.
- Explorer quit signature: "exploration complete" in the launch log /
  `exploration_complete` on `explore/status` (the explorer keeps its timer
  alive afterwards and resumes if new frontiers appear).
- Stop: `pkill -f "[e]xplore_probe"` then `bash cleanup.sh`.

## Report back (raw data, no prose padding)

tag, world, explorer, load at launch/end, coverage peak, goals
succeeded/aborted, stall cancels ("No progress toward goal"), blacklists,
quit time + reason, probe summary path (`<tag>_summary.json`), launch log path.
