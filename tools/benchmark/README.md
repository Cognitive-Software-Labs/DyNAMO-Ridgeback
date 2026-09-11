# Benchmark harness

Instrumented exploration benchmark runs, promoted from the 2026-07-10 tuning
sessions. The `sim-runner` project subagent (`.claude/agents/sim-runner.md`)
automates this recipe.

> ✅ **Exploration runs on Isaac as of 2026-09-11.** The long-standing stall
> (`collision_monitor` pinning `cmd_vel` at zero) was the 2D lidars being
> parented to `base_link`, which has no joint into the articulation — PhysX
> turned `chassis_link` and left the sensors behind. Reparenting them to
> `chassis_link` fixed it: `cmd_vel` flows, the robot drives, and frontier
> goals succeed. See `docs/isaac/open-issues.md` §1.
>
> **No Isaac coverage baseline exists yet**, and every pre-2026-09-11 figure
> is void — they were measured with a sensor that did not rotate with the
> robot. Producing the first real baseline is the current deliverable.

## Canonical run

```bash
# 1. Clean slate (always)
bash cleanup.sh

# 2. Stage the camera-less profile OUTSIDE the repo — the clearpath generator
#    writes urdf/srdf/platform artifacts into the setup_path directory
mkdir -p /tmp/bench-clearpath
cp tools/benchmark/robot_no_camera.yaml /tmp/bench-clearpath/robot.yaml

# 3. Launch. Every flag below is load-bearing — see "Hygiene".
export ROS_DOMAIN_ID=77          # NOT 42: a co-tenant runs a /r100_0001 stack
                                 # there whose hud_node publishes the same
                                 # hud/coverage topic the probe reads
bash start_exploration.sh warehouse_full sim:=isaac \
    sim_mode:=deterministic camera:=false g1_perception_enabled:=false \
    exploration_rviz:=false odom_noise:=0.0 setup_path:=/tmp/bench-clearpath/

# 4. Attach the probe once the explorer node is up (separate shell)
source install/setup.bash
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
       CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
python3 tools/benchmark/explore_probe.py <tag> [max_wall_seconds]
```

## Hygiene — every flag earns its place

| flag | why |
|---|---|
| `sim:=isaac` | gz is still runnable during the port; without this you benchmark the wrong simulator |
| `sim_mode:=deterministic` | `realtime` couples `frame_dt` to render-wall duration, so co-tenant load smears the lidar sweep — a sim artefact, not a nav result |
| `camera:=false` | the runner used to render the D455 unconditionally: RTF 0.33 → 0.65 |
| `g1_perception_enabled:=false` | keeps the GPU for the sim |
| isolated `ROS_DOMAIN_ID` | co-tenant `/r100_0001` stacks otherwise cross-publish `hud/coverage` |
| `setup_path:=/tmp/bench-clearpath/` | the generator writes artefacts into this directory; keep it out of the repo |

Measure all rates in **sim time**. RTF ≈0.65 makes a 40 Hz scan look like
26 Hz on the wall — expected, not a fault.

**Do not launch with `nohup ... &` and then `kill $!`.** That kills the bash
wrapper and leaves the python child running; two stacks then publish to the
same topics and silently corrupt the run. Kill by explicit PID in a *separate*
command from the launch, and verify the count is zero afterwards — `pkill -f`
and `ps | grep <pattern>` both match the calling shell when the pattern
appears in your own command line.

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

Record `uptime` load with every run — sim health tracks total box load
(see ISSUES.md and `~/workstation.md`; co-tenant training/build storms cap
coverage regardless of nav config). Launch logs land in
`logs/ridgeback_<timestamp>_<world>.log`; triage recipes live in
ISSUES.md ("Exploration Quits Early", "Phantom Coverage", "RTF Collapse").
