# Simulation parking demonstration

`start_parking_demo.sh` runs the autonomous parking mission beside a Unitree G1
in one simulator. The mission's components and Nav2 ownership handoff are
described in [exploration architecture](architecture.md#parking-mission-coordination);
the phase-by-phase work is tracked in the
[parking mission migration plan](../plans/EXPLORATION_parking_mission_migration.md).

## Running the demonstration

Run one simulator at a time from the checkout. The wrapper sources this
workspace, uses CycloneDDS on loopback-only domain 83, and refuses a concurrent
simulator instead of killing shared workstation processes. Point inference at
an existing perception environment if it is outside this checkout:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv
bash start_parking_demo.sh gz --plan-only
# After stopping Gazebo with Ctrl-C:
bash start_parking_demo.sh isaac --plan-only
```

`--plan-only` is stationary validation: no frontier explorer, an explicit idle
handoff stub, and parking navigation disabled. Both real simulators still run
SLAM, Nav2, collision monitoring, perception, one mission manager and the
unchanged parking RViz layout. It does not demonstrate active exploration
handoff or autonomous parking. Unknown footprints can correctly prevent a
stationary plan in an initially unmapped scene. To validate the unchanged
exploration-to-planning sequence, use `--planning-only`: the simulated robot
may explore, map around G1 and return to its observation pose, but parking
navigation remains disabled. A path-feasible snapshot expires under the
existing freshness rules; it is not a persistent authorization to drive.

The full-mission commands enable the existing simulation execution path:

```bash
bash start_parking_demo.sh gz
# After stopping Gazebo with Ctrl-C:
bash start_parking_demo.sh isaac
```

Cancelled or vetoed legs hold Nav2 goal ownership until a terminal result, as
described in [mission navigation goal ownership](architecture.md#mission-navigation-goal-ownership).
During both parking legs the live LiDAR clearance monitor checks the stopping
path and the validated route still ahead of the robot; the route it has already
driven is not re-checked. The return to the G1
observation pose uses a temporary tight goal checker and a verified view before
relocalizing ([observation return](architecture.md#observation-return)); it can
still fail explicitly, including in `--planning-only` mode. No execution
safeguard or motion-settlement threshold is relaxed by this launch change.

## Parking display

RViz shows the plan on `parking/visualization`, with a colour legend overlay on
`parking/legend`. The donut is 360 radial lines, one per degree, each drawn in
one colour from the inner to the outer reach limit (grey rings). A ray's
parking positions are the donut cells it crosses; each is judged by the
planner's own candidate decision, and the ray takes the first class that any
position has:

- green: at least one geometrically valid parking pose, not a Nav2 path check;
- orange: no valid pose, and unmapped space blocks a pose that keep-out and
  reach allow, so the direction is undecided;
- red: no valid pose, and obstacles block every pose that keep-out and reach
  allow;
- grey: keep-out or the reach limit rules out every pose on the ray.

Unmapped space wins over obstacles, so an undecided direction is never shown
as definitively blocked. With the live 0.06 m reach allowance, the allowed task
points lie only about 0.51–0.54 m from the generic G1 estimate, less than one
map cell, so some directions cross no allowed cell and stay grey.

Candidates are not drawn as dots. The Nav2-checked choice is the blue selected
pose (arrow, footprint, rear-bin task point) and its validated route.

## Scene, configuration and startup gates

The wrapper calls `ridgeback_mission.launch.py` with `demo_scene_enabled:=true`.
The default Gazebo environment is `depot` (native world `warehouse`); Isaac
uses `parking_demo`, a layer over `initial_test_world` that disables its distant
authored G1. Startup creates exactly one demo G1 at `(3, 0)` and the existing
table at `(3, 1.05)` through `/gzserver` or `/isaac_sim` standard scene services.
Creation failures or existing G1 entities abort startup rather than renaming
or deleting an existing target. Manual `gz service` calls and the removed
`benchmarking.simulation.spawn_model_command` helper are not needed.

The demonstration maps both LiDARs: `ridgeback_mission.launch.py` defaults
`slam_source` to `merged` on Gazebo and Isaac, and an explicit
`slam_source:=...` still wins. Front-only SLAM never
observes the floor behind the spawn pose, which left the start of every
parking route unknown, so path feasibility rejected all of them
(`path_footprint_unknown`). The merge, its frames and its range limits are
owned by the [LiDAR pipeline](../isaac/lidar-pipeline.md#merger-and-range-limits);
navigation, the collision monitor and the mission's LiDAR clearance keep the
two raw scans, and the shared exploration default stays `front_only`.

The mission explicitly selects this checkout's `clearpath/robot.yaml`; an
explicit `setup_path:=...` remains supported, but Isaac rejects a sensor rig
different from its committed USD. Manager odometry defaults to
`platform/odom/filtered` on Gazebo and `platform/odom` on Isaac, with
`odometry_topic:=...` available as an override. Nav2 retains filtered odometry.
Use the normal pinned dependency installation, including the maintained
Clearpath sensor and namespaced SLAM patches; an old system-only generator is
not the repository's sensor contract.

Startup waits for advancing clock, scene services, verified spawns, fresh
LiDAR/odometry/perception messages, sensor/map TF, active Nav2 lifecycle nodes
(including collision monitor), and MissionGuard interfaces. In a full demo,
the existing guard starts latched. The one manager saves its start pose before
startup resumes exploration and admits detections. `startup_timeout_s:=300.0`
bounds each startup gate; failure shuts down the launch's processes. Ctrl-C
stops the owned stack. Additional `name:=value` arguments are forwarded, for
example `headless:=true` for Isaac without its viewport; RViz remains enabled.

Each run writes its ROS logs to a new
`artifacts/exploration/parking_<date>_<backend>.<suffix>/ros/` directory, which
the wrapper prints at startup. `LOG_DIR` moves that root and an explicit
`ROS_LOG_DIR` replaces the `ros/` directory; console output stays on the
terminal.
