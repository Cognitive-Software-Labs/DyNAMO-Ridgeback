# Parking scenarios

The parking demonstration spawns one of several named scenes. Every scene puts G1
at a *workstation*: standing at a worktable, facing it, out of the robot's sight
from its start. The robot has to explore, detect and localize G1, plan a parking
pose beside it, park, wait and return. The scenes vary what stands around G1 and
which way G1 faces, to test the G1-relative preference order, blocked sides,
obstacles above the LiDAR plane, a scene with no safe pose, and G1 headings. The
mission, planner and safety checks are the same in every scene, and the mission is
given nothing about the scene. The [scenario page](autonomous_parking_scenarios.html)
shows each scene in both simulators; the [results page](autonomous_parking_results.html)
lists every recorded run; the dated records are under
[archived evidence](#archived-evidence).

## Running a scenario

`demo_scenario` selects a scene from
[`parking_scenarios.yaml`](../../src/ridgeback_autonomy/config/parking_scenarios.yaml);
the default is `standard`. The wrapper forwards it:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv
bash start_parking_demo.sh gz demo_scenario:=right_blocked
# After stopping Gazebo with Ctrl-C:
bash start_parking_demo.sh isaac demo_scenario:=right_blocked
```

Startup spawns G1 and each object through the simulator's standard scene
services and refuses a world that already holds a G1 or a `bench_parking_demo_*`
entity. Objects are logical names from each backend's `sim/benchmark_assets.yaml`,
so Gazebo and Isaac spawn the same collision geometry. A scene's `purpose` and
`expected` texts document intent; the mission never reads them.

## Workstations and layout

The robot starts at both simulators' default spawn, the world origin facing +x,
so the SLAM map frame is the world frame. Each demo world has one workstation pose
for G1 (schema `ridgeback_parking_scenarios/2`):

| World | G1 at the workstation | Why the robot cannot see it from the start |
|---|---|---|
| Gazebo `depot` (warehouse) | (-4.5, -0.6) m, facing -x | it stands behind the camera at the start; the explorer heads down this lane first and meets G1 from behind |
| Isaac `parking_demo` (hospital floor) | (0.5, -4.5) m, facing -y | the corridor wall hides the south room; the robot enters through the doorway at x 2.7-5.3 m, behind G1's left shoulder |

Two earlier Gazebo workstations were dropped (2026-11-05/08; see the dated record).
The explorer never entered an aisle behind the long shelf south of the start. North
of the start, G1 was found before the robot left its spawn, so every route began in
the unscanned wedges beside the robot's flanks, where both LiDARs are blocked by the
chassis, and the planner rightly rejected them. A workstation works when the robot
has driven into its area before it sees G1.

A scenario places its objects in the workstation frame: `forward` from G1 toward
its table, `left` to G1's left when it faces the table, `yaw_deg` relative to that
heading; `g1_turn_deg` turns G1 away from the table. A scenario therefore has the
same layout around G1 in both worlds, and only the surroundings differ. The
worktable (1.20 x 0.70 m, top 0.75 m) stands 0.75 m ahead of G1, its long side
toward G1.

| Scenario | Around G1 at its worktable | Intended outcome |
|---|---|---|
| `standard` | nothing else | left or right, never under the tabletop |
| `right_blocked` | 1.7 m panel 0.8 m to G1's right | left |
| `left_blocked` | 1.7 m panel 0.8 m to G1's left | right |
| `both_sides_blocked` | panels on both sides | behind G1 |
| `restricted` | side tables 1.2 m left and right (the base fits under, the mast and basket do not); a 2.1 m passage behind | behind G1, never under a tabletop |
| `no_feasible` | panels on both sides and a 0.5 m bed behind: a pen G1 is visible over | no manoeuvre; bounded planning, return to start with `no_valid_candidate` |
| `heading_90` | G1 turned 90 degrees to its left, the table on its right | left; right-side poses would be under the table |
| `heading_away` | G1 turned 180 degrees, its back to the table | left or right; behind G1 is the table |

The P7 scenes (G1 3 m in front of the start, schema 1, plus a separate `worktable`
scene) are retired; their runs keep the scenario file they ran, and the evaluation
tools still read it. A scenario that does not fit a world can list where it does,
for example `backends: [gz]`, and startup then refuses it elsewhere; every current
scene fits both worlds.

OWLv2 scores a Gazebo G1 at about 0.42-0.68 depending on the view; side-on and
partly hidden views sit near 0.50. The depot's walking person scored at most 0.19
as `humanoid robot`, and nothing else above 0.22. The parking launch runs the
detector at 0.40 with the hysteresis described under
[observation return](architecture.md#observation-return). Keep objects low or away
from G1's outline as seen on the robot's way in: a static view of a side-on G1 in
front of a matching background still scores below 0.40 (see the
[G1 detection backlog item](../BACKLOG.md#g1-detection-in-parking-scenes)).

Left and right are G1's, counter-clockwise and clockwise from the way it faces,
as estimated from depth. The order, sectors and fallback without a heading are
owned by the [G1 heading reference](architecture.md#g1-heading):
left and right equally, then behind, then in front, then the rest, then a return
to the start. Preference only orders safe candidates; it never relaxes a check.

## Robot height and the basket

The planner and the LiDAR monitor work in the plane of two LiDARs 0.18 m above
the floor. A table is a real collision object in both simulators, but only its
legs reach that plane. Two things keep the mast, camera and basket out from
under a tabletop: the camera overlay marks depth points 0.25-1.15 m high within
1.6 m of G1 in the planning grid ([overhead obstacles](architecture.md#obstacles-above-the-lidar-plane)), and the route and final pose are checked
against that grid. Beyond 1.6 m of G1, or where the camera has not looked, there
is no height check, and Nav2's costmaps never see a tabletop.

The robot carries a top-open crate behind the mast. The model's basket is
[provisional](../robot/geometry.md#photographed-payload-and-the-provisional-basket):
no dimension has been measured. Four distances describe a parked pose:

1. G1 body centre to the robot base centre, which only places the robot;
2. G1 body axis to the task point, 0.28 m behind the base centre at the basket's
   modelled centre. This is the planned reach, 0.30-0.60 m;
3. G1's hands to the basket opening. Nothing models or verifies this; with the
   held forearms the nearest hand is about 0.53 m from the task point beside G1
   ([hand access](architecture.md#g1-heading));
4. clearance between any robot part and G1 or an obstacle, which the planner
   checks in the plane and the overlay above it.

Parking plans and checks with Nav2's shared outline and the basket together
(`parking_footprint`, their convex hull): since the group's mesh-shaped outline
(rear edge at -0.476 m, angled rear ends) the basket's rear corners at (-0.48,
±0.19) m reach up to 0.035 m beyond it. Nav2 and its collision monitor keep the
group's outline; the parking planner, its route checks and the parking LiDAR check
use the hull. In open space this widens the valid reach band from 0.526-0.540 m to
0.491-0.540 m: the rear edge stops 0.291-0.340 m from G1's axis (0.308-0.323 m
before), so 0.121-0.170 m from G1's legs as depth sees them.

The task point is the centre of the thesis bin the provisional basket copies. It
stays the interaction point, and the reach band stays 0.30-0.60 m, until the
crate's position on the deck and G1's reach are measured.

## Evaluating a run

The tools under [`tools/parking_scenarios/`](../../tools/parking_scenarios/) run
one scenario in its own process session, sample Gazebo ground truth at about
1 Hz, take RViz screenshots and simulator views, and stop only that session with SIGTERM:

```bash
tools/parking_scenarios/run_scenario.sh gz right_blocked artifacts/exploration/scenarios/gz_right_blocked
python3 tools/parking_scenarios/scenario_metrics.py artifacts/exploration/scenarios/gz_right_blocked
# Scene views, then the two pages (CODE in make_results_page.py labels each recorded HEAD):
tools/parking_scenarios/preview_scenario.sh gz right_blocked artifacts/exploration/scenes
python3 tools/parking_scenarios/make_scenarios_page.py artifacts/exploration/scenes
python3 tools/parking_scenarios/make_results_page.py --runs artifacts/exploration/scenarios/*/ \
    --images artifacts/exploration/scenarios/gz_right_blocked
```

`preview_scenario.sh gz|isaac SCENARIO OUT_DIR` starts a scene with
`--plan-only` (the robot stays at its start) and saves the documentation views:
top-down views of the whole task, of the start and G1 framed together, and of the
workstation, a close-up of G1 and its table, an oblique view, an angled view from
above of the robot and G1, and one frame of the robot's own camera. The scenarios
page leads each scenario with Isaac Sim's top-down, close-up and angled views and
keeps the Gazebo views of the same layout in a collapsed section.
`capture_scene_views.py` renders them in the simulator itself, so a locked or
disconnected desktop cannot blank them: Gazebo through temporary camera sensors
that are removed afterwards, Isaac through the runner's opt-in view cameras
(`RIDGEBACK_ISAAC_VIEW_CAMERAS`, a JSON list from `scene_views.py`; experiment runs
carry only the top-down two). Each image is checked and listed in a manifest.
`run_scenario.sh` takes the same top-down views while parked and at the end, and
RViz screenshots at the mission stages. `make_scenarios_page.py` and
`make_results_page.py` build the two pages; the results page labels every image
with the mission state active when it was captured and shows a top-down view as
the parked position only if the robot was `PARKED` then.

`FRAMES=1` also saves a camera frame every `FRAMES_PERIOD` seconds (default 1;
`FRAMES_EXT=png` is lossless and about 0.4 MB a frame).
`score_frames.py`, run with the perception venv's Python, scores them with the
mission's OWLv2 checkpoint down to 0.05, for reading margins offline. JPEG frames
score a few hundredths differently from the live detector; PNG frames score the same.

Mission metrics come from the console log: estimated state. Reach, side, pose
error and clearance come from Gazebo ground truth. Clearance is the smallest
plan distance between a robot part (base, mast, camera, basket) and an
obstacle collision box at an overlapping height, or G1's 0.19 m lower body. It is
sampled at about 1 Hz, so it can miss a brief closer approach. Isaac runs report
estimated state only. Bags hold the mission state, parking display, plan, map,
detections and both 2D scans. The G1 localization error is the distance from the
position the mission used to G1's ground truth, with the SLAM map frame taken as
the world frame (both start at the spawn), so it includes SLAM drift.

`probe_g1_views.sh SCENARIO OUT_DIR NAME:X:Y:YAW_DEG...` (Gazebo) starts a
`--plan-only` scene, moves the robot to each pose with `capture_g1_views.py` and
saves what the camera sees of G1 there: depth frames, the camera pose, the
detector's boxes and G1's ground truth, for offline localization review. The
robot is moved, so that session's map is not usable afterwards.
`state_timing.py RUN_DIR...` measures how long each mission state lasted in recorded
runs ([states and timing](parking_mission_states.md)). `record_demo_run.sh gz|isaac
SCENARIO OUT_DIR` runs one scenario with the Isaac task and angled view cameras and
the compact RViz layout, and writes `OUT_DIR/demo.mp4` at 8x speed: every frame holds
those views, the RViz screen and the mission state at the same moment of that run
(`record_demo.py`). `make_g1_view_fixture.py` cuts small test fixtures from those recordings
(`test/data/g1_views_2026-11-09.npz`).

## Limitations

- The basket, the mast-top enclosure and G1's hand reach are not measured.
- Height is checked only near G1 and only where the camera has looked.
- Side poses come close to G1's legs: the reach band leaves 0.12-0.15 m between
  the rear edge and G1's legs, and the LiDAR monitor stops approaches whose map
  pose has drifted that far relative to G1's estimate; a retry moves to another
  side.
- A static view of a side-on G1 against a matching background can still score
  below 0.40, and relocalization then times out without parking.
- During G1-centred exploration the shared explorer can stop within a few
  centimetres of the scene's objects (ground truth); parking legs keep their margins.
- One or two runs per scenario show behaviour, not a rate.

## Archived evidence

- [Parking scenarios](../../archive/engineering/2026-11-05-parking-scenarios.md) — every run by scenario and simulator, the provisional basket, and the layouts stopped by G1 detection.
- [Parking detection gaps](../../archive/engineering/2026-11-05-parking-detection-gaps.md) — why G1 detection stopped `heading_90` and `no_feasible`, the detector floor and relocalization hysteresis, the observation-fallback race, and the reruns.
- [Parking workstation scenes](../../archive/engineering/2026-11-05-parking-workstation-scenes.md) — the workstation layouts tried in Gazebo, every run on them, why routes near the spawn were rejected, and the perception limits that stopped most runs.
- [Parking after merging the group's dev](../../archive/engineering/2026-11-14-parking-dev-integration.md) — the merge, the basket-aware outline, the regressions the runs found and the completed missions in both simulators.
- [Reacquisition and recorded demonstration](../../archive/engineering/2026-11-11-parking-reacquisition-and-demo.md) — the stalled observation returns and the step back, target identity with several humanoids, and the recorded Isaac runs.
- [G1 position check](../../archive/engineering/2026-11-09-parking-g1-position-check.md) — why `no_feasible` parked beside a worktable edge and reported success, the head and torso check, the first-localization changes, and the workstation reruns.
