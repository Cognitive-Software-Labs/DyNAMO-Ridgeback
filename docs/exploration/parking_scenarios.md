# Parking scenarios

The parking demonstration can spawn one of several named scenes around G1. Each
scene tests one part of the existing mission: the G1-relative preference order,
blocked sides, obstacles above the LiDAR plane, a scene with no safe pose, and
G1 headings. The mission, planner and safety checks are the same in every scene;
only the spawned objects differ. Results are in the
[scenario gallery](autonomous_parking_gallery.html) and the dated record under
[archived evidence](#archived-evidence).

## Running a scenario

`demo_scenario` selects a scene from
[`parking_scenarios.yaml`](../../src/ridgeback_autonomy/config/parking_scenarios.yaml);
the default `standard` is the original G1 and table. The wrapper forwards it:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv
bash start_parking_demo.sh gz demo_scenario:=right_blocked
# After stopping Gazebo with Ctrl-C:
bash start_parking_demo.sh isaac demo_scenario:=right_blocked
```

Startup spawns G1 and each object through the simulator's standard scene
services, as for the original scene, and refuses a world that already holds a G1
or a `bench_parking_demo_*` entity. Objects are logical names from each backend's
`sim/benchmark_assets.yaml`, so Gazebo and Isaac spawn the same collision
geometry at the same world pose. A scene's `expected` text documents the intent;
the mission never reads it.

| Scenario | Layout around G1 at (3, 0); the robot starts at the origin | Intended outcome |
|---|---|---|
| `standard` | facing the start; table on G1's right | side parking, left preferred |
| `worktable` | back to the start, facing a table 0.40 m ahead | left or right, never under the tabletop |
| `right_blocked` | facing the start; 1.7 m panel along G1's right | left |
| `left_blocked` | facing the start; 1.7 m panel along G1's left | right |
| `both_sides_blocked` | facing the start; panels along both sides | behind G1 |
| `restricted` | facing away; tables both sides (the base fits under, the mast does not), low bed ahead, 2.1 m passage | behind G1, never under a tabletop |
| `no_feasible` | facing the start inside a closed pen of low beds and side panels | no manoeuvre; bounded planning, return to start with `no_valid_candidate` |
| `heading_90` | side-on, facing +y, no objects | sides rotate with G1 |
| `heading_away` | facing away from the start, no objects | heading from behind, or the fallback |

`restricted` and `no_feasible` are Gazebo-only (`backends: [gz]`): Isaac's demo
world has corridor walls at |y| 1.5-1.7 m that their objects would overlap, and
startup refuses them there. OWLv2 scores a Gazebo G1 at about 0.42-0.68 depending on
the view; side-on and partly hidden views sit near 0.50. The depot's walking person
scored at most 0.19 as `humanoid robot`, and nothing else above 0.22. With the generic 0.55 detector floor `heading_90` and `no_feasible`
stopped before planning; the parking launch now runs the detector at 0.40 with the
hysteresis described under [observation return](architecture.md#observation-return).
Keep objects low or away from G1's outline as seen from the start: a static view of a
side-on G1 in front of a matching background still scores below 0.40 (see the
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

The task point is the centre of the thesis bin the provisional basket copies. It
stays the interaction point, and the reach band stays 0.30-0.60 m, until the
crate's position on the deck and G1's reach are measured.

## Evaluating a run

The tools under [`tools/parking_scenarios/`](../../tools/parking_scenarios/) run
one scenario in its own process session, sample Gazebo ground truth at about
1 Hz, take RViz and Gazebo screenshots, and stop only that session with SIGTERM:

```bash
tools/parking_scenarios/run_scenario.sh gz right_blocked artifacts/exploration/scenarios/gz_right_blocked
python3 tools/parking_scenarios/scenario_metrics.py artifacts/exploration/scenarios/gz_right_blocked
# Rebuild the gallery from chosen runs; CODE in the script labels each recorded HEAD:
python3 tools/parking_scenarios/make_gallery.py artifacts/exploration/scenarios/gz_*
```

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
estimated state only. Bags hold only the mission state, parking display, plan
and map; scans are not recorded.

## Limitations

- The basket, the mast-top enclosure and G1's hand reach are not measured.
- Height is checked only near G1 and only where the camera has looked.
- Side poses come close to G1's legs; the LiDAR monitor stops such approaches,
  and a retry moves to another side.
- A static view of a side-on G1 against a matching background can still score
  below 0.40, and relocalization then times out without parking.
- During G1-centred exploration the shared explorer can stop within a few
  centimetres of the scene's objects (ground truth); parking legs keep their margins.
- One or two runs per scenario show behaviour, not a rate.

## Archived evidence

- [Parking scenarios](../../archive/engineering/2026-11-05-parking-scenarios.md) — every run by scenario and simulator, the provisional basket, and the layouts stopped by G1 detection.
- [Parking detection gaps](../../archive/engineering/2026-11-05-parking-detection-gaps.md) — why G1 detection stopped `heading_90` and `no_feasible`, the detector floor and relocalization hysteresis, the observation-fallback race, and the reruns.
