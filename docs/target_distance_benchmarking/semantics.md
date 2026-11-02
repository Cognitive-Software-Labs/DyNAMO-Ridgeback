# Benchmark semantics

What a scenario, an event, an observation, an instance row, and a miss reason
mean, and which runs may be compared with which. Start at
[running benchmarks](running_benchmarks.md) if you are choosing how to launch.

## Scenario and comparison discipline

The packaged full set is `config/benchmark_scenarios_full.yaml` (88 scenes,
20 multi-robot in the recorded 2026-08-24 transition; verify current source
before reporting counts). It includes occlusion, clutter, traps, and probe
scenes; the [gallery](benchmark_scenarios_v2_gallery.html) is a visual reference.

- Compare runs against the exact same scenario YAML and revision. Regenerating
  with a new seed expands coverage; it is not an A/B baseline.
- A scene can override repeats. `repeats:=1` still allows a probe with
  `repeats_override: 8` to collect its intended within-pose sample.
- Record current parameters and code provenance; an older scene/model/render
  configuration is not directly interchangeable with a current run.
- Gazebo and Isaac publish the same UST-10LX scan contract: 1081 bins at
  0.25°, 0.06–10 m, 40 Hz. Neither simulator lets `polar_profiling` reach a
  surface beyond 10 m; such a ray is `+inf`, and the row reports `no_value`
  instead of latching a far wall. A Gazebo run from before the
  [UST-10LX patch](../project/dependencies.md#ust-10lx-gazebo-lidar-patch),
  with 540 bins out to 25 m, is a different population and cannot serve as a
  polar baseline.
- Both simulators add LiDAR range noise, and neither is bit-reproducible on
  the LiDAR path. A/B comparisons of `polar_profiling` need repeats.
  LiDAR noise does not change the camera inputs; the two examples runs used
  to measure this effect do not establish general camera-path determinism.
  How the two noise models differ is described in
  [the LiDAR pipeline](../isaac/lidar-pipeline.md#gazebo-parity).
- Never run `cleanup.sh` between persistent sweep configurations. A sweep owns
  one simulator/RViz/detector environment and restarts the per-config layer.

### Simulator assets and cross-simulator comparison

The scenario names a world, the target and each object model only logically.
Each simulator adapter installs `sim/benchmark_assets.yaml`, a catalog that
maps those names to its own world and spawnable resources:
- Gazebo maps them to SDF; `target_distance_calibration` is its own world.
- Isaac maps them to USD. The public `target_distance_calibration` runs as its
  native `g1_distance_calibration`, which has the same geometry.

The runner places, reads and removes entities only through the standard ROS
`simulation_interfaces` services the selected adapter serves, so it contains
no simulator code. Ground truth comes from one state snapshot taken through
those services. Both catalogs spawn at the literal scenario height, so a model
stands identically in both simulators: its origin at z = 0, 5 cm into the
0.05 m floor slab.

What a Gazebo-to-Isaac comparison may claim:
- **Same-revision baseline.** Compare an Isaac run only with a Gazebo run of
  the same revision, scenario bytes and estimator settings. A pre-patch run is
  not that baseline.
- **Differences are expected, not defects.** The renderers differ, so detector
  inputs and masks differ. The LiDAR noise models differ (see the LiDAR bullet
  above), and Isaac optionally adds D455-like depth (`depth_fidelity:=d455`).
- **Coverage beside error.** Report coverage next to every error. Compare
  estimators on the instances both simulators scored, because each simulator's
  own scored set differs, and a lower error on fewer instances is not an
  improvement.
- **Isaac A/Bs need repeats for every estimator.** Isaac is not reproducible
  run to run on any estimator path: same-code runs differ in detector outcomes
  and camera-path estimates.

### Background props

`background:=lab` furnishes the room with one fixed layout of static props,
`config/benchmark_background_lab.yaml`. Props are never moved or randomized.
An empty `background` runs the bare room with the robot at the origin, as
every run did before backgrounds existed.

**Placement.** With a background, each scene is captured shifted along world
x by its own placement `dx`: the robot, its targets and its objects move
together, so every pose relative to the robot stays as the scenario file
declares it. Ranges, bearings and each scene's occlusion design are
unchanged; only the room behind the targets changes. Ground truth is
robot-relative (it comes from the state snapshot), so `true_forward_m` and
`true_distance_m` do not depend on `dx`; `spawn_world_x` includes it. The
robot moves only along x and always faces +x.

`dx` is chosen offline from the multiples of 0.05 m that keep the robot, every
target and every object 0.30 m inside the walls. The anchor is the target with
the largest camera range. Its backdrop is the nearest kept prop inside the
anchor's silhouette cone (half-width `atan(0.40 m / range)`) and beyond it,
otherwise the wall; the gap is how far behind the anchor's centre the
backdrop's nearest point stands. Among the candidates with a prop backdrop,
the one whose gap is nearest `backdrop_gap_m` (1.0 m) wins, then the fewest
drops, then the smallest `|dx|`, then the smaller `dx`; wall backdrops compete
only when no candidate sees a prop. In the full set `dx` runs from −3.9 to
8.3 m and every anchor has a prop backdrop. 77 of 88 scenes land within
0.025 m of the 1.0 m gap; the other 11 (1.15–3.61 m) are `interfere_*` and
`trap_layered_*` scenes whose designed object behind the target keeps props
out from in front of it.

**Drop rule.** At its placement, a scene drops every prop that would
conflict with it, and records why:
- `target_footprint`: within 0.50 m of a target's spawn point (the G1's
  outline reaches 0.39 m from it);
- `object_footprint`, `robot_footprint`: within 0.30 m of an object's
  footprint, or of the robot's Nav2 footprint at its placement;
- `target_sightline`, `object_sightline`: within 0.45 m of a camera-to-target
  or camera-to-object sight line;
- `behind_target_object`: only for a target with a designed object behind
  it, inside that target's cone (its width plus the margin) nearer than the
  object.

Every other prop stays, so clutter close behind a target is intended: near
background is now part of every placed scene, not only of the `interfere_*`
and `trap_layered_*` scenes.

**Generated data.** `tools/benchmark/build_background.py` writes each
packaged scene's `dx`, anchor (target index, backdrop and gap) and drop list
to `config/benchmark_backgrounds/lab.yaml`, with the rule constants and
`backdrop_gap_m`. The file pins the layout and every scenario file by
SHA-256. The runner refuses a layout or scenario file that changed since then,
a scenario file the drop lists do not cover, any scene without an entry, and
a drop file from the earlier schema; regenerate after editing either. Its
summary reports how many scenes each prop is the backdrop for and flags any
prop above a third. Footprints come from the Gazebo models, the geometry's
source of truth; the Isaac models are generated from them (see the
[asset notes](#simulator-assets-and-cross-simulator-comparison)).

**Run sequence.** A placed run needs the simulator to advertise entity-state
setting (`simulation_interfaces` feature 11); both adapters do. The runner
first moves the robot home if an interrupted run left it elsewhere, then
spawns the whole layout once, after the readiness gates and before the first
trial, as `bench_<run>_bg_<prop>`. Each trial then:
1. removes the props its scene drops, starting from what is actually present;
2. moves the robot to `(dx, 0)` facing +x with `set_entity_state` and waits
   until the simulator reports it there (every trial moves it, even to the
   placement it already has, so every trial takes the same steps);
3. restores the props an earlier scene removed;
4. spawns the scene's objects, then its targets, and settles;
5. checks the state before capturing.

The state check takes one snapshot. The `bench_` entities present must be
exactly the kept props, objects and targets, each within 0.005 m and
0.002 rad of its commanded pose, with the robot within the same tolerance of
its placement. Otherwise the trial fails with `state_mismatch` and is skipped,
with every difference logged. The check also records a SHA-256 fingerprint
over run-independent labels (`robot`, `bg_<prop>`, `obj_<i>`, `target_<i>`),
so the same scene gives the same fingerprint in any run and any order. The
trial's own rollback still covers only its objects and targets. At the end of
the run every prop is deleted and the robot returns home. An interrupted run
leaves the rest under the `bench_` prefix, which the sweep reaps and which
makes the next single run refuse to start; a robot it leaves placed is moved
home by the next placed run.

**Control arm.** `background_props:=false` (only with a background) places
every scene by the same `dx` and runs the same moves and state checks in the
bare room. It is the control for the props: compare a props run with the
control arm of the same revision. `scene_ids:=a,b,c` runs those scenes of the
scenario file in that order, which is how scene-order independence is tested.

**Comparability.** Placed runs, with props or without, are not comparable with
runs that kept the robot at the origin: that is every run without a
background, and every background run recorded before placement existed
(2026-11-01, drop schema 1). The room behind every target differs between
them, walls included. Props use Gazebo Fuel meshes (CC BY 4.0, attribution
beside each model) and in-repo primitives, one mesh set for both simulators.
Expect them to move coverage and extra-detection numbers.

**Galleries.** The [Gazebo](benchmark_background_gallery_gazebo.html) and
[Isaac](benchmark_background_gallery_isaac.html) placed-scene galleries show
every scene of the full set at its placement with the `lab` background: the
robot camera beside a fixed room overview, with each scene's backdrop and
drops. They are a visual reference, not estimator evidence.

### Scenario schema

Scenario YAML is declarative. A file contains optional defaults and a non-empty
`scenes` list. Every scene has a unique id and at least one target; objects and
per-scene repeat overrides are optional:

```yaml
defaults:
  robot_yaw_rad: 3.141592653589793
scenes:
  - id: example
    repeats_override: 2
    robots:
      - {x: 3.0, y: 0.4, yaw: 3.14}
    objects:
      - {model: hospital_bed, x: 2.0, y: 0.3, yaw: 1.57}
```

Coordinates are world-frame planar poses: `x` is forward, `y` is lateral
(positive left), and `yaw` is radians about +Z. Scenarios carry no height. The
runner spawns each model at its catalog `spawn_z_m`, which is 0 for every
packaged scene model (background props stand on the floor top, 0.05).
Within a trial the runner spawns the scene's objects first, then its targets,
so the scene is complete before a target appears. `benchmarking/scenarios.py` is the authoritative parser and
validation contract; scenario-file comments record generation-specific design
and certification details.

## Event, observation, and instance semantics

One **event** is one aligned measurement frame keyed by frame id, exact stamp,
detection count, and bounding boxes. Pointcloud and mask messages with that key
merge. A multi-robot frame is still one event.

One **observation** is one detected box. Status histograms count observations,
so a two-box event contributes two statuses per estimator. A statusless
pointcloud value gets coarse OK/UNSET inferred from finiteness.

One **instance row** is a ground-truth target for one trial and estimator.
Association uses per-estimator planar positions, so every estimator scores on
its own usable events. Each final trial estimate is the median of that
estimator's associated values. Consequences:

- An estimator can miss while another scores; there is no common-event gate.
- A zero-detection trial records every instance as missed for every estimator.
- `detector_miss`, `gate_miss`, and `no_value` are scored outcomes. A skipped
  trial means infrastructure failure (for example spawn/stream failure), not a
  perception miss.
- No-detection frames and per-frame raw estimates are not themselves accuracy rows.

## Miss reasons and attribution boundary

Mask measurement messages carry a status per detection and estimator.
`compute_status_histogram` counts every box; a specific failure outranks UNSET
when selecting a dominant explanation. Trial CSV `miss_reason` is populated only
for `no_value`; the other miss categories are already named by `outcome`.

Attribution never borrows another estimator's position or assumes detection
order. In a multi-instance trial, a box status reaches an instance row only when
that same estimator's locator associates the box to that ground-truth instance.
Statuses on unmatched boxes remain observation-level evidence. The one exception
is an event containing exactly one target and one box, where identity is
unambiguous even without an estimator locator. When a `no_value` row has no safe
association, `miss_reason` is blank and the report renders `no_value (reason
unknown)` rather than copying the trial's dominant reason onto one or more
unproven identities.

`scored` rows need no reason; `gate_miss` and `detector_miss` are already explicit
outcomes. The collage remains a trial-level evidence view and may show a dominant
estimator reason; it is not an instance attribution source.

Accuracy/miss fields are per instance. `observations` and `reason_histogram` are
per detected box. Do not add those granularities together or interpret coverage
as accuracy. The authoritative enum is `ridgeback_common/miss_reason.py`; the pipeline
[glossary](../target_localization/target_localization_pipeline.md)
explains stages.

Historical scenario/status transitions are retained in the Archived evidence section below.

## Archived evidence

- [benchmark history](../../archive/engineering/benchmark_evolution.md)
- [Gazebo and Isaac backend qualification](../../archive/engineering/2026-10-26-benchmark-gazebo-isaac-qualification.md)
