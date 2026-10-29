# Benchmark semantics

What a scenario, an event, an observation, an instance row, and a miss reason
mean, and which runs may be compared with which. Start at
[running benchmarks](running_benchmarks.md) if you are choosing how to launch.

## Scenario and comparison discipline

The packaged full set is `config/benchmark_scenarios_full.yaml` (88 scenes,
20 multi-robot; verify current source before reporting counts). It includes
occlusion, clutter, image-edge, trap, and probe scenes; the
[gallery](benchmark_scenarios_v2_gallery.html) renders every scene from its
[certification](#scene-certification) audit.

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
  [UST-10LX patch](../project/dependency_patches.md#ust-10lx-gazebo-lidar-patch),
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
those services. Both catalogs stand each model on the floor's top surface, so
it stands identically in both simulators (see [scene certification](#scene-certification)).

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
runner spawns each model at its catalog `spawn_z_m`: every packaged model's
origin is its ground contact, and both catalogs stand it on the calibration
floor's top surface at z = 0.05. Ground truth is planar, so the height changes
only what the sensors see. `benchmarking/scenarios.py` is the authoritative
parser and validation contract; scenario-file comments record each scene's
intended condition and its certified values.

## Scene certification

`scene_audit` certifies scenes geometrically, without a simulator. It ray-casts
a depth-ordered id buffer of the Gazebo visual geometry standing on the floor
through the sensor geometry expanded from `clearpath/robot.yaml`, and casts the
front LiDAR in its scan plane. The `benchmarking/scene_audit.py` docstring owns
the exact definitions:

| Quantity | Meaning |
|---|---|
| visible fraction | target pixels visible in the scene ÷ its pixels with all other geometry removed |
| in-frame fraction | target pixels inside the image ÷ its pixels on an unbounded image plane |
| centre ratio | `abs(u - cx) / cx` of the projected torso centre; above 1 the centre is outside the frame |
| LiDAR | beams on each target and pole, each shin's visible angular width, pole inset in the detection box |

Comments are certified at 640x480, the profile both simulators default to and
the Thor perception host uses. Family names record the intended condition; a
comment outside its family band says so. `edge640_*` scenes are edge cases only
at 640x480: the frame edge sits at ±39.8° there and at ±45° at 1280x720.

The Isaac stage matches the geometry the audit models: camera pose, intrinsics,
LiDAR pose and every object agree exactly. The G1 is the exception: Gazebo
renders the STL G1 under `ridgeback_autonomy_gz/sim/models/g1`, Isaac the
Unitree 29-DoF Dex1 USD under `ridgeback_autonomy_isaac/sim/isaac/usd/models/g1`
(provenance in its `NOTICE.md`).
Both stand on the floor with their origin at ground contact, but silhouettes,
limb gaps and LiDAR leg returns differ, so certifications describe the Gazebo
G1 and a target-dependent result does not transfer between backends. Re-run
the audit after any change to camera or LiDAR geometry, assets or the scenario
file:

```bash
ros2 run ridgeback_autonomy scene_audit \
  --scenario src/ridgeback_autonomy/config/benchmark_scenarios_full.yaml \
  --profile 640x480 \
  --gallery docs/target_distance_benchmarking/benchmark_scenarios_v2_gallery.html
```

Its `certifications_640x480.csv` compares every comment with the audited value.

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
- [scene recertification for the current camera](../../archive/engineering/2026-10-29-scene-recertification.md)
