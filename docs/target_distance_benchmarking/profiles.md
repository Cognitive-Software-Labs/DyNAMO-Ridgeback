# Replay profiles and frozen evidence

The two offline formats, what each freezes, and what a run against each may
claim. Start at [running benchmarks](running_benchmarks.md) to pick a profile;
[outputs](outputs.md) owns what lands on disk.

## Legacy measurement replay

The legacy measurement profile is the measurement-accuracy path for box-gated stereoscopic
`projective_ranging` parameter sweeps: use it when only that recipe or its
numeric parameters change. The live runner captures a [quota](#capture-quota)
of raw detector batches per trial. It stores raw depth ROIs and camera context,
including invalid values and explicit missing matches; it never stores isolated foreground pixels or final
measurements.

The offline executor loads one trial per worker task and applies every variant
while that trial's arrays are resident. The detector implementation, checkpoint,
labels, threshold and cadence were fixed when the evidence was captured; offline
profiles reject overrides and preserve that provenance when present. A legacy
capture with missing identity is reported as unknown, not inferred from current
launch defaults. It reuses the live capture reduction,
association, scoring, trial-row, JSON/CSV, and sweep-report code. Per-config
keys that do not affect projective ranging are rejected; live-only sweep
defaults are removed from variant provenance.

A dataset is loadable only when every scenario trial produced a payload and the
manifest reports zero skipped trials. A bounded drain may legitimately retain
`NO_DEPTH_FRAME` evidence, but an infrastructure failure that skips the trial
marks the complete capture `incomplete`.

Replay is not authoritative for detector behavior, ROS delivery, timing,
throughput, models, or final integration; those questions need live runs, and
the offline loader refuses an `incomplete` dataset rather than compare a
reduced scenario set. Batch counts and worker scaling
require evidence for the actual workload; a result on one host is not a
universal throughput guarantee.

### Capture quota

Replay capture counts raw detector batches rather than elapsed seconds:
`capture_batches` per trial, default five, with empty batches counted. Once
the quota arrives, the runner drains only until every selected stamp has
matching depth, camera context and a live measurement, or `capture_drain_sec`
expires. Missing matches remain explicit replay evidence. Five batches is the
validated legacy default for the scenario and detector cadence it was measured
on, not a universal sensor statistic.
[Launch arguments](launch_arguments.md#replay-capture) lists the capture
settings.

### Legacy smoke workflow

One live run freezes the dataset; the offline command then evaluates every
configuration of the projective sweep over those same inputs. The dataset
directory and the offline output directory must both be new. This five-scene
example is a smoke workflow at the default five batches:

```bash
SCENARIO="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml"
SWEEP="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_sweep_projective_parameters.yaml"
DATASET="$PWD/artifacts/benchmarks/replay_projective_smoke_dataset"
LIVE_OUTPUT="$PWD/artifacts/benchmarks/replay_projective_smoke_live"
OFFLINE_OUTPUT="$PWD/artifacts/benchmarks/replay_projective_smoke_offline"

ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  scenario:="$SCENARIO" repeats:=1 estimators:=projective_ranging \
  mask_gate:=box depth_source:=stereoscopic record_video:=false gz_gui:=false \
  output_dir:="$LIVE_OUTPUT" run_dir_name:=matching_live \
  replay_dataset_dir:="$DATASET" \
  capture_drain_sec:=2.0 capture_timeout_sec:=30.0 \
  shutdown_on_complete:=true

ros2 run ridgeback_autonomy target_offline_replay_benchmark \
  "$DATASET" "$SWEEP" --output-dir "$OFFLINE_OUTPUT" --workers 4
```

Choose `--workers` for the machine rather than every logical CPU: past the
host's knee, more workers add overhead without reducing wall time. The
[outputs](outputs.md#sweep-outputs) page owns the offline root's layout.

## Layered replay profiles

`benchmarking/replay_profiles.py` is the ROS-free authority for the four stable
profile IDs, question mappings, mutable/frozen stages, legal axes, types,
ranges, compatible estimators, supported claims, and limitations. The command
`target_replay_describe --json` exposes that exact contract to CLIs and the
configurator. Invalid upstream knobs use structured errors; for example,
`segmentation_model` under `mask-output` recommends `mask-model`.

The offline artifact graph uses a version-2 typed envelope while the compact
legacy dataset keeps its own schema; historical `schema_version: 1` files
continue to load unchanged:

```text
sensor-capture (exact detections + RGB + depth + LiDAR scan + calibration + TF)
    ├── box mask-cache ───────────┐
    └── SlimSAM mask-cache ───────┴── measurement variants
```

Each typed manifest records `format: dynamo-replay`, manifest/payload versions,
a content-derived artifact ID, completeness, payload hashes, producer signature,
Git/dependency/model provenance, and trial/event/detection counts. A mask cache
also pins its sensor parent by artifact ID and exact manifest SHA-256. Loading a
cache without that parent, with a different parent, with a changed manifest, or
with a changed payload fails before evaluation.

### Sensor payloads

Sensor payloads contain one losslessly compressed NumPy file per trial. Every
raw batch remains present, including empty detections. RGB is exact `uint8` on
the detection grid; aligned depth is exact float32 metres with NaN/Inf/zero
semantics retained. Missing RGB, depth, intrinsics, or transforms remain
explicit `null` evidence. Ground truth is stored only in trial scoring metadata
and is never passed into a mask producer.

Sensor payload version 2 adds the LiDAR scan the live polar row measured from:
float32 `ranges` as its own payload array, the beam geometry and source frame
beside it, and the scan→camera-optical rotation and translation resolved at the
event stamp. A version-1 capture has no scan and still loads; an event that
never matched a scan keeps explicit `null` the same way a missing RGB frame
does. Layered replay projects that scan through the same `scan_points_optical`
the live row uses, so `polar_profiling` is measurable offline on a version-2
capture.

Legacy schema version 2 adds the same scan to that format, stored the same way:
float32 `ranges` as its own payload array beside the depth ROIs, beam geometry
and source frame in the JSON, and the scan→camera-optical rotation and
translation at the event stamp. One benchmark run therefore records beams into
whichever format it is writing. A schema-1 dataset has no scan and still loads —
it is the historical record the depth-path miss reasons were validated against.

### Mask caches

Mask caches store one index-aligned outcome per parent detection. `None` is an
explicit producer miss and differs from a valid empty `MaskRegion`. Regions
retain origin, full-grid size, `rect|tight` precision, shape, and bit-packed
boolean pixels. Tight regions use their actual nonzero extent, which may extend
outside the padded prompt box. Changing any producer parameter or provenance
creates a separately identified immutable cache: a different checkpoint,
revision, prompt padding, IoU floor, device or dtype, dependency version, or
code provenance yields a different producer signature and artifact identity.

`target_replay_materialize_masks` loads one model once and streams all parent
trials through it; the box producer needs no model. A canonical `mask-model` job
may name several materializations. Its default is one model worker, and parallel
model workers require distinct explicit devices so a parameter grid cannot load
many copies onto one GPU accidentally.

### Which estimators an offline job may select

Two questions, not one. The profile answers for its executor: every offline
profile now runs all three mask rows, since both frozen formats record a scan,
and only the live system reads the organized point cloud. The artifact then
answers for its own channels, through `unavailable_estimators`, keyed on the
version at which each format began recording a scan — a payload-version-1
capture and a schema-version-1 dataset both refuse polar profiling because they
predate it. Both refusals carry their reason. An estimator is never dropped
silently: a job selecting one the evidence cannot feed is refused before the
run, and one dropped from a variant is named with its reason in the report (see
[outputs](outputs.md)).

Polar profiling reads the scan, not depth, so an event whose depth frame never
arrived still yields a polar row while the depth rows record `NO_DEPTH_FRAME` —
the same split the live kernel makes. On the legacy dataset its mask is always
the frozen detection box, because that format stores no RGB to re-segment.

### Evaluation

`target_replay_benchmark` validates a job and all lineage before work. CPU
workers load one trial plus its selected caches and evaluate every compatible
numeric variant while the arrays are resident. Both depth rows call the live
`fill_path_measurements` kernel and therefore share one prepared depth region
per detection; polar profiling enters the same kernel with the stored scan and
is projected only when it was selected. Process-map ordering plus ordered reduction
makes sequential and parallel result rows byte-stable. Offline jobs accept only
the selected profile's axes in both sweep defaults and individual configs; use
a replay-specific sweep rather than carrying ignored live-only defaults into a
job.

Offline reports carry the profile boundary. `measurement` and `mask-output`
cannot support model/runtime claims. `mask-model` may retain isolated producer
duration as diagnostics, but only `live-system` can establish ROS delivery,
end-to-end latency, throughput, GPU contention, simulator real-time factor, or
integration behavior. Each offline report states its profile, supported
claims, artifact hashes, cache lineage, worker count and limitations. Remaining
live-parity, real-model, storage and scaling gates are owned by the
[benchmarking backlog](BACKLOG.md#layered-replay-validation).

### Layered workflow

Print the four profiles and their legal axes, without importing ROS or a model:

```bash
ros2 run ridgeback_autonomy target_replay_describe --json
```

Capture one full sensor parent with the live benchmark launch. The capture
follows the same [quota](#capture-quota) as the legacy format:

```bash
SCENARIO="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml"
SENSOR="$PWD/artifacts/benchmarks/sensor_capture"
LIVE_OUTPUT="$PWD/artifacts/benchmarks/sensor_capture_live"

ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  scenario:="$SCENARIO" repeats:=1 estimators:=projective_ranging \
  mask_gate:=box depth_source:=stereoscopic record_video:=false gz_gui:=false \
  output_dir:="$LIVE_OUTPUT" run_dir_name:=sensor_capture_live \
  sensor_capture_dir:="$SENSOR" capture_batches:=5 \
  capture_drain_sec:=2.0 capture_timeout_sec:=30.0 \
  shutdown_on_complete:=true
```

Derive immutable mask caches from it independently:

```bash
ros2 run ridgeback_autonomy target_replay_materialize_masks \
  "$SENSOR" --producer box \
  --output-dir "$PWD/artifacts/benchmarks/masks_box"

ros2 run ridgeback_autonomy target_replay_materialize_masks \
  "$SENSOR" --producer slimsam \
  --model Zigeng/SlimSAM-uniform-50 \
  --output-dir "$PWD/artifacts/benchmarks/masks_slimsam"
```

`target_replay_benchmark` runs one canonical job YAML or JSON. `mask-output`
requires the sensor parent plus one or more caches; `mask-model` requires the
sensor parent plus `materializations`, and creates its caches once before
running every measurement variant:

```yaml
job_version: 1
question: compare-mask-outputs
profile: mask-output
inputs:
  sensor_capture: ./artifacts/benchmarks/sensor_capture
  mask_caches:
    - ./artifacts/benchmarks/masks_box
    - ./artifacts/benchmarks/masks_slimsam
sweep:
  sweep:
    name: paired_mask_measurements
    description: Compare two measurement bands over every frozen mask cache.
  configs:
    - name: baseline
      estimators: projective_ranging,euclidean_reconstruction
      isolation_2d_band_m: 0.35
    - name: wider_2d_band
      estimators: projective_ranging,euclidean_reconstruction
      isolation_2d_band_m: 0.50
resources:
  measurement_workers: 4
comparison_baseline: masks_box:baseline
output_dir: ./artifacts/benchmarks/paired_masks
```

```bash
ros2 run ridgeback_autonomy target_replay_benchmark replay-job.yaml
```

Relative job paths resolve against the job file's own directory
([conventions](../project/conventions.md#generated-state)). Outputs appear only
after the whole job succeeds ([outputs](outputs.md#sweep-outputs)).

## Archived evidence

- [the validation history](../../archive/engineering/offline_measurement_replay_validation.md)
- [the layered replay validation note](../../archive/engineering/layered_replay_implementation_validation.md)
