# Replay profiles and frozen evidence

The two offline formats, what each freezes, and what a run against each may
claim. Start at [running benchmarks](running_benchmarks.md) to pick a profile;
[outputs](outputs.md) owns what lands on disk.

## Legacy measurement replay

The legacy measurement profile is the measurement-accuracy path for box-gated stereoscopic
`projective_ranging` parameter sweeps. The live runner counts an exact number
of raw detector batches per trial, then performs a bounded exact-stamp drain.
It stores raw depth ROIs and camera context, including invalid values and
explicit missing matches; it never stores isolated foreground pixels or final
measurements.

The offline executor loads one trial per worker task and applies every variant
while that trial's arrays are resident. It reuses the live capture reduction,
association, scoring, trial-row, JSON/CSV, and sweep-report code. Per-config
keys that do not affect projective ranging are rejected; live-only sweep
defaults are removed from variant provenance.

A dataset is loadable only when every scenario trial produced a payload and the
manifest reports zero skipped trials. A bounded drain may legitimately retain
`NO_DEPTH_FRAME` evidence, but an infrastructure failure that skips the trial
marks the complete capture `incomplete`.

The root README owns the capture and replay commands. Replay is
not authoritative for detector behavior, ROS delivery, timing, throughput, or
final integration. The 2026-09-09 clean full-scenario run selected five batches,
proved live/offline parity across 135 rows, and cleared the performance gate by
about 50x; the complete evidence is in
[the validation history](../history/offline_measurement_replay_validation.md).

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

In the 109-trial `sensor_capture_full_benchmark` capture, 526 of 545 events
(96.5%) carried both a scan and its extrinsics, against 86.4% for exact RGB and
71.9% for exact depth. The scan is therefore the most reliably captured channel,
not a new completeness risk.

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
creates a separately identified immutable cache.

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
integration behavior. Implementation evidence and the remaining live gates are
recorded in
[the layered replay validation note](../history/layered_replay_implementation_validation.md).
