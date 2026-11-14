# Hardware-bag diagnostic bridge

The workstation command `target_hardware_bag_validation` reads saved hardware
MCAP recordings without publishing ROS messages or starting a simulator. Its
outputs are unscored estimates and input/status evidence, separate from final
hardware accuracy, parameter selection, and live throughput qualification.
Its commands are under [running the diagnostic](#running-the-diagnostic).

## Frozen input and processing contract

`describe` selects exactly 48 unique `capture_complete == true` entries from
`quality/recording-plan-closeout-audit-20261101.json`. It starts with
`02_P01-03_attempt2`, excludes attempts 01 and 25, and uses exact bag names to
join `segments.jsonl`, closeout reference associations, and receipt inventories.
Aliases remain labels on the same physical trial. Supplied reference documents,
chronological coordinate corrections, qualitative headings and turn-carried
position qualifications survive. No heading becomes measured yaw; no reference
is eligible for final scoring.

The job freezes absolute paths, source revision and tracked-patch digest,
Python implementation hashes (including untracked bridge sources), input
metadata hashes, raw file sizes/mtimes and saved SHA256 offload receipts, local
model snapshot contents, dependencies, explicit device, float32 dtype, settings,
and the ten paths. Describe validates metadata hashes against receipts and raw
file sizes; it **does not rehash MCAP**. The receipts retain their capture/offload
provenance. Model weights are checked locally, without downloading or inference.
Model cache namespace and snapshot revision identify the model; config types
and file hashes detect replacements. `check-models` additionally loads all three
models without processing frames. Execution fails on unavailable models/devices;
there is no automatic device or model fallback.

OWL uses `google/owlv2-base-patch16-ensemble`, prompt `humanoid robot`, threshold
0.30 and production parsing/NMS order. SlimSAM and metric indoor Depth Anything
use the pinned snapshots recorded in the job. The monocular ceiling is its
checkpoint's declared `max_depth` times the existing usable-range fraction 0.9.
That fraction is an implementation setting, not a validated reliability cutoff.
Stereo has no invented ceiling. All derivatives explicitly use the measured
[hardware front datum](../robot/geometry.md#hardware-distance-reporting-front-datum)
0.465 m; historical replay and simulator defaults stay unchanged.

| Estimator | Masks | Input | Paths |
|---|---|---|---:|
| projective ranging | box, SlimSAM | stereo, monocular | 4 |
| Euclidean reconstruction | box, SlimSAM | stereo, monocular | 4 |
| polar profiling | box, SlimSAM | recorded front scan | 2 |

Resolved production isolation, segmentation and validity settings are explicit
in the job. Tight silhouettes retain their production depth-isolation behavior;
an empty silhouette never falls back to a box. Each process loads each model
once. OWL and monocular depth run once per selected RGB; segmentation runs once
per nonempty accepted detection batch. Products feed every applicable estimator.
Empty detector batches retain ten explicit empty path results. Missing stereo
or scan affects its own paths; model/infrastructure failures stop the batch.

## Time, decoding and recorded geometry

Settled inclusion uses bag receive time with `[start, end)` boundaries, matching
the recording validator. The first eligible RGB header stamp anchors 100 ms
buckets; keep the first source-order frame in each occupied bucket. Empty
buckets create no event. Duplicate eligible header stamps are counted and kept
once; a header regression or malformed timestamp is an explicit input fault.
Event IDs contain bag, selected ordinal, header stamp and source order.

Stereo pairing requires exact RGB/depth header equality, using the first source
message for duplicate matches. Scan pairing chooses the nearest front scan
within inclusive 50 ms, then the earlier stamp and source order for ties.
Candidates outside the receive-time settled interval remain legitimate context;
no additional RGB outside the interval is selected.

One forward ROS bag traversal creates a temporary SQLite/disk image index. All
aligned depths and selected RGB are spooled to disk, with only one bag's scan,
calibration and TF metadata in memory. Events assemble after EOF, allowing late
messages to match without a guessed receive-time horizon. This adds temporary
disk I/O and holds approximately one raw bag's relevant images on disk; it
avoids rereading MCAP for another estimator and keeps image memory to a frame
pair plus model products. No whole-session image list is held in RAM.

Colour CameraInfo must have finite positive focal lengths and matching grid and
optical frame; the latest header not later than RGB is retained. Recorded
distortion, K/R/P and context source identities are saved. Production projection
is pinhole; the bridge does not undistort or recalibrate. Aligned depth must
share the colour grid/frame. Shared image decoders honor row byte strides and
endianness; published `16UC1` millimetres become float32 metres once, while
`32FC1` is already metres. Invalid zero/NaN/infinity values survive. Content
counts and positive-depth extrema are saved. The deployed capture-time driver
binary remains incompletely pinned by the original evidence.

TF comes entirely from recorded messages, including static context before the
settled window. Quaternion/translation validity is checked; contradictory static
transforms fail explicitly. Dynamic TF interpolates between recorded bracketing
samples, with no extrapolation. Camera-to-base is resolved at RGB time;
scan-to-camera uses scan time and requires the recorded camera mount to agree
at RGB and scan times. Unavailable or moving-mount context yields an explicit
transform miss rather than an invented cross-time compensation. The base frame
is an explicit recorded identity, independent of topic namespace.

## Evidence, completion and recovery

The output root holds immutable `job.json`, atomic `progress.json` and
`report.md`. Each bag holds:

- `source/`: per-event JSON + compressed NumPy payloads, retaining RGB, stereo,
  calibration/TF, raw scan, header/receive stamps, source order and missing reasons;
- `inference/`: cached detections, box/SlimSAM regions and statuses, monocular
  depth and its ceiling, in the same hashed per-event envelope;
- `events.jsonl`: ten unscored path rows per event, all detections in producer
  order, numeric statuses, F-frame forward/lateral/radial and recovered base
  planar points; height is unavailable from the shared kernel;
- `summary.json`: event/detection denominators, successes, separate input/mask/
  estimator failures, status histograms, references, effective settings and
  inspection classifications;
- up to three `overlay-*.png` panels: RGB, box, silhouette and projected scan;
- `complete.json`: job identity and hashes of all committed bag evidence.

A JSON commit marker hashes its NPZ partner; interrupted uncommitted files are
not completion. A bag completes only after its ten-path accounting, summaries,
overlays and evidence are validated. Resume verifies source/config/model/code/
dependency identities and completed-bag hashes; it skips completed bags and
reuses committed source/inference events in incomplete bags. A crash during
raw indexing starts a visibly numbered new indexing attempt. A crash during
an uncommitted model event may repeat that event. Scratch and failure evidence
remain inspectable; successful source indexing removes its scratch directory.

A corrupt single raw input becomes `input_failed`, and unaffected bags continue.
Resume retains that failure rather than automatically retrying it. Inspect the
reason and use a new output after recovery. Shared processing failures leave the
current bag incomplete and stop before processing subsequent bags. Completed
outputs are never overwritten for a changed configuration.

`report` validates completed evidence and recreates the compact report without
inference. `remeasure` creates a separate derivative from completed caches using
the current measurement kernel/settings. It can accept a JSON mapping of exact
bag names to revised qualified reference documents. Reference changes never
enter measurement or final scoring. The derivative freezes parent hashes,
current code/settings and overrides; original completed evidence is preserved.
Full-height base points cannot be reconstructed from planar-only kernel output.
Radial distance is recomputed after front-coordinate translation, never adjusted
by subtracting 0.465 m from an off-axis radius.

Classifications retain evidence and actions: diagnostic usable, remotely
recoverable software issue, reference limitation requiring qualified claims, or
possible re-record need after recovery and limited-use options are exhausted.
The report does not automatically infer a re-record need from an OWL miss, empty
mask, distance error or assumed position. A raw input failure first requires
inspection to establish a concrete unrecoverable observation. No nominal
reference differences or final accuracy aggregates are computed by this tool.

## Running the diagnostic

Build the changed packages after `tools/check_dependencies`, then use the
perception environment with Jazzy's decoding libraries. No ROS graph or playback
is needed. Describe reads metadata and validates pinned local model snapshots;
it performs no frame inference. Paths resolve against this workspace.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "${RIDGEBACK_PERCEPTION_VENV:-$PWD/perception_venv}/bin/activate"

SESSION=/home/stefi/ScenesForDetectionTesting/20261029T141724Z-distance-benchmark-raw
JOB=artifacts/benchmark-jobs/hardware_bridge_cpu.json
OUT=artifacts/benchmarks/hardware_bridge_cpu
ros2 run ridgeback_autonomy target_hardware_bag_validation describe \
  --session "$SESSION" --job "$JOB" --output "$OUT" --device cpu
ros2 run ridgeback_autonomy target_hardware_bag_validation check-models --job "$JOB"
```

Use an explicit `--device cuda:0` and fresh job/output for GPU processing after
checking the driver, free GPU/RAM/disk and contention. There is no automatic
fallback. Real-bag execution is a separate assignment from preparing the job.
When execution is assigned:

```bash
ros2 run ridgeback_autonomy target_hardware_bag_validation run --job "$JOB"
# After an interruption, keep the same job/configuration:
ros2 run ridgeback_autonomy target_hardware_bag_validation resume --job "$JOB"
# Inspect validated committed outputs without inference:
ros2 run ridgeback_autonomy target_hardware_bag_validation report --output "$OUT"
# Recompute a separate software derivative without raw reads or inference:
ros2 run ridgeback_autonomy target_hardware_bag_validation remeasure \
  --evidence "$OUT" --output artifacts/benchmarks/hardware_bridge_remeasured
```

Optional `remeasure --references corrections.json` accepts exact bag names
mapped to revised qualified reference documents. All outputs remain unscored.
Job/output names must be new; changed code, settings, input identity, models or
dependencies refuse incompatible resume. Exit 2 indicates explicit single-input
failures after unaffected bags finish; exit 1 indicates a stopped processing or
validation fault. Both retain evidence. Offline duration establishes no live
throughput or final hardware accuracy claim.
