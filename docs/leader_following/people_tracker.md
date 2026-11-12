# People tracker

The `people_tracker` ROS node combines camera person detections, registered
D455 depth and front/rear lidar detections into planar constant-velocity Kalman
tracks. It publishes `leader_follow/people` using the
[People interface](architecture.md#messages). Candidate selection and simulation
evidence are owned by the [benchmark reference](benchmarks.md).

## Sensor contract

All topics are relative to the robot namespace, normally `r100_0001`.

| Input | Default topic |
|---|---|
| RGB | `sensors/camera_0/color/image` |
| Registered metric depth | `sensors/camera_0/depth/image` |
| Color intrinsics | `sensors/camera_0/color/camera_info` |
| Front lidar | `sensors/lidar2d_0/scan` |
| Rear lidar | `sensors/lidar2d_1/scan` |
| Transforms | namespaced `tf` and `tf_static` |

RGB and depth must have the same optical frame and dimensions, matching color
intrinsics, rectified pinhole geometry, and at most 40 ms timestamp skew.
Supported encodings are RGB/BGR/RGBA/BGRA 8-bit color and `32FC1` metres or
`16UC1` millimetres for depth. Unregistered hardware depth must be registered
upstream; changing its frame label is not registration.

The node chooses the newest queued RGB frame with a complete synchronized
input bundle and sensor-to-output TF at that frame's stamp. Sensor/TF callbacks
continue while camera inference runs; fusion updates remain serialized. Depth, intrinsics and both scans come from
that timestamp or earlier, within the sensor-age limit. Each lidar uses TF at
its own scan stamp. There is no fallback to a latest transform at an unrelated
time. The queue contains up to 30 messages per input.

F1 recordings include warmup messages and a transient-local static TF
subscription with enough history for multiple publishers. Sensor readiness
requires a complete camera/lidar-to-odometry tree. The F2 corpus additionally
checks freshness and transforms at every evaluation tick.

## Measurement and fusion

The detector returns image boxes and confidence. The central torso portion of
each box supplies a median valid depth, excluding background near box edges.
At least 12 valid pixels between 0.15 and 15 m are required. Color intrinsics
back-project the box centre, and recorded/live TF transforms it to the output
frame. Measurement variance uses depth-dependent uncertainty and the median
absolute depth deviation, with a 4 cm minimum standard deviation.

Each lidar is processed with its native angle/range metadata. Its detections
are transformed into the common output frame, then merged with a 25 cm overlap
suppression radius. This preserves the two sensors' separate origins and blind
sectors; the learned adapters run on native scans rather than assuming a
co-located polar sensor. This detection-level merge supplies the planned
360-degree lidar coverage.

A camera detection can create a person track. Lidar observations can refine or
sustain an existing camera-confirmed track; lidar clusters alone cannot create
people. The filter predicts `[x, y, vx, vy]`, associates each sensor's measurements
once per tick using gated Hungarian assignment, and updates uncertainty with
the Joseph covariance form. ByteTrack image IDs prefer compatible camera assignments without bypassing the
position gate. Its low-confidence preset follows the selected detector score
scale; the benchmark reference records the preset. Camera updates precede lidar
updates. Unobserved
tracks persist internally only until expiry and are not emitted as detections.
IDs are stable within an association session; they are not appearance identity
and may change after expiry or a simulator clock reset.

The node publishes positions and velocities in the `People.header.frame_id`,
normally `odom`. The F2 default leaves `appearance` empty and cannot confirm
a leader identity. The optional F3 encoder supplies current-image embeddings
as described below. This node publishes no motion commands.

With appearance enabled, an unavailable RGB-D bundle can fall back to a fresh
front/rear lidar bundle with stamped TF. These updates sustain observed tracks
without adding appearance; they cannot create a person from lidar alone.
A complete queued bundle is processed before considering a heartbeat.

If input/TF becomes stale, the node emits empty `People` messages after 150 ms
of advancing ROS time by default. The F3 profile permits 250 ms for delayed
sensor/TF bundles. `observation_timeout` bounds this interval; a paused
simulator clock does not advance it.
A reset starts a new track session.

### Lidar walkers

Lidar detections that match no person track feed a separate constant-velocity
Kalman tracker (`LidarMovers` in `fusion.py`) with Hungarian association
inside a 0.6 m gate. Only tracks with at least 3 hits that have moved at least
0.3 m and walk at least 0.3 m/s are published, in the people frame, on
`leader_follow/moving_people` with the same header as `People`. Walls and
furniture therefore stay out, while a person approaching from outside the
camera view appears before the camera sees them. These walkers have no
appearance and serve collision avoidance only
([`people_predictor`](architecture.md#people_predictor-node)); they are never
leader candidates.

## Runtime

Build and source the workspace, prepare the candidate dependencies/checkpoints
using the [benchmark workflow](benchmarks.md#candidate-environments), then run:

```bash
ros2 launch ridgeback_leader_follow people_tracker.launch.py namespace:=r100_0001 use_sim_time:=true model_dir:="$PWD/artifacts/leader-following/f2-models"
```

`RIDGEBACK_PERCEPTION_VENV` selects the compute interpreter; otherwise the
launch uses `perception_venv` under the current workspace. `device:=cpu` permits
a CPU camera run. Benchmark-only TensorFlow and the separate MOT environment
are unnecessary for a selected runtime that does not use those adapters.
Prepare weights before running without network access.

| Parameter | Default | Meaning |
|---|---|---|
| `camera_candidate` | `owlv2` | Selected quality-first camera; `rfdetr` is the faster alternative |
| `lidar_candidate` | `lfe-ppn` | Selected native-scan adapter, bounded CPU thread pools |
| `model_dir` | `artifacts/leader-following/f2-models` | Model storage and adjacent pinned vendor sources |
| `device` | `cuda` | Camera/compatible lidar compute device; LFE-PPN and legacy adapters use CPU |
| `output_frame` | `odom` | Position and velocity frame |
| `maximum_sensor_age` | `0.075` | Maximum preceding-input age, seconds |
| `association_gate` | `0.75` | Maximum position association distance, metres |
| `track_maximum_age` | `0.75` | Unobserved-track expiry, seconds |
| `observation_timeout` | `0.15` | Maximum gap before an empty heartbeat, seconds; F3 selects `0.25` |
| `rgb_topic`, `depth_topic`, `camera_info_topic` | above | Camera topic overrides |
| `front_scan_topic`, `rear_scan_topic` | above | Raw scan topic overrides |

The selected camera favors detection quality over latency; offline scores do
not establish a real-time deadline. Dense crowds still cause association errors.
The robot/Thor runtime and performance checks remain pending until hardware
access. The current path is a stationary-observer simulation tracker; it does
not certify camera calibration, latency, collision avoidance or closed-loop
following on hardware.

## Optional appearance encoding

`appearance_candidate` defaults to `none`, preserving the F2 profile. Setting it
to `osnet-ain`, `clip-reid`, `transreid`, `solider` or `dinov2` extracts normalized
features from associated RGB detection crops before camera/lidar fusion.
`appearance_model_dir` defaults to `artifacts/leader-following/f3-models`.
Only the current camera observation supplies appearance; lidar-only updates
publish an empty vector. The [recognition reference](recognition.md) owns
model variants, setup, matching and validation. OSNet-AIN is selected for simulation; the recognition reference records the
passed F3 checks and remaining coverage limitations.
