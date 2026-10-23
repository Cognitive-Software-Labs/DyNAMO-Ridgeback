# Intel–Thor perception frame timing and depth-one deployment

Recorded dates: 2026-10-22, 2026-10-23
Tested revisions: unknown
Provenance: partial

## Scope and preserved evidence

Measurements were taken on October 22; this record was assembled October 23.
The system was r100_0160: D455 colour and aligned depth on Intel, OWLv2 on
Thor, and result visualization on Intel. Thor's configuration selected
`google/owlv2-base-patch16-ensemble`, CUDA, `full body person`, threshold 0.26,
maximum detector cadence 10 Hz, stereoscopic depth, box masks, and projective,
euclidean and polar estimators. The camera operated at VGA / approximately
30 Hz. Operator confirmation preceded the temporary diagnostic restart:
stationary robot and autonomous motion stopped.

The exact dirty state and dependency/model versions of both installed hosts
were not snapshotted before testing. No exact tested commit is asserted.
The local documentation/source base at record preparation was
`0bce8919e471fce94bce1a2242619acae682af19`; this is context, not a tested
revision. The detector queue modification was subsequently built and its live
DDS endpoint inspected, but no full host patch/config snapshot was preserved.

Preserved evidence under [assets](assets/2026-10-22-perception-frame-timing/):

- [Warm diagnostic log](assets/2026-10-22-perception-frame-timing/thor.log).
- [Diagnostic Thor frame metadata](assets/2026-10-22-perception-frame-timing/thor-frames.json).
- [Baseline matched-frame summary](assets/2026-10-22-perception-frame-timing/baseline-summary.json).
- [Baseline Intel metadata](assets/2026-10-22-perception-frame-timing/baseline-intel.json) and [Thor metadata](assets/2026-10-22-perception-frame-timing/baseline-thor.json).
- [Depth-one Intel metadata](assets/2026-10-22-perception-frame-timing/depth1-intel.json).

Metadata contains host/stage, original header stamp, callback wall time, DDS
source/publication time and RMW take time. Raw subscriptions parsed serialized
header stamps without decoding pixels; later captures also decoded the small
detection/measurement messages to count detections. RMW take time is not network
packet arrival. Probe callbacks are independent observers, not timestamps from
inside the production detector's callback. Clocks were PTP synchronized; nearby
Thor logs reported master-offset maxima of about 0.05 ms, without an independent
clock-error bound for every sample. The temporary probe and launch wrapper were
not retained, so these data do not constitute a fully reproducible benchmark.

## Frame age at observable boundaries

A 25-second baseline capture matched 73 source-frame stamps across both hosts.
Values below are milliseconds from the camera header timestamp. The header
does not independently establish the instant of physical exposure.

| Boundary | Median | 95th percentile |
|---|---:|---:|
| Camera DDS publication | 18.464 | 21.121 |
| Thor colour observer callback | 35.570 | 38.314 |
| Detection DDS publication | 359.546 | 376.528 |
| Measurement DDS publication | 360.206 | 377.239 |
| Intel measurement observer callback | 361.066 | 377.598 |

The matched per-frame differences had medians of 16.904 ms for camera
publication to the Thor observer, 324.127 ms from that observer to detection
publication, 0.682 ms from detection to measurement publication, and 0.495 ms
from measurement publication to the Intel observer. The unusually small
measurement interval must not be interpreted as full target-estimator runtime:
this baseline did not record detection/status fields.

After the RGB subscription changed from sensor-data depth 5 to best-effort
keep-last depth 1, a separate 25-second Intel capture observed 82 detection and
82 measurement messages; all reported detections. Result statuses were recorded
but are not an independent accuracy or target-association validation.

| Boundary | Median age (ms) | Range (ms) | 95th percentile (ms) |
|---|---:|---:|---:|
| Detection receipt | 356.907 | 340.016–375.305 | 370.457 |
| Measurement receipt | 368.740 | 349.860–391.840 | 383.490 |
| Overlay receipt (100 publications) | 423.324 | 387.237–489.478 | 460.920 |

These are different scenes/populations, not a controlled before/after result.
There was no demonstrated latency reduction from the queue change. Deployed
overlay source inspection showed timestamp-keyed colour/depth caching with a
latest-frame fallback; the earlier statement in chat that it always rendered
the newest image was based on another checkout and was too broad. Overlay age
is not the additional delay of a particular measurement without confirming
the match, and neither overlay receipt nor publication measures RViz screen
presentation. Re-renders can produce multiple overlay messages per frame.

## Warm internal timing

A temporary launch set constructor-time `detector_debug` and
`depth_match_debug`. It stopped only Thor localization, warmed models, captured
30 seconds of metadata, and ended after roughly 55 seconds. The original
service was restored and reported active; its journal confirmed model load and
all three workers restarted. Debug CUDA barriers add overhead, so these figures
describe the instrumented run. Each statistic excludes its first five samples.

| Stage | Warm median (ms) | Warm p95 (ms) | Warm samples |
|---|---:|---:|---:|
| Detector cadence wait | 0.002 | 0.003 | 156 |
| Selected image pending-slot age | 17.734 | 32.412 | 156 |
| Image decode / preparation | 0.549 | 0.868 | 156 |
| Detector adapter inference call | 307.948 | 310.385 | 155 |
| Parse boxes | 0.014 | 0.035 | 155 |
| Publish detections | 0.207 | 0.236 | 155 |
| Total detector step | 308.889 | 311.736 | 155 |
| Measurement worker queue wait | 0.210 | 0.377 | 142 |
| Prepare mask region | 0.068 | 0.087 | 16 |
| Prepare stereoscopic depth input | 0.400 | 0.930 | 16 |
| Estimator reduction (selected paths together) | 12.515 | 14.166 | 16 |

The inference timer covers the detector adapter call, including model pipeline
pre/postprocessing and synchronization, not pure GPU kernels. The 0.400 ms
stereo-depth stage prepares an already-produced depth image; it does not measure
the D455 ASIC's stereo computation. The estimator timer does not separately
resolve projective, euclidean and LiDAR/polar costs.

The last diagnostic report had 160 detector steps, no detector failures,
3.24 Hz achieved cadence, and 1,336 superseded pending RGB frames out of 1,498
received. This is expected latest-frame selection, not inference queue growth.
The initial cold inference took 569.308 ms, versus about 308 ms warm.

The mask report had 147 completed batches, zero failed batches, 147 exact-depth
lookup hits and no misses; no pending detection replacements. Only 21 batches
reached the logged estimator stages, leaving 16 warm samples. The 30-second
observer window contained 97 results, only two with `detected=true`. Consequently
the aggregate mask-worker median of 0.248 ms and receipt-to-completion median
(`publication_age_ms`) of 0.561 ms mostly describe empty batches. They are not
representative target-estimation durations. `publication_age_ms` starts at
detection receipt, not camera acquisition.

In that observer window, median image age was 36.409 ms on Thor, 362.539 ms at
detection receipt and 363.994 ms at measurement receipt. Stage medians above
describe different sample sets; adding them is not a traced single frame.

## Decisions and limits

- The detector's RGB DDS subscription was changed to best-effort, volatile,
  keep-last depth 1. The worker already overwrote one pending image and selected
  it after cadence waiting; inference already in progress still finishes.
- Only the detector RGB subscription changed. Depth and scan subscriptions
  remained sensor-data depth 5; timestamp lookup histories retained 15 depth
  frames and 20 scans, with exact depth and nearest scan within 50 ms. Matching
  historical inputs is necessary while detection runs.
- A perception-only rebuild on Thor installed the changed executable. After
  operator restart, endpoint inspection confirmed best-effort keep-last depth 1
  and health `processing`, with input/output stamps advancing.
- The initial full-workspace dependency check failed because Thor lacked unrelated
  Clearpath/Gazebo repositories. Build dependencies for the selected localization
  package were verified from the installed underlay before rebuilding it.
- The temporary script first failed before service stop: ROS setup was incompatible
  with shell nounset. Its next preflight also caught that a systemd EnvironmentFile
  containing unquoted label spaces cannot be sourced as shell code. The corrected
  wrapper disabled nounset for setup and parsed each complete environment value.
- About 308 ms of detector adapter work is the main measured bottleneck; the
  10 Hz configured cadence did not impose a significant wait. No accelerated
  model, precision change, TensorRT conversion or accuracy comparison was tested.
- Physical exposure timing, exact wire arrival, per-estimator timing, and RViz
  screen presentation remain unmeasured. The captures establish neither a hard
  latency bound nor behavior under motion, different scenes or sustained load.
