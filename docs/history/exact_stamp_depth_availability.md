# Exact-stamp depth availability investigation

2026-09-01. This investigation traced intermittent `NO_DEPTH_FRAME` results to
ROS image delivery under Fast DDS and selected CycloneDDS as the repository's
quick-start default. Exact integer-stamp matching, buffer depth, estimator
recipes, gates, scenario bytes, and miss accounting were not changed.

## Result

The fresh Fast DDS baseline lost 121 of 703 requested depth observations for
each depth estimator. The matched three-repeat CycloneDDS run delivered all 706
requested observations to both estimators. Scoring and every reported accuracy
statistic were unchanged.

| Metric | Fast DDS pre | CycloneDDS post |
|---|---:|---:|
| Trials / instances | 15 / 24 | 15 / 24 |
| Scored instances, each estimator | 18 / 24 | 18 / 24 |
| Detected-box observations, each estimator | 703 | 706 |
| `OK`, each estimator | 582 | 706 |
| `NO_DEPTH_FRAME`, each estimator | 121 | 0 |
| Coverage, each estimator | 82.788% | 100.000% |
| Matched launch wall duration | 243 s | 243 s |

| Estimator | Metric | Fast DDS pre | CycloneDDS post |
|---|---|---:|---:|
| `projective_ranging` | MAE / median / P95 (m) | 0.060235 / 0.059884 / 0.073259 | 0.060235 / 0.059884 / 0.073259 |
| `euclidean_reconstruction` | MAE / median / P95 (m) | 0.065023 / 0.059079 / 0.093979 | 0.065023 / 0.059079 / 0.093979 |

The three-observation denominator difference is detector cadence, not hidden
depth loss: the post histogram contains only `OK`. Both runs reported six
detector misses and no gate or estimator-value miss. The result therefore fixes
input availability without changing the estimates produced when input exists.

Temporary artifacts are under:

- Fast DDS pre: `/tmp/dynamo-exact-depth-current-5LwPlp/pre-host/benchmark`
- CycloneDDS post: `/tmp/dynamo-exact-depth-current-5LwPlp/post-cyclone-3x/benchmark`

Both use commit `df744b2`, branch `g1-distance-benchmarks`, the same scenario
bytes (`sha256:6200a4d4ea2a94a121978f5e028dce74dfcd8c786852db8f63a6bc81f1f943c6`),
five scenes, eight instances, three repeats, 2 s settle, 10 s capture,
`mask_gate:=box`, `depth_source:=stereoscopic`, and exactly
`projective_ranging,euclidean_reconstruction`.

## Where the frames disappeared

Default-off, bounded diagnostics recorded exact stamps and monotonic receipt,
lookup, worker, and lock timings. They retain at most 256 stamp-only miss records
and no image payloads.

The clean Fast DDS trace recorded 5,088 depth callbacks and 881 lookups: 770 hits
and 111 misses. Of those misses, 96 targets fell inside the buffered stamp range,
14 were newer, one was older, and none saw an empty buffer. No missed exact stamp
arrived later. There were no duplicate or out-of-order depth callbacks, no
pending detection replacement, mean detection dequeue age was 0.266 ms, mean
worker time was 2.972 ms (21.446 ms max), and lock waits/holds were sub-millisecond
apart from a 1.661 ms maximum depth hold. The maximum gap between depth callbacks
was 629 ms.

Those counters reject late arrival, buffer eviction/order, pending-slot loss,
node lock contention, worker starvation, and executor starvation. The missing
stamps never reached `depth_callback`.

An external ROS depth subscriber then reported 1,174 DDS `A message was lost!!!`
events in about 95 seconds. Among sampled internal misses, it received two exact
stamps that the mask node did not. This proves the bridge emitted those frames
and DDS delivered them to one subscriber but not the other. The observer reduced
benchmark coverage to 54.3%, so its rates are evidence of transport pressure,
not a production baseline.

The source boundary could not be independently enumerated without contamination:
Gazebo partition isolation did not separate another same-named world already
running on the shared host. That does not weaken the subscriber-specific proof,
but the contaminated GZ stamp probe is excluded from the result.

## Hypotheses tested

| Hypothesis / axis | Predicted counter | Observation | Decision |
|---|---|---|---|
| Depth subscription history 5 → 30 | Fewer internal unresolved misses | 25.2% → 27.8% internal miss rate | Reject; reverted |
| Fast DDS UDP-only profile | Fewer transport losses if SHM is responsible | 57.6% internal miss rate, 39.4% coverage | Reject for image delivery |
| CycloneDDS | Fewer transport losses | 0 `NO_DEPTH_FRAME`, 100% coverage in one-repeat and matched three-repeat runs | Select |
| Late exact arrival / retry | Miss stamps appear later | 0 of 111 | Reject |
| Buffer ordering / depth | Duplicates, out-of-order delivery, old-target misses | 0 / 0 / 1 | Reject |
| Detection replacement | Pending detection batches replaced | 0 of 881 | Reject |
| Lock / worker / executor starvation | Long waits, holds, or worker times correlate with misses | Sub-ms lock timings; 2.972 ms mean worker | Reject |

Queue and transport experiment details are retained in
[exact-stamp depth delivery](../do_not_try_again/exact_stamp_depth_delivery.md).

## Runtime and RTF limits

The matched launch transcripts both span 243 seconds, so the selected transport
did not increase end-to-end benchmark wall duration. The diagnostic Fast DDS run
measured 2.972 ms mean mask-worker latency; diagnostics were disabled in the
matched A/B to avoid observer effect, so there is no matched per-frame Cyclone
latency number.

Fast DDS baseline RTF sampled at 0.927947. A post-run Cyclone RTF sample was not
captured before shutdown. A supplemental attempt was rejected: the Gazebo stats
stream alternated between the task world and another same-named world, then the
task controller discovery failed. No Cyclone RTF is claimed from that run. This
is the precise remaining measurement blocker on the shared host; use an idle
host or a proven Gazebo partition before repeating it.

One matched Cyclone shutdown raised a post-benchmark rclpy message-conversion
exception while subscriptions were being torn down. It occurred after `run.json`
was complete and the launch command still exited zero. A separate one-repeat
Cyclone trial logged 98 controller `No clock received` fallbacks; the matched
three-repeat run logged none. Neither symptom affected the matched results, but
both should be rechecked on the Isaac Sim 6/Cyclone stack rather than generalized
from Gazebo.

## Implemented boundary

`start_exploration.sh` now defaults an unset `RMW_IMPLEMENTATION` to
`rmw_cyclonedds_cpp`. An explicit caller choice remains authoritative. The
legacy UDP-only FastDDS toggle and XML profile were removed after the experiment
showed substantially worse image delivery. `manual_mapping.launch.py` no longer
forces that profile, and cleanup/diagnostic scripts no longer carry its special
handling. `ridgeback_autonomy/package.xml` declares the Cyclone RMW runtime
dependency.

The diagnostics remain behind the existing `depth_match_debug` parameter,
default `false`, so normal processing has no stamp recording or timing work.

## Scope and limitations

- No tolerance, nearest-frame fallback, retry, or status relabeling was added.
- No estimator, gate, geometry, detector, scenario, or scoring change was made.
- The host was shared with user-owned simulator and visualization processes;
  they were preserved. This is not an idle-host benchmark.
- Isaac Sim 6 was not installed or launched by this investigation. CycloneDDS is
  now the selected transport, but Isaac camera QoS, `/clock`, control, and clean
  shutdown still need migration validation.
