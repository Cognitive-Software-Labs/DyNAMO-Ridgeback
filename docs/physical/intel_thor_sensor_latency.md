# Intel–Thor sensor latency qualification

This reference owns the sensor timing tools and their report semantics.
The [transport reference](intel_thor_transport.md) owns DDS settings and the
[robot-local deployment](robot_local_deployment.md) owns camera/time-service
installation. The [active plan](../plans/PHYSICAL_intel_thor_sensor_latency.md)
owns remaining live tuning and adoption.

## Measurement contract

`tools/camera_contract_check --timing` retains serialized image subscriptions
and decodes only image headers. Callback entry captures wall time before
parsing. Colour, aligned depth, CameraInfo, three hardware scans and dynamic
TF record RMW source/received timestamps. TF statistics cover individual
transforms, including multiple stamps within a single TFMessage.

| Component | Calculation |
|---|---|
| Driver | Intel source timestamp minus header stamp |
| Network + DDS | received timestamp minus source timestamp minus Thor–Intel offset |
| Executor | callback entry minus received timestamp |
| Total | callback entry minus header stamp minus Thor–Intel offset |

Components report sample count, median, p95, p99 and maximum in milliseconds,
using linear interpolation. Missing/zero RMW timestamps leave the affected
components empty. Negative samples are retained to expose timestamp faults.
Header stamps remain unchanged; they are driver time estimates rather than
independently verified acquisition timestamps.

`--host thor` requires an explicit namespace and does not read Intel's
`/etc/clearpath` defaults. `--clock-offset-ms` is **Thor minus Intel**;
`--clock-bound-ms` must be finite and at most 0.2 ms for cross-host components.
Without a qualifying bound, driver and executor components remain available;
network and total components are unresolved. Intel timing uses one host clock.
The caller must select the matching DDS environment and deployment domain.

`--samples-output` preserves raw timestamps for later clock validation.
Reports include exact pairing in both directions, observed rate, an expected
cadence deficit estimate, and RMW publication-sequence loss when supported.
The installed Jazzy RMW may omit sequence numbers; cadence deficit does not
prove network-only packet loss. CPU busy percentage is host-wide; NIC byte
rates cover all traffic on each interface, without subtracting other workloads.

## Qualification collector

Run `tools/intel_thor/check_link` before deployment. The collector runs on Intel
with `/etc/clearpath/setup.bash` sourced and `ROS_DOMAIN_ID=0`. It requires
identical committed tooling on both hosts, with Thor using its separate latency
worktree. It runs Intel then Thor, three interleaved repeats by default, each
with 10 seconds of warm-up and 60 seconds of measurement. A simultaneous Intel
contract observer guards each Thor run; this adds a local camera subscriber,
which is recorded in provenance and must be kept constant for comparisons.

The collector owns the robot experiment lock. It stops the sequence on a
contract failure, a scan receipt gap over 250 ms on Intel, changed sensor/camera
PID or restart count, or a camera `[ERROR]` journal entry. The guard uses its
own subscriptions and system counters; it does not poll the ROS graph.

Before and after every observation, `check_clock` takes persistent SSH round
trips. Its midpoint estimates the offset and half the round-trip duration
bounds asymmetry. The collector expands uncertainty by half the observed
before/after offset drift and recomputes components from raw samples. If that
bound exceeds 0.2 ms, cross-host components remain unresolved. Endpoint checks
cannot exclude an intervening clock step; avoid service changes during runs.
Time-service status, host CPU/NIC counters, guard reports, logs, source revisions
and raw samples stay under `artifacts/hardware/<UTC>-thor-latency-baseline/`.

The report's `working_targets_passed` checks VGA at 30 Hz with at least 59
seconds of observed data, network + DDS p95 at most 20 ms, total p95 at most
40 ms, loss at most 1%, exact pairing at least 99%, and the camera/scan contract.
The one-second duration tolerance admits scheduling at the window boundary.
Short or 15 Hz runs are diagnostic evidence and cannot pass the 30 Hz targets.
No configuration is adopted automatically. Commands live in the
[README](../../README.md#intel-thor-sensor-latency).

## Limits and pending adoption

As inspected on 2026-10-06, the running camera remains on the previously
qualified VGA 30 Hz service profile. The new camera DDS profile is an opt-in
candidate; chrony installation and measured transport selection require the
operator. Raw/compressed comparisons, detector accuracy under lossy colour,
CPU/NIC tuning and a camera move to Thor remain in the active plan.
