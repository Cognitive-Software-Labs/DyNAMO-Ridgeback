# PHYSICAL — Intel–Thor sensor latency and transport tuning

Status: **tooling, clock sync and the raw VGA 30 Hz baseline done; the baseline meets the working targets; tuning candidates pending.** Owner: an agent at the robot, with the
operator for every `sudo` step. Governing plan:
[PHYSICAL — Intel–Thor deployment and transport qualification](PHYSICAL_intel_thor_deployment.md),
whose sections 2 (time) and 3 (transfer cost) this plan executes for the
sensor inputs. The [transport reference](../physical/intel_thor_transport.md)
owns the DDS contract; [robot-local deployment](../physical/robot_local_deployment.md)
owns the camera service this plan tunes.

Perception will run on Thor, so every colour frame, aligned-depth frame,
CameraInfo, scan and transform it uses crosses the Intel–Thor Ethernet link.
This plan measures where the time goes for the real D455 and LiDAR streams,
then tunes the transport. The perception environment on Thor is the next plan;
it starts from the configuration this one installs.

## Decisions (user, 2026-10-06)

| Decision | Choice |
|---|---|
| Clock sync | chrony on Intel keeps UTC from its internet servers, slewed at most 500 ppm; PTP carries Intel's wall clock to Thor (both NICs timestamp in hardware), replacing Thor's chrony (user, 2026-10-06, after chrony alone left 0.2–1.9 ms disagreement) |
| Frame rate | 30 Hz only; reduced-rate operation is not an option (user, 2026-10-06) |
| Where tuning lives | A camera-only DDS profile rendered by `camera_service`. The services profile `/etc/clearpath/cyclonedds.xml` is not changed |
| Tooling on Thor | Pushed to `origin` from Intel and pulled on Thor into a separate worktree; Thor's existing checkout stays untouched |
| Working targets, VGA at 30 Hz | Network + DDS p95 ≤ 20 ms; sensor stamp to Thor callback p95 ≤ 40 ms; loss ≤ 1 %; exact colour/depth pairing ≥ 99 %; LiDAR continuity unchanged |
| If raw VGA misses the targets | Choose between compression and moving the D455 to Thor using the measurements; neither is pre-selected |

These targets choose between transport options. They do not replace the
deployment plan's acceptance of the localization pipeline, and runs below
30 Hz are recorded as reduced-rate evidence, never as a pass of the 30 Hz check.

## Starting point (inspected 2026-10-06)

| Item | Intel `cpr-r100-0160` | Thor `nvidia-thor-r100-0160` |
|---|---|---|
| Platform | x86_64, i7-9700TE | aarch64, L4T R38.2.2, 14 cores, 122 GB |
| Robot Ethernet | `br0` 192.168.131.1 on `eno1`, e1000e, 1 Gb/s, PTP clock, MTU 1500 | `enP2p1s0` 192.168.131.51, 1 Gb/s, PTP clock, MTU 1500 |
| Segment | Thor and both Hokuyos share the switch behind `eno1` | — |
| Socket limits | `rmem_max` 16 MiB; `wmem_max` 212,992 | same |
| DDS | services profile on `br0`+`wlp3s0`, discovery-only multicast; the camera service inherits it | Ethernet-only `cyclonedds_thor.xml` through `dds_env.sh` |
| Image-transport plugins | compressed, compressedDepth, zstd, ffmpeg, theora | none |
| Repository | camera service from `feat/hardware-camera-contract` | `950aacb`, clean, no build, no perception environment |
| Time | `systemd-timesyncd`, internet NTP over Wi-Fi | same, a different server |
| Measured offset | — | Thor 2.01 ms ahead of Intel, bound ±0.18 ms |

The September synthetic benchmark measured a 13.9 ms median round trip for VGA
colour and depth on CycloneDDS; the real driver has not been measured on Thor.

Raw VGA colour (`rgb8`, 921,600 B) and aligned depth (`16UC1`, 614,400 B) at
30 Hz need 369 Mbit/s, 37 % of the link. One frame pair takes 12.3 ms on the
wire at 1 Gb/s, so no DDS setting can bring raw VGA below about 12 ms; only
fewer bytes or no network hop can. Raw HD needs 1.1 Gbit/s and does not fit.
The three scans need about 4 Mbit/s.

## What is measured

For every message received on Thor:

| Component | Definition | Clock |
|---|---|---|
| Driver | Intel RMW source timestamp − header stamp | Intel |
| Network + DDS | Thor RMW received timestamp − Intel source timestamp | cross-host |
| Executor | Thor callback − Thor received timestamp | Thor |
| Total | Thor callback − header stamp | cross-host |

Streams: colour, aligned depth, colour CameraInfo, the front, rear and merged
scans, and `/tf`. Each run also records rate, loss, exact colour/depth pairing,
link utilisation on both hosts, and CPU on both hosts.

Every run records the clock offset and its bound before and after. Cross-host
components are reported only when the bound is at most 0.2 ms; otherwise they
are marked unresolved and only same-clock components are kept. RealSense
header stamps use the driver's host-synchronized time; LiDAR stamps come from
the driver's receive-time estimate. Neither is corrected.

Runs use 10 s of warm-up, 60 s of measurement and three interleaved repeats.
Stop a run on any scan gap above 250 ms on Intel, a driver error, or a
`clearpath-sensors` restart. Measure only through the tool's own
subscriptions; do not run `ros2 topic hz` or `ros2 node list` loops on the
robot during runs (they coincided with the 2026-09-18 Hokuyo lockout).

## Implementation status (2026-10-06)

The timing mode, Thor defaults, independent LAN clock check, interleaved run
collector, chrony installer with rollback, and camera-only DDS/fps candidates
are implemented. Unit tests and the isolated two-process localhost timing
smoke pass. Installed robot services remain on the previously qualified VGA
30 Hz configuration; no tuning candidate has been adopted.

The [sensor timing reference](../physical/intel_thor_sensor_latency.md) owns
implemented report semantics and qualification commands. The tooling was pushed to `origin` and fetched into the separate Thor
worktree. A short guarded run passed the Intel and Thor camera/scan contracts;
it remains preflight evidence, with the full baseline pending. Chrony was installed on both hosts, but default 64-second Thor polling did
not hold the clocks within 0.2 ms during Intel's initial internet-clock slews.
The operator installed faster LAN polling on Thor. All six full baseline
observations passed their camera/scan contracts, but two of three Thor runs
had unresolved cross-host timing under a constant whole-run correction.
The collector now calibrates continuously using short adjacent intervals and
retains the same 0.2 ms uncertainty limit.

With that method (`artifacts/hardware/20261006-thor-latency-baseline-lan-calibration/`
on Intel, tooling `5f9467e`), all three Thor observations resolved the clock,
passed the camera/scan contracts and met every working target, with no frame
missing against the Intel observer. Step 4 is complete for the deployed raw
VGA 30 Hz configuration. Step 3 is met for measurement, since the offset is
known within about 0.08 ms and corrected, but not for agreement: Thor ran
0.20–0.24 ms ahead of Intel on average in those runs, just outside the
0.2 ms exit while `chronyc` on Thor reported microseconds (see the reference's
note on chrony's virtual clock). Find the residual's source before cross-host
TF lookups rely on agreement rather than correction.

The breakdown points the tuning: the network component is close to the wire
time of each frame, while the driver component (stamp to Intel publish) is the
largest and grows when Thor subscribes. That is consistent with publishing
blocking on Intel's 212,992-byte send buffer, which O1 addresses, so O1 runs
first. Since `camera_service` now restarts only the camera for camera-only
changes, candidates no longer restart the platform.

O1 and O4 were measured and not adopted, O2 was skipped and O3 withdrawn;
see [Intel–Thor camera transport candidates](../do_not_try_again/intel_thor_camera_transport.md).
The raw services profile stays installed.

Clock agreement varies with Intel's internet corrections: Thor was 0.20–0.24 ms
ahead at the baseline and 1.5–1.9 ms behind during O1, because chrony serves
Intel's estimate of true time while sensor stamps use Intel's actual wall
clock. Continuous calibration still resolved every run. Distributing Intel's
wall clock with PTP would make the clocks agree; that is the plan's listed
alternative to chrony. Compression, host
power/coalescing changes and physically moving the camera remain
measurement-dependent work; their outcomes are not presumed.

The O4 tooling (`thor_decoder`, the collector's transport, encoder and decoder
options, and NVENC settings in `d455.yaml`) remains for a future
bandwidth-limited case; the
[sensor timing reference](../physical/intel_thor_sensor_latency.md#compressed-transport-candidates)
owns its semantics.

## Steps

### 1. Timing mode in the contract check (agent, no robot change)

- `tools/camera_contract_check`: add `--timing`. Keep the raw, header-only image
  subscriptions; take the `MessageInfo` argument in each callback and record
  source and received timestamps with the callback time. Report the four
  components above as median, p95, p99 and maximum per stream. Add `/tf` and
  the scans to the timed streams.
- Add `--host thor`: the tool runs on Thor under
  `source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh thor`
  with `ROS_DOMAIN_ID=0`, applies the same contract criteria, and does not
  require the Intel-only defaults (`/etc/clearpath` files).
- Add `--clock-offset-ms` and `--clock-bound-ms` inputs, recorded in the report
  and used for the cross-host gating above.
- Unit tests in `src/ridgeback_autonomy/test/test_camera_contract_check.py`:
  component arithmetic, percentile reporting, the unresolved-clock rule, and
  the header parser with `MessageInfo` timestamps.

Exit: tests pass; an isolated two-process smoke on one host (domain 99,
localhost discovery) produces all components.

### 2. Tooling on Thor (agent; Thor's existing checkout untouched)

Code reaches Thor only through `origin`, so both hosts run a commit that
exists on the shared remote. From Intel, push the branch:
`git push -u origin feat/intel-thor-sensor-latency`. It carries the unmerged
camera-branch commits beneath it. On Thor, fetch and check it out in a separate
worktree:
`git -C ~/DyNAMO-Ridgeback fetch origin feat/intel-thor-sensor-latency`, then
`git -C ~/DyNAMO-Ridgeback worktree add --track -b feat/intel-thor-sensor-latency ~/DyNAMO-Ridgeback-latency origin/feat/intel-thor-sensor-latency`.
Later changes follow the same path: commit and push on Intel, then
`git -C ~/DyNAMO-Ridgeback-latency pull --ff-only` on Thor. Never edit code on
Thor. The tool runs from that worktree without a build. Record the commit hash
both hosts ran with each measurement.

Exit: on Thor, under `dds_env.sh thor` and domain 0, the tool sees the three
camera topics and the scans and passes its contract criteria in a short run.

### 3. Clock synchronization (operator `sudo` on both hosts; agent writes the files)

- Track `src/ridgeback_autonomy_hardware/config/intel_thor/chrony_intel.conf`
  (keeps the existing internet upstreams, adds `allow 192.168.131.0/24` and
  `local stratum 10`) and `chrony_thor.conf` (`server 192.168.131.1 iburst
  prefer`, no other sources, `makestep 1 3`).
- Add `tools/intel_thor/time_sync` with `status`, `apply` and `rollback`,
  following `intel_services_rmw`: install `chrony` (replacing
  `systemd-timesyncd`), place the host's file in `/etc/chrony/conf.d/`, snapshot
  what it replaces, and restore `systemd-timesyncd` on rollback. It acts on the
  local host; run it on each.
- Record both hosts' time sync in robot-local deployment and Thor's in the
  transport reference.

Exit: `chronyc tracking` on Thor reports an offset within 0.2 ms of Intel, and
an independent direct-LAN round-trip check agrees within its own bound.

The chrony stage was installed and measured: the clocks agreed only to within
Intel's pending internet correction, because chrony serves an estimate of true
time while sensor stamps use Intel's wall clock. PTP replaces Thor's chrony:

- `chrony_intel.conf` adds `maxslewrate 500`, so a 1 ms internet correction
  takes 2 s instead of tens of milliseconds; re-apply `time_sync` on Intel.
- `config/intel_thor/ptp/` holds both hosts' `ptp4l` files and the
  `dynamo-ptp4l`/`dynamo-phc2sys` unit templates. Intel's `phc2sys` copies its
  wall clock into `eno1`'s hardware clock and `ptp4l` serves it over raw
  Ethernet (Intel's address is on `br0`, not the port). Thor's `ptp4l` follows
  on `enP2p1s0` and its `phc2sys` sets the wall clock, which may step once by
  the current offset. Both hardware clocks hold UTC (`-O 0`).
- `tools/intel_thor/ptp_sync` has `status`, `apply --host intel|thor` and
  `rollback`, following `time_sync`: it snapshots the package choice and
  chrony's state, refuses on Intel until the slow slew rate is installed,
  stops chrony on Thor and restores it on rollback, takes the robot lock, and
  fails if either daemon does not stay running.

Order: `time_sync apply --host intel`, `ptp_sync apply --host intel`, then
`ptp_sync apply --host thor` from Thor's worktree. Exit: `check_clock` from
Intel bounds the wall-clock offset within 0.2 ms and keeps it there across an
Intel internet correction.

**Met on 2026-10-06** (`artifacts/hardware/20261006T145000Z-ptp-verification/`,
tooling `de29c63`): the operator installed PTP on both hosts; Intel's `ptp4l`
serves and Thor's follows, and Thor's chrony is disabled. Eighteen direct-LAN
samples over three minutes measured Intel–Thor wall-clock offsets of −38 to
+1 µs, each with a 52–130 µs bound, through two Intel internet corrections of
+1.59 ms and −1.10 ms that slewed out over tens of seconds. Under chrony alone
the offset had moved between +0.2 and −1.9 ms.

### 4. Baseline (agent; deployed configuration unchanged)

- Same-host run on Intel with the robot's environment
  (`source /etc/clearpath/setup.bash`), which isolates driver time.
- Thor run at the installed VGA 30 Hz profile.

Exit: the four-component breakdown for every stream, with loss, pairing, link
load and CPU, under `artifacts/hardware/<UTC>-thor-latency-baseline/`.

### 5. Tuning, one option at a time (agent; operator applies service changes)

Each option is installed through `camera_service apply`, measured exactly like
the baseline, and recorded even when it loses.

| Option | Change | Where |
|---|---|---|
| O1 | Camera-only DDS profile: `br0` only, discovery-only multicast, `SocketSendBufferSize` 16 MiB, plus Intel `wmem_max` 16 MiB | New template `config/cyclonedds_camera.xml`, rendered by `camera_service`; the unit sets `CYCLONEDDS_URI` after sourcing `/etc/clearpath/setup.bash`; `wmem_max` added to the tracked sysctl file. **Measured, not adopted** |
| O2 | CycloneDDS `MaxMessageSize` 65500 against the default | Same profile, so it includes O1's send buffer. **Skipped** (user, 2026-10-06) |
| O3 | 15 frames/s against 30 | **Withdrawn**: reduced rate is not an option |
| O4 | Colour H.264 through `ffmpeg_image_transport` (NVENC on the RTX 5060) or JPEG; depth `zstd` or RVL, lossless only | Driver parameters for the encoder; on Thor, install the matching plugins (operator `sudo apt`) and a republisher that restores raw `Image` topics under the contract names. **Measured, not adopted** |
| O6 | Driver pipeline on Intel (the largest component): measure camera+USB against Intel processing per frame, then cut Intel processing | Contract check records the driver's per-frame metadata. **Measured**: performance policy adopted; publish order left as an open decision. See O6 below |
| O5 | Thor power mode and CPU governor; NIC interrupt coalescing on both hosts | **Decided** (user, 2026-10-06): both hosts always at the performance governor and Thor in MAXN, enforced by `host_performance` and `check_link`; NIC tuning not pursued (at most ~1 ms above wire time). Original: | Recorded host settings; restored afterwards unless adopted |

**O6 driver pipeline, measured.** The Intel observer splits each frame's
driver time at librealsense's host arrival; the
[timing reference](../physical/intel_thor_sensor_latency.md#measurement-contract)
owns the arithmetic. The performance policy (both hosts at the performance
governor, Thor in MAXN, installed by `host_performance`) is adopted; its
measured effect and the full split are in the
[archived record](../../archive/engineering/2026-10-06-r100-0160-thor-transport-candidates.md#o6-driver-pipeline-and-performance-policy).
With it, all three Thor observations met every target: colour 28 / 31 ms and
depth 20 / 23 ms total (median / p95).

What remains is mostly fixed by the hardware. Colour arrives on Intel about
12 ms after its stamp (sensor readout and USB), and depth waits for it. Intel
then needs about 3 ms. The driver publishes aligned depth before colour, and
with Thor reading, depth's send occupies the 1 Gb/s link for about 4.6 ms
before colour is published. Turning off the driver's pairing (`enable_sync`)
cannot help, since colour already arrives last and alignment needs it; see
[do not try again](../do_not_try_again/intel_thor_camera_transport.md).

Open decision: publishing colour before depth would let a detector start
about 4.6 ms sooner while depth follows during inference. A consumer that
needs both frames gains nothing, because both still cross the link in turn.
This needs a patched driver (the apt `realsense2_camera` fixes the order), so
it waits until the perception pipeline's consumption order is known.

No lossy colour option is adopted until the detector's output on Thor has been
compared with raw input on the same frames; that comparison belongs to the
perception-environment plan.

### 6. Choose and install (operator and agent)

Pick the cheapest configuration that meets the working targets at 30 Hz. Make
it the `camera_service` default, re-run the contract check on Intel and the
timing run on Thor, write an archive record, and update the transport
reference's configuration contract and known limits.

### 7. Camera on Thor (only if step 6 finds no configuration that meets the targets)

Assess the D455 on Thor before any physical change: `librealsense2` and
`realsense2_camera` for aarch64 (USB backend on JetPack 7 unverified), the USB
route from the mast, and the camera service ported to Thor. Present the
evidence and a matched comparison plan, as the governing plan requires.

## Risks

| Risk | Handling |
|---|---|
| Camera load on the port the Hokuyos share | Full duplex (camera out, scans in), but every run watches scans and stops on a gap |
| Time-service change on both hosts | Tracked installer with rollback; offset checked before and after |
| The other agent's DDS work | The services profile is not touched; only the camera's own profile changes |
| Thor shared with another project | Record Thor CPU and GPU load before each run |
| Lossy colour changing detections | Not adopted without the detector comparison |
| Unitree DDS traffic on domain 0 | Seen by Intel over Wi-Fi on 2026-10-06; Thor is Ethernet-only; record it if it appears in Thor's graph |

## Commits

1. `feat(tools): measure cross-host sensor latency in the camera contract check`
2. `feat(physical): synchronize Thor to Intel with chrony`
3. `feat(hardware): give the camera service its own DDS profile`
4. `feat(hardware): apply the chosen camera transport settings`
5. `docs(physical): record Intel–Thor sensor latency and the chosen transport`
