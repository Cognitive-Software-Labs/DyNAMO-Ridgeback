# Intel DDS local-reader screening and warmed follow-up

Recorded dates: 2026-10-05
Tested revisions: `2c866dcbdda5c8c8ddb274cce7f6c6356f95844a`
Provenance: partial

The checkout was dirty. The experiment preserves its patch, untracked harness
inputs, installed configuration, service activity, process environments,
middleware libraries and package versions under
`artifacts/hardware/20261005-intel-dds-screening/before/`. The complete external
vendor checkout and every remote DDS participant were not archived. Results
describe this recorded run, not a clean build of the base revision.

## Method and deployment

The operator confirmed the Ridgeback stationary with an operator present and
e-stop available. No navigation or base commands were started. Intel host
`cpr-r100-0160`, namespace `r100_0160`, domain 0 used CycloneDDS 0.10.5; the
scan-merger workers retained their installed Fast DDS override. The D455 used
the original native VGA30 startup configuration throughout this screening.
Alignment was not changed.

A used the original installed CycloneDDS profile. B added only
`<AllowMulticast>spdp</AllowMulticast>` to the services profile and corresponding
test-reader profile. The controller explicitly restarted the originally active
DDS services and verified their descendant settings. Each configuration began
with a sixty-second scan-only baseline. Five-second trials added color, native
depth, one pair, then two independent pairs. Sustained repeats, alignment and
the soak were excluded at the operator's request.

Scan monitoring began before reader startup. A scan gap above 250 ms triggered
reader termination and evidence preservation; sensor-only recovery preceded B.
Packet capture used 192-byte headers and bounded rotating captures on `eno1`
and `enp2s0`. Wi-Fi counters were also recorded; desktop reconnects and other
traffic prevent attributing their changes to camera readers alone.

## Observations

Both scan-only baselines received all three scans at approximately 40 Hz.
Baseline physical-port TX was 3.65/3.64 Mbit/s for A and 0.144/0.150 Mbit/s
for B (`eno1`/`enp2s0`).

| Trial | Received images | Scan continuity | Added physical TX, eno1/enp2s0 | Short safety result |
|---|---|---|---|---|
| A, one color reader | 3 images in an aborted 0.398-second reader window | Front gap reached 260 ms; observation recorded front/merged gaps of about 1.12 seconds and rear gap of 1.01 seconds before ending | +20.42/+20.44 Mbit/s over the aborted scan measurement window | Failed; remaining A combinations skipped |
| B, one color reader | 29.0 FPS over five seconds | About 40 Hz; maximum gap across scans 44.9 ms | +0.01/+0.01 Mbit/s | Passed |
| B, one native-depth reader | 29.0 FPS over five seconds | About 40 Hz; maximum gap 57.0 ms | Approximately 0.00/+0.01 Mbit/s | Passed |
| B, one color/depth pair | 28.2 FPS each over five seconds | About 40 Hz; maximum gap 46.2 ms | Approximately 0.00/0.00 Mbit/s | Passed |
| B, two independent pairs | 28.8 FPS each for reader 1; 29.2 FPS each for reader 2 | About 40 Hz; maximum gap 63.2 ms | +0.01/+0.01 Mbit/s | Passed |

A's failed color trial captured 5,885/5,854 IPv4 multicast packets directed to
`239.255.0.1`; B's two-pair trial captured 90/91. These are header-sample counts
over different capture durations, include unrelated traffic and do not identify
camera payloads by themselves. They support the observed traffic change but do
not establish its causal path to LiDAR interruption.

The A reader's calculated 7.54 FPS reflects only three received frames before
automatic termination. It does **not** reproduce the historical sustained
7.7 FPS measurement or establish a steady-state camera rate. Likewise this run
did not reproduce a sustained 92 Mbit/s physical-port load.

B's short trials included initial image receive gaps of 157–297 ms and rates
below 29 FPS in some combinations. Their safety pass means requested streams
arrived and no scan/driver/traffic stop condition occurred. It is not a pass of
the warmed camera-rate/gap acceptance gate. No image stamp duplicates or
regressions were recorded in the short B captures. Aligned stamp matching was
not tested.

## Restoration and limits

The run completed at 13:18:43 UTC after 4 minutes 43 seconds. Test readers and
captures exited. Original XML bytes and original service activity were restored,
effective descendant profiles were checked, and preserved deployment files and
legacy backup hashes were verified. All three scans passed final verification
at approximately 40 Hz. B was not retained.

Observed: one local color reader coincided with scan interruption and increased
physical-port traffic under A; all four B startup trials avoided that immediate
failure. Not reproduced in B: the scan/traffic stop condition during these brief
trials. Still untested: sustained native delivery, alignment, soak, LiDAR timeout
and reconnect behavior, repeatability, and multicast causality. No captured
Hokuyo timeout establishes the reconnect bug's cause.

The installed vendor webserver also owns VNC and its desktop processes. Its
restart terminated the two earlier foreground attempts. The final screening
used an independent systemd controller, which survived the expected VNC
disconnects and completed restoration. The host did not reboot.

The local evidence root is `artifacts/hardware/20261005-intel-dds-screening/`;
`report.md` and `SHA256SUMS` identify measurements, packet captures and saved
inputs. These generated artifacts are local evidence, not Git-tracked
attachments. The procedure used for this run is preserved with the untracked
inputs at `before/untracked/docs/physical/intel_dds_testing.md`.

## Warmed native follow-up and desktop separation

A separate run at `artifacts/hardware/20261005-intel-dds-focused/` tested B only,
after installing independent VNC and port-9005 proxy services. Its own `before/`
directory preserves the updated dirty source inputs and installed settings;
the base revision stayed the same. Desktop installation inputs, preserved
configuration hashes and restart verification are saved separately under
`artifacts/hardware/20261005-vnc-separation/`.

The installer caused one planned desktop reconnect. VNC then retained PID
308180/start tick 1795820, and its proxy retained PID 309131/start tick 1796130,
across a deliberate webserver restart, B installation and original-DDS
restoration. The services remained active independently of the webserver.
Password, display :1, resolution and existing proxy ports were retained. Boot
behavior was not tested.

The follow-up measured a sixty-second scan-only B baseline, then started two
independent color/native-depth reader pairs. After ten seconds of warm-up, all
four image subscriptions were measured for sixty seconds. Scan and driver
safety monitoring included subscriber startup and warm-up.

| Measurement | Reader 1 | Reader 2 |
|---|---|---|
| Color received rate | 29.9667 FPS | 29.9667 FPS |
| Native-depth received rate | 29.95 FPS | 29.95 FPS |
| Color maximum receive gap | 87.18 ms | 91.67 ms |
| Depth maximum receive gap | 106.69 ms | 130.84 ms |
| Color/depth maximum stamp gap | 33.49/66.80 ms | 33.49/66.80 ms |
| Image stamp duplicates/regressions | None | None |

All required acceptance gates passed. Measured front/rear/merged scan rates
were 39.92/39.95/39.92 Hz. Across continuous monitoring, maximum front/rear/
merged receive gaps were 55.0/51.4/68.6 ms. Physical-port TX was approximately
0.1445/0.1497 Mbit/s, slightly below its quiet baseline, with no incremental
load attributable to the added readers by that metric. Bounded captures
recorded 462/455 IPv4 multicast packets; these remain aggregate evidence,
not camera-payload classification or causal proof.

The focused run completed at 14:02:09 UTC. It restored the exact original DDS
XML, original camera settings and original vendor service activity; removed
all test readers/captures; and passed final three-scan verification. Independent
VNC services remained installed. No candidate DDS setting was retained.

This establishes one passing warmed native workload under B. The immediate
scan/traffic failure observed under A was not reproduced during this B capture.
Repeatability, the complete native matrix, aligned-depth matching, the soak,
actual perception-compute load and Hokuyo timeout/reconnect behavior remain
untested. It does not identify the failure mechanism under A.

## Publication timing follow-up and retained configuration

The later run at `artifacts/hardware/20261005-intel-image-timing/` added
Cyclone RMW source and take timestamps to the two independent readers. Every
analyzed measurement-window image pair had valid timestamp ordering and passed
the wall/monotonic continuity check. In installed RMW 2.2.3, source time is set
before `dds_write_ts`, while received time is set when taking the sample, rather
than at packet arrival. Raw events preserve those fields and callback entry time.

The largest color gap was 86.15/84.10 ms in the two readers. The corresponding
header stamps were 33.48 ms apart, but publication timestamps were 86.45 ms
apart. The later frame took approximately 1.73/1.72 ms from publication to take,
then 0.22/0.19 ms to callback. The long spacing was therefore already present at
the publication boundary. This does not separate USB acquisition, driver work,
scheduling, or a preceding blocking publication. Other received frames showed
publication-to-take delays up to 35.40 ms, which combine middleware write,
transport, queueing and executor scheduling.

Both readers recorded the same depth stamp gaps of 100.42, 66.68 and 66.83 ms.
Received frames alone cannot establish whether intermediate frames were never
published or were dropped afterward. No acquisition or complete publisher-call
trace was taken. This run passed all focused gates and restored the original
DDS settings; all 136 recorded checksums verified.

The operator then approved a focused repeat that would keep B if the capture
and final verification passed. That run is saved under
`artifacts/hardware/20261005-intel-dds-retain/`, with a separate snapshot of the
dirty inputs at the same base revision. Its report retains the distinction
between this focused workload and the untested full matrix, alignment and soak.

| Measurement | Reader 1 | Reader 2 |
|---|---|---|
| Color / depth received rate | 29.95 / 29.93 FPS | 29.97 / 29.95 FPS |
| Color / depth maximum receive gap | 55.68 / 63.16 ms | 58.73 / 60.61 ms |
| Color / depth maximum stamp gap | 33.49 / 66.70 ms | 33.49 / 66.70 ms |
| Duplicate or regressing image stamps | None | None |

Measured front/rear/merged scan rates were 39.90/39.93/39.90 Hz. Continuous scan
monitoring recorded maximum gaps of 41.68/38.86/56.18 ms. Physical-port TX was
0.1462/0.1508 Mbit/s, with increments of only 0.00118/0.00063 Mbit/s over its
quiet baseline. Bounded captures recorded 464/464 multicast packets, including
discovery and unrelated traffic.

The capture and final three-scan verification passed. The run completed with
`candidate-retained`, no owned reader/capture process groups, and installed XML
matching exactly the saved original plus the one `AllowMulticast=spdp` insertion.
Robot YAML, camera startup, service activity and the legacy backup were checked.
Independent VNC/proxy PIDs stayed 308180/309131. All 136 evidence checksums
verified. The saved `--recover` command remains an exact-original rollback.

Observed: three focused native B captures passed on this recorded Intel host;
the last explicitly retained B. Not reproduced in those captures: the earlier
scan/traffic stop condition. Still untested: full matrix repeatability, aligned
streams, soak, actual perception-compute load, distributed Thor traffic, motion,
boot behavior and Hokuyo timeout/reconnect qualification. No causal attribution
to multicast, USB, camera hardware or TCP recovery follows from these results.
