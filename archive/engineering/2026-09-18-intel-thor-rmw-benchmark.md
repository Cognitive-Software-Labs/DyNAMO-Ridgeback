# Intel–Thor middleware benchmark

Recorded dates: 2026-09-18
Tested revisions: `a01a4532a170fcb85df1a5aaffb8c28bf1601f96`
Provenance: partial

## Scope and decision

Intel's robot services ran Fast DDS by Clearpath default, while the repository
selects CycloneDDS for simulation. This benchmark compared Fast DDS,
CycloneDDS, and a mixed pair (Fast DDS publisher on Intel, CycloneDDS subscriber
on Thor) across the Intel–Thor Ethernet link. Each variant used an
Ethernet-only interface restriction and 16 MiB receive buffers. Within the
link's capacity the variants tied. The user selected CycloneDDS for both hosts,
to match the simulator middleware. Fast DDS was marginally ahead, but that did
not decide the choice. The resulting contract is maintained in
`docs/physical/intel_thor_transport.md`.

Provenance is partial. The listed revision was checked out on Intel. The
benchmark driver, probe, and configurations were uncommitted working-tree files
when the run was made. The manifest in the run directory records
`dirty_tree: true`, package versions, and sysctl values. The committed Intel
CycloneDDS configuration differs from the tested one: the tested file set a
hard 16 MiB buffer minimum (`min="16MB"`) and an address-based
Ethernet-only interface. Raw results were written to
`artifacts/hardware/rmw_benchmark/20260918T170204Z/`, which is not tracked.

## Conditions

- Intel `cpr-r100-0160` (i7-9700TE): Fast DDS 2.14.6 / `rmw_fastrtps_cpp`
  8.4.4, CycloneDDS 0.10.5 / `rmw_cyclonedds_cpp` 2.2.3. Thor
  `nvidia-thor-r100-0160` (AGX Thor, 120 W mode): `rmw_fastrtps_cpp` 8.4.3,
  `rmw_cyclonedds_cpp` 2.2.4, installed that day. Both hosts had `rmem_max` and
  `rmem_default` at 16 MiB during the run.
- Synthetic frames from `dds_link_probe.py` (Python, best-effort, depth 5) at
  30 Hz: rgb8 colour, optionally with a same-sequence 16UC1 depth frame, and an
  acknowledgement per colour frame for round-trip timing. Measurement windows
  were 15 s, three repeats, interleaved by scenario and variant, in ROS domain
  88. The robot was stationary with its services running on Fast DDS in
  domain 0. The D455 service was stopped.

## Results

All variants delivered with 0% sequence loss, 100% exact-stamp colour/depth
pairing, no probe socket drops, and at most 2.7 Mbit/s of Wi-Fi traffic. The
table lists medians over three repeats. Repeats differed by 0.5 ms or less
except where noted.

| Scenario | Metric | Fast DDS | CycloneDDS | Mixed |
|---|---|---|---|---|
| 640×480 colour, Intel → Thor | rate / RTT p50 | 30.07 Hz / 11.0 ms | 30.07 Hz / 11.1 ms | 30.07 Hz / 10.7 ms |
| 640×480 colour + depth | rate / RTT p50 / p95 | 30.0 Hz / 13.1 / 13.8 ms | 30.0 Hz / 13.9 / 15.2 ms | 30.07 Hz / 13.1 / 13.9 ms |
| 640×480 colour + depth | publisher / subscriber CPU | 10.1% / 16.1% | 14.7% / 14.5% | 14.2% / 14.4% |
| 1280×720 colour | rate / RTT p50 | 30.07 Hz / 29.1 ms | 30.07 Hz / 29.6 ms | 30.07 Hz / 28.6 ms |
| 1280×720 colour + depth (about 1.1 Gbit/s) | rate / RTT p50 | 25.6 Hz / 1.08 s | 23.7 Hz / 1.36 s | 25.6 Hz / 11.3 s |
| 640×480 colour, Thor → Intel | rate / RTT p50 | 30.07 Hz / 11.4 ms | 30.07 Hz / 12.2 ms | — |

RTT is one colour frame's transfer plus a small reply. CPU is the Python
probe's process time as a percentage of one core, and it includes probe
overhead. In the over-capacity scenario the publishers fell behind the 30 Hz
schedule, which is why sequence loss stayed at 0 while the delivered rate
dropped. The frames that did arrive were stale.

## Interface follow-up

A later check tested an Intel configuration listing both `br0` and `wlp3s0`
against Ethernet-only Thor. Intel → Thor stayed on Ethernet, with 2.7 Mbit/s
of background Wi-Fi traffic. Thor → Intel carried 466 Mbit/s of Wi-Fi traffic
across both hosts, and the round trip rose to 20.9 ms. The Intel reader
advertised its Wi-Fi address and Thor sent there. This result set the rule
that Intel readers of Thor output use the Ethernet-only configuration. The
dual-interface configuration is kept for Intel robot services, which only
publish to Thor.

Two further checks used the committed configurations. A local Intel publisher
on the services configuration delivered 30.17 Hz to an Ethernet-only
subscriber, with a 3.3 ms median round trip. A subscriber on Thor pinned to its
Wi-Fi address, standing in for a lab laptop, received the services
configuration's stream at the published 5 Hz.

## Limits

Frames were synthetic and came from a Python client on both ends. Same-host
delivery and exact-stamp delivery from the real D455 driver were not measured,
and the robot services themselves had not been switched. A single session
cannot show day-to-day variation.
