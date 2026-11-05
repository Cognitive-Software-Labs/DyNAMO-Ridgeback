# r100_0160 Thor image copies and 1280×720 camera trials

Recorded dates: 2026-10-16
Tested revisions: unknown
Provenance: partial

Two linked measurements of camera bandwidth on the 1 Gb/s robot Ethernet
between Intel host `cpr-r100-0160` and Thor: the Intel→Thor load before and
after `localization_health` stopped subscribing to the images, and the S2
qualification of the 1280×720 camera profile. The text was extracted from the
[Intel–Thor transport reference](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/physical/intel_thor_transport.md#topology)
and the
[camera deployment reference](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/10dc4940c8663619de1a88c2aa21e1ab5715b2e7/docs/physical/robot_local_deployment.md)
at a later documentation snapshot.

Provenance gaps: the source text names no commit for the installed camera
service, Thor's localization checkout or the reader tool. The health change is
[commit `f3df2cddbeaf5acb4d935503d8a25415b0b93473`](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/commit/f3df2cddbeaf5acb4d935503d8a25415b0b93473),
committed 2026-10-16 09:06 UTC, but the text does not record which revision
Thor ran for either measurement. The first 1280×720 trial and the 640×480
restore carry no clock time, so their order relative to the timed diagnosis
and reader matrix is not established. Reports live only on the Intel host
under `artifacts/hardware/` and were not committed.

## Thor image copies

CycloneDDS sends each subscribing process its own unicast copy. Until this
date `localization_health` on Thor subscribed to the colour and depth images
only to read their stamps, beside the detector (colour) and the mask node
(depth). The transport reference attributed the 2026-10-13 packet captures'
destination ports 7411, 7413 and 7415 (see the
[local reader screens](2026-10-13-r100-0160-local-camera-reader-screens.md)) to
three Thor participants carrying colour + depth, colour, and depth: about
754 Mbit/s in total.

Health was changed to watch each stream's `camera_info`, which carries the same
stamps. Under the same Thor launch, Intel→Thor traffic was 763 Mbit/s before
and 382.6 Mbit/s steady after, with Thor receiving 377 Mbit/s. A header capture
after the change showed one copy each of colour (226 Mbit/s) and depth
(151 Mbit/s). Readings near 860 Mbit/s taken between those two measurements
were made while the camera was temporarily at 1280×720 for the trials below,
so they are not 640×480 evidence.

## First 1280×720 trial

The operator selected 1280×720 at 30 FPS with the maintained camera installer.
The robot declaration did not change, so only the camera service restarted.
A guarded five-second screen after three seconds of warm-up delivered 22.40 Hz
for both images. A second guarded run after ten seconds of warm-up measured
colour at 22.58 Hz and depth at 22.08 Hz over 60 seconds; the largest depth
stamp gap was 400.3 ms and colour-to-depth exact stamp pairing was 97.79%.
Both failed the 29 Hz image gate; the front, rear and merged scans stayed near
39.9 Hz. The reports are `artifacts/hardware/20261016-s2-hd-short.json` and
`20261016-s2-hd-full.json`.

During diagnosis `eno1` transmitted about 865 Mbit/s with Thor localization
active. The camera journal confirmed that both 1280×720 sensor profiles opened
at 30 FPS over USB 3.2, but it does not count frames produced at the sensor.
Thor's localization launch was running from an abandoned interactive session
rather than a restartable service, so its active consumer was left untouched.
The original 640×480 profile was restored and passed a guarded 12-second
screen at 29.92/30.00 Hz colour/depth with all three scans near 39.92 Hz
(`20261016-s2-vga-restored.json`).

## Link diagnosis (09:55–10:00 UTC)

Raw 1280×720 colour (`rgb8`, 2,764,800 B) plus aligned depth (`16UC1`,
1,843,200 B) at 30 Hz is about 1,106 Mbit/s of payload for a single remote
copy, before DDS and Ethernet overhead. With the profile at 1280×720 and the
same guarded 60-second check
(`artifacts/hardware/20261016T095428Z-s2-hd-diagnosis/`):

| Run | Thor localization | Colour / depth | Largest depth gap | `eno1` TX |
|---|---|---|---|---|
| A | stopped | 29.92 / 29.92 Hz, 100% pairing | 66.9 ms | 0.3 Mbit/s |
| B | running (one subscriber per image) | 22.45 / 22.37 Hz | 300.5 ms | 856.6 Mbit/s |

856.6 / 1,106 × 30 Hz ≈ 23 Hz, the observed rate. The driver itself published
fewer frames while the link was saturated, so local Intel readers also dropped
to about 22 Hz whenever Thor pulled raw 1280×720; the LiDAR scans were
unaffected. The cause of the first trial's failure was therefore the 1 Gb/s
robot Ethernet, not the camera or Intel.

**Decision (user, 2026-10-16): Thor stays at 640×480.** The deployment profile
remained 640×480 raw at 30 Hz.

## Local reader matrix at 1280×720 (10:22–10:33 UTC)

With 1280×720 selected and Thor localization stopped, the nine-run procedure
used at 640×480 (`artifacts/hardware/20261016T101433Z-s2-hd-reader-matrix/`)
passed eight runs: 29.4–30.0 Hz colour and depth, 99.77–100% exact pairing, the
TF lookup, all three scans near 39.9 Hz, and no wired traffic rise. Run
`single_1` failed only its stamp-gap limit: one 166.8 ms gap (five frames) in
colour and depth together, against 150 ms. Other 1280×720 runs reached
100–133 ms, where 640×480 never exceeded 67 ms. The camera journal showed no
error and the profile stayed at 1280×720 throughout, so these were source-side
frame dropouts.

## Limits

- Local 1280×720 delivery was not qualified under the frozen criteria; the
  dropouts were not characterised in a longer capture.
- These trials measured no compressed or reduced-rate 1280×720 stream to Thor.
- The before/after copy measurement covers one Thor launch; the source text
  records no repeat and no run with additional Thor image subscribers.
