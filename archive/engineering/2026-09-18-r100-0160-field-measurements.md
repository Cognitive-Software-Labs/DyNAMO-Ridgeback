# r100_0160 field mount readings and LiDAR box checks

Recorded dates: 2026-09-18
Tested revisions: `a01a4532a170fcb85df1a5aaffb8c28bf1601f96`
Provenance: partial

The revision is the repository checkout on the robot PC during the visit. The
checks exercised the deployed Clearpath/MyBotShop services, not repository
nodes, and used ad-hoc rclpy probes. The raw captures, probe scripts, parameter
dumps, TF inventory and checksums stay on the robot PC under the ignored
`artifacts/hardware/20260918T144317-stationary-validation/` and are not
reproduced here. The deployed `/etc/clearpath/robot.yaml` was later changed
(RMW switch, 2026-09-18 17:55 UTC) without touching the LiDAR entries.

## Scope

A stationary field visit on robot `r100_0160` (host `cpr-r100-0160`): tape
readings of the sensor mounts and gross range/side checks of both Hokuyo
UST LiDARs with an upright box. Nothing was driven. No robot configuration was
changed during the measurements.

## Mount readings

Tape precision is about ±1 cm. The people at the robot confirmed that the camera is centred and
level; photos were waived.

| Mark | Reading | Model/config comparison |
|---|---|---|
| A: floor → deck top | 30.6 cm | Isaac model deck top 0.280 m + wheel contact 0.026 m = 30.6 cm |
| B: deck top → camera housing bottom | 74 or 75 cm (two readings, 2026-09-10 and 2026-09-18) | Repo camera mount uses 74 cm; the difference is within tape precision |
| C: floor → LiDAR window | 25 cm, accepted for both units | Deployed laser origin ≈ 25.9 cm, repo/Isaac ≈ 25.3 cm; the reading cannot separate them |
| D: deck front edge → camera front face | 18 cm | Model ≈ 18 cm |
| E: LiDAR housing face → body edge | front 6.3 cm, rear 6.1 cm | Equal within reading precision: symmetric mounts |

## LiDAR configuration observed

Both units ran `urg_node` from `clearpath-sensors.service` (front `lidar2d_0`
at 192.168.131.21, rear `lidar2d_1` at 192.168.131.22). The deployed config
requests ±π, but both published ±2.356 rad (±135°) with 0.25° increments, 1081 beams,
0.02–60 m, a 25 ms scan period, and 40.0 Hz over a 60 s capture with no duplicate or
regressing stamps. Header stamps led the receive clock by about 2.4 ms on both
units. Deployed mounts (parent `chassis_link`): front `[0.3922, 0, 0.1856]`,
rear `[-0.4278, 0, 0.1856]` with yaw π; `lidar2d_N_link → lidar2d_N_laser` adds
0.0474 m in z.

## Box checks

Scan ranges are 5 s per-beam medians (typical spread 3 mm). Tape distances
run from the centre of the named body edge to the box's near face.

| Case | Tape | Scan | Result |
|---|---|---|---|
| Front 1 m | 1.00 m from the front edge | 1.065 m at 0°; box centre ~9 cm right (tape: 7–8 cm right) | Laser origin 6.5 cm inside the edge vs 7.0 cm configured |
| Front far | 2.40 m (a first reading of 2.60 m was a tape error) | 2.494 m at 0° | 2.494 − 0.070 = 2.424 m, 2.4 cm from the tape, within the UST's ±4 cm rating |
| Front-left | 1.5 m | +24…+36°; face centre 1.499 m from the edge at 31.6° | 0–2 cm; left side correct |
| Rear 1 m | 1.00 m from the rear edge | 1.085 m at 0° | Origin 8.5 cm inside the edge |
| Rear far | 2.15 m | 2.213 m at 0° | Origin 6.3 cm inside the edge |
| Left side, both units | none | Box face ≈ 0.61 m from the centreline, seen by both | See below |

Both LiDARs pass the gross mount/range/side check. Edge-referenced single-scanner
cases scatter by 2–3 cm (tape endpoints and per-unit range bias), so none of them
alone resolves a few-centimetre mounting question.

## Rear LiDAR position

The shared side target is independent of body edges. Using box points only,
the rear-scanner points align with the front-scanner points after a
**+3.5 cm** x shift under the deployed rear x = −0.4278 (median nearest-neighbour
distance 12.1 → 5.7 mm). With −0.3922 the residual is 5.6 mm without a shift.
The mean rear edge offset (7.4 cm) also matches −0.3922 (predicted 7.8 cm) better
than −0.4278 (predicted 4.3 cm). The E readings are symmetric.

Conclusion at this date: the deployed rear x is about 3.6 cm too far back and the
symmetric −0.3922 fits. The Clearpath sample `r100_dual_laser_hokuyo.yaml`
(clearpath_config 2.9.5) and the repository's `clearpath/robot.yaml` both use
±0.3922. Limit: a rear yaw error of about 1.7° would produce a similar side-target
shift at this range. The yaw-insensitive rear-axis cases also favour −0.3922.

Origin of −0.4278: it first appears in `/etc/clearpath/robot.yaml` on
2026-08-06 (an earlier backup lists only the front unit). The MyBotShop backup of
2026-08-10 is identical. The MyBotShop scan-merger configuration
(`params_ridgeback.yaml`, 2026-08-10) copies the same offset, and its live
parameter was −0.4278. Its comments say the offsets still needed live
calibration. No vendor handover note records a measurement.

## Not covered

The merged-scan comparison, a rendered scan overlay, camera checks,
camera–LiDAR alignment, IMU/odometry and command-chain inspection, and
target tests were outside this record.
