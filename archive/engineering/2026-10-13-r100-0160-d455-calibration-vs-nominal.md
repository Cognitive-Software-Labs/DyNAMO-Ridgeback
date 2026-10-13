# r100_0160 D455 factory calibration against the nominal simulator profiles

Recorded dates: 2026-09-18, 2026-10-13
Tested revisions: `5751d96ba264f7456ddb3b3cbf44c9659d749708`
Provenance: partial

D455 serial 338522302134 on `r100_0160`. The 640×480 values were read live on
2026-10-13 from `dynamo-camera.service` (the repository camera service) with the
robot stationary. The service was not switched to 1280×720 for this check. The
1280×720 values therefore come from the 2026-09-18 cold-start captures of the
same device under the earlier vendor `realsense-camera` service, so the revision
does not apply to them. Raw evidence was not committed. On Intel it is under
`/home/robot/DyNAMO-Ridgeback-rviz/artifacts/hardware/20261013T095809Z-d455-calibration/`
(CameraInfo, extrinsics, TF and `comparison.json`), and the HD captures are under
`/home/robot/DyNAMO-Ridgeback/artifacts/hardware/20260918T181451-p2-coldstart-repeat/captures/`
(`coldstart_B1.json`, `coldstart_B2.json`).

## Intrinsics

| Profile | Nominal `(fx, fy, cx, cy)` | Live `(fx, fy, cx, cy)` | Focal | Principal point | HFOV nominal / live |
|---|---|---|---|---|---|
| 640×480 | 384, 384, 320, 240 | 387.926, 387.427, 322.202, 245.478 | +1.02 % / +0.89 % | +2.2 / +5.5 px | 79.61° / 79.04° |
| 1280×720 | 640, 640, 640, 360 | 647.071, 646.238, 643.671, 369.130 | +1.10 % / +0.97 % | +3.7 / +9.1 px | 90.00° / 89.37° |

Colour distortion at 640×480 is `plumb_bob`
`[-0.05547, 0.06546, 5.5e-05, 0.00105, -0.02142]`. `R` is the identity and `P`
equals `K`; the aligned-depth CameraInfo carries the same `K` and grid. The
VGA focal length is 0.6 times the HD one (388.24 against 387.93). That fits the
reference's description of VGA as a centred 4:3 crop of the 1280×800 sensor,
scaled down. Two HD cold starts differed by 0.03 % in focal length (B1 647.071,
B2 647.246) with the same principal point; the cause is not known.

## Extrinsics

Live TF, full precision: `base_link → camera_0_link` is
`[0.28035, 0.0475, 1.0345]`, equal to the nominal description. The factory
colour frame sits at `[0.28009, −0.01161, 1.03456]`, against the nominal
`[0.28035, −0.0115, 1.0345]`, rotated 0.057° / −0.215° / 0.150° (roll / pitch /
yaw). The published depth-to-colour translation is −0.05911 m, against the
nominal 0.059 m.

## Distortion in the pipeline

`intrinsics_from_camera_info` reads `K` only, so the estimators treat the
unrectified colour image as an ideal pinhole. Using the live `D` at 640×480, the
error is zero at the centre and grows toward the edges:

| Position | Pixel shift | Bearing error | Lateral error at 2 m |
|---|---|---|---|
| u = 480 | +1.0 px | +0.13° | +0.5 cm |
| u = 560 (the 2026-10-13 person) | +2.7 px | +0.29° | +1.4 cm |
| u = 630, right edge | +3.7 px | +0.33° | +1.9 cm |

## Conclusion

Keep the nominal simulator profiles. They are a reproducible approximation, they
lie within about 1 % in focal length and 9 px in principal point of this unit,
and hardware runs already read the live `K`. Ignoring distortion costs at most a
few centimetres of lateral error at the image edge. That is small next to the
+5–9 % range offsets measured on 2026-10-12, and it does not explain them. This
was not done: a live HD read under `dynamo-camera`, depth-imager
intrinsics, or a range check that isolates the distortion term.
