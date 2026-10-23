# r100_0160 Thor perception environment and first live localization

Recorded dates: 2026-10-09, 2026-10-12
Tested revisions: `211ed1c116a36e853dfb87704aaa91fdb3dd745e`, `9a48e016f8a160dba75486fd71c41df2bbf2711c`, `33aaafc00a653aa916b784be576a1438544c220d`
Provenance: partial

Raw evidence stays on the hosts and was not committed: on Thor under
`~/DyNAMO-Ridgeback-perception/artifacts/hardware/` (`20261009T131223Z-thor-perception-env/`,
`20261009T131626Z-thor-model-probe/`, `20261012T104641Z-thor-model-probe-120w/`,
`20261012T110753Z-thor-localization-smoke/`) and on Intel under
`/home/robot/DyNAMO-Ridgeback-camera/artifacts/hardware/` (`20261012T132245Z-localization-smoke-intel-person-0.22/`
and the earlier attempts listed below). Thor's power was not logged during the
final run, and the simultaneous Intel scan guard was not run during it.

## Environment

Thor: JetPack 7.0 (L4T R38.2.2), CUDA 13.0, Python 3.12.3, `~/venvs/dynamo-perception`
(system site packages) from `requirements-perception-thor.txt`: `torch 2.11.0+cu130`
and `torchvision 0.26.0+cu130` with `sm_110` kernels, cuDNN 9.19 from the wheels,
`transformers 5.10.2`, `accelerate 1.13.0`. `python3.12-venv` was installed by the
operator, which also updated the system Python from `3.12.3-1ubuntu0.13` to
`0ubuntu0.17`. `tools/intel_thor/check_perception_env` passed. Checkpoint
snapshots: OWLv2 `cfd3195ba4ea`, SlimSAM `d67c795ed0ca`, Depth-Anything V2
small `8078d68a9c75`.

## Models on one 640×480 frame

Warm median / p95 over 20 runs, labels `person` and `humanoid robot`, through
the repository's model classes:

| Model | 120W (2026-10-12) | MAXN (2026-10-09) | Peak GPU memory |
|---|---|---|---|
| OWLv2 detector | 607 / 657 ms | 546 / 558 ms | 780 MB |
| SlimSAM, one box | 196 / 215 ms | 186 / 200 ms | 2296 MB |
| Depth-Anything V2 small | 57 / 59 ms | 55 / 56 ms | 1037 MB |

At 120W Thor drew 68 W median and 89 W at most at its input during that probe
(GPU rail at most 44 W, junction at most 64.8 °C). The localization unit tests
that do not import autonomy (27 files, 439 tests) passed on Thor in the venv
and on Intel. After the MAXN probe, a detector precision diagnostic was running
when Thor stopped responding; the Ridgeback USER POWER ports then read 0 V.
Thor kept no logs from that day, so the cause is unconfirmed; Thor runs in
120W since.

## Findings before the run

**Person score.** With `person`, OWLv2 scored the operator facing the camera
at 0.44 on a JPEG copy and 0.31 on raw frames, below the detector's default
threshold of 0.55, so no person was ever published. A skeleton poster in view
scored 0.16–0.28. Other phrasings on one raw frame: `man` 0.40 (poster 0.11),
`standing person` 0.37 (0.25), `a person` 0.22, `human` 0.24 for the poster
above the person. The run used `person` with the new launch argument
`detection_threshold:=0.22` (user decision; deviation from defaults).

**Static TF missing on Thor.** Camera frames and scans reached Thor, but
`tf_static`, `robot_description` and node names did not, so both estimators
reported `TF_MISS_EXTRINSIC` (status 4) with NaN distances. A once-published
transient-local message from Intel's services profile (br0 + Wi-Fi) never
reached Thor in domain 0, and arrived within 3 s with Wi-Fi removed from the
publisher's profile, with the Ethernet-only profile, or in an isolated domain.
Tracing showed Thor learning Intel's locators Wi-Fi first and Intel never
seeing Thor's reader. Thor and Intel cannot reach each other over the lab
Wi-Fi (`ASUS_20`, 100% ping loss both ways), so Thor's replies to
192.168.123.14 were lost. A route for that address over the cable fixed it
(`thor_lan_route`); see [troubleshooting as of this record](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/c0b5e6f5058e08dd457803bf5343bc48bbfcf979/docs/troubleshooting.md#thor-receives-sensor-streams-but-no-static-tf).

Restarted Thor localization processes took up to about 2 minutes to connect
to the camera before the route existed. A launch started in the background
ignores SIGINT; SIGTERM stops it.

## Live run (2026-10-12, 13:23–13:31 UTC)

Robot stationary and e-stopped. Thor: `localization.launch.py` with
`projective_ranging,euclidean_reconstruction`, `depth_source:=stereoscopic`,
`mask_gate:=box`, `target_labels:=person`, `detection_threshold:=0.22`, at
`9a48e01`. Intel: `tools/intel_thor/record_localization_run` under
`dds_env.sh intel`. Marks were taped from the deck's front edge (1.50, 2.00,
2.50 m); the camera front face is 0.18 m behind it, and reported forward
distances were converted to the camera face with the camera's base-frame X
(0.280 m) against the pipeline's 0.25 m front offset. Each segment is the 30 s
after the operator reported being in place, less 3 s to settle.

| Mark | Tape from camera | Projective | Euclidean | Error | Lateral | Frames with target |
|---|---|---|---|---|---|---|
| A | 1.68 m | 1.82 m | 1.81 m | +0.13 m (+8%) | −0.19 m | 99 / 99 |
| B | 2.18 m | 2.39 m | 2.38 m | +0.20 m (+9%) | −0.14 m | 96 / 99 |
| C | 2.68 m | 2.80 m | 2.81 m | +0.12 m (+5%) | +0.01 m | 99 / 99 |
| D | 2.18 m | 2.38 m | 2.38 m | +0.20 m (+9%) | −0.07 m | 90 / 99 |

Medians shown; the estimators agreed within about 1 cm and every error was
positive, which points to a common offset (the estimators see the body rather
than the toes on the tape is one candidate; not tested). Health reported
`processing` throughout; the detector ran at 3.3 Hz. Source image stamp to
result received on Intel: detections 365 / 380 ms, measurements 369 / 388 ms
(median / p95), dominated by detector inference.

Not achieved: the empty-scene segment had a person detected about 0.8 m ahead
for part of it (the operator leaving), and at threshold 0.22 the skeleton
poster was detected in 81 of 95 frames after the run (2.83 m, 2.14 m right),
a sustained false measurement. D was meant to be 0.6 m to the robot's left
but was measured on the centre line, and the retry segment had no person. The
camera contract check before (11:01) and after the run passed, with all three
scans at 39.9 Hz and gaps under 40 ms; no sensor service restarted.

Earlier attempts that day, kept for the findings above:
`20261012T111047Z-localization-smoke-intel` (threshold 0.55, no person
detected), `20261012T112546Z-localization-smoke-intel` (marks A and A2 at
0.55), `20261012T113530Z-localization-smoke-intel-threshold-0.40` (0.40 and
0.22 before the route; NaN distances), `20261012T124534Z-localization-smoke-intel-person-0.22`
(recorder stopped by a session restart before the run).

## Redo of the open checks (2026-10-12, 14:26–14:42 UTC)

Label selection first: 24 raw frames on Thor (6 each with the scene empty and
the operator at A, C and D, poster in view) were scored per phrasing; margin is
the operator's lowest score minus the empty scene's highest.

| Label | Person lowest | Empty highest | Margin |
|---|---|---|---|
| `person` | 0.296 | 0.254 | +0.04 |
| `standing person` | 0.311 | 0.261 | +0.05 |
| `a standing person` | 0.249 | 0.190 | +0.06 |
| `full body person` | 0.302 | 0.225 | +0.08 |
| `man` | 0.334 | 0.130 | +0.20 |
| `human`, `human body` | — | — | negative |

The user chose `full body person` at `detection_threshold:=0.26` (neutral;
`man` separated best but is not a general person label). Thor ran the launch
at `9a48e01` with power logged; Intel ran the recorder and, simultaneously,
`camera_contract_check --guard` for 600 s (`20261012T143120Z-localization-redo-intel/`).

- Empty scene: 0 of 98 detection messages had a target over 30 s; the poster
  was no longer detected. A second empty segment had 4 of 99, all the
  operator leaving at about 0.7 m.
- Person detected in every frame while in view, score about 0.33, 3.3 Hz,
  result 364–374 ms after the source stamp (median), health `processing`.
- Off-centre not captured: no segment had the operator still at the planned
  0.6 m left. Positions measured (Euclidean forward from the camera face,
  lateral): 1.76 m / +0.03 m, 2.13 m / −0.14 m, 2.42 m / +0.11 m,
  2.32 m / −0.08 m; projective agreed within 1 cm each time. Lateral values on
  both sides of the centre line were measured, but none against a taped
  off-centre mark. The camera's view of the room appeared shifted against the
  morning's frames (the poster nearer the image edge), so the robot or camera
  may have turned after the marks were taped; not checked.
- Scan guard: contract PASS over 600 s; front, rear and merged scans 39.9 Hz,
  largest gap 102 ms; colour and depth 29.96 Hz.
- Thor at 120W over the 684 s run: input 73 W median, 93 W at most, GPU rail
  at most 45 W, junction at most 70.0 °C.

## Limits

One operator, one indoor scene, stationary robot, box masks and stereo depth
only. The threshold of 0.22 was chosen against one frame's scores and admits
the poster. Distances were compared with a hand-taped reference; this is a
smoke, not an accuracy qualification.
