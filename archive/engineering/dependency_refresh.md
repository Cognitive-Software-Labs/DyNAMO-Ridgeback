# Upstream dependency refresh

Recorded dates: 2026-09-11

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

On 2026-09-11, all five external ROS repositories were advanced together from
their reproducibility pins to the then-recorded `jazzy` branch tips. The manifest
still records immutable commits; it does not track moving branch names. This page preserves the transition and its bounded evidence.

## Revision transition

| Dependency | Previous pin | New pin | Upstream commits | Release at new pin |
|---|---|---|---:|---|
| `clearpath_simulator` | `25997cb` | `590a451` | 13 | 2.9.4 |
| `clearpath_common` | `9960354` | `33e4b31` | 44 | 2.9.16 |
| `clearpath_config` | `5c92caa` | `b2a64ba` | 26 | 2.9.7 |
| `clearpath_msgs` | `5d04171` | `797be60` | 18 | after 2.7.0 |
| `slam_toolbox` | `ec8f763` | `02afdde` | 4 | 2.8.5 |

The useful upstream changes include Gazebo teleop topic automapping and an IMU
filter node-name fix; controller-spawner and absent-joystick fixes; safer YAML
directory creation and dictionary merging; D405 and Hesai support; new device
and power-control messages; and SLAM lifecycle, RViz initial-pose, launch
substitution, and map-name validation fixes. Dependency update and CI-only
commits make up the rest. This list explains why the repositories moved; the
upstream histories remain the authority for exhaustive changelogs.

## Local-change reconciliation

The three maintained patches remain necessary and are recorded from clean
checkouts at the new pins:

- `clearpath_gz_customizations.patch` retains the two repository-owned worlds,
  resource/plugin paths, `SpawnG1`, and headless rendering while preserving the
  newer upstream `setup_path` forwarding.
- `clearpath_realsense_sim_frames.patch` was rebased over upstream D405 support.
  The refreshed simulation extrinsics were forwarded for D405 as well as the
  previously supported RealSense models; hardware still leaves calibrated
  internal frames to the driver.
- `slam_toolbox_tf_namespace.patch` retains the namespace-aware TF listener.

Two modifications found only in the ignored live dependency checkouts were not
carried forward:

- The extra `g1_distance_calibration` allowed-world entry was stale: no such
  world exists in this repository. The two existing worlds remain in the
  Gazebo patch.
- An extra `os.makedirs(...sensors...)` in `clearpath_common` duplicated parent
  directory creation already performed by the then-recorded generator and by
  `clearpath_config.write_yaml`; upstream also added a general missing-directory
  fix. Keeping a second owner would obscure failures rather than improve them.

## Compatibility evidence

Fresh temporary dependency clones and isolated build/install/log directories
verified exact application of the three maintained patches. A 25-package build
passed; camera-frame checks and five public launch argument loads passed.
An isolated headless hospital smoke published camera images and resolved the
color optical TF. No detector or GUI was started for that first gate.

After migration of the live dependency checkouts, a second headless hospital
run loaded the CUDA detector, activated SLAM/Nav2, and completed three of four
selected frontier goals before the bounded run ended. Recorded poses advanced
from about (-0.01, 0.00) to (10.25, -1.40). These were compatibility controls,
not matched performance comparisons across dependency sets.

The read-only dependency gate was tested against clean, patched, missing-patch,
and reapplied states. Existing stale-velocity/filter-queue messages and
shutdown publisher exceptions were observed; this refresh did not fix them.

## Provenance limitations

The table records upstream revisions, separate from the root repository's
tested state, which was not identified. The archive preserves the transition
and compatibility scope; operational refresh and rollback steps belong to the
maintained runbook. Exact raw-run paths were not supplied in the original note.
