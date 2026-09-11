# Upstream dependency refresh

On 2026-09-11, all five external ROS repositories were advanced together from
their reproducibility pins to the then-current `jazzy` branch tips. The manifest
still records immutable commits; it does not track moving branch names. The
[dependency runbook](../project/dependencies.md) owns the current procedure;
this page preserves the evidence from this refresh.

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
  Nominal simulation extrinsics are now forwarded for D405 as well as the
  previously supported RealSense models; hardware still leaves calibrated
  internal frames to the driver.
- `slam_toolbox_tf_namespace.patch` retains the namespace-aware TF listener.

Two modifications found only in the ignored live dependency checkouts were not
carried forward:

- The extra `g1_distance_calibration` allowed-world entry was stale: no such
  world exists in this repository. The two existing worlds remain in the
  Gazebo patch.
- An extra `os.makedirs(...sensors...)` in `clearpath_common` duplicated parent
  directory creation already performed by the current generator and by
  `clearpath_config.write_yaml`; upstream also added a general missing-directory
  fix. Keeping a second owner would obscure failures rather than improve them.

## Reproducibility gate

`tools/check_dependencies` now verifies every checkout against `.repos`. It
accepts either a clean dependency or the exact maintained patch for that
dependency, rejects revision drift, untracked files, and additional tracked
changes, and can idempotently apply missing patches with `--apply`.

The checker was exercised from clean repositories in both directions: clean to
patched, patched to verified, reverse-check, and re-apply. A deliberately
missing patch failed the read-only gate, was restored by `--apply`, and passed
on the next run.

## Validation evidence

Initial validation used fresh temporary clones, separate build/install/log
directories, and the repository's packages overlaid into that workspace. The
primary ignored dependency checkouts were not used for that compatibility gate.

- All three patches applied cleanly and their resulting diffs exactly matched
  the recorded patch files.
- A 25-package `colcon build --symlink-install` completed. Two upstream packages
  emitted deprecation warnings for `tl_expected`; there were no build failures.
- `colcon test-result` reported 847 tests with zero errors, failures, or skips
  across `clearpath_gz` and the 47 registered `ridgeback_autonomy` test groups.
- The six camera-description checks passed, including simulation/hardware frame
  ownership, exactly one RGBD sensor, and Gazebo SDF conversion at the colour
  frame pose.
- All five public launch files loaded successfully with `--show-args`.
- An isolated `mock_hospital` headless run published the camera image topic and
  resolved `base_link` to `camera_0_color_optical_frame` on the namespaced TF
  streams. No detector model or GUI was started.

After those gates passed, the live ignored dependency checkouts were moved to
the same revisions and patched through `tools/check_dependencies --apply`. The
normal workspace build then completed all 25 packages. A second isolated
`mock_hospital` run exercised full autonomous exploration headlessly with
CycloneDDS and target localization enabled:

- the CUDA detector loaded, subscribed to the colour stream, and processed
  frames while advertising `detections/target/raw`;
- SLAM activated and Nav2 reported all managed nodes active;
- the frontier explorer selected four successive goals and Nav2 completed the
  first three before the bounded run ended;
- logged navigation poses advanced from approximately `(-0.01, 0.00)` through
  `(-3.66, -0.80)` and `(5.43, -3.04)` to `(10.25, -1.40)`;
- neither Gazebo nor RViz GUI was launched.

The run also reproduced existing load/shutdown noise: occasional stale velocity
commands and full SLAM message-filter queues while running, followed by
publisher-context exceptions in several Python nodes and a collision-monitor
exit during coordinated shutdown. Those messages occurred after the interrupt
or did not stop navigation; this migration does not claim to fix lifecycle
shutdown behavior.

These checks establish compatibility and simulator-frame continuity. They do
not make historical benchmark measurements comparable to new runs: benchmark
provenance must continue to record dependency revisions, and results from
different dependency sets must not be presented as a matched comparison.

## Rollback

The root commit immediately before this migration restores the old `.repos`
pins, patch files, and manual setup instructions as one compatibility unit. A
fresh `vcs import` at either manifest state is the clean rollback path. Do not
mix old patches with new pins or selectively roll back one dependency.
