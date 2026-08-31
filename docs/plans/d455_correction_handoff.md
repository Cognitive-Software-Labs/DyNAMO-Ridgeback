# Handoff: correct D435 assumptions for the robot's D455

Status: plan only; implementation has not started. Prepared on 2026-08-31 against `f16afd0`. The user confirms the physical camera is a **D455**. This overrides earlier documentation that inferred the hardware model from a D435 configuration.

## Objective and scope

Correct the repository's camera selection, make simulated camera geometry internally consistent, and correct first-party documentation. Cover the shared exploration/benchmark stack. Treat deployed configuration and physical-hardware verification as separate rollout stages, not something a simulation test can prove.

This is a configuration/geometry correction, not a purely behavior-preserving refactor. A changed nominal camera origin can intentionally change simulated measurements. Estimator algorithms, tuning, synchronization, input contracts, and unrelated behavior must remain unchanged.

Read `AGENTS.md`, `AI_CONTEXT.md`, and the current Graphify report before editing. Use migration discipline: baseline, staged implementation, verification, and an explicit rollback path. Inspect root and nested dependency worktrees before modifying them. There is concurrent TF-fallback work in `common/tf_utils.py`, its tests, and CMake; preserve it and do not combine the investigations.

## 1. Established facts

| Layer | Current evidence | Implication |
|---|---|---|
| Hardware identity | User confirms D455; README already says D455 | Do not continue treating D435 as physical hardware truth |
| Repo configuration | [robot.yaml](/home/stefi/DyNAMO/DyNAMO-Ridgeback/clearpath/robot.yaml:41) explicitly selects `device_type: d435`; introduced by `9af681c` on April 9 | Correct the explicit selection, not upstream defaults |
| Clearpath support | [IntelRealsense configuration](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/clearpath_config/clearpath_config/sensors/types/cameras.py:236) supports D455 but defaults to D435 | Removing the override would not fix this |
| Model generation | [Camera description generator](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/clearpath_common/clearpath_generator_common/clearpath_generator_common/description/sensors.py:261) passes the resolved device type into the sensor model | The setting affects robot description as well as driver parameters |
| Simulation TF | [Optical TF include](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/launch/includes/camera_optical_tf.launch.py:20) publishes a D435-derived `+0.015 m` Y translation | Do not leave this behind when changing the camera model |
| D455 model | [D455 description](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/clearpath_common/clearpath_sensors_description/urdf/intel/d455.urdf.xacro:10) defines nominal depth-to-color Y offset `-0.059 m`, plus its own mount-to-link transform | The full frame chain matters, not just one replacement number |
| Rendered camera | [RealSense wrapper](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/clearpath_common/clearpath_sensors_description/urdf/intel_realsense.urdf.xacro:37) attaches an RGBD sensor to the camera link, but labels optical image frames separately | A TF name/offset change alone does not prove it matches the rendered viewpoint |
| Local setup copy | [/home/stefi/clearpath/robot.yaml](/home/stefi/clearpath/robot.yaml:33) omits device type and uses mount `[0, 0, 0.72]`; repo uses `[0.3, 0, 0.85]` | Do not overwrite the whole local file or assume either mount is verified on hardware |
| Pointcloud | Checked-out Clearpath has `POINTCLOUD_ENABLED = True`; the app's [RealSense input contract](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/common/camera_inputs.py:49) deliberately leaves the organized-cloud input unspecified | Driver defaults, actual topics, and app eligibility are different facts |

Reconfirm these facts at implementation time. Generated and installed files may differ from source.

The RealSense driver uses `device_type` as a device-name regular-expression filter; D435 is not a generic D400-family selector. See the [official driver parameter documentation](https://github.com/realsenseai/realsense-ros#parameters-that-cannot-be-changed-in-runtime). The D455 has an IMU, but that does not mean this stack enables or consumes it. See the [manufacturer's D455 specifications](https://www.realsenseai.com/products/real-sense-depth-camera-d455f/).

## 2. Decisions and boundaries

- Set the repository's target camera explicitly to `d455`. Keep `model: intel_realsense`, camera name, namespace, topics, and device-independent interfaces.
- Preserve unspecified stream profiles and current 640x480@30 Clearpath defaults. Do not reintroduce 1280 profiles.
- Preserve `align_depth.enable: true`, `enable_sync: true`, exact RGB/depth matching, and existing missing-frame behavior.
- Keep repository mount XYZ/RPY unchanged in this correction. D455's internal model offsets are distinct from the physical bracket pose. Do not invent a hardware measurement or compensate for geometry errors by changing the mount.
- Preserve the current simplified Gazebo optics: FoV, near/far clipping, resolution, and rate. D455 body/frame geometry plus an idealized RGBD renderer is the scoped target, not a calibrated D455 stereo/noise simulator. Document that distinction.
- Do not change depth gates, segmentation thresholds, model choices, estimator defaults, or pointcloud/IMU enablement to match product marketing specifications.
- Keep organized pointcloud input optional until its actual grid, frame, and ordering are verified. Correct claims about defaults without automatically wiring a new hardware input.
- Keep physical camera TF driver-owned. Any nominal internal camera TF must be simulation-only, with no duplicate publishers.
- No mass D435-to-D455 replacement across vendored code, historical measurements, datasheet quotations, or other-camera support.
- No automatic write to the home-directory setup copy, robot-side configuration, services, or firmware. Prepare a narrow deployment diff and obtain explicit authorization for that stage.

## 3. Implementation sequence

### A. Capture baseline and resolve configuration ownership

1. Record root HEAD, root/nested worktree changes, Clearpath dependency revisions, installed package paths, and the effective launch `setup_path`.
2. Run existing configuration, camera-contract, geometry, launch-layout, and full package tests before editing. Record actual results; historical counts are not the current baseline.
3. Make a throwaway setup directory with `mktemp -d` for generated files and simulation checks. Use the repository configuration there. Do not generate into the user's default setup directory during development.
4. Capture resolved driver parameters and generated D435 URDF/SDF before the change. Identify the mount, camera link, RGB render pose, optical frame names, and TF ownership. Save these as comparison artifacts outside production paths.
5. Inspect the default local setup copy read-only and report its differences. The known mount difference is not authorized to be resolved by copying the repo file over it.

### B. Correct model selection

1. Change only `ros_parameters.intel_realsense.device_type` to `d455` in the repository robot configuration.
2. Extend [test_robot_config.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/test/test_robot_config.py) to assert the explicit model and preservation of alignment, sync, enabled RGB/depth, profile omission, and mount values.
3. Instantiate the checked-out Clearpath parser and assert both `sensor.device_type` and emitted `intel_realsense.device_type` equal `d455`. Assert the description generator selects the D455 model. A YAML string assertion alone is insufficient.
4. Do not change Clearpath's global D435 default or device catalogue. Existing D435 support remains legitimate.
5. Check effective serial-number selection separately during hardware rollout. Do not assume the existing `serial_no: "0"` has the intended meaning in every wrapper version, and do not invent a replacement serial number.

### C. Give simulation one consistent source of camera geometry

Preferred implementation: use the selected model's nominal frames in simulation and let `robot_state_publisher` publish them. Remove reliance on the separately hardcoded D435 optical transform.

1. Use Clearpath's existing `is_sim` Xacro input: [description.launch.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/clearpath_common/clearpath_platform_description/launch/description.launch.py:47) already derives it from `use_sim_time`. There is no need for a new public camera-model or TF-offset parameter.
2. Add the narrow wrapper support needed to pass simulation-only nominal-extrinsics selection into the supported camera macro. Verify macro signatures; the D455 macro already exposes `use_nominal_extrinsics`. The hardware expansion must retain its false behavior. Keep existing D435 fixture coverage while removing the old helper dependency.
3. In simulation, place the single RGBD render sensor at the selected model's **color-camera pose**, consistent with the color optical frame used by its RGB and aligned-depth products. Prefer referencing the generated color frame over copying a nominal translation into a second file.
4. Inspect the expanded URDF and Gazebo-converted SDF, including fixed-joint reduction, sensor pose, axes, `gz_frame_id`, `optical_frame_id`, and emitted RGB/depth/cloud headers. Frame labels and rendered viewpoint must agree. Preserve the public topic names and coordinate conventions expected by consumers; changing a header string without transforming its data is not a fix.
5. Retire the redundant static publisher and its internal include/call sites only once the generated model supplies the complete chain. Update the launch-layout test's expected internal includes if the file is removed. Exploration, benchmark, and manual mapping must remain usable. Do not remove the helper while leaving an unpatched model with missing frames.
6. Prove hardware expansion contains no competing nominal internal camera joints or fallback publisher; calibrated internal TF must come from the physical driver. Mount-to-camera attachment remains URDF-owned.
7. If a required dependency hook is absent, report the exact limitation and implement only a narrowly justified alternative meeting these same pose/ownership tests. Do not replace `+0.015` with `-0.059` in Python and declare geometry verified.

The wrapper lives in the ignored, nested `clearpath_common` checkout. Any change there must be captured as a focused, root-tracked patch (for example `patches/clearpath_realsense_sim_frames.patch`) and documented in the existing README dependency-patch workflow. Record the dependency revision and prove the patch applies on a clean copy of that revision, without reverting unrelated nested changes. Do not hide a required fix only in a local dependency checkout or modify installed `/opt/ros` files.

### D. Correct documentation and comments by meaning

Update the targeted claims in:

- [README.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/README.md): consistent D455 identity, patch/setup instructions, distinction between capabilities and configured/verified streams, safe handling of an existing setup directory.
- [AI_CONTEXT.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/AI_CONTEXT.md): D455 identity, sim/hardware TF ownership, optional cloud contract, and no unconditional claim that a physical pointcloud row is permanently missing.
- [ISSUES.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/ISSUES.md:159): replace the current D435-specific workaround with the new ownership model. Keep historical explanation explicitly historical if useful.
- [object_localization_pipeline.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/localization/object_localization_pipeline.md): hardware model, IMU capability versus usage, pointcloud configuration, and simplified simulation geometry/optics.
- [aligned_depth.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/localization/aligned_depth.md), [pointcloud_provenance_test.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/history/pointcloud_provenance_test.md), and [segmentation_component.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/localization/segmentation_component.md): model-specific statements and current verification caveats.
- Comments/docstrings in [depth_sources.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/depth_sources.py), [segmentation.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/core/segmentation.py), [mask_measurement_node.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/mask_measurement_node.py), and [launch.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/launch.py): use “RealSense driver” for generic alignment/encoding behavior and “D455” for this robot's hardware.

Rules for this pass:

- Do not relabel D435-specific datasheet numbers as D455 numbers. Remove irrelevant quotations or replace them with verified primary-source statements; preserve historical attribution where necessary.
- Do not turn a recommended/ideal depth range into a runtime validity cutoff. Remove any unsupported guarantee that all unreliable stereo depths necessarily arrive as zero/NaN/inf; finite positive depth can still be inaccurate. This is a wording correction, not permission to add filtering.
- Explain that D455 IMU availability does not mean IMU streams are enabled or fused. No new IMU subscriptions or SLAM changes.
- Explain the distinction between Clearpath's checked-out pointcloud default, RealSense wrapper defaults, actual published cloud layout, and the application's optional organized-cloud input. No new cloud wiring in this task.
- Correct outdated claims that the current mask estimators derive their intrinsics from the JSON FoV constants: they use `CameraInfo`. Inspect remaining JSON consumers before describing its role; do not change its values here.
- Retain past benchmark artifacts/results with their actual D435-model provenance. Mark the geometry transition so old and new runs are not presented as identical setups.
- Leave unrelated refactor documentation, ROI work, and TF-fallback coverage work alone.

### E. Validate the repository implementation

| Gate | Required proof |
|---|---|
| Config | Explicit D455 survives parser and generated driver/model configuration; no profile, synchronization, mount, or stream-toggle drift |
| URDF/TF | Full mount-to-color-optical chain comes from the selected model in simulation; exactly one owner per transform; actual parent/child relationships and nominal offsets asserted |
| Sim/hardware separation | `is_sim=true` supplies nominal frames; `is_sim=false` leaves internal calibrated frames to the driver; no old static-publisher conflict |
| Render geometry | Converted SDF sensor pose, optical axes, and observed image/cloud frames agree with TF, including fixed-joint reduction |
| Projection | Known 3D targets at center, image edges, and multiple depths project consistently through `CameraInfo` and TF; organized-cloud/deprojection comparisons remain consistent; LiDAR overlays align |
| Existing behavior | Camera contract, exact stamp/RGB sharing, estimator math, masks, miss reasons, enabled paths, and unrelated TF tests remain green |
| Launches | Exploration, benchmark wrapper/environment/config, and manual mapping still resolve; internal include expectations updated only for the scoped TF ownership change |
| Reproducibility | Root patch applies cleanly against recorded dependency revision; required installed packages rebuilt; new test files registered with CMake |
| Docs | Every first-party D435 hit is classified as corrected, generic wording, or explicitly historical; no unsupported hardware-verified claim |

Use the ROS-sourced `perception_venv` for Python tests. Run at least `test_robot_config.py`, `test_camera_inputs.py`, `test_launch_layout.py`, `test_intrinsics.py`, `test_pointcloud_ranging.py`, `test_mask_measurement_node.py`, and import/shared-default guards, then the full suite. New parser/Xacro/geometry integration checks must actually run in the configured workspace; `importorskip` alone is not proof of parser/model correctness.

```bash
cd /home/stefi/DyNAMO/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash
source install/setup.bash
env ROS_LOG_DIR=/tmp/dynamo_d455_test_logs perception_venv/bin/python3 -m pytest -q src/ridgeback_autonomy/test
```

Before interpreting final installed-code results, rebuild every changed package and its required dependents, not only `ridgeback_autonomy` if the camera description changed. Re-source the install overlay, repeat the tests, run the registered colcon tests, check public launches with `--show-args`, and run `git diff --check` plus:

```bash
bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"
```

Run an isolated short benchmark with a visible target and inspect actual measurements, mask/RGB overlays, and polar beams. Use the throwaway `setup_path`, record it in the report, and include a short exploration startup smoke. Do not assert numerical parity with the old camera pose; explain any geometry-induced change and evaluate against known scene coordinates. Keep the separate pre-existing depth-coverage concern separate from camera-model correctness.

Coordinate runtime access before using cleanup or starting simulator processes. Do not terminate another agent's run. If live validation is unavailable, complete independent static/unit checks and report the live gate as pending, not passed.

### F. Controlled local/robot rollout — explicit authorization required

1. Resolve the exact intended setup path on the local machine or robot. Do not assume the home-directory copy is the robot's deployed file.
2. Show the camera-only proposed diff, effective generated parameters, and all remaining differences from repo defaults. Ask the user to resolve the conflicting mount pose if deployment would change it. Leave unrelated platform, lidar, networking, and robot identity configuration intact.
3. With deployment permission, make a recoverable backup of the exact target file, apply the narrow update, regenerate only that setup's outputs, and restart only its affected services/processes. Record what was changed. Do not do this automatically as part of the repository build.
4. On the physical device, use available enumeration/driver information to confirm D455 identity and effective model/serial selection. Do not install drivers, update firmware, enable additional streams, or move the robot without separate need and authorization.
5. Verify actual RGB and color `CameraInfo`; aligned-depth topic/encoding/grid; stamp relationship under sync; intrinsics; complete calibrated TF chain; and no duplicate optical TF publisher. Record active profiles rather than inferring them from YAML.
6. Inventory existing pointcloud/IMU topics read-only. If a pointcloud exists, verify organization, color-grid indexing, frame, and timestamps before claiming it can serve the pointcloud estimator. Its presence does not automatically authorize changing that estimator's input.
7. Report hardware/deployment pending if access or user choices are missing. This must not block writing/tests for the repository fix, but it does block claiming the physical system is verified.

## 4. Rollback and partial-state handling

- Keep configuration, sim-model/TF changes, and deployment output provenance together as one coherent rollout. Do not run a new nominal-TF model alongside the old static publisher.
- Roll back only this task's patches, not unrelated root or nested work. Restore the matching model/TF arrangement and regenerate from its matching configuration. Never use broad resets or discard other agents' edits.
- For an authorized deployment, restore the exact backed-up file if needed and regenerate that setup's artifacts. Label the restored D435 setting as the previous configuration, not a correct D455 fix.
- Patch setup instructions should distinguish “not applied,” “already applied,” and “conflicts with local changes”; a second run must not stack the same change or silently proceed on a failed patch.
- Keep old benchmark data intact; report which configuration each run used. Do not rewrite historical evidence to appear D455-based.

## 5. Completion report and stop condition

Report three statuses separately:

1. **Repository correction:** config, model/TF ownership, docs, tests, build, and reproducible dependency patch.
2. **Simulation verification:** generated pose/frame checks and observed benchmark/exploration results.
3. **Deployment/hardware verification:** exact setup path, approved changes, device identity, topics, timestamps, and TF evidence — or explicitly pending.

Include intentional behavior changes, preserved settings, exact validation commands/results, rollback location for any authorized deployment, and remaining blockers. Stop after this camera correction; do not start ROI migration, TF fallback changes, latency tuning, IMU integration, pointcloud enablement, or full D455 sensor-fidelity simulation.
