# Isaac Sim 6.1 migration

Status: **approved, not started**. This is the canonical migration and
acceptance plan for moving the Ridgeback backend from Isaac Sim 6.0.1 to
6.1.0.0. Current runtime documentation must continue to describe 6.0.1 until
the migration passes every gate below.

## Decisions

- Migrate now, after a **bounded 6.0.1 control**, rather than waiting for the
  robot-seating work and a statistical multi-seed baseline. The control proves
  compatibility; it is not a publishable 6.0-versus-6.1 benchmark.
- Qualify 6.1 in a side-by-side virtual environment. Do not overwrite the
  working 6.0.1 environment until 6.1 is accepted.
- Use only a driver configuration supported by NVIDIA. Isaac Sim 6.1 requires
  Linux driver 595.58.03 or newer; this workstation currently has 580.173.02.
- Protect the workstation's active Isaac Sim 5.1 workflow with the same
  representative G1 evaluation before and after the driver upgrade.
- Land basic 6.1 compatibility before removing any 6.0.1 workaround. Remove a
  workaround only when a focused test proves it obsolete.
- Use the 6.1 native stereo-depth path if it returns valid measurements. If it
  does not, use geometric depth from the physical left-depth origin plus our
  D455 disparity/noise model. Both paths align into the colour frame.
- Do not spend this migration moving Gazebo lidar mounts. Their known physical
  divergence remains documented and accepted.

Official references:

- [Isaac Sim 6.1 system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)
- [Isaac Sim release notes](https://docs.isaacsim.omniverse.nvidia.com/latest/overview/release_notes.html)
- [Isaac Sim known issues](https://docs.isaacsim.omniverse.nvidia.com/latest/overview/known_issues.html)
- [NVIDIA CUDA compatibility](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)

## 1. Preserve work and capture the 6.0.1 control

The in-progress camera/depth diff is a separate feature, not migration code.
Preserve it on a local checkpoint branch, never push that checkpoint, and
squash it into the finished camera deliverable after the 6.1 depth decision
gate. Start the runtime migration from a clean commit so experimental private
APIs and the known zero-output native-depth path cannot ride the dependency
commit.

Run one deterministic, camera-disabled warehouse control under 6.0.1 using
the recipe in `../exploration/benchmarking.md`, plus:

- `tools/isaac/smoke_test.py`, including exact fixed-step `/clock` behaviour
- installed package versions, enabled extensions and registered ROS OmniGraph
  node identifiers
- runner readiness, expected ROS topics and QoS, TF topology and robot motion
- both assembled scans: 270 degrees, 1081 bins, -135 to +135 degrees, with
  analytic obstacle-distance spot checks
- achieved RTF and one short navigation run

Keep the raw evidence, but label it **bounded compatibility control**. Do not
quote it as a statistical performance comparison; no valid current 6.0.1
multi-seed baseline exists.

## 2. Protect Isaac Sim 5.1 users during the driver change

The workstation-local checkout `/home/deivid/dev/isaac` is the selected 5.1
gate. It contains Isaac Lab v2.3.1, an Isaac Sim 5.1.0.0 Python environment,
the active custom G1 grasp-lift task, camera observations, custom hammer USDs
and trained SKRL checkpoints.

The gate was trialled successfully on 2026-09-13 with driver 580.173.02. It
created the G1 scene, ran CUDA PhysX and policy inference, rendered 120 camera
frames, wrote a valid 1280x720 H.264 video and exited zero. The first attempted
asset path was stale; the command below contains the corrected current path.

Run this exact workload before and after the driver upgrade:

```bash
cd /home/deivid/dev/isaac
/home/deivid/micromamba/envs/env_isaaclab/bin/python \
  scripts/reinforcement_learning/skrl/play_g1_custom_object.py \
  --task Isaac-G1-GraspLift-Direct-v0 \
  --object_usd_path sam3d_objects/hammers/usd_versions/hammer_usds_noinst/converted_hammer_blue_granite/hammer_blue_granite.usd \
  --object_size 0.05 0.05 0.12 \
  --object_scale 1.0 1.0 1.0 \
  --object_euler_deg 90 0 0 \
  --object_spawn_z 0.71 \
  --checkpoint logs/skrl/g1_grasp_lift_direct/2026-04-16_21-23-41_ppo_torch_fanta/checkpoints/best_agent.pt \
  --num_envs 1 \
  --enable_cameras \
  --headless \
  --device cuda:0 \
  --video \
  --video_length 120
```

For each run, retain stdout, the Kit log, package versions, video metadata and
video checksum outside tracked source. Acceptance is:

- package metadata reports Isaac Sim 5.1.0.0
- the RTX PRO 6000 is the active Vulkan device
- the custom environment and checkpoint load
- the process exits zero and produces exactly 120 frames at 1280x720
- no traceback, segmentation fault, CUDA error or Vulkan error appears

This validates the active Isaac Lab/CUDA/rendering workflow, not ROS 2. If a
5.1 user also depends on ROS, their representative ROS command is an
additional pre/post gate rather than a replacement for this one.

## 3. Upgrade the shared workstation driver

Coordinate a maintenance window with the 5.1 users. Before changing anything,
record the running kernel and exact installed 580-open packages and confirm
that they remain available for rollback.

Stay with Ubuntu's signed open-driver packages, matching the existing install;
do not mix in NVIDIA's `.run` installer. At planning time Ubuntu 24.04 offers
`nvidia-driver-595-open` 595.84, which satisfies the 595.58.03 minimum. Recheck
the candidate immediately before installation because repository versions can
change.

After reboot, require all of the following before installing or qualifying
Isaac Sim 6.1:

- the signed NVIDIA module loads with Secure Boot enabled
- `nvidia-smi` reports the RTX PRO 6000 and the intended 595-or-newer driver
- Vulkan selects the NVIDIA GPU successfully
- the Isaac compatibility checker passes
- the identical 5.1 G1 gate above passes

If the 5.1 gate regresses, stop. Restore the recorded 580-open package set,
reboot and rerun the pre-upgrade gate. Do not qualify 6.1 on the failed host
state.

## 4. Migrate the dependency and runtime

- Pin `isaacsim[all,extscache]==6.1.0.0` in `requirements-isaac.txt`.
- Keep Python 3.12, ROS 2 Jazzy and CycloneDDS unchanged.
- Extend `tools/install_isaac_venv.sh` with `--venv <path>` and update its
  driver preflight to 595.58.03. Install the candidate beside 6.0.1.
- Make the smoke test assert the exact Isaac version as well as headless boot,
  bridge nodes, `/clock` and fixed-step behaviour.
- Audit imports, extension identifiers, OmniGraph node registrations, ROS
  bridge initialization, application lifecycle and render settings against
  the 6.1 APIs. Change only what 6.1 requires; do not build a permanent
  dual-version compatibility layer.
- Qualify launches with `ISAAC_PYTHON` pointing at the candidate. Promote 6.1
  to the canonical `isaac_venv` only after all acceptance gates pass, and keep
  6.0.1 available until the first complete 6.1 benchmark succeeds.

## 5. Resolve the depth source on 6.1

Before adapting the camera implementation, run a minimal native
`SingleViewDepthCameraSensor` probe against a plane at 3 m. With noise
disabled, require the requested dimensions, nonzero finite output, at least
99% valid pixels in the unobstructed centre and median depth within 1% of the
ideal geometric result. Then enable the D455 model and require bounded,
non-constant residuals rather than zero or invalid frames.

The probe selects the implementation without another design decision:

- **Native probe passes:** use the supported 6.1 native sensor from the
  left-depth optical origin.
- **Native probe fails:** record the evidence and use geometric depth from the
  same origin, followed by D455 disparity-domain noise, quantization, range
  masking and distortion.

Both paths explicitly reproject into the colour camera and resolve collisions
with a nearest-depth z-buffer. Publish aligned depth and the organized point
cloud in `camera_0_color_optical_frame`, preserving the existing topics, QoS
and time base. Remove all temporary debugging, environment overrides and
private lifecycle calls.

The stable cross-backend options are:

- `camera_profile:=d455_640|d455_1280`, default `d455_640`
- `depth_fidelity:=ideal|d455`, default `ideal`

Real hardware intrinsics, extrinsics and noise remain a deployment validation
TODO. Simulation must not be retuned from unverified hardware assumptions.

## 6. Revalidate 6.0.1 workarounds

Land the pure 6.1 compatibility change first. Then test these independently:

- native ROS LaserScan versus the two-prim point-cloud assembler
- URDF importer visual-mesh, collision and de-instancing repairs
- headless initialization order and forced shutdown handling
- collider debug draw and render-settings workarounds

Remove a workaround only when its focused test and the bounded end-to-end
control both pass. Otherwise retain it and document that it remains necessary
on 6.1. Do not regenerate or replace the committed robot USD merely because
the importer version changed; first generate to temporary output and compare
visuals, prim structure, articulation and collision behaviour.

## Acceptance and delivery

- Unit and launch-layout tests pass for backend selection, installer options,
  camera profiles and depth-fidelity forwarding.
- Both 640 and 1280 work with ideal and D455 depth in Isaac. Gazebo retains the
  shared resolutions without gaining migration-specific depth machinery.
- A two-plane occlusion scene proves colour alignment: visible control points
  reproject within one pixel, the nearest surface wins, output dimensions and
  frame match colour, and timestamps differ by no more than one simulation
  tick.
- Lidar retains 270 degrees, 1081 ordered bins and correct analytic ranges.
- The bounded 6.1 run has no topic, QoS, TF or control regression and achieves
  RTF >= 0.8 on a suitably quiet host.
- Run the full current 6.1 benchmark before publishing new performance or
  coverage numbers. Do not present the bounded 6.0.1 control as a statistical
  A/B baseline.
- Update current-state documentation to 6.1 only when these gates pass;
  preserve explicitly historical 6.0.1 evidence.
- Run relevant tests and `tools/rebuild_graphify` before each commit, never as
  a post-commit repair.

Deliver the dependency/runtime migration as
`build(isaac): migrate runtime to 6.1`. Verified workaround removals may be
separate stage commits. Finish the camera work as
`feat(camera): add selectable aligned D455 depth`, squashing the local camera
checkpoint into that deliverable. Tests and documentation ride with their
corresponding commit.
