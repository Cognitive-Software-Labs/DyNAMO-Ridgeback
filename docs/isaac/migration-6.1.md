# Isaac Sim 6.1 migration

Status: **completed and promoted on 2026-09-13**.
The pre-driver controls and post-upgrade Isaac Sim 6.1 acceptance gates passed
on signed Ubuntu 595.84-open. The protected Isaac Sim 5.1 workload reproduced
the known RTX scene-database crash twice; the operator explicitly accepted
that regression on 2026-09-13 and an offline 580.173.02 rollback is preserved.
This is the canonical migration and acceptance record for moving the
Ridgeback backend from Isaac Sim 6.0.1 to 6.1.0.0. Current runtime
documentation now describes the promoted 6.1 environment; historical controls
below retain their original 6.0.1 labels.

## Decisions

- Migrate now, after a **bounded 6.0.1 control**, rather than waiting for the
  robot-seating work and a statistical multi-seed baseline. The control proves
  compatibility; it is not a publishable 6.0-versus-6.1 benchmark.
- Qualify 6.1 in a side-by-side virtual environment. Do not overwrite the
  working 6.0.1 environment until 6.1 is accepted.
- Use only a driver configuration supported by NVIDIA. Isaac Sim 6.1 requires
  Linux driver 595.58.03 or newer; this workstation currently has 580.173.02.
- Measure the workstation's active Isaac Sim 5.1 workflow with the same
  representative G1 evaluation before and after the driver upgrade. The
  post-upgrade failure is now an explicitly accepted compatibility loss, not
  a passed migration gate; every workstation user receives the shared 595
  driver.
- Land basic 6.1 compatibility before removing any 6.0.1 workaround. Remove a
  workaround only when a focused test proves it obsolete.
- Use the 6.1 native stereo-depth path if it returns valid measurements. If it
  does not, use geometric depth from the physical left-depth origin plus our
  D455 disparity/noise model. Both paths align into the colour frame.
- Keep the shared measured lidar mounts unchanged. Gazebo uses the same
  physical poses and ±135° aperture; its consumer integration was
  [verified on 2026-09-18](../history/2026-09-18-gazebo-lidar-integration.md).

Official references:

- [Isaac Sim 6.1 system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)
- [Isaac Sim 6.1 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/overview/release_notes.html)
- [Isaac Sim 6.1 known issues](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/overview/known_issues.html)
- [Isaac Sim MCP](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/development_tools/isaac_sim_mcp.html)
- [NVIDIA CUDA compatibility](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)

## Execution record: 2026-09-13

The pre-driver gates are complete:

- the retained environment reports `isaacsim==6.0.1.0`
- the strengthened smoke gate booted on the RTX PRO 6000 with driver
  580.173.02, registered the required ROS OmniGraph nodes, received 150
  monotonic `/clock` messages through system CycloneDDS and advanced 40
  fixed-step frames by exactly 1.0000 seconds
- the bounded camera-disabled warehouse control reached 44.1% coverage at
  96.7% map accuracy and RTF 0.803, with five successful goals and no aborts;
  these are compatibility evidence, not benchmark claims
- the exact Isaac Sim 5.1 G1 gate loaded the custom environment and checkpoint,
  ran on the RTX PRO 6000, emitted 120 frames of 1280x720 H.264 and exited
  cleanly
- Ubuntu still offers every recorded 580.173.02 rollback package, and the
  selected signed open-driver candidate is 595.84
- `nvidia-driver-595-open` and `nvidia-dkms-595-open` 595.84 are installed;
  DKMS built the signed module for kernel 7.0.0-31 with Secure Boot enabled,
  package audit is clean and the host reports that a reboot is required

After that reboot, `nvidia-smi`, Vulkan enumeration, Secure Boot module
signing and the Isaac compatibility checker all passed. The exact Isaac Sim
5.1 G1 gate then crashed twice during RTX initialization in
`librtx.scenedb.plugin.so`, before the custom environment or checkpoint
loaded. The second run rules out a one-time shader/cache failure. The generic
IOMMU warning is not evidence for this crash: the host has one NVIDIA GPU and
its self P2P checks completed, while the crash signature matches the earlier
595 regression recorded in the workstation log.

The plan originally required an immediate rollback at this point. On
2026-09-13 the operator instead chose to accept the local Isaac Sim 5.1 loss
and proceed with Isaac Sim 6.1 on 595.84. Driver selection cannot be scoped by
user account because the kernel module and host libraries are system-wide.
The exact 580.173.02 package set was therefore downloaded and checksummed at
`~/.local/share/workstation-driver-rollback/nvidia-580.173.02/`, with the
guarded rollback helper in
`~/workstation-config/gpu/rollback-to-nvidia-580.173.02`.

The side-by-side `isaac_venv_6_1` environment reports exact package version
`6.1.0.0`. Its strengthened smoke test booted headless on the RTX PRO 6000,
registered every required ROS OmniGraph node, received 150 monotonic `/clock`
messages through system CycloneDDS and advanced 40 fixed-step frames by exactly
1.0000 seconds.

The native depth decision probe failed cleanly. In a controlled 3 m scene,
ordinary RTX `distance_to_image_plane` returned 100% valid pixels with a 3.000 m
median while `SingleViewDepthCameraSensor` returned finite all-zero output after
90 updates. NVIDIA's authored D455 asset reproduced the same split: geometric
depth was valid and native depth was all zero. The selected production path is
therefore geometric depth from the physical left imager, followed by the D455
disparity/noise model, 1/32-pixel quantization, range masking, colour-frame
reprojection and nearest-depth z-buffering. Live ROS checks passed at 640x480
and 1280x720 in both `ideal` and `d455` modes; image and point-cloud timestamps
and colour optical frames matched. The pure occlusion check confirms that the
nearest surface wins when two samples project to the same colour pixel.

The bounded 6.1 camera-disabled warehouse run reached 44.4% coverage at 96.5%
map accuracy and RTF 0.868, with 17 successful goals and no aborts. This clears
the RTF >= 0.8 compatibility gate and exercises ROS topics, QoS, TF, SLAM,
Nav2, lidar scan assembly and robot control. As with the pre-driver control,
these figures are compatibility evidence rather than a statistical benchmark.

Promotion preserves both environments without rewriting either venv:
`isaac_venv` is a symlink to `isaac_venv_6_1`, while the previous exact
6.0.1 environment lives at `isaac_venv_6_0_1`. To roll only this workspace
back after restoring the 580 driver, repoint the symlink to
`isaac_venv_6_0_1`; the old environment's original absolute entry-point paths
then become valid again.

Raw logs and summaries live under the ignored
`artifacts/isaac-migration-6.1/{pre-driver,post-driver}/` directories. The
retained 6.0.1 environment has not been overwritten.

The MCP client and transport do not need a protocol change: `.mcp.json` uses
the current streamable-HTTP `/mcp` endpoint, and the healthy local service
reports MCP 1.25.0 with the expected five tools. Its embedded documentation
corpus is still generated from Isaac Sim 6.0. A full-LFS audit of NVIDIA's
current `kit-usd-agents` main branch at commit
`9e9d69561f81fa368343382a53535f78f8c769fc` found only
`isaacsim_fns/data/6.0`; its default `MCP_ISAACSIM_VERSION` is also `6.0`.
Rebuilding would update the service implementation but would not produce 6.1
answers, so the working port-9904 container is deliberately retained. MCP
answers must not be used as 6.1 evidence until NVIDIA publishes a 6.1 corpus;
the installed runtime and official 6.1 documentation remain authoritative.

## 1. Preserve work and capture the 6.0.1 control

The camera/depth prototype is a separate checkpoint commit, not migration
code. Squash it into the finished camera deliverable after the 6.1 depth
decision gate. Keep experimental private APIs and the known zero-output
native-depth path out of the dependency commit.

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

The 5.1 gate did regress exactly as described above. The original automatic
stop-and-rollback rule was superseded by the operator's explicit 2026-09-13
decision to prioritize Isaac Sim 6.1. This is a scoped exception, not a claim
that 5.1 works: do not launch or support the local 5.1 workload on 595. To
restore it, run the recorded offline rollback helper, reboot and rerun the
pre-upgrade gate. Do not delete the rollback bundle until 6.1 is accepted and
the operator deliberately retires the 5.1 fallback.

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

- `camera_profile:=640x480|1280x720`, default `640x480`
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
