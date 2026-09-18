# Recover the retained Isaac environment

This workstation-specific procedure restores the pre-migration 580 driver and
retained 6.0.1 environment. It changes the driver for **every user** and requires
a reboot. Coordinate host downtime before executing it. The archived migration
record explains why the
5.1 regression was accepted; it is not authorization for a new host change.

## Preflight and retained resources

Read-only inspection on 2026-09-18 confirmed these paths exist:

- `~/.local/share/workstation-driver-rollback/nvidia-580.173.02/`, including
  `SHA256SUMS` and the offline packages;
- `~/workstation-config/gpu/rollback-to-nvidia-580.173.02`;
- workspace directories `isaac_venv_6_0_1` and `isaac_venv_6_1`, with
  `isaac_venv` pointing to `isaac_venv_6_1`.

Existence is not recovery qualification. Before using them, record the current
kernel, `nvidia-smi`, `dkms status`, Secure Boot status, active venv target and
running GPU jobs. Inspect the helper and confirm the bundle is still complete.
The inspected helper checks SHA256 hashes, exactly 21 packages, and package
version `580.173.02-0ubuntu0.24.04.1` before requesting `ROLLBACK-580`.
Do not substitute a partial bundle or mix Ubuntu packages with a `.run` installer.

## Recovery order

1. Stop the affected simulator/GPU workloads in the agreed maintenance window.
2. Run `~/workstation-config/gpu/rollback-to-nvidia-580.173.02`. The helper
   performs the validated package installation, removes 595 driver holds and
   holds the restored 580 driver packages. Abort if its validation fails.
3. Reboot. Verify the intended GPU/driver with `nvidia-smi`, the loaded module
   and DKMS state, Secure Boot signing, and NVIDIA Vulkan enumeration. An
   installed package version alone does not prove the running driver changed.
4. From the workspace root, verify `isaac_venv` is still a symlink before
   repointing it with `ln -sfn isaac_venv_6_0_1 isaac_venv`. Stop if it is a
   real directory; do not overwrite it. Keep both versioned environments.
5. Source ROS Jazzy and the workspace. Check package metadata and run
   `isaac_venv/bin/python tools/isaac/smoke_test.py --expected-version 6.0.1.0`.
   Require bridge registration, monotonic clock delivery and fixed-step
   advancement. Current 6.1 application code is not certified by this smoke
   test: a full application rollback also requires a matching reviewed code
   revision in an isolated checkout and fresh compatibility checks.
6. Re-run the representative 5.1 workload below before claiming that workflow
   is restored. Preserve logs, video metadata/checksum and package versions.

No rollback or GPU qualification was executed during the documentation cleanup.

## Representative Isaac 5.1 gate

The recorded workload uses Isaac Lab v2.3.1 in `/home/deivid/dev/isaac`.
Verify the script, asset and checkpoint still exist before running:

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


Require Isaac Sim package version 5.1.0.0, the intended NVIDIA device, successful
scene/checkpoint loading, exit zero, exactly 120 video frames at 1280×720, and
no traceback, segmentation fault, CUDA or Vulkan error. This exercises
CUDA/PhysX/rendering, not ROS; add the actual ROS workload if its recovery is
required. Returning to 6.1 requires a separately qualified supported driver;
repointing the venv alone does not restore driver compatibility.

## Archived evidence

- [migration evidence](../../archive/engineering/2026-09-13-isaac-6.1-migration.md)
