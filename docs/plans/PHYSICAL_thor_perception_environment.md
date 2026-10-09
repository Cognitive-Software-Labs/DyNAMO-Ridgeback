# PHYSICAL — Thor perception environment and first live localization

Status: **proposed; decisions below await the user.** Owner: an agent at the
robot, with the operator for any `sudo` step (none is expected on Thor).
Governing plan:
[PHYSICAL — Intel–Thor deployment and transport qualification](PHYSICAL_intel_thor_deployment.md),
whose section 1 (reproducible installation, proven GPU execution) this plan
executes for Thor, followed by one stationary live run. The
[distributed localization contract](../target_localization/distributed_deployment.md)
owns launch roles, topics and health; the
[transport reference](../physical/intel_thor_transport.md#sensor-timing-and-time-synchronization)
owns the qualified camera and LiDAR path to Thor this plan builds on.

Perception runs on Thor. This plan gives Thor a verified GPU inference
environment for the repository's models, builds the localization packages
there, and runs the real camera through the real detector and estimators once,
stationary, with Intel observing the results. Measuring the four model modes
(the governing plan's section 3) is the next plan; it starts from the
environment installed here.

## Decisions (proposed; user to confirm)

| # | Decision | Recommendation | Alternative |
|---|---|---|---|
| D1 | Environment type | `venv` over the system Python 3.12 with `--system-site-packages`, as on the workstation, so compute nodes import ROS Jazzy's `rclpy`, `cv_bridge` and NumPy directly | Conda: its Python cannot reliably load the system ROS build; the existing `lerobot` environment belongs to another project and is not reused |
| D2 | Environment location | `~/venvs/dynamo-perception` on Thor, outside every checkout, selected with `RIDGEBACK_PERCEPTION_VENV` | `perception_venv` inside a worktree, which disappears with the worktree |
| D3 | PyTorch source | The PyTorch CUDA 13.0 aarch64 index: `torch 2.11.0+cu130` and `torchvision 0.26.0+cu130`, the workstation's versions in Thor's CUDA variant. Accepted only if the step 3 gate passes | NVIDIA's Jetson AI Lab index, only if the gate fails; its versions differ, so that is a new decision |
| D4 | Requirements | New tracked `requirements-perception-thor.txt`: the D3 pins plus the workstation's unchanged model stack (`transformers 5.10.2`, `accelerate 1.13.0`, Pillow) | Edit `requirements-perception.txt` per host by hand |
| D5 | Model revisions | Record each checkpoint's snapshot hash from the first download; pinning them in code is a separate later change | Pin first |
| D6 | Smoke target and bound | A person (`target_labels:=person`) at taped 1.5, 2.5 and 4.0 m straight ahead and 2.5 m off-centre; projective and Euclidean distances within ±15 % of tape (a smoke, not an accuracy qualification) | The G1 (`humanoid robot`) first; a tighter bound |
| D7 | Code on Thor | A new branch `feat/thor-perception-env` stacked on the latency branch, pushed to `origin` and pulled into a new Thor worktree `~/DyNAMO-Ridgeback-perception`; Thor's other checkouts stay untouched | Switch the latency worktree's branch |

## Starting point (inspected 2026-10-09, read-only)

| Item | Thor `nvidia-thor-r100-0160` |
|---|---|
| Platform | aarch64, L4T R38.2.2, JetPack 7.0, Ubuntu 24.04.4, 14 cores, 122 GB unified memory (119 GB available) |
| GPU | NVIDIA Thor, compute capability 11.0 (`sm_110`), driver 580.00, CUDA 13.0 (`nvcc` 13.0.48), cuDNN 9.12, TensorRT 10.13 |
| Power | MAXN, performance governor (`host_performance`) |
| Python | system 3.12.3; NumPy 1.26.4, OpenCV 4.6.0, Pillow 10.2.0 from Ubuntu; no PyTorch for the system Python |
| Other environments | `~/miniforge3` with `lerobot` (another project): PyTorch 2.10 on CUDA 13.0 with `sm_110` kernels, so CUDA PyTorch runs on this board |
| ROS | Jazzy desktop, 406 packages; `colcon` and `rosdep`; `rosdep check` for the three localization packages: all system dependencies satisfied |
| Disk | 513 GB free |
| Network | PyPI, `download.pytorch.org/whl/cu130`, `pypi.jetson-ai-lab.io` and Hugging Face reachable |
| Wheels | `torch-2.11.0+cu130` and `torchvision-0.26.0+cu130` exist for `cp312` `manylinux_2_28_aarch64`; whether they carry `sm_110` kernels is unverified |
| Hugging Face cache | 19 GB, all another project's; none of this repository's models |
| Repository | latency worktree `~/DyNAMO-Ridgeback-latency` at `68d3570`, no build |

The repository's models are OWLv2 `google/owlv2-base-patch16-ensemble`
(detection), Depth-Anything V2 `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`
(monocular depth only) and SlimSAM `Zigeng/SlimSAM-uniform-50` (silhouette mask
only); no revision is pinned in code. `resolve_torch_device()` falls back to
CPU silently when CUDA PyTorch is missing, so every GPU claim below is checked
explicitly.

## Steps

### 1. Branch and Thor worktree (agent)

Push `feat/thor-perception-env` (D7). On Thor:
`git -C ~/DyNAMO-Ridgeback fetch origin feat/thor-perception-env`, then
`git -C ~/DyNAMO-Ridgeback worktree add --track -b feat/thor-perception-env ~/DyNAMO-Ridgeback-perception origin/feat/thor-perception-env`.
Later changes follow the same path through `origin`; never edit code on Thor.

Exit: the Thor worktree is at the pushed commit and clean.

### 2. Build the localization subset on Thor (agent, no `sudo`)

From the Thor worktree, following the README's
[distributed localization](../../README.md#distributed-localization) commands:
build `ridgeback_interfaces`, `ridgeback_common` and `ridgeback_localization`
into a fresh prefix under `artifacts/colcon/thor-*`, then run
`tools/check_localization_install`.

Exit: the build succeeds and the install check passes.

### 3. Inference environment and GPU gate (agent, no `sudo`)

- Add `requirements-perception-thor.txt` (D4) with the README's setup lines.
- On Thor: create the venv (D1, D2), add the ROS `.pth` file, upgrade `pip`,
  install the requirements, run `pip check`.
- Gate, inside the venv: `torch.cuda.is_available()`; device name contains
  `Thor`; capability `(11, 0)`; `'sm_110'` in `torch.cuda.get_arch_list()`; a
  4096² matrix product and `torchvision.ops.nms` both run on `cuda`; `rclpy`
  and `cv_bridge` import.

Stop rule: if any gate check fails, stop and bring the evidence back for the
D3 alternative. Do not continue on CPU.

### 4. Models on the GPU (agent)

- Load all three models through the repository's own loaders, not ad-hoc
  model IDs, so first-launch downloads cannot stall the live run. Each must
  report `cuda`. Record each snapshot hash (D5).
- Save one colour and aligned-depth frame pair from the live topics with a
  single one-shot subscriber (no `ros2 topic hz` or `node list` loops). Run
  OWLv2 on it, then SlimSAM on its boxes and Depth-Anything on the frame.
  Record cold load time, first and warm inference times, and peak GPU memory
  (`torch.cuda.max_memory_allocated`).
- Run the localization unit tests with the venv's Python against the Thor
  build.

Exit: all three models infer on `cuda`; the unit tests give the same results as
on the workstation, plus any tests that need PyTorch.

### 5. Stationary live run (agent; robot stationary, motion off)

- Before: `tools/camera_contract_check --backend hardware` on Intel passes, and
  `check_link` passes, including the performance-policy rows. Record Thor's
  load and running GPU processes; the board is shared with another project.
- Thor, in the Thor worktree under `dds_env.sh thor` and `ROS_DOMAIN_ID=0`,
  with `RIDGEBACK_PERCEPTION_VENV=~/venvs/dynamo-perception`:
  `localization.launch.py namespace:=r100_0160 base_frame:=base_link`
  `estimators:=projective_ranging,euclidean_reconstruction depth_source:=stereoscopic mask_gate:=box target_labels:=person displays:=false`.
  The camera topic defaults already match the installed contract.
- Intel: the stationary observer from the README, built in a fresh prefix in
  the worktree, sourced through `dds_env.sh intel` so Thor's output stays off
  Wi-Fi.
- Scene (D6): empty for 30 s, then the person at each mark for 30 s.
- Record throughout: health state and progress, detector rate (configured
  limit 10 frames/s), detection and measurement source stamps, the time from
  source stamp to measurement on Thor, Thor CPU/GPU load and temperature
  (`tegrastats`), and scan continuity through a simultaneous Intel guard
  observer.
- After: the contract check passes again.

Exit: health reports `processing` with advancing progress; detections and
measurements carry the source image stamp; distances are within the D6 bound;
the empty scene gives no sustained false measurement; scans have no gap above
250 ms.

Stop rule: any scan gap above 250 ms, a camera driver error, or a GPU memory
error stops the launch; record it.

### 6. Record (agent)

Evidence under `artifacts/hardware/<UTC>-thor-perception-env/` and
`<UTC>-thor-localization-smoke/` with `pip freeze`, the gate output, snapshot
hashes and timings. Add the Thor environment to the README's distributed
localization section, write an archive record, update the governing plan's
installation and timing rows, and update the backlog's physical camera item.
The measured stage timings also answer the governing plan's open publish-order
question: the detector consumes colour first and the mask node then fetches
depth at the detection stamp.

## Excluded

The four model-mode performance measurements and matched replay (governing
plan section 3), failure injection (section 4), silhouette and monocular modes
in the live run (step 4 only proves they load and run), the G1 as target,
robot motion, and the colour-before-depth driver patch.

## Risks

| Risk | Handling |
|---|---|
| The PyTorch wheels lack `sm_110` kernels | Step 3 gate before anything depends on them; fallback is a user decision |
| Silent CPU fallback | Explicit `cuda` checks in steps 3 and 4; GPU load recorded in step 5 |
| Thor shared with another project's GPU work | Record running processes and load before every run; do not stop them |
| Unpinned model revisions change between hosts | Snapshot hashes recorded (D5) |
| Extra camera readers on Thor | The localization nodes are the planned readers; scans are guarded throughout |
| Intel viewer pulls Thor output over Wi-Fi | The observer sources `dds_env.sh intel` |
| NumPy 1.26 from Ubuntu under newer wheels | `pip check` and the step 4 inference prove the combination |

## Commits

1. `docs(plans): plan the Thor perception environment`
2. `feat(localization): add the Thor perception requirements`
3. `docs(physical): record the Thor perception environment`
4. `docs(physical): record the first live localization run on Thor`
