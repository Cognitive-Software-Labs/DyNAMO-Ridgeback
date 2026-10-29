# Ridgeback Exploration, Target Localization, and Benchmarking

Autonomous exploration, reusable target localization, and distance benchmarking
for a [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/)
robot on physical hardware, in Gazebo Harmonic, or in NVIDIA Isaac Sim 6.1 with
ROS 2 Jazzy. The same `ridgeback_autonomy` application package runs in all three
environments; thin backend packages own simulator- or hardware-specific
bringup. The shipped simulation benchmark uses a Unitree G1 as its target model;
the ROS interfaces and localization package are target-generic.

This README owns installation, public commands and operator workflows. It is a
router: detailed references and physical runbooks live in the pages it links.

## Docs

| Area | Pages |
|------|-------|
| Start here | [Documentation index](docs/index.md) · [project context](docs/project/PROJECT_CONTEXT.md) · [documentation ownership](docs/project/documentation.md) · [conventions](docs/project/conventions.md) · `AGENTS.md` / `CLAUDE.md` (thin agent entrypoints) |
| Exploration | [Architecture](docs/exploration/architecture.md) · [launch arguments](docs/exploration/launch_arguments.md) · [evaluation](docs/exploration/evaluation.md) (coverage and navigation outcomes) |
| Target localization | [Pipeline](docs/target_localization/target_localization_pipeline.md) · [camera stack](docs/target_localization/camera_stack.md) · [distributed deployment](docs/target_localization/distributed_deployment.md) |
| Benchmarking | [Overview](docs/target_distance_benchmarking/overview.md) · [running benchmarks](docs/target_distance_benchmarking/running_benchmarks.md) · [launch arguments](docs/target_distance_benchmarking/launch_arguments.md) · [sweeps](docs/target_distance_benchmarking/sweeps.md) · [outputs](docs/target_distance_benchmarking/outputs.md) · [replay profiles](docs/target_distance_benchmarking/profiles.md) · [configurator](docs/target_distance_benchmarking/configurator.md) |
| Robot model | [Geometry](docs/robot/geometry.md) (mounts, dimensioned model, validation limits) · [collision model](docs/robot/collision_model.md) (physical envelope, navigation footprint) |
| Isaac Sim backend | [Overview](docs/isaac/overview.md) · [rollback](docs/isaac/rollback.md) (retained environments, driver recovery) · [camera depth](docs/isaac/camera-depth.md) (D455 depth modes and alignment) · [lidar pipeline](docs/isaac/lidar-pipeline.md) (how the RTX lidar reaches ROS) |
| Physical robot | [Recovery runbook](docs/physical/ridgeback_recovery.md) · [robot-local deployment](docs/physical/robot_local_deployment.md) (startup changes, re-apply order, camera service) · [driving readiness](docs/physical/driving_readiness.md) · [Intel–Thor transport](docs/physical/intel_thor_transport.md) · [operator runbooks](#physical-operations) |
| Maintenance | [Troubleshooting](docs/troubleshooting.md) (current failure signatures and recovery) · [dependency runbook](docs/project/dependencies.md) · [project backlog](docs/BACKLOG.md) · [benchmarking backlog](docs/target_distance_benchmarking/BACKLOG.md) |

## Stack

| Layer | Package | Purpose |
|-------|---------|---------|
| Backend adapter | `ridgeback_autonomy_gz`, `_isaac`, or `_hardware` | Supplies the common ROS sensor, odometry, TF, and clock contract |
| Simulation | Gazebo Harmonic + clearpath_simulator, or Isaac Sim 6.1 | Optional physics, sensors, and world providers |
| Perception | Hokuyo UST-10LX 2D lidar | Obstacle detection + SLAM input |
| Perception | Intel RealSense D455 (~1m height) | Depth/RGB for overlay and distance estimation |
| SLAM | slam_toolbox (online async, from source) | Map building + localization |
| Navigation | Nav2 (MPPI omni controller) | Path planning + obstacle avoidance |
| Exploration | in-repo `frontier_explorer_node` | Frontier detection + goal selection |
| Perception | Staged target-localization pipeline | Raw OWLv2 detections plus the point-cloud and mask measurement nodes and an optional overlay |
| Benchmarking | Target-distance benchmark runner | Controlled evaluation of the point-cloud and mask-based localization paths |

## Prerequisites

- Ubuntu 24.04
- GPU with OpenGL support (Gazebo); an RTX GPU and compatible NVIDIA driver for Isaac

## Installation

The steps below install this workspace. To rebuild the robot host after SSD
replacement, start with the [Ridgeback recovery runbook](docs/physical/ridgeback_recovery.md)
and its [deployed scripts/configs](deployment/r100_0160/README.md); the
[hardware/desktop recovery recipe](docs/physical/ridgeback_public_recovery.md)
uses pinned public dependencies, and the vendor dashboard is optional.

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep \
  python3-vcstool ros-jazzy-navigation2 ros-jazzy-nav2-bringup
# Gazebo hosts only: the ROS/Gazebo bridge; rosdep resolves the Qt/Gazebo
# development packages after the Gazebo profile is imported
sudo apt install -y ros-jazzy-ros-gz
sudo rosdep init 2>/dev/null || true
rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

### 2. Build the workspace

```bash
cd /path/to/DyNAMO-Ridgeback
vcs import < dependencies/core.repos   # common source dependencies, all deployments
# Gazebo hosts only: Clearpath simulation, the patched gz-sim8 source, and
# ros_gz (only its patched bridge packages are built)
vcs import < dependencies/gz.repos
# Hosts that run RViz (both simulators launch it by default): the patched RViz
# default plugins; `--profile all` below expects this checkout
vcs import < dependencies/viz.repos
# Verify every checkout is pinned and apply the recorded patches idempotently
tools/check_dependencies --apply --profile all
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
source install/setup.bash
```

Isaac-only and hardware hosts omit `dependencies/gz.repos` and use
`tools/check_dependencies --apply --profile core` (plus `--profile viz` where
RViz runs); the core closure never builds Gazebo. After moving the workspace,
regenerate `build/` and `install/`, whose setup files hold absolute paths;
preserve `artifacts/`, which holds evidence, not build products. Updating from a
revision that generated messages in `ridgeback_autonomy` needs the rebuild and
joint restart in the [interface contract](docs/target_localization/target_localization_pipeline.md#package-and-deployment-interfaces).

### 3. (Optional) Set up target localization venv

Target localization is **on by default** (`target_localization_enabled:=true`)
and **requires** this venv; without it the detector logs one clear error and
exits cleanly. `target_localization_enabled:=false` needs no venv and loads no
models. Pinned dependencies live in [`requirements-perception.txt`](requirements-perception.txt):

```bash
# Create venv that can see ROS 2 packages
python3 -m venv --system-site-packages perception_venv
echo "/opt/ros/jazzy/lib/python3.12/site-packages" > perception_venv/lib/python3.12/site-packages/ros2.pth
# Install pinned perception dependencies
perception_venv/bin/python3 -m pip install -U pip
perception_venv/bin/python3 -m pip install -r requirements-perception.txt
```

The pinned `cu128` torch build targets Blackwell GPUs. Set
`RIDGEBACK_PERCEPTION_VENV` for a venv outside the checkout and
`RIDGEBACK_MODEL_DOWNLOADS=1` for a first launch that must download weights
([inference environments](docs/target_localization/distributed_deployment.md#workstation-inference-environment)).

### 4. (Optional) Set up Isaac Sim 6.1

Isaac runs from its own Python environment. The installer pins the version in
`requirements-isaac.txt` and checks the driver/free VRAM; `--warmup` warms the shader cache:

```bash
tools/install_isaac_venv.sh --warmup
source install/setup.bash
OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 tools/isaac/smoke_test.py
```

The first install is roughly 30–50 GB. Stock Isaac worlds need NVIDIA's asset
root; the repo-local `initial_test_world` does not. Canonical `isaac_venv` points
to 6.1; [Isaac rollback](docs/isaac/rollback.md) covers the retained 6.0.1 environment.

### 5. (Optional) Install the repository Git hooks

```bash
perception_venv/bin/python3 -m pip install -r requirements-dev.txt
bash tools/install_hooks
```

The hooks (sources in `tools/hooks/`) rebuild and stage the graphify graph when
code is staged and run the documentation checks when Markdown is staged;
[conventions](docs/project/conventions.md#graphify) own the graphify rules.

### 6. Set up robot config

The simulator adapters expect the development robot config at `~/clearpath/`.
**If `~/clearpath/robot.yaml` already exists, diff it first:** a copy there can
carry legitimate host, namespace or mount settings, or simply be stale.

```bash
mkdir -p ~/clearpath
diff -u ~/clearpath/robot.yaml clearpath/robot.yaml    # existing copy only
cp ~/clearpath/robot.yaml ~/clearpath/robot.yaml.bak   # rollback point before writing
cp clearpath/robot.yaml ~/clearpath/robot.yaml
```

To change one sensor, edit that block in place and regenerate the outputs.
Writing a *robot's* deployed configuration is a separate, authorized step.

## Quick start

Start every run from a sourced workspace (`source /opt/ros/jazzy/setup.bash`,
then `source install/setup.bash`). `start_exploration.sh` launches exploration:
an optional world is the first positional argument, and later `key:=value`
arguments are forwarded. For simulation it runs the simulator cleanup and
defaults `ROS_DOMAIN_ID` to 42; for `backend:=hardware` it keeps running ROS
processes and the caller's domain.

```bash
bash start_exploration.sh                                # initial_test_world
bash start_exploration.sh office backend:=isaac          # Isaac office world
bash start_exploration.sh headless_rendering:=true       # default world, EGL server rendering
bash start_exploration.sh office backend:=isaac estimators:=pointcloud
bash start_exploration.sh initial_test_world backend:=isaac sim_mode:=deterministic
bash start_exploration.sh backend:=hardware autonomous_motion_enabled:=false base_frame:="${BASE_FRAME:?Set the observed base TF frame}"
RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh office backend:=isaac
```

The script selects CycloneDDS unless `RMW_IMPLEMENTATION` is set; entrypoints
set `CYCLONEDDS_URI` to `config/cyclonedds.xml` only when it is unset
([conventions](docs/project/conventions.md#entrypoints-and-launch-ownership),
[participant ceiling](docs/troubleshooting.md#cyclonedds-participant-ceiling)).
`build_and_start_expl.sh` rebuilds first, then forwards its arguments:

```bash
bash build_and_start_expl.sh
bash build_and_start_expl.sh office backend:=isaac
bash build_and_start_expl.sh office backend:=isaac headless:=true
```

## Public workflows

The three human workflows are exploration with target localization,
target-distance benchmarking, and manual mapping. The six top-level launch files:

| Launch file | Role |
|-------------|------|
| `ridgeback_exploration.launch.py` | [Exploration](#exploration) on Gazebo, Isaac or hardware |
| `target_distance_benchmark.launch.py` | One [benchmark](#benchmarks) run |
| `manual_mapping.launch.py` | [Manual mapping](#manual-mapping) |
| `localization_observer.launch.py` | Intel result observer for [distributed localization](#distributed-localization) |
| `target_benchmark_env.launch.py`, `target_benchmark_config.launch.py` | Benchmark environment and configuration layers, public because the sweep supervisor starts them as separate processes |

Internal includes live under `ridgeback_autonomy/launch/includes/`, and each
backend package has a `launch/backend.launch.py` adapter. Bringup stages wait on
readiness gates with a `--timeout` fallback ([bringup sequence](docs/exploration/architecture.md#bringup-sequence)).

## Exploration

`ridgeback_exploration.launch.py` starts the backend (Gazebo or Isaac spawn, or
hardware attach), then SLAM, Nav2, the in-repo `frontier_explorer_node` and
target localization as their readiness gates pass
([bringup sequence](docs/exploration/architecture.md#bringup-sequence)). All four
estimator rows run by default, each with an RViz ring, plus a wide distance HUD;
the camera overlay is on `debug/target/overlay`. Worlds and every argument are in
[exploration launch arguments](docs/exploration/launch_arguments.md), and
`--show-args` prints the generated list.

```bash
bash cleanup.sh   # simulation only: clean up stale processes first
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=initial_test_world
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=depot exploration_rviz:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=coworking_space
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py backend:=isaac world:=warehouse exploration_rviz:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=initial_test_world target_localization_enabled:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py backend:=isaac world:=office headless:=true
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py backend:=isaac world:=initial_test_world sim_mode:=deterministic
# One ring instead of four: the pre-mask-row exploration stack
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py estimators:=pointcloud
# Hardware-safe smoke: attach only, use real-sensor topics, and never send
# frontier goals. Keep the robot's approved platform/sensor services running.
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
  backend:=hardware autonomous_motion_enabled:=false base_frame:="${BASE_FRAME:?Set the observed base TF frame}"
# Quick-start wrappers forward launch arguments untouched, e.g. an explicit detector baseline
bash start_exploration.sh initial_test_world detector:=owlv2 \
  detection_model:=google/owlv2-base-patch16-ensemble \
  target_labels:="humanoid robot" detection_threshold:=0.55 detector_fps:=10.0
```

`localization_mode:=external` consumes an already-running detector and rejects
local detector overrides; `target_localization_enabled:=false` starts none.
Hardware is attach-only with frontier goals off by default; the platform starts
only with `start_hardware_platform:=true`. Before `autonomous_motion_enabled:=true`,
validate the command chain, stop behaviour and speed limits with the
[driving qualification reference](docs/physical/driving_readiness.md) and the
[hardware safety boundary](docs/exploration/architecture.md#hardware-safety-boundary);
the [qualification plan](docs/plans/PHYSICAL_remaining_qualification.md) holds
every physical gate's status.

## Manual mapping

`manual_mapping.launch.py` brings up Gazebo, `slam_toolbox`, RViz and
Clearpath's interactive-marker teleop with `twist_mux`, without Nav2 or frontier
goals, so an operator drives the robot through the space. It activates the
inactive `platform_velocity_controller` once the controller manager is ready.
Arguments: `world` (`initial_test_world`, `depot` or `coworking_space`),
`mapping_rviz` (default `true`), `namespace`, `use_sim_time`, `setup_path`.

```bash
ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=initial_test_world
```

Keyboard teleop and map saving are in the [ground-truth maps guide](src/ridgeback_autonomy/sim/ground_truth_maps/README.md);
canonical maps come from world geometry, with manual capture as the fallback.

## Benchmarks

Tune on and report results only from a
[qualified execution environment](docs/target_distance_benchmarking/running_benchmarks.md#qualified-execution-environments);
the CLI does not block other environments. [Running benchmarks](docs/target_distance_benchmarking/running_benchmarks.md) picks configurator or CLI.

### Single run

`target_distance_benchmark.launch.py` compares the registered estimators on
declarative simulator scenarios. `backend:=gz` (default) needs
`dependencies/gz.repos`; `backend:=isaac` needs the
[Isaac Sim setup](#4-optional-set-up-isaac-sim-61) and must start from the
workspace root (or set `ISAAC_PYTHON`); `backend:=hardware` is refused. The
default `shutdown_on_complete:=false` keeps both layers open after the runner
finishes ([shutdown lifecycle](docs/target_distance_benchmarking/overview.md#shutdown-lifecycle)).

```bash
# Default run: compare all four estimators on Gazebo
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py
# The same run on Isaac Sim, with its viewport (from the workspace root)
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py backend:=isaac headless:=false
# The same production baseline, with the detector settings explicit
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  detector:=owlv2 detection_model:=google/owlv2-base-patch16-ensemble \
  target_labels:="humanoid robot" detection_threshold:=0.55 detector_fps:=10.0
# Point cloud only; then point cloud + the mask reference row
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=pointcloud
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=pointcloud,projective_ranging
# Mask-based localization rows, stereoscopic aligned depth
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction,polar_profiling
# Same rows on monocular (Depth-Anything) depth: compare sources with one run per depth_source
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction depth_source:=monocular
# Silhouette (segmentation model) instead of the box gate: one run per mask_gate
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction,polar_profiling mask_gate:=silhouette
# Polar profiling alone needs no depth, so this run builds no depth source
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=polar_profiling
```

Each run writes `summary.md` (start here), `run.json`, one CSV per estimator,
trial collages and an optional RViz video under `artifacts/benchmarks/` or
`output_dir:=...` ([outputs](docs/target_distance_benchmarking/outputs.md)). See
also [every argument](docs/target_distance_benchmarking/launch_arguments.md) and
[scoring rules](docs/target_distance_benchmarking/semantics.md).

### Local benchmark configurator

The configurator builds, validates and runs an offline benchmark job. It binds
only to loopback and opens a browser unless `--no-open` is given:

```bash
ros2 run ridgeback_autonomy target_benchmark_configurator
ros2 run ridgeback_autonomy target_benchmark_configurator --no-open
```

**Start** runs the three offline profiles from the page; live sweeps still
start from a terminal. See the [configurator](docs/target_distance_benchmarking/configurator.md).

### Benchmark sweeps

The sweep supervisor starts the simulator selected by `backend`, RViz, the
robot, camera TF and the detector once, then restarts only the configuration
layer for each entry:

```bash
SWEEP="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_sweep_baseline.yaml"
# Validate all configs and print the trial/time estimate without launching.
ros2 run ridgeback_autonomy target_benchmark_sweep "$SWEEP" --dry-run
# Run all 4 baseline configurations against one simulator boot.
ros2 run ridgeback_autonomy target_benchmark_sweep "$SWEEP"
# Run a named subset.
ros2 run ridgeback_autonomy target_benchmark_sweep "$SWEEP" \
  --only pointcloud,mask_polar_profiling
# Rebuild the cross-config report for a partial or completed sweep.
ros2 run ridgeback_autonomy target_benchmark_sweep \
  --report-only artifacts/benchmarks/<timestamp>_baseline
```

The supervisor runs `cleanup.sh` once before the environment starts; never run
it between configurations. Rerunning an interrupted command resumes it. File
format, shipped sweeps and resume: [sweeps](docs/target_distance_benchmarking/sweeps.md);
report layout: [sweep outputs](docs/target_distance_benchmarking/outputs.md#sweep-outputs).

### Replay

Offline replay re-evaluates frozen live captures: the
[legacy smoke workflow](docs/target_distance_benchmarking/profiles.md#legacy-smoke-workflow)
sweeps box-gated stereoscopic `projective_ranging` parameters, and the
[layered workflow](docs/target_distance_benchmarking/profiles.md#layered-workflow)
compares or reruns masks. Replay does not replace live runs for detector,
transport, throughput, latency, model or integration questions.

## Distributed localization

Use the same commit on Intel and Thor. The
[deployment contract](docs/target_localization/distributed_deployment.md) owns
health, QoS, timeout, frame, and rollback semantics. This is the stationary
deployment interface, not permission to drive the robot.

**Thor — build only localization.** In a new shell, source ROS Jazzy and no old
workspace overlay, and use a fresh prefix (another name for a later candidate).
The subset needs no Clearpath sources, Nav2, simulator or RViz; it includes the
vendored, message-only `rviz_2d_overlay_msgs`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/ridgeback_interfaces src/ridgeback_common src/ridgeback_localization src/rviz_2d_overlay_plugins/rviz_2d_overlay_msgs --ignore-src -r -y
colcon --log-base artifacts/colcon/thor build \
  --base-paths src/ridgeback_interfaces src/ridgeback_common src/ridgeback_localization src/rviz_2d_overlay_plugins/rviz_2d_overlay_msgs \
  --build-base artifacts/colcon/thor-build --install-base artifacts/colcon/thor-install
source artifacts/colcon/thor-install/local_setup.bash
python3 tools/check_localization_install
```

Thor's venv follows the header of `requirements-perception-thor.txt`
([Thor inference environment](docs/target_localization/distributed_deployment.md#thor-inference-environment));
with ROS sourced, `~/venvs/dynamo-perception/bin/python3 tools/intel_thor/check_perception_env`
fails unless the models' operations run on the GPU. Apply `tools/intel_thor/thor_lan_route`
first, or Thor never receives static TF. OWLv2 scores a person below the default
`detection_threshold` of 0.55 on `r100_0160`; set it explicitly and record it.
Launch with `RIDGEBACK_PERCEPTION_VENV=~/venvs/dynamo-perception` (plus
`RIDGEBACK_MODEL_DOWNLOADS=1` on a new host) and the observed topics and base frame:

```bash
: "${ROS_DOMAIN_ID:?Set the deployment domain shared with Intel}"
: "${ROBOT_NAMESPACE:?Set the observed ROS namespace}"
: "${BASE_FRAME:?Set the observed base TF frame ID}"
: "${COLOR_TOPIC:?Set the observed color image topic}"
: "${DEPTH_TOPIC:?Set the observed aligned depth-to-color topic}"
: "${CAMERA_INFO_TOPIC:?Set the observed color CameraInfo topic}"
ros2 launch ridgeback_localization localization.launch.py \
  namespace:="$ROBOT_NAMESPACE" use_sim_time:=false base_frame:="$BASE_FRAME" \
  color_topic:="$COLOR_TOPIC" depth_topic:="$DEPTH_TOPIC" camera_info_topic:="$CAMERA_INFO_TOPIC" \
  estimators:=projective_ranging,euclidean_reconstruction \
  detector:=owlv2 detection_model:=google/owlv2-base-patch16-ensemble \
  target_labels:="humanoid robot" detection_threshold:=0.55 detector_fps:=10.0 \
  displays:=false
```

To start localization at every Thor boot (after Thor's PTP clock services), run
`sudo tools/intel_thor/localization_service apply` from the Thor checkout that
holds the build; it refuses while a hand-started launch runs. Edit settings
(checkout, labels, threshold, mask and depth modes) in `/etc/default/dynamo-localization`,
then `sudo systemctl restart dynamo-localization`. `status` compares the unit
with its template; `remove` keeps the settings unless `--purge` is given.

**Intel — stationary consumers.** After the core dependency installation, build
the four application packages in a fresh prefix (no simulator adapter needed):

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base artifacts/colcon/intel build \
  --base-paths src/ridgeback_interfaces src/ridgeback_common src/ridgeback_localization src/ridgeback_autonomy src/rviz_2d_overlay_plugins/rviz_2d_overlay_msgs \
  --build-base artifacts/colcon/intel-build --install-base artifacts/colcon/intel-install
source artifacts/colcon/intel-install/local_setup.bash
: "${ROS_DOMAIN_ID:?Set the deployment domain shared with Thor}"
: "${ROBOT_NAMESPACE:?Set the observed ROS namespace}"
: "${BASE_FRAME:?Set the observed base TF frame ID}"
: "${COLOR_TOPIC:?Set the observed color image topic}"
ros2 launch ridgeback_autonomy localization_observer.launch.py \
  namespace:="$ROBOT_NAMESPACE" use_sim_time:=false base_frame:="$BASE_FRAME" \
  color_topic:="$COLOR_TOPIC" estimators:=projective_ranging,euclidean_reconstruction
```

The observer attaches to existing topics and starts no navigation or compute.
Read health with `ros2 topic echo "/${ROBOT_NAMESPACE}/localization/health"` and
the Intel summary on `localization/status`; `displays:=false` keeps only the
health consumer. For a person check, restart Thor with `target_labels:=person`
and record it. Stop each launch with Ctrl-C in its own terminal, leave
platform/sensor services running, and never run `cleanup.sh` on hardware.

**Intel — RViz view with the SLAM map.** To watch Thor's results over a live map
on the Intel desktop (VNC display `:1`), run hardware exploration in external
mode. Build the full Intel set with the patched `slam_toolbox` (from
`tools/check_dependencies --apply --profile core`) and the Ethernet-only DDS profile:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base artifacts/colcon/intel-rviz build --symlink-install \
  --base-paths src/ridgeback_interfaces src/ridgeback_common src/ridgeback_localization src/ridgeback_autonomy \
    src/ridgeback_autonomy_hardware src/rviz_2d_overlay_plugins src/slam_toolbox \
  --build-base artifacts/colcon/intel-rviz-build --install-base artifacts/colcon/intel-rviz-install
source artifacts/colcon/intel-rviz-install/local_setup.bash
source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh intel
: "${ROS_DOMAIN_ID:?Set the deployment domain shared with Thor}"
DISPLAY=:1 ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
  backend:=hardware localization_mode:=external autonomous_motion_enabled:=false \
  base_frame:="${BASE_FRAME:?Set the observed base TF frame}" \
  estimators:=projective_ranging,euclidean_reconstruction
```

RViz shows the map, both scans, the rings and the camera overlay; pass the same
`estimators` and `mask_gate` that Thor runs. Hardware RViz drops the
*2D Goal Pose* tool and is throttled to protect the camera
([Intel RViz view](docs/target_localization/distributed_deployment.md#intel-rviz-view)).
Keep the e-stop engaged, because other Nav2 clients can still send goals.

Missions that require localization (`localization_required:=true`, the external
mode default) start paused; keep `autonomous_motion_enabled:=false` throughout
this stationary milestone. In simulation/fake-server qualification, after
healthy advancing input and a resolved previous goal, resume explicitly
([mission pause and resume](docs/target_localization/distributed_deployment.md#mission-pause-and-cancellation)):

```bash
ros2 service call /r100_0001/mission/resume std_srvs/srv/Trigger '{}'
```

## Intel–Thor deployment

The Intel PC and the Jetson Thor both run ROS 2 on CycloneDDS over the robot
Ethernet only. The [transport reference](docs/physical/intel_thor_transport.md)
owns the mechanism and its [blocker procedure](docs/physical/intel_thor_transport.md#camera-subscriber-blocker)
the deployment state; the [guarded hardware workflow](docs/physical/intel_dds_testing.md)
repairs Intel desktop/camera readiness and verifies the retained DDS profile.

### Intel-Thor link setup and pre-deploy check

One-time host setup (receive buffers, the `intel_services_rmw` middleware
switch, `host_performance`, Thor's `thor_lan_route`) is in
[setup commands](docs/physical/intel_thor_transport.md#setup-commands);
`check_link` fails until it is done. An unknown host or invalid role makes
`dds_env.sh` return nonzero without changing the environment; then pass `intel`
or `thor` explicitly.

```bash
# Every shell that starts cross-host ROS processes or inspects Thor's topics, both hosts
source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh
# Before every deployment, robot stationary, on Intel
tools/intel_thor/check_link
```

It checks routing, link speed, buffer limits, Intel's service middleware, the
configurations, clock offset and a 30 Hz camera-sized stream in both
directions, and exits nonzero on any failure; `--no-traffic` skips the stream.
`tools/intel_thor/rmw_benchmark` repeats the Fast DDS/CycloneDDS comparison.

### Intel-Thor sensor latency

The latency collector runs on Intel against a Thor worktree at the same
revision, after `check_link`. Clock (chrony/PTP) installation, baseline
collection and transport candidates are in
[running the qualification](docs/physical/intel_thor_sensor_latency.md#running-the-qualification);
report definitions in the [sensor timing reference](docs/physical/intel_thor_sensor_latency.md).

## Hardware camera service

On the robot, `dynamo-camera.service` owns the D455 from boot and publishes the
simulators' camera topics, frames and TF; the vendor camera units are retired.
Install it after `intel_services_rmw apply`, robot stationary and e-stop in
reach. `rollback`, and an `apply` that changes the camera declaration, restart
`clearpath-robot`, dropping motor power for about a minute; a re-apply that
changes only the profile, rate or DDS tuning restarts the camera alone. Let each
printed step finish.

```bash
tools/intel_thor/camera_service status
sudo tools/intel_thor/camera_service apply --profile 640x480
# Check the running camera against the contract, with the robot's environment
source /etc/clearpath/setup.bash && tools/camera_contract_check --backend hardware
```

```bash
sudo tools/intel_thor/camera_service rollback
```

`apply` is idempotent: re-run it after editing the driver configuration. The
robot runs 640x480 only; 1280x720 is not supported on the robot: it is not tested or developed further, and anyone who needs it must develop and qualify it first. On a replacement host, after
Clearpath and discovery-only DDS are set up, install the serial-pinned service
with `sudo tools/intel_thor/camera_service apply --profile 640x480 --serial 338522302134`.
Do not reapply just because the checkout revision differs while all status
checks match. Installed files, source restore and re-apply order are in
[robot-local deployment](docs/physical/robot_local_deployment.md).

## Physical operations

Physical procedures run on Intel from the robot checkout, stop pressed and an
operator present unless a runbook step says otherwise; recorders and checkers
cannot stop the robot. Check the [installed guard mode](docs/physical/driving_readiness.md#installed-state) first.

| Task | Runbook |
|------|---------|
| Read-only audits, guard install and rollback, recorded stop release without motion, observer checks, bounded moving trials, the forward-pulse helper, controller checks and recovery, evidence finalization | [Driving operator runbook](docs/physical/driving_operator_runbook.md) |
| Puma repair driver, feedback-consumer plugin, field-age driver, post-reboot check, stationary feedback-interruption checks | [Puma feedback runbook](docs/physical/puma_feedback_runbook.md) |
| Desktop/VNC/controller-autoconnect repair, IMU/battery/clock repair, fast health batch, Intel clock sources | [Stationary repairs runbook](docs/physical/stationary_repairs_runbook.md) |
| Command path, installed guard state, remaining physical verification | [Driving readiness](docs/physical/driving_readiness.md) |
| Host reconstruction after reinstall or SSD failure | [Ridgeback recovery runbook](docs/physical/ridgeback_recovery.md) |

With the stop pressed, `python3 tools/hardware/fast_stationary_check.py` runs a
16-second read-only health batch. Operator steps that still lack a maintained
procedure are tracked in the
[project backlog](docs/BACKLOG.md#missing-physical-operator-procedures).

## Configuration and maintenance

- **Robot sensors:** `clearpath/robot.yaml`; see [declared sensors](docs/robot/geometry.md#declared-sensors).
- **Tuning and interfaces:** [exploration tuning parameters](docs/exploration/architecture.md#tuning-parameters),
  [perception node interfaces](docs/target_localization/target_localization_pipeline.md#11-node-interfaces),
  [camera stack](docs/target_localization/camera_stack.md) (intrinsics and profiles).
- **Generated artifacts:** logs and results live under the Git-ignored `artifacts/`;
  layout and the `LOG_DIR`, `output_dir` and `ROS_LOG_DIR` overrides are in
  [conventions](docs/project/conventions.md#generated-state). Never treat
  benchmark results as disposable; nothing prunes them.
- **Dependencies and patches:** patches are pinned to exact upstream commits;
  run `tools/check_dependencies --apply --profile all` after every import and
  before builds. The [patch inventory](docs/project/dependency_patches.md#patch-inventory)
  and [dependency runbook](docs/project/dependencies.md) own list and rollback.
  Never resolve a conflict by discarding unrelated changes in a dependency checkout.
- **Troubleshooting:** [current recovery steps](docs/troubleshooting.md).

## Archived evidence

- [6.1 migration evidence](archive/engineering/2026-09-13-isaac-6.1-migration.md)
- [investigation history](archive/engineering/exact_stamp_depth_availability.md)
- [projective parameter sensitivity](archive/engineering/projective_parameter_sensitivity.md)
- [Offline measurement replay validation](archive/engineering/offline_measurement_replay_validation.md)
- [dependency refresh history](archive/engineering/dependency_refresh.md)
- [operational incident history](archive/engineering/operational_incidents.md)
- [port history](archive/engineering/port-history.md)
