# Ridgeback Exploration, Target Localization, and Benchmarking

Autonomous exploration, reusable target localization, and distance benchmarking
for a [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/)
robot on physical hardware, in Gazebo Harmonic, or in NVIDIA Isaac Sim 6.1 with
ROS 2 Jazzy. The same `ridgeback_autonomy` application package runs in all three
environments; thin backend packages own simulator- or hardware-specific
bringup. The shipped simulation benchmark currently uses a Unitree G1 as its
target model; the ROS interfaces and localization package are target-generic.

This workspace supports 3 main human workflows:
- full autonomous exploration with target localization (any world)
- repeatable target-distance benchmarking across the point-cloud and mask-based measurements
- manual mapping

## Docs

- [Documentation index](docs/index.md): project context, architecture, benchmarking, plans, and historical validation
- `README.md`: installation, public launch usage, and normal human workflows
- `docs/troubleshooting.md`: current failure signatures and recovery steps
- `docs/project/`: project context, documentation ownership, and repository conventions
- `AGENTS.md` / `CLAUDE.md`: thin entrypoints into the shared agent guidance
- [Exploration evaluation](docs/exploration/evaluation.md): coverage and navigation outcomes
- [Target-distance benchmarking](docs/target_distance_benchmarking/overview.md): estimator comparisons

- [Robot geometry](docs/robot/geometry.md): shared mounts, dimensioned model reference, and validation limits
- [Collision model](docs/robot/collision_model.md): physical envelope, navigation footprint, and backend representations
- [Robot-local deployment](docs/physical/robot_local_deployment.md): every startup change made on the robot, re-apply order, and the camera service

Isaac Sim backend:

- [Project backlog](docs/BACKLOG.md): robot, exploration, deployment, and shared gaps
- [Benchmarking backlog](docs/target_distance_benchmarking/BACKLOG.md): target-distance benchmarking and estimator evaluation
- [Isaac rollback](docs/isaac/rollback.md): retained environments and driver recovery
- `docs/isaac/camera-depth.md`: Isaac D455 depth modes, alignment, ROS contract, and runbook
- `docs/isaac/lidar-pipeline.md`: how the RTX lidar reaches ROS

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

For rebuilding the physical robot host after SSD replacement, start with the
[Ridgeback recovery runbook](docs/physical/ridgeback_recovery.md). It covers the
additional host network, services and device setup, with
[small deployed scripts/configs](deployment/r100_0160/README.md). The steps below
install this project workspace.
The selected [hardware/desktop recovery recipe](docs/physical/ridgeback_public_recovery.md)
uses pinned public dependencies; the vendor dashboard is optional.

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

```bash
sudo apt install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup
```

Gazebo hosts additionally need the ROS/Gazebo bridge and the Qt/Gazebo
development packages resolved by `rosdep` after importing the Gazebo profile:

```bash
sudo apt install -y ros-jazzy-ros-gz
```

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

### 2. Build the workspace

```bash
cd /path/to/DyNAMO-Ridgeback

# Clone the common source dependencies (all deployments)
vcs import < dependencies/core.repos

# Gazebo hosts only: add Clearpath simulation, the patched gz-sim8 source,
# and ros_gz (only its patched bridge packages are built)
vcs import < dependencies/gz.repos

# Hosts that run RViz (both simulators launch it by default): add the patched
# RViz default plugins; `--profile all` below expects this checkout
vcs import < dependencies/viz.repos

# Verify every checkout is pinned and apply the recorded patches idempotently
tools/check_dependencies --apply --profile all

# Install any remaining deps
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --base-paths src
source install/setup.bash
```

For Isaac-only or hardware hosts, omit `dependencies/gz.repos` and use
`tools/check_dependencies --apply --profile core` (plus `--profile viz` after
importing `dependencies/viz.repos` where RViz runs). The core dependency closure
does not import or build Gazebo. If you rename or move the workspace directory
later, regenerate `build/` and `install/` before rebuilding so the generated
setup files do not keep stale absolute paths. Preserve `artifacts/`: its logs
and benchmark results are independent evidence, not build products.

Shared localization messages are generated by `ridgeback_interfaces`. After
updating from a revision that generated them in `ridgeback_autonomy`, rebuild
the four application packages and restart all publishers and consumers together:

```bash
colcon build --symlink-install --packages-select ridgeback_interfaces ridgeback_common ridgeback_localization ridgeback_autonomy
source install/setup.bash
```

The ROS message package name changed; old live processes and bags are not
interchangeable with the new types. See the
[interface contract](docs/target_localization/target_localization_pipeline.md#package-and-deployment-interfaces)
for compatibility and rollback constraints. Standalone compute and Intel result-consumer commands are in
[Distributed localization](#distributed-localization).

### 3. (Optional) Set up target localization venv

The target detection/localization stack (detection, the selected measurement
rows, rings, HUD, and camera overlay) is **on by default** —
`ridgeback_exploration.launch.py` ships with
`target_localization_enabled:=true`, which **requires** the venv below; without it the
detector logs a single clear error and exits cleanly. Disable the whole stack
with `target_localization_enabled:=false` for a normal exploration run that needs no
venv and will not try to load any models.

The stack follows a staged pipeline inside the installable Python package `ridgeback_localization`:
- raw detections on `detections/target/raw`
- point-cloud measurements on `measurements/target/pointcloud`
- mask-based measurements on `measurements/target/mask`

It requires a Python venv with PyTorch and Transformers. All dependencies
(pinned, with the correct CUDA torch index) live in
[`requirements-perception.txt`](requirements-perception.txt):

```bash
# Create venv that can see ROS 2 packages
python3 -m venv --system-site-packages perception_venv
echo "/opt/ros/jazzy/lib/python3.12/site-packages" > perception_venv/lib/python3.12/site-packages/ros2.pth

# Install pinned perception dependencies
perception_venv/bin/python3 -m pip install -U pip
perception_venv/bin/python3 -m pip install -r requirements-perception.txt
```

> **GPU note:** `requirements-perception.txt` pins the **CUDA 12.8 (`cu128`)**
> torch build, which carries the `sm_120` kernels needed for Blackwell GPUs
> (e.g. RTX PRO 6000). Installing plain `torch` from PyPI gives a CPU-only build
> and silently runs Depth-Anything / OWLv2 on the CPU. For a different
> GPU/CUDA, change the index URL + torch pins in that file.

The exploration and benchmark-environment launches use this checkout's
`perception_venv` for inference processes when it exists. If the venv lives
elsewhere, set `RIDGEBACK_PERCEPTION_VENV` to its absolute path in the shell
that starts the launch, sweep, or configurator. Restart an already-running
configurator after setting it; child runs inherit the server's environment.
Direct `ros2 run ridgeback_localization target_detector_node` does not use the
launch helper, so activate the venv in that shell first.

Inference launches load checkpoints from the local Hugging Face cache by
default. For a first launch that needs to download uncached weights, set
`RIDGEBACK_MODEL_DOWNLOADS=1` in the launching shell. OWLv2 uses the configured
`detection_model`; `depth_source:=monocular` also needs the Depth-Anything V2
metric checkpoint, and `mask_gate:=silhouette` needs the SlimSAM checkpoint.

### 4. (Optional) Set up Isaac Sim 6.1

Isaac runs from its own large Python environment. The installer pins the
version in `requirements-isaac.txt`, checks the driver/free VRAM, and can warm
the shader cache:

```bash
tools/install_isaac_venv.sh --warmup
source install/setup.bash
OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 tools/isaac/smoke_test.py
```

The first install is roughly 30–50 GB. Stock Isaac worlds also need access to
NVIDIA's asset root; the repo-local `initial_test_world` world does not.
The 2026-09-13 migration keeps the prior exact 6.0.1 environment at
`isaac_venv_6_0_1`; canonical `isaac_venv` points to 6.1.

### 5. (Optional) Install graphify pre-commit hook

If you use the graphify knowledge graph, install the pre-commit hook to rebuild
and stage it whenever staged code changes are committed:

```bash
perception_venv/bin/python3 -m pip install -r requirements-dev.txt
bash tools/install_hooks
```

The hook only triggers on staged code-file changes, calls
`tools/rebuild_graphify`, and stages the generated graph in the same commit.
The hook source lives in `tools/hooks/pre-commit`.

### 6. Set up robot config

The simulator adapters expect the development robot config at `~/clearpath/`:

```bash
mkdir -p ~/clearpath
cp clearpath/robot.yaml ~/clearpath/robot.yaml
```

**If `~/clearpath/robot.yaml` already exists, diff before you overwrite it**, and
work out whether the differences are local customization or just staleness:

```bash
diff -u ~/clearpath/robot.yaml clearpath/robot.yaml
cp ~/clearpath/robot.yaml ~/clearpath/robot.yaml.bak   # rollback point before writing
```

This directory is a *deployment* location, so a copy sitting there can legitimately
carry host, namespace, or mount settings the repo does not know about — but it can
equally be an old `cp` that simply never got refreshed. The two cases call for
opposite actions, and only the diff tells them apart. Copying the repo file
wholesale replaces every setting at once; to change one sensor, edit that block in
place. Regenerate the setup's outputs afterwards, and treat writing to a *robot's*
deployed configuration as a separate, explicitly authorized step.


## Distributed localization

Use the same commit on Intel and Thor. The
[deployment contract](docs/target_localization/distributed_deployment.md) owns
health, QoS, timeout, frame, and rollback semantics. Physical Intel–Thor
qualification remains open; this is the stationary deployment interface,
not permission to drive the robot.

**Thor — build only localization.** In a new shell, source ROS Jazzy and no old
workspace overlay. The subset needs no imported Clearpath source repositories,
Nav2, simulator, or RViz; it includes the vendored, message-only
`rviz_2d_overlay_msgs` that the optional display node imports. Install the
subset's ROS dependencies, then use a fresh prefix (choose another name for a
later candidate):

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/ridgeback_interfaces src/ridgeback_common src/ridgeback_localization src/rviz_2d_overlay_plugins/rviz_2d_overlay_msgs --ignore-src -r -y
colcon --log-base artifacts/colcon/thor build \
  --base-paths src/ridgeback_interfaces src/ridgeback_common src/ridgeback_localization src/rviz_2d_overlay_plugins/rviz_2d_overlay_msgs \
  --build-base artifacts/colcon/thor-build --install-base artifacts/colcon/thor-install
source artifacts/colcon/thor-install/local_setup.bash
python3 tools/check_localization_install
```

Thor's inference environment uses `requirements-perception-thor.txt` (PyTorch
2.11 for CUDA 13.0 on aarch64); its header has the setup commands. It lives
outside the checkout, so set
`RIDGEBACK_PERCEPTION_VENV=~/venvs/dynamo-perception` for the launch. Perception
processes load models only from the local Hugging Face cache, so a Wi-Fi outage
cannot stop startup; set `RIDGEBACK_MODEL_DOWNLOADS=1` for the first launch on a
new host so the models are downloaded once. Check the environment
with `~/venvs/dynamo-perception/bin/python3 tools/intel_thor/check_perception_env`
(ROS sourced), which fails unless the models' operations run on the GPU.

To run localization at every Thor boot, install the boot service from the Thor
checkout that holds the build; it starts after Thor's PTP clock services:

```bash
sudo tools/intel_thor/localization_service apply
```

Settings (checkout, labels, threshold, mask and depth modes) live in
`/etc/default/dynamo-localization`; edit them and run
`sudo systemctl restart dynamo-localization`. `status` compares the installed
unit with the template; `remove` uninstalls it and keeps the settings unless
`--purge` is given. Stop any launch started by hand first; `apply` refuses
while one runs.
`tools/intel_thor/thor_lan_route` must be applied first, or Thor never receives
static TF. For a person target, OWLv2 scores below the default
`detection_threshold` of 0.55 on `r100_0160`; set `detection_threshold:=` explicitly
and record it. Preserve observed sensor topics and the observed TF base frame:

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

**Intel — stationary consumers.** Follow the core dependency installation above,
then build the four application packages in a fresh prefix, with no simulator
adapter required for the observer:

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
View health with `ros2 topic echo "/${ROBOT_NAMESPACE}/localization/health"` and
the Intel summary with
`ros2 topic echo "/${ROBOT_NAMESPACE}/localization/status"`. Displays publish
images/markers for an independently opened RViz; `displays:=false` keeps only
the health consumer. For a person check, restart Thor with `target_labels:=person`
and record that selection. Stop each launch with Ctrl-C in its own terminal;
leave platform/sensor services running. Do not run `cleanup.sh` on hardware.

**Intel — RViz view with the SLAM map.** To watch Thor's results over a live
map on the Intel desktop (VNC display `:1`), run hardware exploration in
external mode instead of the observer. Build the full Intel set in a fresh
prefix: the patched `slam_toolbox` (imported and patched by
`tools/check_dependencies --apply --profile core`), `ridgeback_autonomy_hardware`
and the HUD RViz plugin. Then source the Ethernet-only DDS profile so Thor's
results reach the consumers:

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

Intel runs SLAM, Nav2 and RViz. RViz shows the map, both LiDAR scans, the
target rings and the camera overlay:
- **Colour panel:** each of Thor's measurements, drawn on Intel's copy of the
  colour frame it was measured from (matched by source stamp). The newest frame
  is used only when that one has left the 60-frame buffer (about 2 s), and the
  overlay logs a warning when that happens.
- **Depth panel:** Intel's local aligned depth, from the same stamp.
- **Mask panel:** with `mask_gate:=box`, built from the detections message, so
  no image crosses from Thor. With `silhouette`, Thor's `debug/target/mask`,
  on detection frames.

Pass the same `estimators` and `mask_gate` that Thor runs. Without the patched
`slam_toolbox` no map appears and Nav2 never starts.

**Nav2 output reaches the robot's twist mux.** With `backend:=hardware`, RViz
opens a copy of the exploration layout without the *2D Goal Pose* tool, because
one click would otherwise drive the robot even with frontier goals off. Any other
Nav2 client can still send goals, so keep the e-stop engaged until the command
chain is qualified.

**RViz on Intel is throttled so it cannot starve the camera.** Intel's VNC
desktop renders RViz on the CPU (no GPU). At the layout's 30 fps it took a full
core, and the camera contract check failed with 467–567 ms depth gaps. With
`backend:=hardware` the launch therefore:
- redraws at 10 fps;
- runs RViz at `nice -n 19` on the last two CPU cores;
- limits Mesa's software renderer to two threads (`LP_NUM_THREADS=2`).

Then RViz uses about a quarter of a core and the check passes with RViz open.
Viewing RViz from another computer avoids the load entirely.

For later qualified mission integration, exploration supports
`localization_mode:=external localization_required:=true`; keep
`autonomous_motion_enabled:=false` throughout this stationary milestone.
Required missions start paused. In simulation/fake-server qualification, after
healthy advancing input and a resolved previous goal, explicitly resume with:

```bash
ros2 service call /r100_0001/mission/resume std_srvs/srv/Trigger '{}'
```

The service belongs to the frontier owner and is absent in the stationary
observer. Missing health, frozen progress, or a producer restart latches a pause;
recovery does not resume automatically. Read `mission/status` for pending,
rejected, failed, or overdue cancellation. This is application supervision,
not a physical stop guarantee.


## Usage

Start every run from a sourced workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/DyNAMO-Ridgeback
source install/setup.bash
```

The package has 6 top-level launch files. Exploration, the benchmark wrapper,
and manual mapping are the normal human workflows. The localization observer is
a public physical-deployment role; the benchmark environment/config pair are
also public because the sweep supervisor invokes them as separate processes:

- `ridgeback_exploration.launch.py`
- `target_distance_benchmark.launch.py`
- `manual_mapping.launch.py`
- `localization_observer.launch.py`
- `target_benchmark_env.launch.py`
- `target_benchmark_config.launch.py`

The lower-level dispatcher, SLAM, Nav2, and frontier-exploration launches live
under `ridgeback_autonomy/launch/includes/`. Each backend package exposes an
internal `launch/backend.launch.py` adapter with the same normalized contract.

### `ridgeback_exploration.launch.py`

Launches the selected backend, SLAM, Nav2, optional frontier exploration, and
the target-localization stack in sequence:

1. Gazebo or Isaac + Ridgeback spawn, or attach to physical hardware
2. Exploration RViz config
3. `slam_toolbox` (once the scan and filtered-odometry topics publish)
4. Nav2 (once `/map` publishes)
5. The in-repo `frontier_explorer_node` (once the global costmap publishes and
   autonomous motion is enabled)
6. Target-localization nodes: `target_detector_node`, the measurement nodes for the selected `estimators` (`target_pointcloud_measurement_node` and/or `target_mask_measurement_node`), `target_visualization_node`, `target_overlay_node`, and a second `hud_node` for the distance panel

All four distance estimator rows run by default, the same set the benchmark compares — one RViz ring per row, each with its own bearing, plus a wide distance HUD top-right (`hud_target_overlay`) listing the four side by side with the age of each reading. There is no ground truth in exploration, so that panel carries no truth line and no error column. Per-row visibility is an RViz Displays checkbox under `Target Estimates`, one per estimator.

The perception overlay is published on `debug/target/overlay` and shown by the configured RViz Image display.
For an explicit baseline selection, the quick-start wrappers forward the same
launch arguments untouched (the build wrapper forwards to the start wrapper):

```bash
bash start_exploration.sh initial_test_world detector:=owlv2 \
  detection_model:=google/owlv2-base-patch16-ensemble \
  target_labels:="humanoid robot" detection_threshold:=0.55 detector_fps:=10.0
```

`localization_mode:=external` consumes an already-running detector and rejects
local detector overrides. `target_localization_enabled:=false` starts none.

Bringup is **event-driven** (readiness gates), not fixed timers — each stage starts when its prerequisite exists, with a `--timeout` fallback. See [troubleshooting](docs/troubleshooting.md#event-driven-startup).

Available worlds:

| World | Backend | Source | Notes |
|-------|---------|--------|-------|
| `initial_test_world` | Gazebo and Isaac | Repository SDF and converted USD | Default shared multi-room test geometry |
| `depot` | Gazebo | Clearpath `warehouse.sdf`, translated by the adapter | Warehouse/depot geometry |
| `coworking_space` | Gazebo | Clearpath `office.sdf`, translated by the adapter | Multi-room workplace geometry |
| `warehouse` | Isaac | NVIDIA Simple Warehouse | Cropped warehouse environment |
| `warehouse_full` | Isaac | NVIDIA Simple Warehouse | Full warehouse environment |
| `office` | Isaac | NVIDIA Office | Office environment |
| `hospital` | Isaac | NVIDIA Hospital | Hospital environment |

The Gazebo adapter maps `depot` and `coworking_space` to Clearpath's internal
filenames. Direct Gazebo requests for `warehouse` or `office` are rejected so
distinct geometry never reuses an Isaac world name.

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `world` | `initial_test_world` | World to load (see table above) |
| `backend` | value of `sim` (`gz`) | I/O provider: `gz`, `isaac`, or `hardware` |
| `sim` | `gz` | Deprecated compatibility alias for `backend`; accepts `gz` or `isaac` |
| `namespace` | backend-derived | `r100_0001` in simulation; on hardware, read from `<setup_path>/robot.yaml`; an explicit value overrides either default |
| `use_sim_time` | backend-derived | `true` in simulation; `false` on hardware |
| `setup_path` | backend-derived | `~/clearpath/` in simulation; `/etc/clearpath/` on hardware |
| `color_topic` | `sensors/camera_0/color/image` | Shared camera contract's colour image; the hardware camera service publishes the same name |
| `camera_info_topic` | `sensors/camera_0/color/camera_info` | Compatibility override for color-grid camera intrinsics |
| `depth_topic` | `sensors/camera_0/depth/image` | Colour-aligned depth; the same name on every backend |
| `pointcloud_topic` | backend-derived | Simulation organized points; empty on hardware until its layout is validated |
| `exploration_rviz` | `true` | Launch the custom exploration RViz config |
| `target_localization_enabled` | `true` | Launch the target-localization stack |
| `estimators` | backend-derived | `all` in simulation; aligned-depth-only `projective_ranging,euclidean_reconstruction` on hardware |
| `estimate_viz` | `true` | Publish the estimator rings and the wide distance HUD |
| `detector` | `owlv2` | Startup-only implementation selector; only `owlv2` is supported |
| `detection_model` | `google/owlv2-base-patch16-ensemble` | Checkpoint identifier for the selected detector |
| `target_labels` | `humanoid robot` | Comma-separated detection queries |
| `detection_threshold` | `0.55` | Confidence floor in `(0, 1]` |
| `detector_fps` | `10.0` | Upper bound on detection step starts; every selected measurement row inherits this cadence |
| `detector_debug` | `false` | Log detector cadence, superseded frames, and bounded stage timings |
| `depth_source` | `stereoscopic` | Aligned depth source for the mask rows — `stereoscopic` or `monocular` (Depth-Anything V2; its checkpoint must be cached or downloads enabled) |
| `mask_gate` | `box` | Mask front-end — `box` (no segmentation model) or `silhouette` (SlimSAM) |
| `mppi_visualize` | `false` | Publish MPPI trajectory visualization topics (RViz's namespaced `MPPI Optimal` and `MPPI Samples` displays already subscribe to the relative `optimal_trajectory` and `trajectories` topics) |
| `coverage_overlay_enabled` | backend-derived | `true` in simulation; `false` on hardware, which has no packaged truth map |
| `autonomous_motion_enabled` | backend-derived | `true` in simulation; `false` on hardware until the operator explicitly enables frontier goals |
| `start_hardware_platform` | `false` | Hardware only: attach to existing Clearpath services by default; `true` explicitly includes platform bringup |
| `headless_rendering` | `false` | Optional server-only EGL sensor rendering for SSH/non-seat sessions; the existing `tools/gpu-run` NVIDIA GLX workflow remains the default GUI-capable path (see [troubleshooting](docs/troubleshooting.md#camera-rate-collapses-under-software-rendering)) |
| `camera_profile` | `640x480` | Shared nominal D455 render profile for Gazebo and Isaac; `1280x720` is available for higher-resolution runs and requires a simulator restart |
| `depth_fidelity` | `ideal` | Isaac depth path: clean colour-render depth (`ideal`) or nominal D455 stereo artifacts plus explicit colour alignment (`d455`); ignored by Gazebo and hardware |
| `headless` / `livestream` | `true` / `false` | Isaac window and WebRTC controls; ignored by Gazebo |
| `rtf` / `sim_mode` | `1.0` / `realtime` | Isaac timing controls; use `deterministic` for comparisons |
| `odom_noise` | `1.0` | Isaac odometry-drift scale; `0` gives exact debug odometry |
| `noise_seed` | `0` | Isaac-only repeatability: odometry uses N, IMU noise uses N+1; seed 0 preserves the original streams |
| `camera` | `true` | Attach the Isaac D455 renderer; disable for lidar-only performance runs |
| `sensor_hz` | `40.0` | Isaac deterministic step/lidar rate; camera cadence stays at its authored 30 Hz |
| `slam_source` | `front_only` | Identical default for every backend; `merged` is an explicit opt-in |

Examples:

```bash
# Simulation only: clean up stale processes first
bash cleanup.sh

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
```

The hardware adapter is intentionally attach-only by default. It does not start
the Clearpath platform unless `start_hardware_platform:=true`, and the quick-start
wrapper does not run broad process cleanup or force `ROS_DOMAIN_ID=42` on
hardware. With no `namespace` override, hardware reads the robot identity from
`<setup_path>/robot.yaml` (normally `/etc/clearpath/robot.yaml`) and aborts if it
cannot be read; simulation retains `r100_0001`. Before setting
`autonomous_motion_enabled:=true`, validate the deployed
command chain, stop behaviour and speed limits using the
[driving qualification reference](docs/physical/driving_readiness.md); the
checked-in generated configuration is not proof of the live robot contract.
The D455 section of
[PHYSICAL — Robot measurements and stationary sensor validation](docs/plans/PHYSICAL_robot_measurements_and_validation.md#d455-camera-validation)
owns the physical camera acceptance gates. The
[three coordinated physical plans](docs/BACKLOG.md#physical-stationary-integration)
cover revision handoff, Intel–Thor deployment, and robot
measurements; they do not describe an already qualified distributed deployment.

The explorer is the in-repo `frontier_explorer_node` (sources under `src/ridgeback_autonomy/ridgeback_autonomy/frontier_explorer/`). It consumes the Nav2 global costmap and sends goals via `NavigateToPose`. The component boundary and readiness sequence are documented in [exploration architecture](docs/exploration/architecture.md).

#### Simulation parking demonstration

Run one simulator at a time from the checkout. The wrapper sources this
workspace, uses CycloneDDS on loopback-only domain 83, and refuses a concurrent
simulator instead of killing shared workstation processes. Point inference at
an existing perception environment if it is outside this checkout:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv
bash start_parking_demo.sh gz --plan-only
# After stopping Gazebo with Ctrl-C:
bash start_parking_demo.sh isaac --plan-only
```

`--plan-only` is stationary validation: no frontier explorer, an explicit idle
handoff stub, and parking navigation disabled. Both real simulators still run
SLAM, Nav2, collision monitoring, perception, one mission manager and the
unchanged parking RViz layout. It does not demonstrate active exploration
handoff or autonomous parking. Unknown footprints can correctly prevent a
stationary plan in an initially unmapped scene. To validate the unchanged
exploration-to-planning sequence, use `--planning-only`: the simulated robot
may explore, map around G1 and return to its observation pose, but parking
navigation remains disabled. A path-feasible snapshot expires under the
existing freshness rules; it is not a persistent authorization to drive.

The full-mission commands enable the existing simulation execution path:

```bash
bash start_parking_demo.sh gz
# After stopping Gazebo with Ctrl-C:
bash start_parking_demo.sh isaac
```

Full driving remains subject to the separately identified cancellation and
checked-route execution issues; startup validation does not qualify those
behaviors. The existing navigation back to the G1 observation pose can also
time out before planning, including in `--planning-only` mode. No execution
safeguard or motion-settlement threshold is relaxed by this launch change.

The wrapper calls `ridgeback_mission.launch.py` with `demo_scene_enabled:=true`.
The default Gazebo environment is `depot` (native world `warehouse`); Isaac
uses `parking_demo`, a layer over `initial_test_world` that disables its distant
authored G1. Startup creates exactly one demo G1 at `(3, 0)` and the existing
table at `(3, 1.05)` through `/gzserver` or `/isaac_sim` standard scene services.
Creation failures or existing G1 entities abort startup rather than renaming
or deleting an existing target. Manual `gz service` calls and the removed
`benchmarking.simulation.spawn_model_command` helper are not needed.

The mission explicitly selects this checkout's `clearpath/robot.yaml`; an
explicit `setup_path:=...` remains supported, but Isaac rejects a sensor rig
different from its committed USD. Manager odometry defaults to
`platform/odom/filtered` on Gazebo and `platform/odom` on Isaac, with
`odometry_topic:=...` available as an override. Nav2 retains filtered odometry.
Use the normal pinned dependency installation, including the maintained
Clearpath sensor and namespaced SLAM patches; an old system-only generator is
not the repository's sensor contract.

Startup waits for advancing clock, scene services, verified spawns, fresh
LiDAR/odometry/perception messages, sensor/map TF, active Nav2 lifecycle nodes
(including collision monitor), and MissionGuard interfaces. In a full demo,
the existing guard starts latched. The one manager saves its start pose before
startup resumes exploration and admits detections. `startup_timeout_s:=300.0`
bounds each startup gate; failure shuts down the launch's processes. Ctrl-C
stops the owned stack. Additional `name:=value` arguments are forwarded, for
example `headless:=true` for Isaac without its viewport; RViz remains enabled.

#### Stationary driving readiness checks

These commands collect graph/parameter evidence and test a separate mux;
they do not send commands to the robot or restart its services. Run on Intel
from the workspace root. Neither establishes physical driving readiness.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="$PWD/artifacts/hardware/driving-readiness/ros"
python3 tools/hardware/audit_driving_readiness.py \
  --output "$PWD/artifacts/hardware/driving-readiness/audit.json"
# Synthetic inputs confined to loopback, domain 213 and a private namespace:
python3 tools/hardware/check_driving_mux.py
```

The auditor deliberately selects deployed domain 0 and
`/etc/clearpath/cyclonedds.xml`; its output labels missing evidence. Physical
e-stop, low-speed, timeout and deadman checks require the supervised procedure
in the [driving reference](docs/physical/driving_readiness.md#remaining-physical-verification).

To apply the fixed commissioning caps, keep the robot stationary with the
physical stop engaged and an operator beside it. The installer observes that
state before restarting only the platform and new guard; it sends no motion
commands and requests no camera, sensor or desktop restarts. Choose a fresh
evidence directory:

```bash
sudo python3 tools/hardware/install_commissioning_caps.py \
  --apply --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/20261006-commissioning-caps-install"
```

It limits all mux inputs to a planar magnitude of 0.05 m/s and yaw of 0.1 rad/s.
See the [guard contract](docs/physical/driving_readiness.md#commissioning-caps).
An unsuccessful install leaves the platform stopped. An explicitly requested
rollback restores only the saved files; with the physical stop still engaged,
use the same evidence directory with `--restore --operator-confirmed` instead
of `--apply`. For a first installation this removes the caps and is not
permission to drive.

To upgrade an existing verified capped installation with the e-stop
QoS repair, retain that prior install directory and keep the physical stop
engaged, controller controls centred and an operator present:

```bash
sudo python3 tools/hardware/install_commissioning_caps.py \
  --apply --operator-confirmed \
  --upgrade-from-run-dir "$PWD/artifacts/hardware/20261006-commissioning-caps-install" \
  --run-dir "$PWD/artifacts/hardware/20261009-stop-qos-install"
```

The upgrade requires exact prior installed file hashes, replaces only the same
four owned files and verifies the reliable stop relay/mux remap as well as the
capped controller. It restarts only the platform and guard. Use the new evidence
directory for an explicitly requested `--restore --operator-confirmed`; an
upgrade rollback restores the previous capped installation. An original first
installation rollback removes caps. The
[stop-delivery reference](docs/physical/driving_readiness.md#e-stop-delivery-to-the-mux)
records deployment state. The repair was installed and verified stationary on
`r100_0160` on 2026-10-09; physical disconnection qualification remains open.
For another upgrade, choose a fresh output directory and the record matching
the current installed files.

A temporary commissioning-test lock can be staged by adding `--test-interlock`
to the caps installer. In that mode ordinary driving is blocked unless a fresh
bounded test permit is present; it is not normal manual-driving mode. Stage it
with a fresh evidence directory and the exact currently installed receipt:

```bash
python3 tools/hardware/install_commissioning_caps.py --test-interlock \
  --upgrade-from-run-dir "$PWD/artifacts/hardware/20261009-stop-qos-install" \
  --run-dir "$PWD/artifacts/hardware/test-interlock-stage"
```

For operator-confirmed stationary installation with the physical stop pressed,
add `--apply --operator-confirmed` and use another fresh output directory.
It restarts only the platform and guard. Use `observe_driving_stop.py --test stationary --test-interlock` with a fresh `--run-dir` to inspect it afterward.
Restore the previous capped installation using the new installation receipt:
`sudo python3 tools/hardware/install_commissioning_caps.py --restore
--operator-confirmed --run-dir <new-install-directory>`, keeping the physical
stop pressed. Moving interlock trials are not yet enabled. See the
[test-lock contract](docs/physical/driving_readiness.md#temporary-commissioning-test-interlock).

To observe a held joystick command against the installed software test lock,
start with the physical stop pressed, controller neutral, clear floor and an
operator beside the reachable stop:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-held --test-interlock --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/interlock-held-check"
```

After READY, release only the physical stop with controls centred. Wait for
PHYSICAL RELEASE RECORDED, then gently hold driving-enable and forward for at
most one second. Release the controls and press the physical stop again. The
robot should stay still throughout. Press the stop immediately for unexpected
movement. The observer requires fresh asserted software-lock reports, no test
permit publishers, zero mux/limited/wheel commands and stationary raw odometry.
It sends neither commands nor permits; successful recording needs analysis and
operator confirmation. Moving-disconnect testing remains disabled.

A separately authorized bounded-permit trial uses the explicitly interlocked
installation. It allows one gentle forward request at the existing 0.05 m/s cap:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-permit --test-interlock --operator-confirmed \
  --permit-motion-confirmed \
  --run-dir "$PWD/artifacts/hardware/interlock-permit-check"
```

For a separately approved **backward** trial, add `--direction backward` to
that `interlock-permit` command and use a fresh run directory. Keep the physical
stop pressed until `RECORDING READY`, release with controls centred, then use
backward input only after `PHYSICAL RELEASE RECORDED`. At the stop prompt,
release controls and press the physical stop. This selects the recorder's
expected direction; it does not change the installed one-second/0.05 m/s guard.
Other fault/disconnect modes remain forward-only.

For a **stationary right-stick mapping check**, use `--test stationary
--test-interlock --operator-confirmed --mapping-axis 3` with a fresh `--run-dir`.
Keep the physical stop pressed and driving-enable released throughout. At
`MAPPING READY`, move only the right thumbstick left for one second and let go.
The recorder waits up to 60 seconds, prints when input is received, then requires
one second of neutral input. If driving-enable is held, it waits for its release
and ignores that input rather than immediately aborting. Releasing the physical
e-stop or observing nonzero wheel commands/motion still aborts immediately.
It records the observed axes without issuing
commands or permits; a mapping mismatch is not permission to try movement.

The same plain permit trial supports `--direction left`, `right`, `turn-left`
or `turn-right`, one separately approved trial at a time with a fresh evidence
directory. On the inspected controller mapping, **right-stick horizontal moves
sideways**, while **left-stick horizontal turns**; keep the other stick centred.
Planar commands remain capped at 0.05 m/s and yaw at 0.1 rad/s. Follow the
selected-direction prompts and press the physical stop immediately if movement
does not match the requested direction.

Start with the physical stop pressed, controller neutral and an operator beside
the robot with clear floor and reachable stop. After READY, release only the
physical stop; wait for PHYSICAL RELEASE RECORDED before gently holding forward
and driving-enable. Hold until the stop prompt, about 1.25 seconds after first
input. The guard must block commands after at most one second even while permit
heartbeats continue briefly. At the prompt, release controls and press the
physical stop. Press the stop immediately for unexpected movement, or if the
robot is still moving at the prompt. Do one movement. The observer publishes
only permits, retains their transmit times, cancels when controller input is
released, and rejects stale observations or unexpected command/odometry bounds.
Controller neutrality follows the deployed driving mapping (axes 0, 1, 3, 4),
with both driving-enable buttons released; unused axes may rest at nonzero values.
The forward trial accepts up to 0.005 rad/s command yaw for small stick drift;
its installed speed caps and wheel-feedback abort bounds are unchanged.
It cannot physically inhibit the motors itself; rely on the reachable stop for
unexpected behavior. Review source input, permit receipts/lock state, wheel
commands and actual operator observations before accepting this gate. This mode
does not disconnect the controller or enable the disabled moving-disconnect
workflow.

For a **wheels-raised** check of the installed feedback plugin's response to
an invalid ROS feedback batch while the motor command path stays running, use
the same one-shot interlock. A qualified helper must first secure all four
wheels clear of the floor on rated stands. Keep an operator at the reachable
physical e-stop. This check does not interrupt electrical CAN or measure
loaded stopping distance:

```bash
cd /home/robot/DyNAMO-Ridgeback
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/observe_driving_stop.py \
  --test interlock-feedback-fault --test-interlock --operator-confirmed \
  --permit-motion-confirmed --feedback-fault-confirmed \
  --run-dir "$PWD/artifacts/hardware/interlock-feedback-fault"
```

Begin with the e-stop pressed and controller neutral. After READY, release the
stop, wait for PHYSICAL RELEASE RECORDED, then gently hold driving-enable and
forward until the stop prompt. After measured forward wheel motion, the
recorder publishes one deliberately empty feedback batch. Observe whether the
wheels stop by themselves before pressing the physical stop at the prompt.
Press the stop immediately for any unexpected motion. The result is evidence
to analyze, never an automatic pass; restore the controller with a supervised
platform restart and stationary recovery verification while the stop is
pressed. Use a fresh directory for each run.

After bounded-permit stopping is qualified, a separately authorized moving
controller-disconnect trial uses `--test interlock-disconnect`,
`--test-interlock`, `--operator-confirmed`, `--permit-motion-confirmed`,
`--moving-disconnect-confirmed` and `--disconnect-device <controller-MAC>` with a
fresh `--run-dir`. Start with the physical stop pressed and controller neutral.
The recorder checks the installed interlock and connected device before READY.
After physical release is recorded, gently hold forward and driving-enable.
It requests one device-specific Bluetooth disconnect half a second after the
first nonzero mux command. Keep controls held until disconnect, then release
them. At the stop prompt, press the physical stop; press it immediately for
unexpected movement or continued motion. Do one movement and do not reconnect.
Permits continue briefly after input reset so source-loss stopping can be
identified before the independent one-second guard budget expires. A stop only
at that budget boundary does not qualify disconnect handling. Review driver
logs, transport status, input reset, wheel commands and physical observations.
The device-specific client runs with closed stdin and a one-second CLI timeout;
its exit status is supporting evidence, not proof of actual link loss.
The old un-interlocked `--test disconnect --disconnect-device ...` remains
blocked. This deliberate disconnect does not simulate silent radio loss.

For a supervised first forward movement, use the fixed pulse helper only after
an operator confirms a clear level floor, nobody in the path and the physical
stop within reach. Leave joystick/RC controls untouched. First run its default
passive mode with the stop engaged and a fresh evidence directory:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/commissioning_forward_pulse.py \
  --run-dir "$PWD/artifacts/hardware/forward-pulse-passive"
```

After that succeeds, the operator may release the stop using the normal
physical procedure and explicitly approve one forward test. Run:

```bash
python3 tools/hardware/commissioning_forward_pulse.py \
  --execute --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/forward-pulse-live"
```

After five seconds of passive checks, it requests 0.05 m/s forward for at most
one second through the external mux input, then sends zeros. It records the
command path, raw odometry and stop state, aborts on missing feedback or
unexpected commands, and never resets/enables the base or restarts services.
The nominal displacement is 5 cm; actual travel and stopping are unqualified.
Press the physical stop for unexpected movement, re-engage it after the test,
and record the observed direction and stop separately from odometry.

The same helper accepts `--axis backward|left|right|turn-left|turn-right` for
one separately approved direction at a time. Turning peaks at 0.1 rad/s.
Optional `--ramp` adds 250 ms rise/fall within the one-second pulse; the peak
and abort limits remain fixed. Do not repeat a motion-abort merely to obtain
a pass: preserve its recording and examine
[wheel feedback](docs/physical/drive_feedback_diagnostics.md) first.

To measure stopping after command publication disappears, use a separately
approved forward pulse with `--ramp --stop-mode timeout`. It ramps up, ends
publication at the peak after one second, observes silence for one second,
and then sends cleanup zeros. This qualifies publishing loss separately from
an explicit zero; it does not simulate physical joystick disconnection.

Physical controller checks use a separate observer that publishes no commands.
Run it in the operator's terminal with the physical stop initially engaged:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/observe_driving_stop.py \
  --test stationary --run-dir "$PWD/artifacts/hardware/drive-stop-stationary"
```

Only after separately approving the physical action, choose `--test deadman`,
`--test disconnect` or `--test e-stop`, add `--operator-confirmed`, and use a
fresh directory. The observer checks the engaged stop and capped routing,
then prints the action instructions. It waits up to 60 seconds for the first
nonzero mux command and records ten more seconds after that command, so a late
start does not truncate the stopping observation. Gently request forward with
the joystick for at most one second and perform the selected stop action; keep
a clear floor and reachable stop. For deadman release, release the button first,
then centre the stick: centring alone tests a zero command, not deadman release. It cannot stop
the robot itself. Re-engage the physical stop afterward; reconnect a controller
only with that stop engaged. Record what physically happened separately from
the captured command, wheel and stop-state data. A completed recording is not
an automatic readiness pass.


For neutral physical stop release, use `--test e-stop-release` with
`--operator-confirmed` and a fresh evidence directory. Start with the stop
pressed, both driving-enable buttons released and all sticks centred. After
RECORDING READY, release only the physical stop and leave the controller
untouched. The observer records ten seconds after the released firmware report,
then prompts you to press the stop again and waits for two asserted reports.
It aborts on nonzero commands, wheel movement, non-neutral/stale controller
input, stale stop/odometry observations, or early stop re-engagement. Press the
physical stop immediately for unexpected movement: the observer cannot stop
the robot. A successful recording still requires analysis and operator
confirmation. This checks neutral release; deliberate held-input release and
wireless reconnection are separate tests.

For a wireless controller, first check disconnect propagation while the physical
e-stop stays pressed:

```bash
python3 tools/hardware/observe_driving_stop.py \
  --test disconnect-stationary --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/drive-disconnect-stationary"
```

Keep the physical stop engaged for the entire check. When ready, hold the deadman
and a small forward stick input, switch off/disconnect the controller while
holding that input, then release its controls. The observer waits up to 60 seconds
for joystick input and records 20 more seconds. It aborts if the reported stop
becomes released or a nonzero mux command appears. Reconnect afterward with the
stop engaged and controls centred. This checks stale-input propagation without
qualifying physical stopping during a moving disconnection trial.

For a reproducible stationary device-specific disconnect, add
`--disconnect-device <controller-MAC>` to the command above. Reconnect the
controller first with the physical stop pressed and controls neutral. At READY,
hold driving-enable and small forward input until the recorder disconnects the
selected device, then release controls. Keep the physical stop pressed; do not
reconnect until analysis. Automatic moving disconnection is disabled pending a preflight motion interlock.
Do not run the moving variant. A failed trial never armed because it received
only released stop reports during startup; commands appeared before READY, so
no disconnect request was scheduled. The observer now aborts immediately on
commands or wheel motion during preflight, but cannot itself inhibit the robot.
Stationary automatic disconnect remains available with the physical stop held.
The PS+Triangle binding in the captured config belongs to
an optional keyboard/mouse profile and is not enabled for the driving profile.

#### Quick-start script

`start_exploration.sh` sources the workspace and launches exploration. For
simulation it first runs the simulator cleanup and defaults `ROS_DOMAIN_ID` to
42. For `backend:=hardware` it preserves existing ROS processes and the caller's
domain. It accepts an optional world as the first positional argument and
forwards later `key:=value` launch arguments:

```bash
bash start_exploration.sh                                # initial_test_world
bash start_exploration.sh office backend:=isaac          # Isaac office world
bash start_exploration.sh headless_rendering:=true       # default world, EGL server rendering
bash start_exploration.sh office backend:=isaac estimators:=pointcloud
bash start_exploration.sh initial_test_world backend:=isaac sim_mode:=deterministic
bash start_exploration.sh backend:=hardware autonomous_motion_enabled:=false base_frame:="${BASE_FRAME:?Set the observed base TF frame}"
RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh office backend:=isaac
```

The script defaults to CycloneDDS, selected by the exact-stamp image-delivery
benchmark. An explicit `RMW_IMPLEMENTATION` remains authoritative. The rejected
UDP-only FastDDS experiment is retained in the archived evidence below.

Exploration and benchmark entrypoints also point `CYCLONEDDS_URI` at
`config/cyclonedds.xml`, which raises the participant-index ceiling. The full
exploration process set exceeds Cyclone's default and can abort `slam_toolbox`,
the Nav2 stack, and the explorer — see
[troubleshooting](docs/troubleshooting.md#cyclonedds-participant-ceiling).
Setting `CYCLONEDDS_URI` yourself overrides it; the launch files only fill in a
value when none exists.

`build_and_start_expl.sh` rebuilds the workspace first, then runs the same exploration quick-start (extra args are forwarded to `start_exploration.sh`):

```bash
bash build_and_start_expl.sh
bash build_and_start_expl.sh office backend:=isaac
bash build_and_start_expl.sh office backend:=isaac headless:=true
```

### Hardware camera service

On the robot, `dynamo-camera.service` owns the D455 from boot and publishes the
camera topics, frames and TF the simulators publish, so the exploration launch
attaches to it exactly as it does to the LiDARs. The vendor camera units and its
web camera view are retired. Install it after `intel_services_rmw apply`, with the
robot stationary and the e-stop in reach: `rollback`, and an `apply` that
changes the camera declaration, restart `clearpath-robot`, which drops motor
power for about a minute while the platform stops. A re-apply that changes only
the profile, rate or DDS tuning restarts the camera alone. Let them finish; each
step is printed.

```bash
tools/intel_thor/camera_service status
```

```bash
sudo tools/intel_thor/camera_service apply --profile 640x480
```

Check the running camera against the contract from a shell with the robot's
environment:

```bash
source /etc/clearpath/setup.bash && tools/camera_contract_check --backend hardware
```

```bash
sudo tools/intel_thor/camera_service rollback
```

`apply` is idempotent; re-run it with `--profile 1280x720` to change the
resolution, or after editing the driver configuration in the repository. The
installed files, the re-apply order after a reinstall, and the contract that
robot-side diagnostics should target are in
[robot-local deployment](docs/physical/robot_local_deployment.md).

### Intel-Thor sensor latency

Use the [sensor timing reference](docs/physical/intel_thor_sensor_latency.md)
for report definitions and the [transport reference](docs/physical/intel_thor_transport.md#sensor-timing-and-time-synchronization)
for the selected configuration. Tools run directly from source; no workspace build is needed.

Publish the latency branch through `origin` before qualifying it. On Thor,
leave the existing checkout untouched and create the separate worktree once:

```bash
git -C ~/DyNAMO-Ridgeback fetch origin feat/intel-thor-sensor-latency
git -C ~/DyNAMO-Ridgeback worktree add --track -b feat/intel-thor-sensor-latency ~/DyNAMO-Ridgeback-latency origin/feat/intel-thor-sensor-latency
```

For updates, fetch on the existing Thor checkout and run
`git -C ~/DyNAMO-Ridgeback-latency pull --ff-only`. The operator installs clocks
locally on each host while the robot is stationary:

```bash
# Intel, from this checkout
sudo tools/intel_thor/time_sync apply --host intel
# Thor, from ~/DyNAMO-Ridgeback-latency
sudo tools/intel_thor/time_sync apply --host thor
chronyc tracking
```

`tools/intel_thor/time_sync status` inspects the deployment;
`sudo tools/intel_thor/time_sync rollback` restores the prior time service.
Check the independent bound on Intel with `tools/intel_thor/check_clock`.

chrony alone serves Thor an estimate of true time rather than Intel's wall clock,
so the two clocks agree only to within Intel's pending internet correction. PTP
carries the wall clock itself. With the robot stationary, re-apply `time_sync`
on Intel first (it installs the slow slew rate PTP requires), then install PTP
on each host:

```bash
# Intel, from this checkout
sudo tools/intel_thor/time_sync apply --host intel
sudo tools/intel_thor/ptp_sync apply --host intel
# Thor, from ~/DyNAMO-Ridgeback-latency (stops Thor's chrony)
sudo tools/intel_thor/ptp_sync apply --host thor
```

`tools/intel_thor/ptp_sync status` shows each host's daemons and their latest
offsets; `sudo tools/intel_thor/ptp_sync rollback` removes PTP and restores
chrony on Thor. Confirm the wall-clock agreement from Intel with
`tools/intel_thor/check_clock`.

Then collect the interleaved baseline from Intel:

```bash
tools/intel_thor/check_link
source /etc/clearpath/setup.bash
tools/intel_thor/sensor_latency_run --namespace r100_0160
```

The collector writes reports, raw timestamps, before/after clock checks, CPU
and link rates under `artifacts/hardware/`. Use `--output-dir` to name a new
run directory. Its Intel observer stops the run on scan gaps or sensor faults.

For a short Thor contract diagnostic, from the Thor worktree:

```bash
source /opt/ros/jazzy/setup.bash
source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh thor
export ROS_DOMAIN_ID=0
tools/camera_contract_check --backend hardware --host thor --namespace r100_0160 --timing --duration 10
```

Cross-host components are unresolved until a valid `--clock-offset-ms` and
`--clock-bound-ms` are supplied. The collector handles those checks automatically.
To measure O1/O2, the operator applies one candidate on Intel, then repeats the
collector with a new output directory:

```bash
sudo tools/intel_thor/camera_service apply --dds-profile camera --fps 30 --max-message-size default
# O2: change only --max-message-size to 65500
```

Return to the installed baseline with `camera_service apply --dds-profile
services --fps 30 --max-message-size default`.

Compression candidates (O4) decode on Thor and need its image_transport
plugins once, from the operator:

```bash
sudo apt install ros-jazzy-image-transport-plugins ros-jazzy-ffmpeg-image-transport
```

Then, on Intel, choose one variant per stream. Encoder parameters set with
`--camera-param` apply to the whole run and are restored afterwards:

```bash
tools/intel_thor/sensor_latency_run --namespace r100_0160 --color-transport ffmpeg --depth-transport zstd
tools/intel_thor/sensor_latency_run --namespace r100_0160 --color-transport compressed --depth-transport compressedDepth --camera-param .camera_0.aligned_depth_to_color.image_raw.compressedDepth.format=rvl
```

The deployment runs at 30 Hz; reduced-rate runs are not a tuning option. Compression and host tuning are
selected only after the measured baseline and raw transport candidates.

### Intel-Thor link setup and pre-deploy check

**Hardware qualification remains open:** the September 18 handoffs report camera
delivery failures and LiDAR lockouts with unrestricted multicast. October 5
focused native tests passed with discovery-only multicast, which is now retained
on Intel. Follow the
[transport blocker and recovery procedure](docs/physical/intel_thor_transport.md#camera-subscriber-blocker)
for deployment state and remaining gates. These local tests do not qualify
aligned streams, a soak, distributed localization or motion. The synthetic
check below does not cover those gates.

Use the [guarded hardware workflow](docs/physical/intel_dds_testing.md) to repair
Intel desktop ownership/camera readiness or verify the retained DDS profile with
`--verify-current` without changing it. Desktop repair needs one reconnect;
current-profile verification needs no service reload.

The distributed deployment runs ROS 2 on CycloneDDS on both the Intel PC and
the Jetson Thor. Traffic between them uses the robot Ethernet only. The
mechanism, the configuration files, and the reasons are in the
[Intel–Thor DDS transport reference](docs/physical/intel_thor_transport.md).

One-time setup, on each host from its checkout (Thor also needs
`sudo apt install ros-jazzy-rmw-cyclonedds-cpp`):

```bash
sudo cp src/ridgeback_autonomy_hardware/config/intel_thor/60-dynamo-dds-buffers.conf /etc/sysctl.d/
sudo sysctl --system
```

One-time switch of Intel's robot services from Fast DDS to CycloneDDS. The
services restart, so keep the robot stationary with the e-stop in reach:

```bash
sudo tools/intel_thor/intel_services_rmw apply
```

Both hosts run at full CPU performance, and Thor in its 120W power mode;
`check_link` fails otherwise. Do not select MAXN: on 2026-10-09 the Ridgeback's
user power ports stopped supplying Thor under a heavy GPU load in MAXN. Install
once per host (Thor from its worktree; a power-mode change that needs a reboot
is reported):

```bash
sudo tools/intel_thor/host_performance apply --host intel
sudo tools/intel_thor/host_performance apply --host thor
```

`tools/intel_thor/host_performance status` reports drift; `rollback` restores
the previous governors and power mode.

Thor must reach Intel's Wi-Fi address over the cable, or transient-local data
such as `tf_static` never arrives (see
[troubleshooting](docs/troubleshooting.md#thor-receives-sensor-streams-but-no-static-tf)).
Once, on Thor, from its worktree:

```bash
sudo tools/intel_thor/thor_lan_route apply
```

`tools/intel_thor/intel_services_rmw status` shows the RMW that each service
actually runs, and `sudo tools/intel_thor/intel_services_rmw rollback` restores
the saved middleware, DDS configuration, and service overrides. Backups from
older versions that saved only `robot.yaml` require manual recovery; see the
[rollback contract](docs/physical/intel_thor_transport.md#intel-service-switch).

In every shell that starts cross-host ROS processes or inspects Thor's topics,
on both hosts:

```bash
source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh
```

An unknown host or invalid role returns nonzero without changing the DDS
environment. Supply `intel` or `thor` explicitly when automatic selection is
unavailable, and require successful setup before launching ROS processes.

Before every deployment, with the robot stationary, run on Intel:

```bash
tools/intel_thor/check_link
```

It checks routing, link speed, buffer limits, the middleware of Intel's
services, the configurations, clock offset, and a 30 Hz camera-sized stream in
both directions. It exits nonzero on any failure. Pass `--no-traffic` to skip
the stream when the Ethernet segment must stay quiet.
`tools/intel_thor/rmw_benchmark` repeats the Fast DDS/CycloneDDS comparison.

For an explicitly supervised comparison of the **installed Intel services**
configuration, use the [guarded local DDS procedure](docs/physical/intel_dds_testing.md).
That procedure also separates VNC from the DDS webserver so subsequent service
reloads can keep the operator's desktop connected.
It compares current settings with discovery-only multicast using local camera
readers, scan watchdogs, and packet-header captures. It requires a stationary
robot, an operator present, and sudo for capture and service reloads; its report
does not establish distributed localization or physical motion qualification.

After the focused repeat passes, the procedure's explicit
`--focused-native --retain-passing-native` option keeps discovery-only multicast
on Intel after final verification. It restores the original profile on failure;
alignment, soak and distributed qualification remain separate gates.

### `target_distance_benchmark.launch.py`

Before parameter tuning or using new benchmark comparisons as reportable results,
complete the [execution-environment qualification gate](docs/target_distance_benchmarking/BACKLOG.md#benchmark-execution-environment-qualification).
Qualification and diagnostic runs establish that evidence; the CLI does not
block unqualified runs. Replay tuning also depends on qualified capture provenance.

`backend:=gz` (the default) runs the benchmark on Gazebo and needs
`dependencies/gz.repos`. `backend:=isaac` runs the same scenario file, runner,
estimators, and reports on Isaac Sim 6.1. It needs the
[Isaac Sim setup](#4-optional-set-up-isaac-sim-61), and must start from the
workspace root, where it finds `isaac_venv` (or set `ISAAC_PYTHON`). The backend
belongs to the persistent environment. It selects the simulator adapter, that
adapter's asset catalog, and its scene-control services, and every `run.json`
records it. `backend:=hardware` is refused before any process starts, because
hardware has no scene-control services or ground truth. What may and may not be
compared across the two simulators is in
[benchmark semantics](docs/target_distance_benchmarking/semantics.md#scenario-and-comparison-discipline).

The single-run entrypoint owns separate configuration and environment processes.
Ctrl-C finishes configuration cleanup while the simulator still answers, then
stops the environment. The default `shutdown_on_complete:=false` retains both
layers for inspection after the runner finishes; `true` closes them in order.
See the [shutdown contract](docs/target_distance_benchmarking/overview.md#shutdown-lifecycle).

```bash
# Default run: compare all four estimators on Gazebo
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py

# The same run on Isaac Sim, with its viewport (from the workspace root)
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py backend:=isaac headless:=false

# The same production baseline, with the detector settings explicit
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  detector:=owlv2 detection_model:=google/owlv2-base-patch16-ensemble \
  target_labels:="humanoid robot" detection_threshold:=0.55 detector_fps:=10.0

# Point cloud only
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=pointcloud

# Point cloud + the mask reference row
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=pointcloud,projective_ranging

# Mask-based localization rows, stereoscopic aligned depth
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction,polar_profiling

# Same rows on monocular (Depth-Anything) aligned depth — compare depth
# sources by running the benchmark twice, once per depth_source
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction depth_source:=monocular

# Silhouette mask front-end (segmentation model) instead of the rasterized
# box — compare gates by running the benchmark twice, once per mask_gate
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction,polar_profiling mask_gate:=silhouette

# One mask row on its own — polar profiling needs no depth at all, so this run
# builds no depth source and subscribes to no depth stream
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=polar_profiling
```

This compatibility launch composes two layers. `target_benchmark_env.launch.py` owns the simulator, camera TF, RViz, and `target_detector_node`; `target_benchmark_config.launch.py` waits for the color and warm-detector topics, then starts the selected measurement nodes, visualizations, HUD, overlay, and `target_distance_benchmark_runner`. Every row is individually selectable: `estimators` is split per stack and each measurement node is passed only the rows it owns, so a path that was not selected is never run — its fields stay NaN, it gets no CSV, and the inputs only it needs are never subscribed to. Selecting any mask row launches `target_mask_measurement_node`, which for `projective_ranging`/`euclidean_reconstruction` obtains the aligned depth frame itself, at the detection stamp, through the source `depth_source` selects; `polar_profiling` needs no depth at all, so a polar-only run builds no depth source (and under `depth_source:=monocular`, loads no model). The mask node builds one mask per detection (`mask_gate:=box` rasterizes the detection box; `mask_gate:=silhouette` prompts a segmentation model with the boxes, needs `perception_venv`) and runs the localization paths documented under [`docs/target_localization/`](docs/target_localization/target_localization_pipeline.md).

The mask rows are identified by their config axes — the mask gate (`mask_gate`), the aligned-depth source (`depth_source`), the path, and on the box gate the foreground-isolation recipe (`isolation_2d` for projective ranging, `isolation_3d` for euclidean reconstruction) — so their CSV files fold the axes into a self-describing name. The stereoscopic box-gate run above writes `box_gated_stereoscopic_projective_ranging_nearest_mode_histogram.csv` and `box_gated_stereoscopic_euclidean_reconstruction_height_crop_nearest_mode_band.csv`; silhouette rows drop the isolation token (the tight branches never run a recipe), e.g. `silhouette_gated_stereoscopic_projective_ranging.csv`; polar profiling folds the gate only (`box_gated_polar_profiling.csv`). The `pointcloud` row keeps its plain name.

Each benchmark run writes under
`artifacts/benchmarks/<timestamp>_<scenario>[_<gate>][_<depth_source>]/` by default
(e.g. `20260822_202851_v2_silhouette_stereoscopic`):
- `summary.md` — the readable report; start here
- one trial-level CSV per selected estimator
- one `run.json` — the run-level numbers for machines, plus provenance
- one shared collage image per included trial under `images/`
- one best-effort RViz recording at `video/run.mp4`

Pass `output_dir:=...` to write the timestamped run folder somewhere else.

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `backend` | `gz` | Simulator of the persistent environment: `gz` or `isaac`. Selects the adapter, its asset catalog and scene-control services, and is recorded in every run. In a sweep it is a `defaults` key |
| `namespace` | `r100_0001` | Namespace for simulation, detector, measurement node, and benchmark runner |
| `use_sim_time` | `true` | Use the simulator's `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `world` | `target_distance_calibration` | Public benchmark world. Each backend's asset catalog maps it to that simulator's own world (Isaac: `g1_distance_calibration`, same geometry) |
| `gz_gui` | `true` | Gazebo only: launch Gazebo's GUI; set `false` for unattended benchmark runs |
| `headless` | `true` | Isaac only: run Isaac Sim without its viewport window |
| `rtf` | `1.0` | Isaac only: real-time factor; `0` is unthrottled |
| `sim_mode` | `realtime` | Isaac only: `realtime` or fixed-step `deterministic` stepping |
| `depth_fidelity` | `ideal` | Isaac only: `ideal` renderer depth or D455-like `d455` stereo depth; see [camera depth](docs/isaac/camera-depth.md) |
| `exploration_rviz` | `true` | Launch the persistent benchmark RViz window |
| `estimate_viz` | `true` | Publish estimator rings and the benchmark distance HUD |
| `overlay` | `true` | Publish the camera overlay with selected estimator panels |
| `estimators` | `all` | Comma-separated estimator subset to compare in one run: `pointcloud`, `projective_ranging`, `euclidean_reconstruction`, `polar_profiling`. Any subset works, rows individually — `estimators:=polar_profiling` runs that row alone |
| `scenario` | empty | Scenario YAML; empty uses the packaged full scenario set |
| `depth_source` | `stereoscopic` | Aligned-depth producer for the `projective_ranging`/`euclidean_reconstruction` rows: `stereoscopic` or `monocular`; comparing sources = two runs |
| `mask_gate` | `box` | Mask front-end for the mask-based rows: `box` (rasterized detection box, no model) or `silhouette` (segmentation model prompted with the boxes; needs `perception_venv`); comparing gates = two runs |
| `depth_match_debug` | `false` | Default-off mask-worker evidence logging: exact-depth delivery, pending replacements, batch completion/publication age, and bounded cold/warm percentiles for RGB, SlimSAM, depth production, mask preparation, estimator reduction, worker, lock, and CUDA-synchronization timing |
| `detector` | `owlv2` | Startup-only detector implementation; only `owlv2` is registered. Persistent environment `defaults` in a sweep |
| `detection_model` | `google/owlv2-base-patch16-ensemble` | Checkpoint identifier, distinct from `detector`; persistent environment `defaults` in a sweep |
| `target_labels` | `humanoid robot` | Comma-separated detection queries; persistent environment `defaults` in a sweep |
| `detection_threshold` | `0.55` | Finite confidence floor in `(0, 1]`; persistent environment `defaults` in a sweep |
| `detector_fps` | `10.0` | Upper bound on the detection rate every measurement row inherits. Belongs to the persistent environment layer, so in a sweep it is a `defaults` key and cannot vary per configuration |
| `detector_debug` | `false` | Default-off detector evidence logging: achieved cadence, superseded frames, and bounded cold/warm percentiles for the throttle wait, decode, inference, parse, publish and CUDA synchronization. Environment-layer, like `detector_fps` |
| `headless_rendering` | `false` | Gazebo only: optional default-off server-only EGL sensor rendering. In sweeps this is a persistent-environment `defaults` key and cannot vary per configuration; `tools/gpu-run` remains the GUI-capable NVIDIA GLX path |
| `camera_profile` | `640x480` | Simulator camera profile (`640x480` or `1280x720`). It is a persistent-environment `defaults` key, is recorded in each run, and cannot vary per configuration without restarting the simulator |
| `isolation_2d` | `nearest_mode_histogram` | Projective-ranging box-gate foreground recipe: `nearest_mode_histogram` or `otsu` |
| `isolation_3d` | `height_crop_nearest_mode_band` | Euclidean-reconstruction box-gate foreground recipe: `height_crop_nearest_mode_band`, `height_crop_range_band`, `height_crop`, `nearest_mode_band`, or `range_band`. The two chains differ only in how the background separator anchors — nearest mode vs. percentile; the percentile one slides as background grows and is what the `pointcloud` row does |
| `isolation_3d_floor_margin_m` | `0.05` | Height above the floor a point must clear in Euclidean recipes containing `height_crop` |
| `isolation_3d_percentile` | `25.0` | Range percentile used as the anchor by the Euclidean `range_band` step |
| `isolation_3d_ahead_m` | `0.10` | Euclidean inlier window retained in front of its range anchor |
| `isolation_3d_behind_m` | `0.35` | Euclidean inlier window retained behind its range anchor |
| `isolation_3d_bin_width_m` | `0.05` | Euclidean range-histogram bin width used by `nearest_mode_band` |
| `isolation_3d_min_bin_fraction` | `0.05` | Fraction of Euclidean ranges a histogram bin must hold before it may anchor `nearest_mode_band` |
| `mask_depth_max_meters` | `0.0` | Working depth gate for the mask rows; `0` means no gate, leaving each row bounded only by what its depth source declares it can resolve |
| `isolation_2d_bin_width_m` | `0.05` | Depth-histogram bin width of the selected `isolation_2d` recipe. Applies to whichever recipe is chosen — both bin the masked depths |
| `isolation_2d_band_m` | `0.35` | Depth band kept around the near-surface anchor. `nearest_mode_histogram` only; `otsu` has no band and ignores it |
| `isolation_2d_min_bin_fraction` | `0.05` | Fraction of the masked depths a histogram bin must hold before the anchor may sit on it. `nearest_mode_histogram` only |
| `min_valid_pixels` | `10` | Shared foreground-sample floor for projective ranging and Euclidean reconstruction; both depth rows reject a prepared selection below it |
| `polar_range_jump_m` | `0.30` | Adjacent-beam range step that starts a new polar run |
| `polar_range_band_m` | `0.35` | Maximum offset behind the nearest run for merging another polar run |
| `polar_min_valid_rays` | `2` | Beam floor required for a polar estimate |
| `camera_info_topic` | `sensors/camera_0/color/camera_info` | Compatibility override for the shared camera contract's color-grid intrinsics |
| `repeats` | `5` | Number of positive-trial repeats per spawn pose |
| `output_dir` | `<repo-root>/artifacts/benchmarks` | Root directory that will receive one timestamped subfolder per run |
| `run_dir_name` | empty | Optional exact run-folder name under `output_dir`; sweeps use the configuration name. Empty preserves the timestamped single-run naming |
| `shutdown_on_complete` | `false` | Shut down the config launch service when the runner exits. The sweep sets this to `true`; the compatibility launch leaves the completed stack open for inspection |
| `settle_sec` | `2.0` | Delay after spawning the target before sampling |
| `capture_sec` | `10.0` | Sampling window length for collecting usable detections |
| `replay_dataset_dir` | empty | New directory for the compact legacy measurement replay dataset. Setting it switches this run from elapsed-time capture to an exact raw-batch quota and requires projective ranging, box gating, and stereoscopic depth |
| `sensor_capture_dir` | empty | New immutable full sensor-capture artifact for `mask-output`/`mask-model` replay. Stores exact RGB, float32 aligned depth, raw detections, calibration, and transforms; mutually exclusive with `replay_dataset_dir` |
| `capture_batches` | `5` | Raw detector batches captured per trial when either replay output is set; empty batches count. Five was selected by the 2026-09-09 convergence run |
| `capture_drain_sec` | `2.0` | Maximum post-quota drain for matching exact depth, camera context, and live measurement messages. Capture ends early when every selected stamp is complete and preserves missing matches when the bound expires |
| `capture_timeout_sec` | `30.0` | Hard wall-time bound for obtaining the raw detector-batch quota; it is a stall guard, not the normal capture duration |
| `record_video` | `true` | Best-effort RViz recording to `video/run.mp4`; disable for timing runs where screen capture would be unwanted load |
| `color_topic` | `sensors/camera_0/color/image` | Compatibility override for the shared camera contract's color image; feeds detector, measurements, overlay, runner, and readiness gate |
| `depth_topic` | `sensors/camera_0/depth/image` | Colour-aligned depth of the shared camera contract; the hardware camera service publishes the same name |
| `pointcloud_topic` | `sensors/camera_0/points` | Simulation's organized point cloud. It is optional on RealSense; omit the pointcloud estimator or override this only after confirming driver output |
| `scan_topic` | `sensors/lidar2d_0/scan` | LaserScan topic used by polar profiling |
| `base_frame` | `<namespace>/robot/base_link` | Vehicle frame used for point-cloud and scan projection |

Benchmark semantics:

- each estimator scores its own usable events; one estimator may score while another misses
- multi-target events are associated per estimator from planar positions, never by detection order or another estimator's result
- zero-detection, gate-miss, and no-value outcomes remain scored misses; only infrastructure failures skip a trial
- each estimator CSV stores one row per ground-truth instance, using the median of that estimator's associated values
- the shared trial collage uses one representative event chosen from the available previews and estimates; it is evidence for the trial, not an instance-association source

### Local benchmark configurator

Use the local configurator to build, validate, and run an offline benchmark job.
It binds only to loopback and opens a browser by default; use `--no-open` on a
remote or headless session.

```bash
ros2 run ridgeback_autonomy target_benchmark_configurator
ros2 run ridgeback_autonomy target_benchmark_configurator --no-open
```

The UI supports legacy measurement replay, frozen mask-output comparison,
rerunnable mask-model materialization, and live-system sweeps, validating
selected artifacts against the same replay-job contract used by the CLI. Paths
you type resolve against the workspace root, so `artifacts/benchmarks/…` means
the same directory to the page and to the runner.

**Start** runs the three offline profiles from the page. The job YAML is written
to `artifacts/benchmark-jobs/` and the run is recorded under
`artifacts/configurator/runs/<run id>/` with its log and exit code, so a run
survives closing the tab or restarting the service — reopen the page to
re-attach. One run at a time. Cancel signals the run's whole process group,
escalating SIGINT → SIGTERM → SIGKILL. Save a copy still downloads the job for
archiving or hand-editing.

**Live sweeps still start from a terminal.** They need a GPU-backed X session,
and the page does not yet supervise a live run's conflicts, progress or
cancellation. `cleanup.sh` no longer kills the page or a replay it started — see
[docs/troubleshooting.md](docs/troubleshooting.md#what-cleanupsh-leaves-alone).
Copy the command instead. A sweep the configurator authors carries no `backend`, so it
runs on Gazebo. For Isaac, name a sweep YAML whose `defaults` set `backend: isaac`.
The live-system form shows the supported `owlv2`
choice, checkpoint, labels, threshold, rate and diagnostics once above the
config cards. A referenced sweep owns these settings; the page shows its
resolved values and rejects hidden overrides. Saving the job and copying its
command preserve the same environment settings. The live form retains its
existing `detector_fps` field default of `5.0`; the direct launch and sweep
launch defaults remain `10.0` when unset. Offline profiles reuse frozen
detections and cannot select a new detector.

The Results panel lists typed artifacts and run outputs, and can rename them.
Rename is refused where the directory name is load-bearing — a sweep's config
subdirectories, and staging directories — and renaming a sweep also rewrites the
output paths recorded in its `sweep.json`. See
[benchmark outputs](docs/target_distance_benchmarking/outputs.md) for the rename rules, and
[running benchmarks](docs/target_distance_benchmarking/running_benchmarks.md) to choose between
the configurator and the command line.

### Benchmark sweeps

Use the sweep supervisor when comparing configurations. It starts the simulator
selected by `backend`, RViz, the Ridgeback, camera TF, and the detector once, then restarts only the
measurement/configuration layer for each entry. The detector therefore loads
OWLv2 once per sweep; silhouette configurations still load their segmentation
model inside their per-config mask node.

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
  --report-only artifacts/benchmarks/20260828_141530_baseline
```

The shipped `benchmark_sweep_model_concurrency.yaml` is the controlled
sequential evidence run for the SlimSAM/Depth-Anything question. It starts with
a diagnostics-off/on A/A pair and then runs three rotated replications of the
box/silhouette by stereoscopic/monocular matrix. Run it only from a clean,
committed checkout; its `sweep.json` records the exact sweep and scenario file
hashes. See [BENCHMARKING — Combined-model throughput](docs/plans/BENCHMARKING_model_throughput.md)
for the decision gates and interpretation.

The shipped `benchmark_sweep_projective_parameters.yaml` varies one projective
ranging tuning constant at a time — `isolation_2d_band_m`,
`isolation_2d_min_bin_fraction`, `isolation_2d_bin_width_m` and
`min_valid_pixels` — around their current values, and repeats the default
configuration twice as an in-sweep noise control. See the Archived evidence section below
for what is already settled and what those runs are meant to answer.

Do **not** run `cleanup.sh` between configurations: it kills the simulator
and RViz, which are deliberately persistent, although on Gazebo it leaves `gz sim gui`
running. The supervisor runs it once, immediately before starting that
environment. It then owns each config process group, removes any orphan
`bench_*` entities after an unclean exit through the same scene-control
services the runner uses, and shuts the environment down at the end. Both layers run through `ridgeback_autonomy.benchmarking.interactive_launch`,
`ros2 launch` in its terminal mode, so stopping one delivers a single SIGINT to
every process in it, exactly as Ctrl-C in a terminal does; plain `ros2 launch`
without a terminal would forward a second one to each child.
If a sweep is interrupted, rerun the same command:
the latest incomplete sweep with the same YAML and `--only` selection is resumed,
valid `run.json` configurations are skipped, and partial config folders are
preserved with an `.incomplete_<timestamp>` suffix before retry.

The YAML format is a metadata block, sweep-wide defaults, and named configs:

```yaml
sweep:
  name: baseline
  description: Every estimator once at its defaults.

defaults:
  backend: gz        # persistent; gz or isaac, one simulator per sweep
  scenario: ''       # packaged benchmark_scenarios_full.yaml
  repeats: 1
  camera_profile: 640x480  # persistent; 1280x720 requires a new environment
  detector: owlv2
  detection_model: google/owlv2-base-patch16-ensemble
  target_labels: humanoid robot
  detection_threshold: 0.55
  detector_fps: 10.0

configs:
  - name: pointcloud
    estimators: pointcloud
  - name: mask_projective_ranging
    estimators: projective_ranging
    mask_gate: box
    depth_source: stereoscopic
    isolation_2d: nearest_mode_histogram
```

Config values override `defaults`. Validation happens before the simulator starts:
unknown arguments, duplicate/unsafe names, bad estimators, missing scenario
files, environment keys on individual configs, and estimator-inapplicable
knobs are rejected. `backend`, `world`, `setup_path`, `namespace`, `use_sim_time`,
`color_topic`, `camera_profile`, `detector`, `detection_model`,
`target_labels`, `detection_threshold`, `detector_fps`, and `detector_debug`
may be set only in `defaults` because the environment cannot change mid-sweep.
So may the backend-only settings
(`gz_gui` and `headless_rendering` for Gazebo; `headless`, `rtf`, `sim_mode`, and
`depth_fidelity` for Isaac), and a setting of the other backend is refused.
`backend: hardware` is refused outright.
The dry run prints the persistent environment command. Resume identity includes
the resolved detector settings; a different checkpoint starts a different sweep.
The configuration layer checks its recorded settings against the warm detector
before writing run evidence.

Use `--skip-preflight-cleanup` only when another same-user ROS/Gazebo session
must remain alive. The sweep then skips the aggressive repository cleanup and
warns that any stale processes are the operator's responsibility.

Outputs use one timestamped sweep folder with a resumable manifest and a
comparison report:

```text
artifacts/benchmarks/20260828_141530_baseline/
  sweep.json
  summary.md
  pointcloud/
    run.json  summary.md  pointcloud.csv  images/  video/run.mp4
  mask_projective_ranging/
    ...
```

`summary.md` contains one row per `(config, estimator)`, sorted by MAE. It also
records each configuration's wall time and a pre-run real-time-factor sample;
RTF is diagnostic only, but helps distinguish configuration effects from
observation-coverage drift as a long-lived simulator slows down.

### Legacy measurement replay

Use replay when only the box-gated stereoscopic `projective_ranging` recipe or
its numeric parameters change. One live run freezes raw detections, exact-stamp
depth ROIs, intrinsics, transforms, and ground truth; the offline command then
evaluates every configuration in the existing projective sweep over those same
inputs. It does not replace live runs for detector, transport, throughput,
latency, model, or final-integration questions.

Both the dataset directory and offline output directory must be new. This
five-scene command is a smoke workflow and uses the selected five-batch default.

```bash
SCENARIO="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml"
SWEEP="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_sweep_projective_parameters.yaml"
DATASET="$PWD/artifacts/benchmarks/replay_projective_smoke_dataset"
LIVE_OUTPUT="$PWD/artifacts/benchmarks/replay_projective_smoke_live"
OFFLINE_OUTPUT="$PWD/artifacts/benchmarks/replay_projective_smoke_offline"

ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  scenario:="$SCENARIO" repeats:=1 estimators:=projective_ranging \
  mask_gate:=box depth_source:=stereoscopic record_video:=false gz_gui:=false \
  output_dir:="$LIVE_OUTPUT" run_dir_name:=matching_live \
  replay_dataset_dir:="$DATASET" \
  capture_drain_sec:=2.0 capture_timeout_sec:=30.0 \
  shutdown_on_complete:=true

ros2 run ridgeback_autonomy target_offline_replay_benchmark \
  "$DATASET" "$SWEEP" --output-dir "$OFFLINE_OUTPUT" --workers 4
```

Capture counts raw detector batches rather than elapsed seconds. Once the quota
arrives, it drains only until every selected stamp has matching depth, camera
context, and a live measurement or `capture_drain_sec` expires. Missing matches
remain explicit replay evidence. Any skipped trial leaves the dataset `incomplete`, and the
offline loader refuses it instead of silently comparing a reduced scenario set.

The offline root contains `summary.md` and `sweep.json` for the cross-variant
comparison, `replay.json` for dataset/evaluator provenance and total evaluation
time, and one normal CSV/`run.json`/`summary.md` set per variant. The clean
2026-09-09 full-scenario validation captured 109/109 trials, matched all 135
live rows, and reduced the estimated 6.99-hour 15-variant sweep to 8 minutes 22
seconds end to end (about 50x). Five batches is therefore the legacy capture default. On
the measured 32-thread host, 16 workers was the replay knee; choose workers for
the machine rather than blindly using every logical CPU. See the Archived evidence section below.

### Layered mask replay

Use the typed replay path when masks themselves must be compared or rerun. The
four profiles and their legal axes are available without ROS or model imports:

```bash
ros2 run ridgeback_autonomy target_replay_describe --json
```

Capture one full sensor parent with the existing benchmark launch. The capture
still counts raw batches, including empty batches, and stores missing exact RGB,
depth, or context explicitly after the bounded drain.

```bash
SCENARIO="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml"
SENSOR="$PWD/artifacts/benchmarks/sensor_capture"
LIVE_OUTPUT="$PWD/artifacts/benchmarks/sensor_capture_live"

ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  scenario:="$SCENARIO" repeats:=1 estimators:=projective_ranging \
  mask_gate:=box depth_source:=stereoscopic record_video:=false gz_gui:=false \
  output_dir:="$LIVE_OUTPUT" run_dir_name:=sensor_capture_live \
  sensor_capture_dir:="$SENSOR" capture_batches:=5 \
  capture_drain_sec:=2.0 capture_timeout_sec:=30.0 \
  shutdown_on_complete:=true
```

Derive immutable caches independently. A changed checkpoint, revision, prompt
padding, IoU floor, device/dtype, dependency version, or code provenance yields
a different producer signature and artifact identity.

```bash
ros2 run ridgeback_autonomy target_replay_materialize_masks \
  "$SENSOR" --producer box \
  --output-dir "$PWD/artifacts/benchmarks/masks_box"

ros2 run ridgeback_autonomy target_replay_materialize_masks \
  "$SENSOR" --producer slimsam \
  --model Zigeng/SlimSAM-uniform-50 \
  --output-dir "$PWD/artifacts/benchmarks/masks_slimsam"
```

The generalized executor accepts a canonical job YAML/JSON. `mask-output`
requires the sensor parent plus one or more caches; `mask-model` requires the
sensor parent plus `materializations` and creates its caches once before running
all measurement variants.

```yaml
job_version: 1
question: compare-mask-outputs
profile: mask-output
inputs:
  sensor_capture: ./artifacts/benchmarks/sensor_capture
  mask_caches:
    - ./artifacts/benchmarks/masks_box
    - ./artifacts/benchmarks/masks_slimsam
sweep:
  sweep:
    name: paired_mask_measurements
    description: Compare two measurement bands over every frozen mask cache.
  configs:
    - name: baseline
      estimators: projective_ranging,euclidean_reconstruction
      isolation_2d_band_m: 0.35
    - name: wider_2d_band
      estimators: projective_ranging,euclidean_reconstruction
      isolation_2d_band_m: 0.50
resources:
  measurement_workers: 4
comparison_baseline: masks_box:baseline
output_dir: ./artifacts/benchmarks/paired_masks
```

```bash
ros2 run ridgeback_autonomy target_replay_benchmark replay-job.yaml
```

Outputs appear only after the whole job succeeds. Offline reports state their
profile, supported claims, artifact hashes, cache lineage, worker count, and
limitations. Mask-production time is diagnostic; only the existing live-system
benchmark can support transport, throughput, end-to-end latency, GPU-contention,
or integration claims.

### Perception interfaces

| Node | Output topic | Key params |
|------|--------------|------------|
| `target_detector_node` | `detections/target/raw` | `color_topic`, `detector` (`owlv2` only), `detection_model` (checkpoint), `target_labels`, `detection_threshold`, `detector_fps` (default `10.0`; an upper bound on *step starts*), `detector_debug` |
| `target_pointcloud_measurement_node` | `measurements/target/pointcloud` | `color_topic`, `pointcloud_topic`, `base_frame`, `enabled_estimators` |
| `target_mask_measurement_node` | `measurements/target/mask` (+ subscriber-gated mask/aligned-depth debug images, `visualization/target/polar_rays`, and `debug/target/polar_beams` as selected) | `enabled_estimators`, `depth_source`, `depth_topic`, `camera_info_topic`, `scan_topic`, `base_frame` (**must be passed** — its own default is the bare `base_link`, unlike the other nodes', which are namespace-derived), `pitch_deg`, `front_offset_m`, `isolation_2d` and its three numeric settings, `min_valid_pixels`, `isolation_3d` and its six numeric settings, polar-isolation settings, `mask_gate`, `segmentation_model`, `color_topic`, `ray_marker_topic` |
| `target_overlay_node` | `debug/target/overlay` | `measurement_topic`, `mask_measurement_topic`, `color_topic`, `aligned_depth_topic`, `estimators`, `max_cols`, `rgb_panel_labels` |
| `target_visualization_node` | `visualization/target/estimates` + `hud/target_distances` | `base_frame`, `world_frame`, `marker_lifetime_sec`, `ground_truth_topic`, `estimators` (gates both the HUD rows and the rings; defaults to `all`), `hud_layout` (`rows` — the benchmark's, with truth and error columns — or `wide`, exploration's estimator columns with an age under each) |
| `target_distance_benchmark_runner` | per-estimator CSVs + `run.json` + `summary.md` + trial collage images + optional `video/run.mp4` | `estimators`, `output_dir`, `pointcloud_measurement_topic`, `mask_measurement_topic`, `color_topic`, `record_video`, `record_fps` |

Perception gets camera intrinsics exclusively from the colour camera's live
`CameraInfo`; there is no application-side FoV override to tune. Gazebo and
Isaac share nominal D455 RGB profiles: `640x480` is the default and
`1280x720` is selectable with `camera_profile:=...`, both at 30 Hz. The optics
are derived from the manufacturer's 1280×800, 90-degree horizontal RGB contract
using centred crops and square pixels. Both backends publish the selected
intrinsics through the same `CameraInfo` interface. See the
[camera-stack reference](docs/target_localization/camera_stack.md) for the
exact matrices, assumptions, and pending hardware comparison.

## Configuration

### Robot sensors (`clearpath/robot.yaml`)

- **ROS middleware**: The checked-in Clearpath setup selects CycloneDDS
  (`rmw_cyclonedds_cpp`), matching the measured exact-stamp delivery default.
  Deploy it to the robot only through an approved configuration diff.
- **Hokuyo UST-10LX**: Mounted at front of chassis, provides 2D laser scan for SLAM and costmaps
- **Intel RealSense D455**: Mounted on a riser bracket. `device_type: d455` selects both the driver's device filter and the URDF model, so it decides the camera's geometry as well as which device the driver binds to. Configured streams are RGB and depth with `align_depth.enable: true` and `enable_sync: true`; stream profiles are left unspecified so Clearpath's 640x480 @ 30 defaults apply. It supplies RGB and aligned depth to the target-localization stack. An organized point cloud is enabled in the checked-out Clearpath defaults but is **not** wired as an application input — its layout, frame, and timestamps are unverified on hardware. These driver settings drive the simulators; on the robot the [hardware camera service](#hardware-camera-service) streams colour and aligned depth only, from the same mount. The D455 also has an IMU, which this stack neither enables nor consumes.

### Key parameters to tune

| File | Parameter | Effect |
|------|-----------|--------|
| `config/frontier_explorer_params.yaml` | `min_frontier_size` / `near_frontier_radius` / `goal_advance_cells` | Frontier clustering, near-tier preference, and goal placement past the centroid |
| `config/frontier_explorer_params.yaml` | `progress_timeout` / `abort_blacklist_threshold` | Stall timeout (cancels only when no progress) and failures required before a frontier is blacklisted |
| `config/frontier_explorer_params.yaml` | `distance_weight` / `size_weight` | Scoring trade-off between how close vs. how large a far-tier frontier is |
| `config/frontier_explorer_params.yaml` | `goal_cost_threshold` / `goal_safety_margin` / `lethal_cost_threshold` | Goal-safety filtering against the costmap |
| `config/nav2_params.yaml` | `vx_max` / `vy_max` / `wz_max` | Robot linear and turn-rate limits |
| `config/nav2_params.yaml` | `max_accel` / `max_decel` | Velocity smoother acceleration and braking limits |
| `config/slam_toolbox_params.yaml` | `resolution` | Map resolution (m/pixel) |

## Generated artifacts

Repository-owned logs and results live under the Git-ignored `artifacts/` directory:

| Directory | Contents |
|-----------|----------|
| `artifacts/colcon/` | Colcon's timestamped build/test log directories and its `latest*` links |
| `artifacts/exploration/<run>/` | `console.log` and `ros/` for each quick-start launch |
| `artifacts/benchmarks/<run-or-sweep>/` | Benchmark JSON/CSV reports, images, video, and child-process logs |

`colcon_defaults.yaml` sets the log base for Colcon commands run from the workspace root. Use `colcon --log-base /somewhere/else build ...` to override it. The build helper also uses an absolute log base when invoked from elsewhere and honors `COLCON_LOG_PATH`.

`LOG_DIR` overrides the exploration output root, and `output_dir:=...` (or a sweep's `defaults.output_dir`) overrides benchmark output. `ROS_LOG_DIR` always overrides the managed child ROS-log locations. Without that override, sweep environment logs live in `<sweep>/environment/logs/ros/`, config logs in `<sweep>/<config>/logs/ros/`, and standalone runner command logs in `<run>/logs/ros/`.

Direct `ros2 launch` commands still use ROS's normal log destination unless you set `ROS_LOG_DIR` before launching, for example `ROS_LOG_DIR="$PWD/artifacts/ros" ros2 launch ...`. Existing `~/.ros/log` and `/tmp` logs are not moved.

Existing runs were relocated without rewriting their contents: absolute paths recorded in old run metadata describe the original execution location. Reports and sweep resume use the relocated folders. Do not treat benchmark results as disposable logs; no automatic pruning is enabled.

## Patches, troubleshooting, and history

This project relies on seven maintained dependency patches:

1. `patches/clearpath_gz_customizations.patch` patches `src/clearpath_simulator/clearpath_gz` to expose the custom `SpawnG1` Gazebo GUI plugin. The Gazebo adapter owns this repo's world/model search path.
2. `patches/slam_toolbox_tf_namespace.patch` patches `src/slam_toolbox` so `slam_toolbox` respects namespaced TF remappings.
3. `patches/clearpath_realsense_sim_frames.patch` patches `src/clearpath_common` so `intel_realsense.urdf.xacro` forwards `is_sim` into every supported camera macro as `use_nominal_extrinsics`, and so the Gazebo render sensor sits on the model's colour frame. Without it, simulation has no `camera_0_color_optical_frame` TF and the mask estimators report `TF_MISS_EXTRINSIC`.
4. `patches/gz_sim_sensors_shutdown.patch` patches the pinned Gazebo Sim 8.15.0 Sensors shutdown waiter and adds upstream regression tests. The optional Gazebo source profile builds this compatible workspace overlay; see the [build, loader verification, and rollback instructions](docs/project/dependencies.md#gazebo-sensors-shutdown-patch).
5. `patches/ros_gz_bridge_shutdown.patch` backports upstream ros_gz #955 and #991 and hand-backports #981 to the pinned ros_gz 1.0.24, so `parameter_bridge` no longer keeps its node alive past `main()` and `image_bridge` no longer receives frames into a destroyed handler; bridges stop on Ctrl-C without SIGSEGV or heap corruption. Only `ros_gz_bridge` and `ros_gz_image` are built from this checkout; see the [build, loader verification, and rollback instructions](docs/project/dependencies.md#ros_gz-shutdown-patch).
6. `patches/rviz_shutdown.patch` patches the pinned RViz 14.1.23 Image subscription and view-manager ownership during teardown. The optional `viz` profile builds `rviz_common`, `rviz_default_plugins`, and their test framework; see the [mechanisms, build, loader verification, and rollback instructions](docs/project/dependencies.md#rviz-shutdown-ownership-patch).
7. `patches/clearpath_ust10lx_sim_lidar.patch` patches `src/clearpath_common`, on top of patch 3, so Gazebo simulates each `hokuyo_ust` as the UST-10LX: 1081 bins over ±135° at 0.25°, 0.06–10 m at 40 Hz, with Gaussian range noise. Without it Gazebo publishes Clearpath's generic 540 bins at 0.5° and returns out to 25 m. Hardware frames are unchanged; see [the patch section](docs/project/dependencies.md#ust-10lx-gazebo-lidar-patch).

Because the patches are recorded against specific upstream revisions,
`dependencies/core.repos` and `dependencies/gz.repos` pin every dependency to
an exact commit rather than to a branch tip. Do not
change a pin to `jazzy`/`main` to pick up a fix: a fresh `vcs import` would then
clone commits the patches were never rebased onto. The complete import, refresh,
live-checkout migration, and rollback procedure lives in the
[dependency runbook](docs/project/dependencies.md). The latest completed refresh
and its evidence are recorded in the Archived evidence section below.

`tools/check_dependencies` verifies every checkout revision and rejects
untracked files or changes beyond the recorded patches. Its `--apply` mode
applies missing patches idempotently; without that option it is a read-only
gate. Run it after every import and before builds:

```bash
tools/check_dependencies --apply --profile all
```

Never resolve a reported conflict by discarding unrelated changes in a
dependency checkout.

Current recovery steps live in [troubleshooting](docs/troubleshooting.md), while dated root causes and measurements live in the archived evidence below.

## Archived evidence

- [6.1 migration evidence](archive/engineering/2026-09-13-isaac-6.1-migration.md)
- [investigation history](archive/engineering/exact_stamp_depth_availability.md)
- [projective parameter sensitivity](archive/engineering/projective_parameter_sensitivity.md)
- [Offline measurement replay validation](archive/engineering/offline_measurement_replay_validation.md)
- [dependency refresh history](archive/engineering/dependency_refresh.md)
- [operational incident history](archive/engineering/operational_incidents.md)
- [port history](archive/engineering/port-history.md)

The stationary `interlock-reconnect` observer requires the installed test lock,
operator confirmation and `--reconnect-device <controller-MAC>`. Start with the
physical e-stop pressed and the controller disconnected. It records ten seconds
of disconnected state, then prompts for manual PS-button reconnection with all
driving controls centred/released. It records thirty more seconds, confirms the
selected device connected, and verifies fresh neutral joystick and asserted
physical stop reports afterward. It sends no commands, permits, connect or
disconnect requests. Stop release, software unlock, non-neutral driving input,
unexpected commands/wheel motion, stale stop/wheel observations or a competing
permit publisher abort. Preserve driver journals to establish actual disconnect
and reconnect times. This does not simulate silent radio loss, test reconnection
with the physical stop released, or establish multi-minute outage recovery.

For a controller that is Bluetooth-connected but has no physical reports or ROS
input, keep the physical stop pressed and gather a read-only process diagnostic:

```bash
sudo python3 tools/hardware/diagnose_controller_input.py \
  --run-dir "$PWD/artifacts/hardware/controller-input-diagnostic"
```

Use a fresh directory. It snapshots controller/joystick threads, device handles,
input inventory, service state and driver logs without reading input-device data,
restarting services or publishing commands. Root access is needed for the driver
process inspection. Review recorded permission errors and report delivery before
claiming recovery; see the [driving reference](docs/physical/driving_readiness.md).

The controller diagnostic additionally supports `--sample-states
--operator-confirmed`. With the physical stop pressed and the exact temporary
interlock installed, it verifies stationary stop/routing before and after a
20-second operator input window. It compares cached absolute-axis and button
state using read-only input ioctls for the kernel physical controller and managed
virtual controller. It never consumes event queues, grabs devices, publishes
commands/permits or alters mapping. Require exactly one physical device with the
configured controller MAC and one managed virtual device; ambiguity aborts.
Inspect `input-states.jsonl` to distinguish physical-report reception from
virtual-device forwarding. Keep the physical stop pressed during all actions;
this is input diagnosis, not driving qualification.

For the observed Bluetooth-connected-but-no-input controller fault, keep the
physical stop pressed and software test lock active. Preserve diagnostics before
recovery. Disconnect only the identified controller:

```bash
bluetoothctl --timeout 2 disconnect AC:36:1B:89:A1:0A
bluetoothctl --timeout 1 info AC:36:1B:89:A1:0A
```

Confirm `Connected: no`, then manually press PS once with controls centred and
both driving-enable buttons released. Verify actual button/stick input and its
return to neutral with a fresh stationary recorder; keep the physical stop held.
This restored input in one recorded recovery and is not an automatic-reconnect
qualification. Do not infer live input from Bluetooth connection or zero ROS
messages alone. The [driving reference](docs/physical/driving_readiness.md) owns
failure evidence and the unresolved cause.

A verified pinned motor-driver repair can be rebuilt without changing the live
robot. Download the official Clearpath 2.9.4 archive first; the builder verifies
its SHA-256 `49d0ef5bcc6421856d6a4644aa4bd0dd117b1052d569d5b3a3238385184206df`.
Build and stage into fresh directories:

```bash
python3 tools/hardware/build_puma_repair.py \
  --archive <downloaded-clearpath-2.9.4.tar.gz> \
  --output "$PWD/artifacts/hardware/puma-repair-candidate"
python3 tools/hardware/install_puma_repair.py \
  --candidate "$PWD/artifacts/hardware/puma-repair-candidate" \
  --run-dir "$PWD/artifacts/hardware/puma-repair-install"
```

Inspect the applied-patch provenance, sanitizer results, full-package compiler
output and staged receipt before installation. The October 9 candidate is
withdrawn and rejected by the corrected installer. Apply a verified frozen stage
only with an operator present, physical stop pressed and controls centred:

```bash
sudo python3 tools/hardware/install_puma_repair.py \
  --apply --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/puma-repair-install"
```

Only the platform and guard are restarted; current exact caps/test-lock files
are preserved. The patched Puma package is copied into a root-controlled
`/opt/dynamo-puma/<package-hash>` prefix, not a mutable workspace build. The
three owned additions provide DDS-interface readiness and a scoped platform
package overlay. Failure after platform shutdown leaves the platform stopped.
Restore only those exact additions, with physical stop held, using the same
receipt and `--restore --operator-confirmed` instead of `--apply`. The previous
capped platform is restored; the unused pinned prefix remains. Actual cold boot,
feedback freshness and driver reconnect behavior need separate qualification.
See the [driver reference](docs/physical/drive_feedback_diagnostics.md).

The corrected 2026-10-12 Puma repair is installed and independently verified
stationary; its receipt is under
`artifacts/hardware/20261012-puma-repair-install/`. It preserves the temporary
test lock. Fault-mask decoding now matches raw status; the feedback-rate and
per-wheel freshness investigation remains open. Recreate this state using the
maintained build/stage/install procedure, not the withdrawn October 9 candidate.

The proposed Puma freshness repair is offline only. Verify its extracted routines
and project publication timing without touching ROS, CAN or the installed driver:

```bash
python3 tools/hardware/verify_puma_freshness_patch.py \
  --package <verified-two-patch-source-package> \
  --output "$PWD/artifacts/hardware/freshness-routine-check"
python3 tools/hardware/replay_puma_freshness.py \
  --capture <saved-passive-candump-log> \
  --output "$PWD/artifacts/hardware/freshness-flag-replay.json"
```

Both outputs must be fresh. The replay is a flag/scheduling model, not a live
patched-driver test. It predicts lower output rates and longer publication gaps
when every wheel must provide fresh fields. Do not add the candidate to the
installed repair without qualifying stale-feedback handling in the hardware
consumer. See the [freshness deployment boundary](docs/physical/drive_feedback_diagnostics.md#offline-freshness-candidate-and-deployment-boundary).

To reproduce the Puma hardware consumer’s current behavior offline, with no ROS
or hardware access:

```bash
python3 tools/hardware/verify_puma_feedback_consumer.py \
  --source <matching-clearpath-2.9.4>/clearpath_hardware_interfaces/src/puma/hardware.cpp \
  --output "$PWD/artifacts/hardware/feedback-consumer-check"
```

Use a fresh output directory. It exercises actual extracted method bodies with
inert input/output; it does not interrupt live feedback or establish physical
stopping. The [consumer reference](docs/physical/drive_feedback_diagnostics.md#hardware-consumer-missing-feedback-behavior)
owns findings and the pending age/batch-validation deployment gate.


Build and check the proposed feedback-consumer repair offline:

```bash
python3 tools/hardware/build_puma_feedback_candidate.py \
  --source-archive <official-clearpath-robot-2.9.4-tar.gz> \
  --output "$PWD/artifacts/hardware/feedback-consumer-candidate"
```

Use a fresh output directory. The builder accepts only the pinned archive,
checks the policy and actual adapter/lifecycle bodies, compiles the package and
saves provenance. It does not install or access the live robot. The candidate
is not qualified for deployment; see the
[consumer safety boundary](docs/physical/drive_feedback_diagnostics.md#offline-feedback-consumer-safety-candidate).


Check that compiled candidate using fake wheel publishers and a command sink:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/check_puma_feedback_ros.py \
  --candidate "$PWD/artifacts/hardware/feedback-consumer-candidate" \
  --output "$PWD/artifacts/hardware/feedback-consumer-ros-check"
```

This joins only loopback ROS domain 214 in an isolated namespace, verifies the
loaded library is the candidate and starts no motor driver. It checks compiled
activation, stale-feedback inhibition, latched recovery and deactivation through
real ROS messaging. Controller-manager integration and physical braking still
require separate tests. Use a fresh output directory.


Test controller-manager initialization, feedback-loss inhibition and explicit
neutral recovery with the candidate and fake wheels:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/check_puma_feedback_manager.py \
  --candidate "$PWD/artifacts/hardware/feedback-consumer-candidate" \
  --output "$PWD/artifacts/hardware/feedback-manager-check"
```

Use a fresh output directory. This starts an isolated controller manager on
loopback-only ROS domain 215 with the candidate plugin, fake feedback and a
command sink. It does not start the real motor driver or access CAN. Its recovery
service calls apply only to this fake component; they are not a robot recovery
procedure. Successful software tests do not qualify physical braking.

## Puma feedback plugin installation (stationary)

`tools/hardware/install_puma_feedback_plugin.py` stages a root-pinned plugin
from a verified offline candidate. It preserves the commissioned speed guard,
Puma motor driver, auxiliary launch and other hardware plugin descriptors. A
new platform drop-in selects only the pinned Puma feedback library. The install
requires the temporary commissioning test interlock and an engaged physical
stop. It restarts the platform/guard only; camera, sensors and desktop are
checked for unchanged service state.

Stage from the successful candidate with a fresh directory:

```bash
cd /home/robot/DyNAMO-Ridgeback
python3 tools/hardware/install_puma_feedback_plugin.py --stage \
  --candidate "$PWD/artifacts/hardware/20261012-feedback-consumer-reproducible-5" \
  --run-dir "$PWD/artifacts/hardware/puma-feedback-install"
```

With an operator beside the stationary robot and the physical e-stop pressed:

```bash
sudo python3 tools/hardware/install_puma_feedback_plugin.py --apply \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/puma-feedback-install"
```

Success checks the loaded library in the real controller manager, stop state,
stationary odometry, capped routing and unchanged independent services. A failed
installation attempts to restore the original platform; inspect its saved
`result.json`. Rollback uses the same receipt and conditions:

```bash
sudo python3 tools/hardware/install_puma_feedback_plugin.py --restore \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/puma-feedback-install"
```

If interrupted during apply, restore from that receipt. The pinned package is
retained as evidence after rollback. This verifies startup while stopped; it
does not qualify actual braking, missing CAN feedback under movement or long
runs. The installed library and service drop-in are reconstructed from this
maintained script and the pinned source archive, not from an artifact alone.

After an installed plugin or clock-profile change, prepare a one-reboot check
while stopped, then verify after the desktop reconnects:

```bash
python3 tools/hardware/verify_feedback_boot.py --prepare \
  --run-dir "$PWD/artifacts/hardware/feedback-clock-postboot"
sudo systemctl reboot
# After reconnecting, with the physical e-stop still pressed:
cd /home/robot/DyNAMO-Ridgeback
sudo python3 tools/hardware/verify_feedback_boot.py --verify \
  --run-dir "$PWD/artifacts/hardware/feedback-clock-postboot"
```

The verifier requires a changed boot ID, a successful platform clock gate,
stationary e-stop and service checks, and the exact pinned plugin mapped by the
new controller manager. Its result covers this boot only.

For a supervised stationary check of **real ROS wheel-feedback interruption**,
keep the physical e-stop pressed, red lights flashing, controller centred and
an operator beside the robot. The guarded checker sends no movement or permit;
it pauses only the Puma driver for 0.8 seconds and schedules an independent
one-second resume timer before the pause:

```bash
cd /home/robot/DyNAMO-Ridgeback
sudo python3 tools/hardware/check_live_feedback_outage.py --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/live-feedback-outage"
```

If the controller faults as intended, leave the physical stop pressed and
recover it only after saving the evidence:

```bash
cd /home/robot/DyNAMO-Ridgeback
sudo systemctl restart clearpath-platform.service
sudo python3 tools/hardware/check_live_feedback_outage.py --verify-recovery \
  --operator-confirmed --run-dir "$PWD/artifacts/hardware/live-feedback-recovery"
```

Use fresh run directories. The recovery check waits for platform startup and
requires fresh feedback, zero wheel commands, stationary odometry and an active
controller. This check does not prove braking while moving or an electrical
CAN-wire failure.

To inspect the actual MCU-to-PC motor-CAN ingress while **stationary**, keep
the physical stop pressed, controller centred and an operator beside the
robot. This separate checker drops only incoming UDP from the motor MCU on
port 11412 for 0.8 seconds. It schedules an independent three-second firewall
rollback before adding the rule, records real feedback and wheel commands, and
sends no movement command:

```bash
cd /home/robot/DyNAMO-Ridgeback
sudo python3 tools/hardware/check_motor_can_ingress.py --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/stationary-can-ingress"
```

Use a fresh run directory. Leave the stop pressed while reviewing its
`result.json`; if the controller faults, use the stationary recovery command
above and a new recovery directory. This incoming-only software interruption
does not prove physical braking or the motor controllers' response to a full
electrical link failure. The [feedback reference](docs/physical/drive_feedback_diagnostics.md#remaining-moving-and-motor-link-qualification)
owns those boundaries.

## Stationary desktop and controller setup repair

`tools/hardware/install_stationary_setup.py` stages a six-file repair: one VNC
owner, independent 9005/9100 proxies, a foreground software-rendered desktop with
its own session bus, and a root-pinned controller autoconnect helper. The helper
connects only configured paired/trusted devices; it never restarts ROS or base
services. Bluetooth connection alone does not prove fresh joystick input.

Stage without changing services:

```bash
python3 tools/hardware/install_stationary_setup.py --stage \
  --run-dir "$PWD/artifacts/hardware/stationary-setup-install"
```

Apply only with the robot stationary, an operator present, and physical e-stop
pressed. Sudo authentication belongs to your terminal. The detached installer
survives the planned desktop reconnect:

```bash
sudo python3 tools/hardware/install_stationary_setup.py --apply --detach \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/stationary-setup-install"
```

Read `result.json` after reconnecting. It verifies listener ownership, stationary
state and unchanged hardware service activity. Preflight drift prevents changes;
an installation failure restores original files and prior service activity.
Only desktop/proxy/autoconnect units are stopped or started. This does not deploy
the feedback-consumer candidate or qualify cold boot, joystick reconnection or
physical stopping. Never use the legacy combined vendor-camera repair here.

Exact receipt-based rollback, with the same stationary conditions:

```bash
sudo python3 tools/hardware/install_stationary_setup.py --restore --detach \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/stationary-setup-install"
```

Rollback refuses changed managed files. The stage must use a fresh directory;
application also refuses files or service state changed since staging.

## Hardware camera service

The maintained boot-service sources are restored from the installed camera's
recorded Git revision; its current contract and recovery inputs live in
[robot-local deployment](docs/physical/robot_local_deployment.md).
Inspect installed state without camera subscriptions or restarts:

```bash
tools/intel_thor/camera_service status
```

For a replacement host, after setting up Clearpath and discovery-only DDS,
install the serial-pinned service using:

```bash
sudo tools/intel_thor/camera_service apply --profile 640x480 --serial 338522302134
```

This changes the robot declaration, disables competing vendor camera units,
regenerates Clearpath state and starts the camera. It is a reconstruction step,
not part of the stationary desktop repair. Do not reapply it merely because the
checkout revision differs when all status checks match. Its `rollback` restores
its prior snapshot and may restart services; review the resulting ownership first.

When coordinated with perception, verify actual image delivery with:

```bash
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash
tools/camera_contract_check --backend hardware --guard \
  --output "$PWD/artifacts/hardware/camera-contract.json"
```

This creates image and scan subscribers. Use the stationary/operator conditions
and transport-monitoring precautions from the DDS test workflow. Source
restoration alone does not qualify camera performance or application integration.

## Stationary auxiliary repairs

The IMU/battery/clock repair preserves the commissioning guard and pinned Puma
motor driver. It corrects the MCU filter's mismatched YAML selector, marks
unmeasured individual cell voltages unknown, and adds a read-only platform
startup clock-convergence gate. It does not step the clock, invent calibration
values or add temperature hardware. Battery pack voltage/current/charge estimates
remain unchanged. The Python overlay affects only the battery helper package;
the compiled Puma plugin and other installed hardware libraries are unchanged.

Stage with a fresh directory:

```bash
python3 tools/hardware/install_auxiliary_repairs.py --stage \
  --run-dir "$PWD/artifacts/hardware/auxiliary-repair-install"
```

With the robot stationary, physical e-stop pressed and an operator present:

```bash
sudo python3 tools/hardware/install_auxiliary_repairs.py --apply \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/auxiliary-repair-install"
```

Only the platform is deliberately restarted (its guard follows its normal
service dependency). Camera, sensors, merger and desktop units are preserved and
checked. The installer refuses changed inputs/service state, saves write-ahead
ownership and exact rollback, and restores its prior platform integration on
failed installation. Success requires separate live IMU/battery data verification
before marking the receipt verified. An unknown or changed managed file prevents
rollback overwrite:

```bash
sudo python3 tools/hardware/install_auxiliary_repairs.py --restore \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/auxiliary-repair-install"
```

If an interrupted run leaves the receipt at `applying` but all files/services
are already installed, do not reapply into the same directory. Finalize using
verification only, with the stop pressed:

```bash
sudo python3 tools/hardware/install_auxiliary_repairs.py --verify \
  --operator-confirmed \
  --run-dir "$PWD/artifacts/hardware/auxiliary-repair-install"
```

This checks exact root-owned files, unchanged independent services, stationary
routing, clock readiness and live IMU/battery fields, then finalizes the receipt.
It does not restart services. Partial/mismatched installations are refused;
use the exact-file restore path after inspection rather than overwriting them.

The platform gate allows up to 900 seconds for an external normal chrony source,
remaining system-clock correction at most 5 ms, frequency skew at most 10 ppm,
and a reference at most 180 seconds old. Failure prevents this platform start;
it never resets/releases the physical stop. These provisional local readiness
bounds do not qualify cross-host timing or absolute UTC accuracy. Inspect them
without sudo or clock changes:

```bash
python3 tools/hardware/check_clock_ready.py --timeout 30
```

Stationary IMU characterization, with the physical e-stop pressed:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/characterize_stationary_imu.py --duration 60 \
  --run-dir "$PWD/artifacts/hardware/imu-stationary-characterization"
```

Use a fresh directory. The recorder subscribes to raw IMU, stop, raw odometry,
MCU power and MCU/PCB temperatures. It aborts on released/stale stop, wheel
motion or insufficient data, publishes no commands and applies no calibration.
A single-pose mean cannot separate accelerometer bias from gravity/mount tilt
or establish gyro scale, axis misalignment or thermal calibration.

After the auxiliary repair is installed, record a pre-reboot boot ID in its
receipt directory as `boot-baseline.json`, then reboot only with an operator,
physical e-stop pressed and controller neutral. Reconnect and verify:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/verify_auxiliary_boot.py \
  --receipt "$PWD/artifacts/hardware/auxiliary-repair-install" \
  --output "$PWD/artifacts/hardware/auxiliary-after-reboot"
```

The read-only verifier checks a changed boot ID, repaired file contents, clock
gate success before platform startup, service state/restarts, desktop listener
ownership, asserted-stop stationary routing and actual IMU/battery fields. It
never restarts services or changes the clock. Platform may remain activating
while the bounded clock gate waits up to 15 minutes; inspect its journal rather
than bypassing the gate or assuming it failed immediately. A new boot passing
this check does not qualify prolonged time-source loss or full cold power cycles.

## Fast stationary health batch

Use one command for the current installed stationary checks instead of repeating
separate probes. Keep the physical e-stop pressed; no controller action is needed:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/hardware/fast_stationary_check.py
```

The batch observes 16 seconds of IMU, battery, three scans, stop/relay and raw
odometry, then checks guarded graph routing, service health and local clock
readiness. It creates a unique `artifacts/hardware/stationary-batch-*` directory
and prints one report with only failed check names and explicit remaining gates.
It publishes no commands, subscribes to no camera images, performs no Bluetooth
transitions and restarts no services. Healthy current delivery is not a full
calibration, physical stopping, long-run recovery or camera/perception pass.
Existing reboot evidence is reused; no new reboot is requested by this batch.


If the fast batch reports only clock readiness failure, its `failure_details`
identifies the failed condition and measurements. Keep the passed sensor/route
checks as evidence; inspect/retry only the clock with `check_clock_ready.py`.

For the staged Intel source expansion, keep the physical e-stop pressed and an
operator present. This adds sources without restarting chrony or the desktop:

```bash
cd /home/robot/DyNAMO-Ridgeback
sudo python3 tools/hardware/expand_clock_sources.py --apply --operator-confirmed --run-dir "$PWD/artifacts/hardware/clock-sources-$(date -u +%Y%m%dT%H%M%SZ)"
python3 tools/hardware/check_clock_ready.py --timeout 30
```

An installed result still requires clock qualification. Source scope, recovery
and reconstruction are documented in
[stationary auxiliary checks](docs/physical/stationary_auxiliary_checks.md#intel-clock-source-expansion).

A small remaining clock correction can coexist with large frequency uncertainty;
a noisy NTP source must not be relabelled a timing pass by raising thresholds.


## Intel clock poll bound

If the installed 180-second platform clock gate fails only because chrony's
selected source is older than 180 seconds while still reachable, stage and apply
the 128-second maximum polling interval. The tool changes no clock value and
restarts no service:

```bash
cd /home/robot/DyNAMO-Ridgeback
python3 tools/hardware/bound_clock_poll.py --stage --run-dir "$PWD/artifacts/hardware/clock-poll-install"
sudo python3 tools/hardware/bound_clock_poll.py --apply --operator-confirmed --run-dir "$PWD/artifacts/hardware/clock-poll-install"
```

Use a fresh run directory and an operator beside the stationary robot. Check
`chronyc -n sources` and `python3 tools/hardware/check_clock_ready.py --timeout
30` after the next source measurement. Exact rollback is
`sudo python3 tools/hardware/bound_clock_poll.py --restore --operator-confirmed
--run-dir <same-run-directory>`. The
[clock reference](docs/physical/stationary_auxiliary_checks.md#poll-interval-and-clock-startup-gate)
explains the gate and evidence.
