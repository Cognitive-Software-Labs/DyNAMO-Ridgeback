# External dependency management

This page owns the repository's procedure for importing, patching, verifying,
refreshing, and rolling back external ROS source repositories. The root README
keeps the public installation commands; dated refresh results belong in
[engineering archive](../../archive/engineering/README.md).

## Model and ownership

The dependencies under `src/clearpath_*`, `src/slam_toolbox`, `src/gz-sim`,
`src/ros_gz`, and `src/rviz` are standalone,
Git-ignored repositories populated by `vcstool`. They are not Git submodules and
are not stored in the root repository's history.

The compatibility unit has four root-owned parts:

- [`dependencies/core.repos`](../../dependencies/core.repos) owns the source
  dependencies shared by every deployment.
- [`dependencies/gz.repos`](../../dependencies/gz.repos) owns the optional
  Gazebo-only `clearpath_simulator`, `gz-sim`, and `ros_gz` pins.
- [`dependencies/viz.repos`](../../dependencies/viz.repos) owns the optional
  `rviz` pin for hosts that run RViz.
- [`patches/`](../../patches/) owns intentional changes to those pinned sources.
- [`tools/check_dependencies`](../../tools/check_dependencies) verifies that
  every checkout is at its pin and contains either no changes or exactly its
  recorded patch.

Consequently, root `git status` cannot reveal changes inside the nested
repositories. Use the checker for the workspace-wide contract and `git -C
src/<repository> status` when inspecting one checkout directly.

## Fresh checkout and routine verification

From the workspace root, import only the deployment closure you need:

```bash
# Every deployment
vcs import < dependencies/core.repos

# Gazebo development only
vcs import < dependencies/gz.repos

# Hosts that run RViz
vcs import < dependencies/viz.repos

# Verify both profiles after a Gazebo import
tools/check_dependencies --apply --profile all
```

On an Isaac-only or hardware host, omit the Gazebo import and run
`tools/check_dependencies --apply --profile core`. `--profile gz` checks only
the optional Gazebo checkouts and `--profile viz` the optional RViz checkout;
`all` covers every profile. `vcs import` creates the nested repositories
at the exact manifest commits. The checker then applies missing maintained
patches. Its `--apply` mode is idempotent: an exact already-applied patch is
accepted.

Before later builds, use the read-only form:

```bash
tools/check_dependencies --profile all
```

The checker rejects:

- a missing repository or a checkout at the wrong commit;
- untracked files inside a dependency;
- tracked modifications in an unpatched dependency;
- a missing maintained patch in read-only mode;
- patch conflicts or any changes beyond the recorded patch.

Do not respond to a failure by resetting the nested checkout. Inspect it first;
it may contain intentional work that the root repository cannot see.

## Partial overlays

Some patched repositories hold many packages, of which only the patched ones
need a source build; the rest should keep resolving to the installed binaries.
The checker's `BUILD_ONLY` map names, per checkout, the top-level package
directories to build. Every other top-level directory with a `package.xml`
receives an empty, untracked `COLCON_IGNORE`, so `colcon build --base-paths src`
skips it. A directory upstream already ignores with a tracked `COLCON_IGNORE`,
`AMENT_IGNORE`, or `CATKIN_IGNORE` needs no marker.

For such a checkout the checker additionally:

- accepts exactly that set of markers as untracked files, and rejects any other
  untracked file;
- fails in read-only mode when a marker is missing; `--apply` creates it;
- fails when a kept package is missing or carries an ignore marker of its own.

The markers are untracked, so they never appear in `git diff HEAD`, and the
recorded-patch comparison is the same as for any other checkout. Removing a
package from `BUILD_ONLY` therefore takes a marker the checker will demand on
the next run; adding one requires deleting that package's marker first.

## Refreshing upstream revisions

Refresh all dependencies as one compatibility change. Do not advance a single
pin merely to clear one build error, and do not replace an immutable commit with
a moving `jazzy` or `main` branch name.

### 1. Establish candidates away from the live checkouts

Resolve the current upstream branch tips and review every commit between the old
pin and candidate revision. Use fresh temporary clones or an equivalently clean
workspace so existing ignored checkouts cannot contaminate the evidence.

Record:

- the full old and candidate commits;
- release/tag context and commit count;
- behaviorally relevant changes for this workspace;
- whether every old pin is an ancestor of its candidate.

### 2. Inventory live local state

For every nested repository, record its current revision, branch or detached
state, tracked diff, untracked files, and stashes. Classify each modification as:

- already represented by a maintained root patch;
- intentional but not yet recorded in a patch;
- obsolete or redundant because the repository no longer needs it.

Preserve all uncertain or intentional state before switching commits. A named
stash with `--include-untracked` is suitable for local edits, or commit the work
in the dependency repository when it belongs there. Dropping an edit requires
an explicit reason in the archived refresh record.

### 3. Rebase and re-record patches

Apply each maintained patch to a clean checkout at its candidate pin. If it
conflicts, reproduce the intended behavior against the new upstream structure,
then generate the root patch from the complete nested-repository diff.

The maintained mappings are:

| Dependency checkout | Root patch |
|---|---|
| `src/clearpath_simulator` | `patches/clearpath_gz_customizations.patch` |
| `src/clearpath_common` | `patches/clearpath_realsense_sim_frames.patch` |
| `src/slam_toolbox` | `patches/slam_toolbox_tf_namespace.patch` |
| `src/gz-sim` | `patches/gz_sim_sensors_shutdown.patch` |
| `src/ros_gz` | `patches/ros_gz_bridge_shutdown.patch` |
| `src/rviz` | `patches/rviz_image_display_unsubscribe.patch` |

Update all affected manifest pins and patch files in the same root commit.

### 4. Prove the clean-import contract

Against clean checkouts at the proposed pins:

- confirm each patch passes `git apply --check` before application;
- run `tools/check_dependencies --apply --profile all`;
- rerun `tools/check_dependencies --profile all` and require a clean result;
- confirm each patched checkout's complete `git diff HEAD` exactly equals its
  recorded root patch;
- remove and reapply at least one patch to exercise the missing-patch and
  idempotence paths.

`vcstool validate` is not the acceptance gate here: manifest versions are raw
commit hashes, while `tools/check_dependencies` checks the exact checkout and
patch shape required by this workspace.

### 5. Run compatibility gates

At minimum:

- build the complete source workspace;
- run tests for the changed dependencies and `ridgeback_autonomy`;
- load all public launch files with `--show-args`;
- run the camera-description/SDF checks when Clearpath description code changes;
- run an isolated headless simulator smoke test and resolve
  `base_link -> camera_0_color_optical_frame` on the namespaced TF streams;
- run headless autonomous exploration long enough for Nav2 to complete a
  frontier goal when simulator, control, SLAM, or navigation inputs changed.

Use a temporary Clearpath setup directory, a dedicated `ROS_DOMAIN_ID`, and
targeted process-group shutdown for runtime tests. Do not overwrite a deployed
`~/clearpath` configuration or use machine-wide cleanup as part of an isolated
gate.

Changing dependencies changes benchmark provenance. Rerun an affected benchmark
before quoting its numbers as current, and never present results from different
dependency sets as a matched comparison.

### 6. Move the live ignored checkouts

Only after clean validation:

1. preserve each live local modification with a named stash or repository commit;
2. fetch the validated candidate objects;
3. switch every dependency to its exact new commit in detached-HEAD state;
4. run `tools/check_dependencies --apply`;
5. rebuild the normal workspace and repeat the relevant runtime smoke gate.

The checker must pass after the move. Keep the saved local states until the
refresh is committed and accepted.

### 7. Record and land the refresh

Move the completed item out of the backlog and add a dated record under `archive/engineering/` containing
the revision table, useful upstream changes, local-edit decisions, validation
evidence, limitations, and rollback boundary. Commit the manifest, patches,
checker changes, current documentation, and archived validation evidence as one logical
deliverable.

## Rollback

Treat pins and patches as an inseparable set. The clean rollback is to restore
the previous root commit's `dependencies/*.repos` and `patches/`, recreate or
switch all nested checkouts to those pins, and run the restored checker with
`--apply` and the relevant profile.

Do not selectively combine an old dependency pin with a new patch. Restore a
saved live-checkout stash only on the revision where it was created, inspect the
result, and never discard it merely because the refreshed manifest no longer
accepts it.

## Gazebo Sensors shutdown patch

The Gazebo profile imports gz-sim **8.15.0**, upstream tag `gz-sim8_8.15.0`,
commit `446a44335a45b704b4d36dabcc5508ee34eeb3d8`. This matches the source selected
by `gz_sim_vendor` 0.0.13; that vendor revision supplies no source patches. The
local patch preserves the version and major-8 SONAMEs. It does not import
`gz-common` or the abandoned signal self-pipe backport.

### Synchronization boundary

In `src/systems/sensors/Sensors.cc`, `PostUpdate` waits for pending ECM changes
to reach the rendering scene before replacing them. Upstream's `updateTimeCv`
predicate lacks a shutdown condition, and `Stop` notifies only `renderCv`.
Consequently a pending update can leave a worker waiting after the render
thread has stopped.

The patch serializes each instance's complete `Stop` operation with `stopMutex`,
including connection teardown and render-thread joining. It takes
`renderUtilMutex` before `renderMutex` in `Stop`, changes
`running` while holding both, notifies both condition variables, and releases
both mutexes before joining the render thread. The `PostUpdate` predicate
includes `!running` and returns on shutdown before `UpdateFromECM` and subsequent
sensor work. Coordinating the transition with the wait mutex prevents a lost
notification between predicate evaluation and wait entry; an atomic flag alone
does not supply that coordination.

`RunOnce` releases its `renderMutex` wait before taking `renderUtilMutex`, and
releases `renderUtilMutex` before its later rendering section takes
`renderMutex`. `PostUpdate` and `Reset` likewise do not hold these locks in the
reverse nested order. `Run` is the configure-time transition to running.
The time fields in the predicate are changed under `renderUtilMutex`; the
existing `updateAvailable` flag remains atomic and coordinated with the render
wait by `renderMutex`. The new shutdown wakeup is coordinated with both waits.

The in-tree stop paths are the Stop-event callback and the Sensors destructor;
neither enters holding a Sensors mutex. A server's signal callback and its
destruction path can emit the Stop event concurrently for the same instance.
Without the per-instance guard, both callers can attempt to join the same
`std::thread`, which is undefined. The guard also makes sequential repeated
stops safe without changing the public ABI. No other method takes `stopMutex`,
and the render thread does not need it to exit.

Rendering event callbacks can run while
`renderMutex` is held. Calling server shutdown recursively from such a callback
already risks locking/self-joining upstream and is not a supported extension of
this fix. Repeated stops, same-instance concurrent stops, and independent
instances are tested. Two log messages alone cannot identify their objects;
the same-instance correction was justified by live debugger addresses, caller
stacks, and a blocked join after render-thread exit. This patch does not redesign
general concurrent access to Sensors or cancel all work already in flight.

### Build and regression proof

After the normal Gazebo import and checker application above:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/artifacts/gz-sim-build/ros"
mkdir -p "$ROS_LOG_DIR"
tools/check_dependencies --profile all
CMAKE_BUILD_PARALLEL_LEVEL=6 MAKEFLAGS=-j6 tools/gpu-run \
  colcon build --symlink-install --packages-select gz-sim8 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DBUILD_DOCS=OFF -DSKIP_PYBIND11=ON
source install/setup.bash
tools/gpu-run build/gz-sim8/bin/INTEGRATION_sensors_system \
  --gtest_filter='SensorsTest.*' --gtest_death_test_style=threadsafe
```

`SKIP_PYBIND11` omits unused optional Python bindings; ROS `gzserver` uses the
C++ libraries. The upstream integration target carries six bounded shutdown
tests that include the actual Sensors implementation and access its private
state through a friend fixture. They exercise an already blocked waiter,
contention around entry, pre-entry shutdown, absence of update work after a
shutdown wakeup, joining with locks released, repeated/concurrent stops, and independent
instances. The wait-entry repetition is scheduling stress, not a deterministic
interleaving. These tests use synthetic pending-update state; they do not
construct a working rendering engine. Existing upstream rendering tests and
live installed-launch qualification remain required.

The supplemental protocol reproducer is tracked at
[`tools/tests/gz_sim_sensors_shutdown_protocol_test.cc`](../../tools/tests/gz_sim_sensors_shutdown_protocol_test.cc):

```bash
g++ -std=c++17 -O2 -pthread -DWITH_SHUTDOWN_FIX \
  tools/tests/gz_sim_sensors_shutdown_protocol_test.cc -o /tmp/sensors-protocol
timeout 3 /tmp/sensors-protocol
```

Omit the define for the original protocol; use `WITH_NAIVE_SHUTDOWN_FIX` for
predicate-plus-notify without wait-mutex coordination. The naive variant's
failure rate depends on scheduling. This harness mirrors the protocol and
cannot establish the cause of every historical Gazebo hang or qualify a live
plugin.

### Exact diff and installed loading

Generate patches in a clean **full** clone at the pin, using the same Git
settings and object population as a fresh `vcs import`. `git diff HEAD` includes
index hashes whose automatic abbreviation changes with repository size. A
shallow checkout's applicable patch may therefore fail the exact-diff contract.
Preserve the complete diff, including index metadata; remove only the single
space on blank context lines, as the checker already does. Do not relax that
comparison to accommodate a differently generated patch.

Build/install into the normal workspace, source its setup, and verify a live
`ros_gz_sim/gzserver` process directly:

```bash
awk '/libgz-sim8(\.so|-sensors-system|-rendering)/ {print $NF}' /proc/<pid>/maps | sort -u
```

The core library, rendering library, and **Sensors plugin** must resolve to the
workspace's `install/gz-sim8` (or its symlinked `build/gz-sim8` objects). Record
the complete maps, selected search-path environment, object SHA-256 hashes,
`readelf -d` SONAMEs, source pin, and patch hash. Search-path variables and a
loaded core library alone do not prove the Sensors plugin selection. Check
`gz-common` still resolves to the packaged vendor.

Qualification uses a real foreground PTY and one byte `0x03` for Ctrl-C after
sensor activity is verified. Parse every child exit and launch escalation:
launcher exit 0 plus no process leaks is insufficient if `gzserver` exits -9.
Use an isolated ROS domain, Gazebo partition, temporary Clearpath setup, writable
ROS logs, and scoped shutdown on a shared host. Do not change escalation timing
or benchmark signal policy. Sweep qualification uses `--skip-preflight-cleanup`
after inspecting host ownership. Record standard `GetEntities` results for
`^bench_.*$` and an independent unfiltered name check before shutdown.

### Roll back this optional overlay

First inspect and preserve root and nested changes. To disable the overlay
without discarding work, stop only owned launches, move `src/gz-sim`,
`build/gz-sim8`, and `install/gz-sim8` to a uniquely named location outside the
workspace, and rebuild the workspace setup with the preceding dependency
configuration restored. Start a fresh shell, source ROS and the rebuilt
workspace, and verify maps resolve to `/opt/ros/jazzy/opt/gz_sim_vendor`.
Restore this change's manifest/mapping/ignore/documentation edits as one reviewed
set; do not reset a dirty nested repository or remove user files. Keep the
preserved overlay and evidence until the rollback is accepted.

## ros_gz shutdown patch

The Gazebo profile also imports ros_gz **1.0.24**, upstream tag `1.0.24`, commit
`5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2`, matching the installed
`ros-jazzy-ros-gz-*` 1.0.24 binaries. It is a partial overlay: only
`ros_gz_bridge` and `ros_gz_image` are built; every other ros_gz package keeps
resolving to `/opt/ros/jazzy`.

### Defect

In 1.0.24 the bridge node outlives `main()` through two ownership cycles:

- `RosGzBridge` owns its `BridgeHandle`s through `handles_`, and every handle
  owns the node back through a `rclcpp::Node::SharedPtr`;
- every ROS-to-Gazebo subscription callback captures that `SharedPtr` too, so a
  subscription keeps its own node alive.

The node and its middleware endpoints are then destroyed during process and
shared-library finalization, racing logger teardown, and an active
`parameter_bridge` intermittently dies on Ctrl-C with SIGSEGV or `corrupted
double-linked list` (SIGABRT)
([upstream issue #951](https://github.com/gazebosim/ros_gz/issues/951)).
On 2026-10-22 stock bridges crashed in 7 of 19 Gazebo stops; with the first
cycle alone removed, the bidirectional `cmd_vel_bridge` still crashed in 1 of 10.

Separately, `ros_gz_image`'s `image_bridge` subscribes with a raw `this`, and
Gazebo transport does not wait for a callback already in flight when a
subscription or node is destroyed: a frame arriving during teardown reaches a
destroyed handler and the bridge dies with SIGSEGV
([upstream issue #979](https://github.com/gazebosim/ros_gz/issues/979)).

`patches/ros_gz_bridge_shutdown.patch` holds three upstream changes, in order:

1. [#955](https://github.com/gazebosim/ros_gz/pull/955), backport of #952, commit
   `97a4d64f7907ce1aea01cc1bcc525f4ef2b97952` (released in 1.0.25): the handle's
   back-reference becomes `rclcpp::Node::WeakPtr`, and a protected `RosNode()`
   locks it and throws once the node has expired;
2. [#991](https://github.com/gazebosim/ros_gz/pull/991), backport of #977, commit
   `95e6fdea72fb57ef55a4136292dfa62370ab3fb9` (on `jazzy`, not yet in a release):
   subscription callbacks capture the node's logger instead of the node,
   `parameter_bridge` calls `rclcpp::shutdown()` before returning, and the new
   `test_node_lifetime` gtest checks the node is released for every bridge
   direction;
3. a hand backport of [#981](https://github.com/gazebosim/ros_gz/pull/981),
   commit `43ff7ce1292d37dda2e79ff3c7906508334b0f29`, fixed upstream only on
   `ros2` and not applicable to 1.0.24 as-is: `image_bridge`'s Gazebo callback
   co-owns a guard that `~Handler` closes under its mutex, `main()` destroys
   the handlers before the Gazebo node, and the upstream
   `test_image_bridge_teardown` gtest is carried unchanged.

In `ros_gz_bridge` only private sources change; no installed header or
exported symbol does. The patch creates two test files; the checker records
created files as intent-to-add, so they are part of `git diff HEAD` rather
than untracked.

### Build and loading proof

After the Gazebo import and `tools/check_dependencies --apply --profile all`
(which also writes the `COLCON_IGNORE` markers, see
[Partial overlays](#partial-overlays)):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --base-paths src \
  --packages-select ros_gz_bridge ros_gz_image
source install/setup.bash
```

With a benchmark or exploration environment running, every bridge process must
map the overlay library:

```bash
for pid in $(pgrep -f 'ros_gz_bridge/parameter_bridge|ros_gz_image/image_bridge'); do
  awk '/libros_gz_bridge\.so/ {print $NF}' /proc/$pid/maps | sort -u
done
```

Each line must resolve to `install/ros_gz_bridge/lib/libros_gz_bridge.so`, and
the `image_bridge` processes must run `install/ros_gz_image/lib/ros_gz_image/image_bridge`;
the camera must still stream at its configured rate.

### Exit route and rollback

Once apt ships Jazzy ros_gz releases containing all three changes (1.0.25 has
only the first; #981 has no Jazzy backport yet), retire the overlay:
check first that `apt-get -s upgrade` keeps `gz_sim_vendor` at 0.0.13, upgrade,
then roll back as below and verify `/proc/<pid>/maps` resolves
`libros_gz_bridge.so` to `/opt/ros/jazzy`.

To roll back without discarding work, stop only owned launches, move
`src/ros_gz`, `build/ros_gz_{bridge,image}`, and `install/ros_gz_{bridge,image}` to a uniquely
named location outside the workspace, restore this change's manifest, mapping,
ignore, and documentation edits as one reviewed set, rebuild, and start a fresh
shell. Do not reset a dirty nested checkout.

## RViz Image display teardown patch

The optional `viz` profile imports RViz **14.1.23**, upstream tag `14.1.23`,
commit `feb01669f1297df2af755ce9cd2ed18083e7a8b2`, matching the installed
`ros-jazzy-rviz-*` 14.1.23 binaries. It is a partial overlay: only
`rviz_default_plugins` and `rviz_visual_testing_framework` are built. The second
is built only because `rviz_default_plugins` requires it when `BUILD_TESTING` is
on, as it is in the normal workspace build; the visual tests themselves stay off
(`EnableVisualTests` unset). `rviz2`, `rviz_common`, and `rviz_rendering` keep
resolving to `/opt/ros/jazzy`; pluginlib loads the overlay's
`librviz_default_plugins.so` through the ament index.

### Defect

`ImageTransportDisplay::unsubscribe()` disconnected its
`message_filters::Connection` and then destroyed the `SubscriberFilter`, but kept
the connection. The connection's disconnect closure holds the raw address of
the filter's `Signal1`, so a second `unsubscribe()` -- `onDisable()` followed by
the destructor -- locks a mutex inside the destroyed filter. The lock fails with
`EINVAL` or `EOWNERDEAD`, `std::system_error` escapes the destructor, and RViz
aborts (SIGABRT) on shutdown. The 2026-10-19 diagnosis traced the throw to
`Signal1<Image>::removeCallback` under `~ImageDisplay`; on 2026-10-22 one
SIGINT aborted stock RViz in 6 of 20 starts with `benchmark.rviz` and 10 of 20
with `exploration.rviz`. The defect entered in 14.1.21 (backport #1745); the
apt candidate 14.1.24 still has it.

`patches/rviz_image_display_unsubscribe.patch` disconnects only while the
filter exists and replaces the connection with an empty one before the filter
is reset, so a repeated `unsubscribe()` does nothing. It changes one inline
method in an installed header that only `rviz_default_plugins` compiles.

### Build and loading proof

After `vcs import < dependencies/viz.repos` and
`tools/check_dependencies --apply --profile all`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --base-paths src \
  --packages-select rviz_visual_testing_framework rviz_default_plugins
source install/setup.bash
```

With RViz running, its maps must show the overlay plugin library:

```bash
awk '/librviz_default_plugins\.so/ {print $NF}' /proc/$(pgrep -x rviz2)/maps | sort -u
```

The line must resolve to `install/rviz_default_plugins/lib/`.

### Coupling and rollback

The pin must match the installed RViz binaries: an apt upgrade to 14.1.24
requires moving the pin and rebasing the patch through the refresh procedure
above, in the same change. To roll back, stop only owned RViz processes, move
`src/rviz`, `build/rviz_*`, and `install/rviz_*` outside the workspace, restore
this change's manifest, mapping, ignore, and documentation edits as one reviewed
set, rebuild, and start a fresh shell.

## Archived evidence

- [Gazebo Sensors shutdown races and qualification](../../archive/engineering/2026-10-18-gazebo-sensors-shutdown.md)
- [ros_gz bridge ownership cycles and image_bridge teardown](../../archive/engineering/2026-10-22-ros-gz-bridge-shutdown.md)
- [RViz Image display teardown abort](../../archive/engineering/2026-10-22-rviz-image-display-teardown.md)
