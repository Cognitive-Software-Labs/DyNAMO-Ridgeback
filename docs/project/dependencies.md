# External dependency management

This page owns the repository's procedure for importing, patching, verifying,
refreshing, and rolling back external ROS source repositories. The root README
keeps the public installation commands; dated refresh results belong in
[engineering archive](../../archive/engineering/README.md).

## Model and ownership

The dependencies under `src/clearpath_*`, `src/slam_toolbox`, and `src/gz-sim` are standalone,
Git-ignored repositories populated by `vcstool`. They are not Git submodules and
are not stored in the root repository's history.

The compatibility unit has four root-owned parts:

- [`dependencies/core.repos`](../../dependencies/core.repos) owns the source
  dependencies shared by every deployment.
- [`dependencies/gz.repos`](../../dependencies/gz.repos) owns the optional
  Gazebo-only `clearpath_simulator` and `gz-sim` pins.
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

# Verify both profiles after a Gazebo import
tools/check_dependencies --apply --profile all
```

On an Isaac-only or hardware host, omit the Gazebo import and run
`tools/check_dependencies --apply --profile core`. `--profile gz` checks only
the optional Gazebo checkouts. `vcs import` creates the nested repositories
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

## Archived evidence

- [Gazebo Sensors shutdown races and qualification](../../archive/engineering/2026-10-18-gazebo-sensors-shutdown.md)
