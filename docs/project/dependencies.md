# External dependency management

This page owns the repository's procedure for importing, patching, verifying,
refreshing, and rolling back external ROS source repositories. The root README
keeps the public installation commands; dated refresh results belong in
[engineering archive](../../archive/engineering/README.md).

## Model and ownership

The dependencies under `src/clearpath_*`, `src/slam_toolbox`, `src/urg_node`,
`src/gz-sim`, `src/ros_gz`, and `src/rviz` are standalone,
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
  recorded patches.

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
- patch conflicts or any changes beyond the recorded patches;
- a recorded patch that is not exactly the diff Git generates for its change.

Do not respond to a failure by resetting the nested checkout. Inspect it first;
it may contain intentional work that the root repository cannot see.

## Stacked patches

A checkout may carry more than one recorded patch when the changes are
unrelated: each keeps its own file, name and runbook section. The checker's
`PATCHES` map lists them per checkout, in order and separated by spaces. Each
patch is recorded against the tree its predecessors produce. The checker
applies the list in order to a temporary index built from the pinned commit,
never to the working tree, and requires:

- every patch to apply on top of the ones before it;
- every patch file to equal the diff Git generates for its own step, apart from
  the blank-context-line normalisation the checker documents;
- the checkout's complete `git diff HEAD` to equal the combined result.

A checkout that holds only a leading part of the list, for example one patched
before a later entry was added, fails in read-only mode naming the missing
patches; `--apply` applies exactly those, in order. To record a new entry,
start from a checkout carrying the earlier patches and write only the new
change, so the patch file holds that change alone:

```bash
git -C src/<repository> diff --no-ext-diff HEAD -- <changed files> \
  | sed 's/^ $//' > patches/<name>.patch
```

This command records the whole change only when no earlier patch touches the
same files. Otherwise, diff against a tree with only the earlier patches
applied.

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
then generate the root patch from the complete nested-repository diff. For
[stacked patches](#stacked-patches), rebase them in order and record each one
against its predecessors.

The maintained mappings are:

| Dependency checkout | Root patches, in order |
|---|---|
| `src/clearpath_simulator` | `patches/clearpath_gz_customizations.patch` |
| `src/clearpath_common` | `patches/clearpath_realsense_sim_frames.patch`, `patches/clearpath_ust10lx_sim_lidar.patch` |
| `src/slam_toolbox` | `patches/slam_toolbox_tf_namespace.patch` |
| `src/urg_node` | `patches/urg_node_reconnect_recovery.patch` |
| `src/gz-sim` | `patches/gz_sim_sensors_shutdown.patch` |
| `src/ros_gz` | `patches/ros_gz_bridge_shutdown.patch` |
| `src/rviz` | `patches/rviz_shutdown.patch` |

Update all affected manifest pins and patch files in the same root commit.

### 4. Prove the clean-import contract

Against clean checkouts at the proposed pins:

- confirm each patch passes `git apply --check` before application;
- run `tools/check_dependencies --apply --profile all`;
- rerun `tools/check_dependencies --profile all` and require a clean result;
- confirm each patched checkout's complete `git diff HEAD` exactly equals its
  recorded root patch, or the combined result of its stacked patches;
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

## Maintained patches

Patch-specific defects, build and loading proof, and rollback are in
[maintained dependency patches](dependency_patches.md):

- [Gazebo Sensors shutdown](dependency_patches.md#gazebo-sensors-shutdown-patch)
- [ros_gz shutdown](dependency_patches.md#ros_gz-shutdown-patch)
- [RViz shutdown ownership](dependency_patches.md#rviz-shutdown-ownership-patch)
- [UST-10LX Gazebo LiDAR](dependency_patches.md#ust-10lx-gazebo-lidar-patch)
