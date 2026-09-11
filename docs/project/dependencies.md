# External dependency management

This page owns the repository's procedure for importing, patching, verifying,
refreshing, and rolling back external ROS source repositories. The root README
keeps the public installation commands; dated refresh results belong in
[`docs/history/`](../history/).

## Model and ownership

The dependencies under `src/clearpath_*` and `src/slam_toolbox` are standalone,
Git-ignored repositories populated by `vcstool`. They are not Git submodules and
are not stored in the root repository's history.

The compatibility unit has three root-owned parts:

- [`.repos`](../../.repos) owns upstream URLs and immutable commit pins.
- [`patches/`](../../patches/) owns intentional changes to those pinned sources.
- [`tools/check_dependencies`](../../tools/check_dependencies) verifies that
  every checkout is at its pin and contains either no changes or exactly its
  recorded patch.

Consequently, root `git status` cannot reveal changes inside the nested
repositories. Use the checker for the workspace-wide contract and `git -C
src/<repository> status` when inspecting one checkout directly.

## Fresh checkout and routine verification

From the workspace root:

```bash
vcs import < .repos
tools/check_dependencies --apply
```

`vcs import` creates the nested repositories at the exact commits in `.repos`.
The checker then applies missing maintained patches. Its `--apply` mode is
idempotent: an exact already-applied patch is accepted.

Before later builds, use the read-only form:

```bash
tools/check_dependencies
```

The checker rejects:

- a missing repository or a checkout at the wrong commit;
- untracked files inside a dependency;
- tracked modifications in an unpatched dependency;
- a missing maintained patch in read-only mode;
- patch conflicts or any changes beyond the recorded patch.

Do not respond to a failure by resetting the nested checkout. Inspect it first;
it may contain intentional work that the root repository cannot see.

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
an explicit reason in the refresh history.

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

Update all `.repos` pins and patch files in the same root commit.

### 4. Prove the clean-import contract

Against clean checkouts at the proposed pins:

- confirm each patch passes `git apply --check` before application;
- run `tools/check_dependencies --apply`;
- rerun `tools/check_dependencies` and require a clean result;
- confirm each patched checkout's complete `git diff HEAD` exactly equals its
  recorded root patch;
- remove and reapply at least one patch to exercise the missing-patch and
  idempotence paths.

`vcstool validate` is not the acceptance gate here: versions in `.repos` are raw
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

Move the completed item out of the backlog and add a dated history page containing
the revision table, useful upstream changes, local-edit decisions, validation
evidence, limitations, and rollback boundary. Commit the manifest, patches,
checker changes, current documentation, and validation history as one logical
deliverable.

## Rollback

Treat pins and patches as an inseparable set. The clean rollback is to restore
the previous root commit's `.repos` and `patches/`, recreate or switch all nested
checkouts to those pins, and run the restored checker with `--apply`.

Do not selectively combine an old dependency pin with a new patch. Restore a
saved live-checkout stash only on the revision where it was created, inspect the
result, and never discard it merely because the refreshed manifest no longer
accepts it.
