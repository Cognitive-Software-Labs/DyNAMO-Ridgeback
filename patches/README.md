# Dependency patches

`src/clearpath_*` and `src/slam_toolbox` are **gitignored with zero tracked
files**. They are upstream clones, not our code, so nothing in them is
committed here. That means a dependency tree is only ever as reproducible as
the two records that describe it:

| record | what it pins |
|---|---|
| `../.repos` | the exact upstream **commit** for each dependency |
| this directory | the **local modifications** made on top of that commit |

Anything not in one of those two is lost the next time the workspace is
recreated. That is the whole reason this directory exists.

## Restore or verify a checkout

```bash
bash tools/setup_deps.sh            # import the pins, apply the patches
bash tools/setup_deps.sh --check    # verify a live tree, change nothing
```

Both are idempotent. `--check` is the one worth remembering: it confirms every
dependency sits on its pinned commit, that each patch is applied exactly as
filed, and that there is **no drift** — no hand-edit or untracked file beyond
what the patches describe.

## Adding or updating a patch

Never leave a hand-edit in a dependency tree. Capture it:

```bash
git -C src/<dep> diff > patches/<name>.patch
```

then register it in the `PATCH_TARGET` map at the top of
`tools/setup_deps.sh`, and re-run `--check` to confirm the tree is clean
again. If a change adds *new* files, `git diff` will not capture them — use
`git -C src/<dep> add -N <paths>` first so they appear as additions.

## Moving a pin

Patches only apply against a known base, so a pin bump and a patch refresh go
together:

1. check out the new upstream commit
2. re-apply the patches and fix any conflicts
3. update the hash **and** the date comment in `.repos`
4. `bash tools/setup_deps.sh --check`

## Why the pins are commits and not `jazzy`

`.repos` pinned branches (`version: jazzy`) until 2026-09-11, which made
`vcs import` non-deterministic — you got whatever upstream HEAD was that day.
On 2026-09-10 all five dependencies came down different from the working
checkout, `clearpath_simulator` by five months, and
`clearpath_gz_customizations.patch` stopped applying at all. A faithful patch
is worthless against an unknown base.

## Current patches

| patch | target | what it does |
|---|---|---|
| `clearpath_gz_customizations.patch` | `src/clearpath_simulator` | gz launch/GUI customizations (3 files) |
| `slam_toolbox_tf_namespace.patch` | `src/slam_toolbox` | namespaces slam_toolbox's TF lookups (1 file) |

Both were verified on 2026-09-11 to be byte-identical to the live working
diffs, with no untracked files in any dependency.
