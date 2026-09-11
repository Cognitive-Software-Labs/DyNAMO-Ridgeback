#!/usr/bin/env bash
# Restore the workspace's dependency checkouts to their exact pinned state.
#
#   bash tools/setup_deps.sh          # import pins + apply patches
#   bash tools/setup_deps.sh --check  # verify only, change nothing
#
# src/clearpath_* and src/slam_toolbox are gitignored with zero tracked files,
# so the only record of what they should contain is `.repos` (exact commits)
# plus `patches/` (the local modifications). This script is what turns those
# two records back into a working tree, and `--check` is what proves a live
# tree still matches them.
#
# Both operations are idempotent: an already-imported, already-patched tree
# comes out unchanged.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

CHECK_ONLY=false
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=true

# patch file -> dependency directory it applies to
declare -A PATCH_TARGET=(
    [patches/clearpath_gz_customizations.patch]=src/clearpath_simulator
    [patches/slam_toolbox_tf_namespace.patch]=src/slam_toolbox
)

fail=0
note() { printf '  %s\n' "$*"; }

# ---- 1. the pinned checkouts -------------------------------------------------
if ! $CHECK_ONLY; then
    echo "== importing .repos pins =="
    if ! command -v vcs >/dev/null 2>&1; then
        echo "vcs (python3-vcstool) not found; cannot import" >&2
        exit 1
    fi
    # NOTE: paths in .repos already start with src/, so import from the repo
    # root. `vcs import src < .repos` would create src/src/...
    vcs import < .repos
fi

echo "== checking pinned commits =="
# Parse `.repos` for "src/<name>:" blocks and their `version:` line, ignoring
# comments so the annotations in that file cannot be mistaken for values.
while read -r dir want; do
    [[ -z "$dir" ]] && continue
    if [[ ! -d "$dir/.git" ]]; then
        note "$dir: MISSING (run without --check to import)"
        fail=1
        continue
    fi
    have="$(git -C "$dir" rev-parse HEAD)"
    if [[ "$have" == "$want" ]]; then
        note "$dir: pinned commit OK (${have:0:7})"
    else
        note "$dir: WRONG COMMIT have=${have:0:7} want=${want:0:7}"
        fail=1
    fi
done < <(awk '
    /^[[:space:]]*#/ { next }
    /^[[:space:]]{2}src\// { gsub(/[[:space:]:]/, "", $0); dir=$0; next }
    /^[[:space:]]*version:/ { print dir, $2 }
' .repos)

# ---- 2. the patches ---------------------------------------------------------
echo "== checking patches =="
for patch in "${!PATCH_TARGET[@]}"; do
    target="${PATCH_TARGET[$patch]}"
    if [[ ! -d "$target/.git" ]]; then
        note "$(basename "$patch"): target $target missing"
        fail=1
        continue
    fi
    # Already applied? A clean REVERSE application is the test -- it means
    # every hunk is present in the tree exactly as filed.
    if git -C "$target" apply --reverse --check "$REPO_ROOT/$patch" 2>/dev/null; then
        note "$(basename "$patch"): already applied"
    elif $CHECK_ONLY; then
        note "$(basename "$patch"): NOT applied"
        fail=1
    elif git -C "$target" apply --check "$REPO_ROOT/$patch" 2>/dev/null; then
        git -C "$target" apply "$REPO_ROOT/$patch"
        note "$(basename "$patch"): applied"
    else
        note "$(basename "$patch"): DOES NOT APPLY -- the pin probably moved"
        fail=1
    fi
done

# ---- 3. drift beyond the patches -------------------------------------------
# A dependency tree should contain the pinned commit plus its patch and
# nothing else. Anything extra is an undocumented hand-edit that will vanish
# the next time the tree is recreated -- turn it into a patch instead.
echo "== checking for undocumented drift =="
for target in "${PATCH_TARGET[@]}"; do
    [[ -d "$target/.git" ]] || continue
    live="$(git -C "$target" diff | grep -E '^[+-]' | grep -vE '^(\+\+\+|---)' | sort | md5sum)"
    filed=""
    for patch in "${!PATCH_TARGET[@]}"; do
        [[ "${PATCH_TARGET[$patch]}" == "$target" ]] || continue
        filed="$(grep -E '^[+-]' "$patch" | grep -vE '^(\+\+\+|---)' | sort | md5sum)"
    done
    if [[ "$live" == "$filed" ]]; then
        note "$target: tracked diff == filed patch"
    else
        note "$target: DRIFT -- working diff differs from the filed patch"
        note "    capture it:  git -C $target diff > patches/<name>.patch"
        fail=1
    fi
done
for target in src/clearpath_common src/clearpath_config src/clearpath_msgs \
              src/clearpath_simulator src/slam_toolbox; do
    [[ -d "$target/.git" ]] || continue
    n="$(git -C "$target" status --porcelain | grep -c '^??' || true)"
    [[ "$n" -eq 0 ]] || { note "$target: $n untracked file(s), not captured by any patch"; fail=1; }
done

echo
if [[ "$fail" -eq 0 ]]; then
    echo "dependencies match .repos + patches/"
else
    echo "dependencies DO NOT match .repos + patches/ (see above)" >&2
fi
exit "$fail"
