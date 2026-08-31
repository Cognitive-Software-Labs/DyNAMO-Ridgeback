"""Benchmark-only output, run-folder, and display naming."""

from __future__ import annotations

import os
import re

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_LABELS,
    MASK_GATE_DEFAULT,
    MASK_GATE_SILHOUETTE,
    uses_mask_estimators,
)


def benchmark_output_name(
    estimator: str,
    depth_source: str,
    isolation_2d: str,
    isolation_3d: str,
    mask_gate: str = MASK_GATE_DEFAULT,
) -> str:
    """Filesystem-safe self-describing name for a row's output CSV.

    The snake_case form of the doc prose, in prose order (gate, source, path)
    plus the foreground-isolation recipe -- the four axes a depth-path row
    varies along, so the path name alone collides across runs that vary any of
    the others: ``box_gated_stereoscopic_projective_ranging_nearest_mode_histogram``.

    Silhouette rows drop the isolation token
    (``silhouette_gated_stereoscopic_projective_ranging``): the tight branches
    never run an isolation recipe, and naming code that did not execute would
    mislabel the row.

    Polar profiling is mask-based but LiDAR-sourced, so only the gate applies:
    ``box_gated_polar_profiling``. Non-mask estimators keep their plain key
    (none of these axes apply to them). The spaced, isolation-free prose used
    in logs and the summary column is ``benchmark_display_name``.
    """

    if estimator in DEPTH_PATH_ESTIMATORS:
        if mask_gate == MASK_GATE_SILHOUETTE:
            return f'{mask_gate}_gated_{depth_source}_{estimator}'
        isolation = isolation_2d if estimator == 'projective_ranging' else isolation_3d
        return f'{mask_gate}_gated_{depth_source}_{estimator}_{isolation}'
    if estimator == 'polar_profiling':
        return f'{mask_gate}_gated_{estimator}'
    return estimator


def scenario_slug(scenario_path: str) -> str:
    """Short filesystem-safe name for a scenario file, for the run folder.

    ``benchmark_scenarios_full.yaml`` -> ``full``; anything else keeps its
    stem. The shared prefix carries no information once it is inside a
    ``artifacts/benchmarks`` folder, and dropping it keeps the name short enough
    to read at a glance.
    """

    stem = os.path.splitext(os.path.basename(scenario_path or ''))[0]
    if stem.startswith('benchmark_scenarios_'):
        stem = stem[len('benchmark_scenarios_'):]
    stem = re.sub(r'[^a-z0-9_-]+', '_', stem.lower()).strip('_')
    return stem or 'scenario'


def benchmark_run_folder_name(
    run_label: str,
    scenario_path: str,
    mask_gate: str,
    depth_source: str,
    selected_estimators: tuple[str, ...],
) -> str:
    """Run folder name: timestamp first, then the axes that change the results.

    The timestamp leads so the directory keeps sorting chronologically -- the
    usual question is "what did I run last". After it come only axes that
    actually applied: the mask gate is omitted when no mask estimator ran, and
    the depth source when no depth-path estimator ran, so a pointcloud-only run
    is not labelled with a segmentation gate it never used.
    """

    parts = [run_label, scenario_slug(scenario_path)]
    if uses_mask_estimators(selected_estimators):
        parts.append(mask_gate)
    if any(estimator in DEPTH_PATH_ESTIMATORS for estimator in selected_estimators):
        parts.append(depth_source)
    return '_'.join(part for part in parts if part)


def benchmark_display_name(
    estimator: str,
    depth_source: str,
    mask_gate: str = MASK_GATE_DEFAULT,
) -> str:
    """Human-readable prose name for logs and the summary ``estimator`` column.

    The doc prose form (gate, source, path), spaced and lower-case, without the
    isolation recipe (constant within a run): ``box-gated stereoscopic
    projective ranging``. Polar profiling drops the source
    (``box-gated polar profiling``); non-mask rows use their fixed label
    (``Point Cloud``).
    """

    path_words = estimator.replace('_', ' ')
    if estimator in DEPTH_PATH_ESTIMATORS:
        return f'{mask_gate}-gated {depth_source} {path_words}'
    if estimator == 'polar_profiling':
        return f'{mask_gate}-gated {path_words}'
    return ESTIMATOR_LABELS[estimator]
