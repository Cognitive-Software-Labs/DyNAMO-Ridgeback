from __future__ import annotations

from dataclasses import dataclass
import math
import os
import re


PUBLIC_ESTIMATOR_ORDER = (
    'rgb',
    'sensor_depth',
    'depth_anything',
    'pointcloud',
    'lidar',
    'projective_ranging',
    'euclidean_reconstruction',
    'polar_profiling',
)

IMAGE_BACKED_ESTIMATORS = frozenset({
    'rgb',
    'sensor_depth',
    'depth_anything',
})

RGB_DEBUG_VIEW_ESTIMATORS = frozenset({
    'pointcloud',
    'lidar',
    'projective_ranging',
    'euclidean_reconstruction',
    'polar_profiling',
})

CAMERA_ESTIMATORS = frozenset({
    'rgb',
    'sensor_depth',
    'depth_anything',
    'pointcloud',
})

LIDAR_ESTIMATORS = frozenset({'lidar'})

# The mask-based localization rows (g1_mask_measurement_node), all sharing the
# mask front-end and the camera-optical frame, independent of the legacy camera
# estimator stack. The estimator key is already the self-describing path name.
MASK_ESTIMATORS = frozenset({
    'projective_ranging',
    'euclidean_reconstruction',
    'polar_profiling',
})

# The subset whose accuracy folds the aligned-depth source and a foreground
# isolation recipe into the output name. Polar profiling is mask-based too but
# LiDAR-sourced -- no depth source, no isolation recipe -- so it is excluded.
DEPTH_PATH_ESTIMATORS = frozenset({'projective_ranging', 'euclidean_reconstruction'})

# The mask front-end (gate) axis: the rasterized box (rect) gate and the
# silhouette (tight segmentation) gate. A run picks one via the ``mask_gate``
# parameter; the value folds into every mask-row output name. Tokens mirror
# ``g1_mask_measurement_node.MASK_GATES`` by value (cross-checked in tests).
MASK_GATE_BOX = 'box'
MASK_GATE_SILHOUETTE = 'silhouette'
MASK_GATES = (MASK_GATE_BOX, MASK_GATE_SILHOUETTE)
MASK_GATE_DEFAULT = MASK_GATE_BOX

ESTIMATOR_FIELD_KEYS = {
    'rgb': 'rgb_distance_m',
    'sensor_depth': 'sensor_depth_distance_m',
    'depth_anything': 'mono_depth_distance_m',
    'pointcloud': 'pointcloud_distance_m',
    'lidar': 'lidar_distance_m',
    'projective_ranging': 'projective_ranging_distance_m',
    'euclidean_reconstruction': 'euclidean_reconstruction_distance_m',
    'polar_profiling': 'polar_profiling_distance_m',
}

# Per-detection miss-reason arrays on G1Measurements, for the mask estimators
# that write a status. The legacy estimators have no status field; the benchmark
# infers a coarse OK/UNSET for them from finiteness.
ESTIMATOR_STATUS_FIELD_KEYS = {
    'projective_ranging': 'projective_ranging_status',
    'euclidean_reconstruction': 'euclidean_reconstruction_status',
    'polar_profiling': 'polar_profiling_status',
}

# Ground truth the benchmark runner republishes during a trial's capture
# window and the overlay renders as a reference label line. Benchmark-only:
# nothing publishes this topic in the exploration stack, so the overlay line
# never appears there.
GROUND_TRUTH_TOPIC = 'benchmark/g1/ground_truth'

# How long a truth message stays displayable. The runner publishes only while a
# capture window is open, so the gate is what makes the line disappear between
# trials instead of sitting next to a target that has already been teleported
# away. Defined here rather than in each consumer: the viz node and the overlay
# have to expire on the same schedule or they disagree about the same message.
TRUTH_MAX_AGE_S = 3.0

# Detection-model (forward, lateral) attribute names for the estimators that
# emit a planar position -- used as the sensor-side locator for the scoring
# assignment. sensor_depth / depth_anything report only a distance, so they are
# absent here and fall back to the 1-D distance locator.
ESTIMATOR_POSITION_ATTRS = {
    'rgb': ('rgb_forward_m', 'rgb_lateral_m'),
    'pointcloud': ('pointcloud_forward_m', 'pointcloud_lateral_m'),
    'lidar': ('lidar_forward_m', 'lidar_lateral_m'),
    'projective_ranging': ('projective_ranging_forward_m', 'projective_ranging_lateral_m'),
    'euclidean_reconstruction': ('euclidean_reconstruction_forward_m', 'euclidean_reconstruction_lateral_m'),
    'polar_profiling': ('polar_profiling_forward_m', 'polar_profiling_lateral_m'),
}

ESTIMATOR_LABELS = {
    'rgb': 'RGB',
    'sensor_depth': 'Sensor Depth',
    'depth_anything': 'Depth-Anything',
    'pointcloud': 'Point Cloud',
    'lidar': 'LiDAR',
    'projective_ranging': 'Projective Ranging',
    'euclidean_reconstruction': 'Euclidean Reconstruction',
    'polar_profiling': 'Polar Profiling',
}


@dataclass(frozen=True)
class TruthReading:
    """One trial's ground truth as the display surfaces consume it.

    ``trial_id`` names the trial the numbers belong to, so a reading can be
    checked against the scene on screen instead of being taken on trust.
    """

    lateral_m: float
    forward_m: float
    distance_m: float
    trial_id: str


def truth_reading(msg, now_nanoseconds: int, max_age_s: float = TRUTH_MAX_AGE_S):
    """Unpack a truth message, or ``None`` once it is too old to display.

    Age is measured on the message STAMP, never on when it arrived. A message
    delayed behind a full subscription queue is stale data however recently it
    was handed to the callback, and a receipt-time gate cannot tell those two
    apart -- it reports the delay as freshness and pins the previous trial's
    truth under the current trial's scene.

    Packed by the runner as x=lateral, y=forward, z=distance: the base-frame
    planar measurement every benchmark row shares, not a 3D point in any TF
    frame. The trial id rides in ``header.frame_id`` for the same reason.
    """

    if msg is None:
        return None
    stamp = msg.header.stamp
    stamp_nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if now_nanoseconds - stamp_nanoseconds > max_age_s * 1_000_000_000:
        return None
    return TruthReading(
        lateral_m=float(msg.point.x),
        forward_m=float(msg.point.y),
        distance_m=float(msg.point.z),
        trial_id=str(msg.header.frame_id),
    )


def display_distance(read_distance, index: int) -> float | None:
    """The one distance that speaks for a detection, in canonical order.

    ``read_distance(estimator, index)`` is supplied by the caller, so the same
    rule serves the ``G1Measurements`` array shape and the ``Detection``
    dataclass shape. First usable estimator wins rather than the smallest one:
    producers see different estimator sets, and a rule that ranks by whichever
    number happens to be lowest would let two nodes disagree about the same
    scene far more often than one pinned to a fixed order does.
    """

    for estimator in PUBLIC_ESTIMATOR_ORDER:
        value = read_distance(estimator, index)
        if value is not None:
            return value
    return None


def display_bearing(read_position, index: int) -> float | None:
    """Bearing of a detection in the base frame, in canonical order.

    ``read_position(estimator, index)`` returns that estimator's
    ``(forward_m, lateral_m)`` or ``None``. Ordered exactly like
    ``display_distance`` -- first usable answer wins -- so the direction a
    surface draws and the distance it prints are picked by one rule rather than
    two that can disagree about which estimator speaks for a detection.

    Exists for the depth-only rows (``sensor_depth``, ``depth_anything``), which
    publish a planar distance and no position at all. A borrowed bearing is a
    visualization convenience, never an input to scoring: the benchmark's
    locator stays ``ESTIMATOR_POSITION_ATTRS``, where those two are absent on
    purpose.
    """

    for estimator in PUBLIC_ESTIMATOR_ORDER:
        position = read_position(estimator, index)
        if position is None:
            continue
        forward_m, lateral_m = position
        if forward_m is None or lateral_m is None:
            continue
        if forward_m == 0.0 and lateral_m == 0.0:
            continue
        return math.atan2(lateral_m, forward_m)
    return None


def nearest_instance_index(count: int, read_distance) -> int | None:
    """Index of the closest detection, or ``None`` when nothing is rankable.

    The single source for "which instance do the visualizations speak for".
    Ordering by ``(distance, index)`` breaks a tie on the lower index, so two
    equidistant robots always resolve the same way instead of following
    whichever the detector happened to list first.
    """

    ranked = [
        (distance, index)
        for index in range(count)
        if (distance := display_distance(read_distance, index)) is not None
    ]
    return min(ranked)[1] if ranked else None


def parse_mask_gate(raw_mask_gate: str | None) -> str:
    """Validate the run-level ``mask_gate`` parameter (``box`` | ``silhouette``)."""

    gate = (raw_mask_gate or '').strip()
    if not gate:
        return MASK_GATE_DEFAULT
    if gate not in MASK_GATES:
        supported = ', '.join(MASK_GATES)
        raise ValueError(f'Unsupported mask_gate "{gate}". Expected one of: {supported}')
    return gate


def parse_estimators(raw_estimators: str | None) -> tuple[str, ...]:
    text = (raw_estimators or '').strip()
    if not text or text.lower() == 'all':
        return PUBLIC_ESTIMATOR_ORDER

    requested: set[str] = set()
    for token in text.split(','):
        estimator = token.strip()
        if not estimator:
            continue
        if estimator == 'mono_depth':
            raise ValueError('Use "depth_anything" instead of "mono_depth" in benchmark configuration.')
        if estimator not in ESTIMATOR_FIELD_KEYS:
            supported = ', '.join(PUBLIC_ESTIMATOR_ORDER)
            raise ValueError(
                f'Unsupported estimator "{estimator}". Expected a subset of: {supported}'
            )
        requested.add(estimator)

    if not requested:
        return PUBLIC_ESTIMATOR_ORDER

    return tuple(
        estimator for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in requested
    )


def selected_camera_estimators(selected_estimators: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(
        estimator for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in CAMERA_ESTIMATORS and estimator in selected_estimators
    )


def selected_mask_estimators(selected_estimators: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(
        estimator for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in MASK_ESTIMATORS and estimator in selected_estimators
    )


def uses_camera_estimators(selected_estimators: tuple[str, ...]) -> bool:
    return any(estimator in CAMERA_ESTIMATORS for estimator in selected_estimators)


def uses_lidar_estimators(selected_estimators: tuple[str, ...]) -> bool:
    return any(estimator in LIDAR_ESTIMATORS for estimator in selected_estimators)


def uses_mask_estimators(selected_estimators: tuple[str, ...]) -> bool:
    return any(estimator in MASK_ESTIMATORS for estimator in selected_estimators)


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
    ``benchmark-results`` folder, and dropping it keeps the name short enough
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
    the depth source when no depth-path estimator ran, so a lidar-only run is
    not labelled with a segmentation gate it never used.
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
    (``RGB``, ``LiDAR``, ...).
    """

    path_words = estimator.replace('_', ' ')
    if estimator in DEPTH_PATH_ESTIMATORS:
        return f'{mask_gate}-gated {depth_source} {path_words}'
    if estimator == 'polar_profiling':
        return f'{mask_gate}-gated {path_words}'
    return ESTIMATOR_LABELS[estimator]
