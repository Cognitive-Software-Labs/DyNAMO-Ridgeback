from __future__ import annotations


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
