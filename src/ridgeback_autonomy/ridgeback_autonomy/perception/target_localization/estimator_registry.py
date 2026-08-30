from __future__ import annotations

PUBLIC_ESTIMATOR_ORDER = (
    'pointcloud',
    'projective_ranging',
    'euclidean_reconstruction',
    'polar_profiling',
)

# The organized-``PointCloud2`` row (target_pointcloud_measurement_node), the one
# path that reads the cloud directly rather than deprojecting depth itself --
# which is what the mask stack's deprojection was validated against.
POINTCLOUD_ESTIMATORS = frozenset({'pointcloud'})

# The mask-based localization rows (target_mask_measurement_node), all sharing the
# mask front-end and the camera-optical frame, independent of the pointcloud
# row. The estimator key is already the self-describing path name.
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
# ``measurement_pipeline.MASK_GATES`` by value (cross-checked in tests).
MASK_GATE_BOX = 'box'
MASK_GATE_SILHOUETTE = 'silhouette'
MASK_GATES = (MASK_GATE_BOX, MASK_GATE_SILHOUETTE)
MASK_GATE_DEFAULT = MASK_GATE_BOX

ESTIMATOR_FIELD_KEYS = {
    'pointcloud': 'pointcloud_distance_m',
    'projective_ranging': 'projective_ranging_distance_m',
    'euclidean_reconstruction': 'euclidean_reconstruction_distance_m',
    'polar_profiling': 'polar_profiling_distance_m',
}

# Per-detection miss-reason arrays on TargetMeasurements, for the mask estimators
# that write a status. The pointcloud row has no status field; the benchmark
# infers a coarse OK/UNSET for it from finiteness.
ESTIMATOR_STATUS_FIELD_KEYS = {
    'projective_ranging': 'projective_ranging_status',
    'euclidean_reconstruction': 'euclidean_reconstruction_status',
    'polar_profiling': 'polar_profiling_status',
}

# Detection-model (forward, lateral) attribute names for the estimators that
# emit a planar position -- used as the sensor-side locator for the scoring
# assignment. Every registered estimator places its detection, and the display
# surfaces rely on that: a distance-only row would have no direction of its own
# and would draw its ring down the boresight, wrong by the whole lateral
# component. ``test_target_visualization`` asserts this covers PUBLIC_ESTIMATOR_ORDER.
ESTIMATOR_POSITION_ATTRS = {
    'pointcloud': ('pointcloud_forward_m', 'pointcloud_lateral_m'),
    'projective_ranging': ('projective_ranging_forward_m', 'projective_ranging_lateral_m'),
    'euclidean_reconstruction': ('euclidean_reconstruction_forward_m', 'euclidean_reconstruction_lateral_m'),
    'polar_profiling': ('polar_profiling_forward_m', 'polar_profiling_lateral_m'),
}

ESTIMATOR_LABELS = {
    'pointcloud': 'Point Cloud',
    'projective_ranging': 'Projective Ranging',
    'euclidean_reconstruction': 'Euclidean Reconstruction',
    'polar_profiling': 'Polar Profiling',
}

# The same names as column HEADERS rather than row labels. A wide HUD layout
# stacks the four estimators side by side, and 'Euclidean Reconstruction' at 24
# columns is four times the width the number under it needs -- the layout would
# be all header and no reading. Kept distinct from the prose labels above so
# neither has to compromise for the other's surface.
ESTIMATOR_SHORT_LABELS = {
    'pointcloud': 'Cloud',
    'projective_ranging': 'Project',
    'euclidean_reconstruction': 'Euclid',
    'polar_profiling': 'Polar',
}


def display_distance(read_distance, index: int) -> float | None:
    """The one distance that speaks for a detection, in canonical order.

    ``read_distance(estimator, index)`` is supplied by the caller, so the same
    rule serves the ``TargetMeasurements`` array shape and the ``Detection``
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


def selected_pointcloud_estimators(selected_estimators: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(
        estimator for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in POINTCLOUD_ESTIMATORS and estimator in selected_estimators
    )


def selected_mask_estimators(selected_estimators: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(
        estimator for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in MASK_ESTIMATORS and estimator in selected_estimators
    )


def uses_pointcloud_estimators(selected_estimators: tuple[str, ...]) -> bool:
    return any(estimator in POINTCLOUD_ESTIMATORS for estimator in selected_estimators)


def uses_mask_estimators(selected_estimators: tuple[str, ...]) -> bool:
    return any(estimator in MASK_ESTIMATORS for estimator in selected_estimators)
