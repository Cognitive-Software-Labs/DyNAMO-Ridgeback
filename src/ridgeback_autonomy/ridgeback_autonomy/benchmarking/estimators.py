from __future__ import annotations


PUBLIC_ESTIMATOR_ORDER = (
    'rgb',
    'sensor_depth',
    'depth_anything',
    'pointcloud',
    'lidar',
    'path_a',
    'path_b',
)

IMAGE_BACKED_ESTIMATORS = frozenset({
    'rgb',
    'sensor_depth',
    'depth_anything',
})

RGB_DEBUG_VIEW_ESTIMATORS = frozenset({
    'pointcloud',
    'lidar',
    'path_a',
    'path_b',
})

CAMERA_ESTIMATORS = frozenset({
    'rgb',
    'sensor_depth',
    'depth_anything',
    'pointcloud',
})

LIDAR_ESTIMATORS = frozenset({'lidar'})

# The mask-based localization rows (g1_mask_measurement_node): masks + the
# aligned depth frame, independent of the legacy camera estimator stack.
MASK_ESTIMATORS = frozenset({'path_a', 'path_b'})

ESTIMATOR_FIELD_KEYS = {
    'rgb': 'rgb_distance_m',
    'sensor_depth': 'sensor_depth_distance_m',
    'depth_anything': 'mono_depth_distance_m',
    'pointcloud': 'pointcloud_distance_m',
    'lidar': 'lidar_distance_m',
    'path_a': 'path_a_distance_m',
    'path_b': 'path_b_distance_m',
}

ESTIMATOR_LABELS = {
    'rgb': 'RGB',
    'sensor_depth': 'Sensor Depth',
    'depth_anything': 'Depth-Anything',
    'pointcloud': 'Point Cloud',
    'lidar': 'LiDAR',
    'path_a': 'Path A (2D depth)',
    'path_b': 'Path B (3D points)',
}


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
