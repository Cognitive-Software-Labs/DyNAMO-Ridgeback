"""Shared visual tokens for target-localization HUD and marker renderers."""

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    PUBLIC_ESTIMATOR_ORDER,
)


RING_RADIUS_M = 0.15
RING_POINTS = 32
DOT_RADIUS_M = 0.04
RING_LINE_WIDTH_M = 0.06
MARKER_Z_M = 0.05

# Warm pointcloud versus cold mask hues visually group estimator families on
# surfaces that intentionally carry no extra category label.
ESTIMATOR_COLOURS = {
    'pointcloud':               (1.0, 0.85, 0.0, 1.0),
    'projective_ranging':       (0.0, 0.85, 0.85, 1.0),
    'euclidean_reconstruction': (0.2, 0.55, 1.0, 1.0),
    'polar_profiling':          (0.7, 0.4, 1.0, 1.0),
}

# Fixed ids prevent a nearest-detection or selected-estimator change from
# stranding old markers or making one estimator overwrite another.
ESTIMATOR_MARKER_ID_BASES = {
    name: index * 100 for index, name in enumerate(PUBLIC_ESTIMATOR_ORDER)
}
