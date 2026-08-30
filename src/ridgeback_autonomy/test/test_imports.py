from __future__ import annotations

import ast
import importlib
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1] / 'ridgeback_autonomy'


def _imports_from(path: Path, package: str) -> list[int]:
    """Line numbers importing ``package`` or one of its submodules."""

    imported_at = []
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            names = (node.module,)
        else:
            continue
        if any(name == package or name.startswith(f'{package}.') for name in names):
            imported_at.append(node.lineno)
    return imported_at


def test_packaged_modules_import() -> None:
    module_names = [
        'ridgeback_autonomy.common.camera_config',
        'ridgeback_autonomy.common.coverage_utils',
        'ridgeback_autonomy.common.launch_wait',
        'ridgeback_autonomy.common.messages',
        'ridgeback_autonomy.common.models',
        'ridgeback_autonomy.common.tf_utils',
        'ridgeback_autonomy.benchmarking.alignment',
        'ridgeback_autonomy.benchmarking.naming',
        'ridgeback_autonomy.common.camera_inputs',
        'ridgeback_autonomy.perception.core.depth_common',
        'ridgeback_autonomy.perception.core.depth_sources',
        'ridgeback_autonomy.perception.core.detection',
        'ridgeback_autonomy.perception.core.image_utils',
        'ridgeback_autonomy.perception.core.intrinsics',
        'ridgeback_autonomy.perception.core.isolation_2d',
        'ridgeback_autonomy.perception.core.isolation_3d',
        'ridgeback_autonomy.perception.core.mask',
        'ridgeback_autonomy.perception.core.projective_ranging',
        'ridgeback_autonomy.perception.core.euclidean_reconstruction',
        'ridgeback_autonomy.perception.core.pointcloud_ranging',
        'ridgeback_autonomy.perception.core.polar_profiling',
        'ridgeback_autonomy.perception.core.vehicle_frame',
        'ridgeback_autonomy.perception.core.rendering',
        'ridgeback_autonomy.benchmarking.metrics',
        'ridgeback_autonomy.benchmarking.reduction',
        'ridgeback_autonomy.benchmarking.rendering',
        'ridgeback_autonomy.benchmarking.summary',
        'ridgeback_autonomy.perception.g1_detector_node',
        'ridgeback_autonomy.perception.estimators',
        'ridgeback_autonomy.perception.g1_estimate_viz_node',
        'ridgeback_autonomy.perception.g1_launch',
        'ridgeback_autonomy.perception.ground_truth',
        'ridgeback_autonomy.perception.g1_mask_measurement_node',
        'ridgeback_autonomy.perception.g1_pointcloud_measurement_node',
        'ridgeback_autonomy.perception.g1_overlay_node',
        'ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node',
        'ridgeback_autonomy.diagnostics.hud_node',
        'ridgeback_autonomy.diagnostics.coverage_overlay_node',
        'ridgeback_autonomy.frontier_explorer.navigator',
        'ridgeback_autonomy.frontier_explorer.path_finding',
        'ridgeback_autonomy.frontier_explorer.frontier_explorer_node',
    ]

    for module_name in module_names:
        assert importlib.import_module(module_name) is not None


def test_package_dependencies_flow_one_way() -> None:
    """Keep common <- perception <- benchmarking as a one-way dependency chain."""

    violations = []
    boundaries = {
        'common': (
            'ridgeback_autonomy.perception',
            'ridgeback_autonomy.benchmarking',
        ),
        'perception': ('ridgeback_autonomy.benchmarking',),
    }
    for source_package, forbidden_packages in boundaries.items():
        for path in sorted((PACKAGE_ROOT / source_package).rglob('*.py')):
            for forbidden in forbidden_packages:
                violations.extend(
                    f'{path.relative_to(PACKAGE_ROOT)}:{line} imports {forbidden}'
                    for line in _imports_from(path, forbidden)
                )

    exploration_launch = PACKAGE_ROOT.parent / 'launch' / 'ridgeback_exploration.launch.py'
    violations.extend(
        f'{exploration_launch.name}:{line} imports ridgeback_autonomy.benchmarking'
        for line in _imports_from(exploration_launch, 'ridgeback_autonomy.benchmarking')
    )

    assert not violations, (
        'dependencies must flow common <- perception <- benchmarking: '
        f'{violations}'
    )
