from __future__ import annotations

import ast
import importlib
from pathlib import Path
import re


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
        'ridgeback_autonomy.benchmarking.simulation',
        'ridgeback_autonomy.benchmarking.trial_results',
        'ridgeback_autonomy.common.camera_inputs',
        'ridgeback_autonomy.perception.target_localization.core.depth_common',
        'ridgeback_autonomy.perception.target_localization.core.depth_sources',
        'ridgeback_autonomy.perception.target_localization.core.detection',
        'ridgeback_autonomy.perception.target_localization.core.image_utils',
        'ridgeback_autonomy.perception.target_localization.core.intrinsics',
        'ridgeback_autonomy.perception.target_localization.core.isolation_2d',
        'ridgeback_autonomy.perception.target_localization.core.isolation_3d',
        'ridgeback_autonomy.perception.target_localization.core.mask',
        'ridgeback_autonomy.perception.target_localization.core.projective_ranging',
        'ridgeback_autonomy.perception.target_localization.core.euclidean_reconstruction',
        'ridgeback_autonomy.perception.target_localization.core.pointcloud_ranging',
        'ridgeback_autonomy.perception.target_localization.core.polar_profiling',
        'ridgeback_autonomy.perception.target_localization.core.vehicle_frame',
        'ridgeback_autonomy.perception.target_localization.core.rendering',
        'ridgeback_autonomy.perception.target_localization.contracts',
        'ridgeback_autonomy.perception.target_localization.hud_rendering',
        'ridgeback_autonomy.perception.target_localization.marker_rendering',
        'ridgeback_autonomy.perception.target_localization.measurement_pipeline',
        'ridgeback_autonomy.perception.target_localization.synchronization',
        'ridgeback_autonomy.benchmarking.metrics',
        'ridgeback_autonomy.benchmarking.reduction',
        'ridgeback_autonomy.benchmarking.rendering',
        'ridgeback_autonomy.benchmarking.summary',
        'ridgeback_autonomy.perception.target_localization.detector_node',
        'ridgeback_autonomy.perception.target_localization.estimator_registry',
        'ridgeback_autonomy.perception.target_localization.visualization_node',
        'ridgeback_autonomy.perception.target_localization.visualization_readings',
        'ridgeback_autonomy.perception.target_localization.visualization_style',
        'ridgeback_autonomy.perception.target_localization.launch',
        'ridgeback_autonomy.perception.target_localization.ground_truth',
        'ridgeback_autonomy.perception.target_localization.mask_measurement_node',
        'ridgeback_autonomy.perception.target_localization.pointcloud_measurement_node',
        'ridgeback_autonomy.perception.target_localization.overlay_node',
        'ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node',
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


def test_reusable_target_helpers_do_not_depend_on_node_orchestration() -> None:
    """Keep pure/reusable target layers below their ROS orchestration modules."""

    target_root = PACKAGE_ROOT / 'perception' / 'target_localization'
    helper_names = (
        'synchronization.py',
        'measurement_pipeline.py',
        'visualization_readings.py',
        'visualization_style.py',
        'hud_rendering.py',
        'marker_rendering.py',
    )
    node_modules = (
        'ridgeback_autonomy.perception.target_localization.mask_measurement_node',
        'ridgeback_autonomy.perception.target_localization.visualization_node',
    )
    violations = []
    for helper_name in helper_names:
        path = target_root / helper_name
        for node_module in node_modules:
            violations.extend(
                f'{helper_name}:{line} imports {node_module}'
                for line in _imports_from(path, node_module)
            )

    assert not violations, (
        'reusable target helpers must not import node orchestration: '
        f'{violations}'
    )


def test_benchmark_domain_modules_do_not_depend_on_the_runner_node() -> None:
    """Keep the benchmark runner as the top orchestration layer."""

    benchmarking_root = PACKAGE_ROOT / 'benchmarking'
    runner_module = (
        'ridgeback_autonomy.benchmarking.target_distance_benchmark_runner_node')
    violations = []
    for path in sorted(benchmarking_root.glob('*.py')):
        if path.name == 'target_distance_benchmark_runner_node.py':
            continue
        violations.extend(
            f'{path.name}:{line} imports {runner_module}'
            for line in _imports_from(path, runner_module)
        )

    assert not violations, (
        'benchmark domain modules must stay below runner orchestration: '
        f'{violations}'
    )


def test_target_wire_topics_have_one_owner() -> None:
    """Keep every generic target topic literal in the contracts module."""

    contracts_path = PACKAGE_ROOT / 'perception' / 'target_localization' / 'contracts.py'
    topic_literal = re.compile(
        r"['\"](?:detections|measurements|debug|visualization|benchmark|hud)/target[^'\"]*['\"]"
    )
    violations = []
    for path in sorted(PACKAGE_ROOT.rglob('*.py')):
        if path == contracts_path:
            continue
        for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            if topic_literal.search(line):
                violations.append(f'{path.relative_to(PACKAGE_ROOT)}:{line_number}')

    assert not violations, f'target topic literals belong in contracts.py: {violations}'


def test_removed_g1_software_api_names_do_not_return() -> None:
    """Reserve G1 naming for the actual simulator model/plugin, not stack APIs."""

    package_dir = PACKAGE_ROOT.parent
    checked_paths = [
        *PACKAGE_ROOT.rglob('*.py'),
        *(package_dir / 'launch').rglob('*.py'),
        *(package_dir / 'msg').glob('*.msg'),
        package_dir / 'CMakeLists.txt',
    ]
    removed_names = (
        'G1Detections',
        'G1Measurements',
        'g1_detector_node',
        'g1_pointcloud_measurement_node',
        'g1_mask_measurement_node',
        'g1_estimate_viz_node',
        'g1_overlay_node',
        'g1_distance_benchmark_runner',
        'g1_benchmark_sweep',
        'g1_benchmark_env',
        'g1_benchmark_config',
        'g1_distance_benchmark',
        'detections/g1',
        'measurements/g1',
        'debug/g1',
        'visualization/g1',
        'benchmark/g1',
        'hud/g1',
        'ridgeback_autonomy.perception.core',
        'ridgeback_autonomy.perception.estimators',
        'ridgeback_autonomy.perception.ground_truth',
    )
    violations = []
    for path in sorted(checked_paths):
        text = path.read_text(encoding='utf-8')
        violations.extend(
            f'{path.relative_to(package_dir)} contains {name}'
            for name in removed_names
            if name in text
        )

    assert not violations, f'removed generic G1 APIs returned: {violations}'
