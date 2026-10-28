from ridgeback_localization.environment import compute_environment
"""Persistent simulator, visualization, transforms, and target detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ridgeback_common.detector_config import (
    DETECTOR_DEFAULT, DETECTION_MODEL_DEFAULT, DETECTION_THRESHOLD_DEFAULT,
    TARGET_LABELS_DEFAULT,
)
from ridgeback_common.camera_profiles import (
    CAMERA_PROFILE_CHOICES,
    DEFAULT_CAMERA_PROFILE,
)

from ridgeback_autonomy.benchmarking.backends import (
    BENCHMARK_BACKENDS,
    DEFAULT_BACKEND,
    benchmark_backend,
    resolve_benchmark_environment,
)
from ridgeback_autonomy.localization_launch import (
    RAW_DETECTIONS_TOPIC,
    SIMULATION_CAMERA_INPUTS,
    cyclonedds_actions,
    detector_launch_parameters,
    resolved_camera_inputs,
)


def build_detector(context, *args, **kwargs):
    """Resolve the compatibility color override before starting the detector."""

    inputs = resolved_camera_inputs(context, 'color_topic')
    return [Node(
        package='ridgeback_localization',
        executable='target_detector_node',
        additional_env=compute_environment(),
        name='target_detector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'color_topic': inputs.color_image_topic,
            'detections_topic': RAW_DETECTIONS_TOPIC,
            **detector_launch_parameters(context),
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )]


def build_simulation(context, *args, **kwargs):
    """Start only the selected simulator, on its native name for the public world.

    Runs before anything else starts, so an unsupported backend, an invalid
    setting, or a world the selected catalog lacks fails the launch with
    nothing to tear down. Only the selected adapter package is looked up, and
    only its own settings are forwarded; the sweep validator is what refuses a
    setting written for the other simulator, since a launch cannot tell an
    explicit value from a declared default.

    The native world deliberately replaces ``world`` for the rest of this
    launch: an adapter may read it only when a later process starts (Isaac
    resolves it after its robot description is generated), long after this
    include returns, so a scoped value would be gone by then. The per-config
    layer captures the public name before this runs.
    """

    backend = benchmark_backend(LaunchConfiguration('backend').perform(context))
    environment = resolve_benchmark_environment(
        backend.name,
        LaunchConfiguration('world').perform(context),
        {name: LaunchConfiguration(name).perform(context) for name in backend.settings},
    )
    arguments = {
        'backend': backend.name,
        'setup_path': LaunchConfiguration('setup_path'),
        'world': environment.native_world,
        'namespace': LaunchConfiguration('namespace'),
        'clearpath_rviz': 'false',
        'camera_profile': LaunchConfiguration('camera_profile'),
        **environment.settings,
    }
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    return [
        LogInfo(msg=(
            f'Benchmark backend {backend.name}: world "{environment.public_world}" '
            f'runs as "{environment.native_world}" (asset catalog '
            f'{environment.catalog.path}, sha256 {environment.catalog.sha256})')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_this, 'launch', 'includes', 'simulation.launch.py')
            ),
            launch_arguments=arguments.items(),
        ),
    ]


def generate_launch_description():
    pkg_this = get_package_share_directory('ridgeback_autonomy')
    rviz_config = os.path.join(pkg_this, 'sim', 'rviz', 'benchmark.rviz')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration_rviz = LaunchConfiguration('exploration_rviz')
    gz = BENCHMARK_BACKENDS['gz'].settings
    isaac = BENCHMARK_BACKENDS['isaac'].settings

    return LaunchDescription([
        *cyclonedds_actions(pkg_this),
        # Shared with target_benchmark_config.launch.py. Keep these defaults
        # byte-identical: the first declaration inherited by a wrapper wins.
        DeclareLaunchArgument('detector', default_value=DETECTOR_DEFAULT),
        DeclareLaunchArgument('detection_model', default_value=DETECTION_MODEL_DEFAULT),
        DeclareLaunchArgument('detection_threshold', default_value=str(DETECTION_THRESHOLD_DEFAULT)),
        DeclareLaunchArgument('target_labels', default_value=TARGET_LABELS_DEFAULT),
        DeclareLaunchArgument(
            'backend', default_value=DEFAULT_BACKEND,
            description='Benchmark simulator, fixed for the persistent environment: gz or isaac'),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='target_distance_calibration'),
        DeclareLaunchArgument(
            'camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
            choices=CAMERA_PROFILE_CHOICES,
            description='Simulated D455 render profile; fixed for the persistent environment'),
        DeclareLaunchArgument(
            'color_topic', default_value=SIMULATION_CAMERA_INPUTS.color_image_topic),
        DeclareLaunchArgument(
            'setup_path',
            default_value=os.path.expanduser('~/clearpath/'),
        ),
        DeclareLaunchArgument(
            'exploration_rviz',
            default_value='true',
            description='Launch the persistent benchmark RViz2 window',
        ),
        # Backend-only settings. Only the selected backend's reach its adapter.
        DeclareLaunchArgument(
            'gz_gui', default_value=gz['gz_gui'].default,
            description=gz['gz_gui'].description),
        DeclareLaunchArgument(
            'headless_rendering', default_value=gz['headless_rendering'].default,
            description=gz['headless_rendering'].description),
        DeclareLaunchArgument(
            'headless', default_value=isaac['headless'].default,
            description=isaac['headless'].description),
        DeclareLaunchArgument(
            'rtf', default_value=isaac['rtf'].default,
            description=isaac['rtf'].description),
        DeclareLaunchArgument(
            'sim_mode', default_value=isaac['sim_mode'].default,
            description=isaac['sim_mode'].description),
        DeclareLaunchArgument(
            'depth_fidelity', default_value=isaac['depth_fidelity'].default,
            description=isaac['depth_fidelity'].description),
        # The detector belongs to this persistent layer, so its rate is fixed
        # for a whole sweep: a per-config layer cannot change it.
        DeclareLaunchArgument(
            'detector_fps',
            default_value='10.0',
            description='Upper bound on detection rate, in frames per second',
        ),
        DeclareLaunchArgument(
            'detector_debug',
            default_value='false',
            description=(
                'Default-off detector evidence logging: achieved cadence, '
                'superseded frames, and bounded cold/warm percentiles for the '
                'throttle wait, decode, inference, parse, publish and CUDA '
                'synchronization'
            ),
        ),

        OpaqueFunction(function=build_simulation),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            condition=launch.conditions.IfCondition(exploration_rviz),
        ),

        OpaqueFunction(function=build_detector),
    ])
