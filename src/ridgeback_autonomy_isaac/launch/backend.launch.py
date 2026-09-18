"""Isaac Sim 6.1 provider for the backend-neutral autonomy stack.

Event chain (all publisher-gated downstream, no timers):
  generate_description (robot.yaml -> robot.urdf.xacro)
  -> OnProcessExit:
     - robot_state_publisher (simulation URDF frames; the D455 description
       owns the nominal colour and optical frames exactly once)
     - imu_filter_madgwick  (sensors/imu_0/data_raw -> data)
     - robot_localization EKF (platform/odom + imu -> platform/odom/filtered
       + odom->base_link TF)
     - isaac_runner.py under isaac_venv (world, robot, rig, GPU sensors)

The runner needs the workspace's isaac_venv interpreter; override with
ISAAC_PYTHON if the venv lives elsewhere.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ridgeback_common.camera_profiles import (
    CAMERA_PROFILE_CHOICES,
    DEFAULT_CAMERA_PROFILE,
    DEFAULT_DEPTH_FIDELITY,
    DEPTH_FIDELITY_CHOICES,
)


def generate_launch_description():
    pkg_adapter = get_package_share_directory('ridgeback_autonomy_isaac')
    pkg_core = get_package_share_directory('ridgeback_autonomy')
    namespace = LaunchConfiguration('namespace')

    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    livestream = LaunchConfiguration('livestream')
    rtf = LaunchConfiguration('rtf')
    odom_noise = LaunchConfiguration('odom_noise')
    noise_seed = LaunchConfiguration('noise_seed')
    camera = LaunchConfiguration('camera')
    camera_profile = LaunchConfiguration('camera_profile')
    depth_fidelity = LaunchConfiguration('depth_fidelity')
    sim_mode = LaunchConfiguration('sim_mode')
    sensor_hz = LaunchConfiguration('sensor_hz')

    isaac_python = os.environ.get(
        'ISAAC_PYTHON', os.path.join(os.getcwd(), 'isaac_venv/bin/python3'))
    runner = os.path.join(pkg_adapter, 'sim', 'isaac', 'isaac_runner.py')

    generate_description = ExecuteProcess(
        cmd=['ros2', 'run', 'clearpath_generator_common',
             'generate_description', '-s', setup_path],
        name='generate_description',
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', setup_path, 'robot.urdf.xacro is_sim:=true']),
                value_type=str),
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('imu/data_raw', 'sensors/imu_0/data_raw'),
            ('imu/data', 'sensors/imu_0/data'),
            ('/tf', 'tf'), ('/tf_static', 'tf_static'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        namespace=namespace,
        output='screen',
        parameters=[os.path.join(pkg_core, 'config', 'ekf_isaac.yaml'),
                    {'use_sim_time': True}],
        remappings=[
            ('odometry/filtered', 'platform/odom/filtered'),
            ('/tf', 'tf'), ('/tf_static', 'tf_static'),
        ],
    )

    isaac_runner = ExecuteProcess(
        cmd=[isaac_python, runner,
             '--world', world,
             '--namespace', namespace,
             '--headless', headless,
             '--livestream', livestream,
             '--rtf', rtf,
             '--odom-noise', odom_noise,
             '--noise-seed', noise_seed,
             '--camera', camera,
             '--camera-profile', camera_profile,
             '--depth-fidelity', depth_fidelity,
             '--sim-mode', sim_mode,
             '--sensor-hz', sensor_hz,
             '--animate-g1', LaunchConfiguration('animate_g1')],
        name='isaac_runner',
        output='screen',
        additional_env={'OMNI_KIT_ACCEPT_EULA': 'YES'},
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'setup_path',
            # repo-local clearpath/ (approved improvement: no ~/clearpath
            # mirror). Launches happen from the workspace root, like the
            # isaac_venv lookup above.
            default_value=os.path.join(os.getcwd(), 'clearpath/'),
            description='Path to clearpath config directory (robot.yaml)',
        ),
        DeclareLaunchArgument('namespace', default_value='r100_0001'),
        DeclareLaunchArgument('world', default_value='mock_hospital',
                              description='Isaac world name or USD path'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('livestream', default_value='false',
                              description='WebRTC livestream'),
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='real-time factor; 0 = unthrottled'),
        DeclareLaunchArgument('odom_noise', default_value='1.0',
                              description='odometry drift scale; 0 = perfect'),
        DeclareLaunchArgument(
            'noise_seed', default_value='0',
            description='reproducible Isaac odometry/IMU noise seed'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='attach D455 camera; false = '
                                          'lidar-only (saves GPU/RTF)'),
        DeclareLaunchArgument('camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
                              choices=CAMERA_PROFILE_CHOICES,
                              description='Nominal D455 render profile'),
        DeclareLaunchArgument(
            'depth_fidelity', default_value=DEFAULT_DEPTH_FIDELITY,
            choices=DEPTH_FIDELITY_CHOICES,
            description='ideal renderer depth or D455-like geometric stereo depth'),
        DeclareLaunchArgument('sim_mode', default_value='realtime',
                              description='realtime | deterministic '
                                          '(fixed-step contention-immune A/B)'),
        DeclareLaunchArgument('sensor_hz', default_value='40.0',
                              description='lidar rate = deterministic fixed dt'),
        DeclareLaunchArgument('animate_g1', default_value='false',
                              description='demo idle for world-authored G1 '
                                          'figures; keep off for benchmarks'),
        generate_description,
        RegisterEventHandler(OnProcessExit(
            target_action=generate_description,
            on_exit=[robot_state_publisher, imu_filter, ekf, isaac_runner],
        )),
    ])
