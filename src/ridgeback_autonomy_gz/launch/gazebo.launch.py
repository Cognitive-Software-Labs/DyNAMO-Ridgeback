"""Gazebo server, optional GUI client, clock bridge, and the Clearpath robot.

The server runs as the ``ros_gz_sim`` ``gzserver`` node rather than through
the ``gz sim`` command line, because that node is what serves the standard
``simulation_interfaces`` scene-control services, under ``/gzserver``. The GUI
is a separate client of the same server. The Clearpath generators, robot
spawn, and platform/sensor bridges are included unchanged.

Internal to this adapter: ``backend.launch.py`` owns the public arguments and
passes the Clearpath world name and the resolved rendering mode here.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable, UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


# Matches the verbosity the Clearpath command line used (gz sim -v 4).
GZ_VERBOSITY = '4'


def _world_search_dirs():
    return [
        os.path.join(get_package_share_directory('clearpath_gz'), 'worlds'),
        os.path.join(get_package_share_directory('ridgeback_autonomy_gz'), 'sim', 'worlds'),
    ]


def resolve_world_sdf(world, search_dirs):
    """Absolute path of ``<world>.sdf`` in the first directory that has it."""

    for directory in search_dirs:
        path = os.path.join(directory, f'{world}.sdf')
        if os.path.isfile(path):
            return path
    raise ValueError(
        f'Gazebo world {world!r} has no {world}.sdf in: {", ".join(search_dirs)}')


def _gui_config(setup_path):
    """Clearpath's GUI layout with the Teleop plugin aimed at the robot namespace."""

    static_config = os.path.join(
        get_package_share_directory('clearpath_gz'), 'config', 'gui.config')
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    if not os.path.isfile(robot_yaml):
        return static_config
    namespace = ClearpathConfig(robot_yaml).system.namespace
    topic = f'/{namespace}/cmd_vel' if namespace not in ('', '/') else '/cmd_vel'
    with open(static_config) as handle:
        content = handle.read()
    content = content.replace(
        '<plugin filename="Teleop">',
        f'<plugin filename="Teleop">\n    <topic>{topic}</topic>')
    with tempfile.NamedTemporaryFile(
            prefix='ridgeback_gz_gui_', suffix='.config', mode='w', delete=False) as handle:
        handle.write(content)
    return handle.name


def _launch_gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    setup_path = LaunchConfiguration('setup_path').perform(context)
    headless = LaunchConfiguration('headless_rendering').perform(context) == 'true'

    server = GzServer(
        world_sdf_file=resolve_world_sdf(world, _world_search_dirs()),
        verbosity_level=GZ_VERBOSITY,
    )
    if not headless:
        # Started directly rather than through a shell, so the stop signal
        # reaches the GUI itself; a shell wrapper exits and orphans the GUI.
        gui = ExecuteProcess(
            name='gazebo_gui',
            cmd=['gz', 'sim', '-g', '-v', GZ_VERBOSITY,
                 '--gui-config', _gui_config(setup_path)],
            output='screen',
        )
        return [server, gui]

    # The server node has no headless-rendering switch. Without a display,
    # Gazebo's rendering engine falls back to EGL headless rendering, so the
    # server alone is started with the display variables removed.
    return [GroupAction(scoped=True, actions=[
        UnsetEnvironmentVariable('DISPLAY'),
        UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        server,
    ])]


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_adapter = get_package_share_directory('ridgeback_autonomy_gz')
    package_shares = [
        os.path.join(prefix, 'share')
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if prefix
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument(
            'setup_path', default_value=os.path.expanduser('~/clearpath/')),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'headless_rendering', default_value='false', choices=['true', 'false'],
            description='Render server sensors via EGL without a display and start no GUI'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Model and world lookup for the server, GUI, and robot spawn: the
        # Clearpath assets, this adapter's worlds and models, and every
        # sourced package share.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join([
            os.path.join(pkg_clearpath_gz, 'worlds'),
            os.path.join(pkg_clearpath_gz, 'meshes'),
            os.path.join(pkg_adapter, 'sim', 'worlds'),
            os.path.join(pkg_adapter, 'sim', 'models'),
            *package_shares,
        ])),
        SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', os.path.join(
            pkg_adapter, '..', '..', 'lib', 'ridgeback_autonomy_gz')),
        OpaqueFunction(function=_launch_gazebo),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py')),
            launch_arguments={
                'setup_path': LaunchConfiguration('setup_path'),
                'world': LaunchConfiguration('world'),
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),
    ])
