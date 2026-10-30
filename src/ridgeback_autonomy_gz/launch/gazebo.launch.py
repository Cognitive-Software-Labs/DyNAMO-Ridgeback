"""Gazebo server, optional GUI client, clock bridge, and the Clearpath robot.

The server runs as the ``ros_gz_sim`` ``gzserver`` node rather than through
the ``gz sim`` command line, because that node is what serves the standard
``simulation_interfaces`` scene-control services, under ``/gzserver``. The GUI
is a separate client of the same server. The Clearpath generators, robot
spawn, and platform/sensor bridges are included unchanged, except that each 2D
LiDAR bridge publishes to ``scan_gz`` for ``lidar_first_ray_guard``, which
republishes the scan on its normal topic with the corrupted first ray invalid.

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
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
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
        return static_config, None
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
    return handle.name, handle.name


def _remove_gui_config(_context, *, path):
    """Remove the generated GUI layout after the owning launch shuts down."""

    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    return []


def _lidar_names(setup_path):
    """(namespace, 2D LiDAR names) from the robot declaration, or None without one."""

    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    if not os.path.isfile(robot_yaml):
        return None
    config = ClearpathConfig(robot_yaml)
    return config.system.namespace, [lidar.name for lidar in config.sensors.get_all_lidar_2d()]


def _launch_robot(context):
    """Clearpath robot spawn, with each 2D LiDAR scan relayed through the first-ray guard."""

    setup_path = LaunchConfiguration('setup_path').perform(context)
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('clearpath_gz'), 'launch', 'robot_spawn.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in ('setup_path', 'world', 'rviz', 'use_sim_time', 'x', 'y', 'z', 'yaw')
        }.items(),
    )
    lidars = _lidar_names(setup_path)
    if not lidars or not lidars[1]:
        return [spawn]
    namespace, names = lidars
    sensors = f'/{namespace}/sensors' if namespace not in ('', '/') else '/sensors'
    # The spawn starts the generated bridges from event handlers after this
    # function returns, so a scoped group would be gone by then. Each rule names
    # the generated bridge node (``<lidar>_gz_bridge``) and renames only its
    # publisher; every other node keeps the normal scan topic.
    remaps = [SetRemap(f'{name}_gz_bridge:{sensors}/{name}/scan', f'{sensors}/{name}/scan_gz')
              for name in names]
    guard = Node(
        package='ridgeback_autonomy_gz',
        executable='lidar_first_ray_guard',
        namespace=sensors.lstrip('/'),
        output='screen',
        parameters=[{'scans': names, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    return [*remaps, spawn, guard]


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
        gui_config, temporary_gui_config = _gui_config(setup_path)
        gui = ExecuteProcess(
            name='gazebo_gui',
            cmd=['gz', 'sim', '-g', '-v', GZ_VERBOSITY,
                 '--gui-config', gui_config],
            output='screen',
        )
        actions = [server, gui]
        if temporary_gui_config is not None:
            actions.append(RegisterEventHandler(OnShutdown(
                on_shutdown=[OpaqueFunction(
                    function=_remove_gui_config,
                    kwargs={'path': temporary_gui_config},
                )],
            )))
        return actions

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
        OpaqueFunction(function=_launch_robot),
    ])
