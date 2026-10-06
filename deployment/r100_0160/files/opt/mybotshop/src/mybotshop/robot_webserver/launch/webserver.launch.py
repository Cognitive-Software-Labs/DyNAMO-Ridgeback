#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author    Salman Omar Sohail

import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0, os.path.join(get_package_share_directory('robot_webserver'), 'launch'))
from foxglove_whitelist import build_topic_whitelist
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    GroupAction,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nsp = os.environ.get('ROBOT_NS', 'robot_unit_0')
    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    filepath_config_ros2_utils = PathJoinSubstitution(
        [FindPackageShare('robot_webserver'), 'config', ('robot_webserver.yaml')]
    )

    # -------------------- Webserver
    node_webserver = Node(
        namespace=nsp,
        remappings=rmp,
        name='robot_webserver',
        package='robot_webserver',
        executable='webserver',
        output='screen',
        parameters=[filepath_config_ros2_utils]
    )

    # -------------------- LLM Action Server
    node_llm_server = Node(
        namespace=nsp,
        name='llm_server',
        package='robot_webserver',
        executable='llm_server',
        output='screen',
        parameters=[filepath_config_ros2_utils],
    )

    # -------------------- H.264-over-WebSocket Streaming Node (C++)
    filepath_config_webrtc = PathJoinSubstitution(
        [FindPackageShare('robot_webserver'), 'config', 'robot_webrtc.yaml']
    )
    node_robot_webrtc = Node(
        namespace=nsp,
        name='robot_webrtc',
        package='robot_webserver',
        executable='robot_webrtc',
        output='screen',
        parameters=[filepath_config_webrtc],
    )

    # -------------------- Annotate Server
    node_annotate_server = Node(
        namespace=nsp,
        name='annotate',
        package='robot_webserver',
        executable='annotate_server',
        output='screen',
        parameters=[filepath_config_ros2_utils],
    )

    # -------------------- Start VNC server (skip if already listening)
    vncserver_process = ExecuteProcess(
        cmd=['bash', '-c',
             'if ss -tlnp 2>/dev/null | grep -q ":5901 "; then '
             'echo "VNC server :1 already listening, skipping"; '
             'else '
             'export HOME="$(getent passwd "$(id -un)" | cut -d: -f6)"; '
             'mkdir -p "$HOME/.vnc"; '
             'rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 "$HOME"/.vnc/*:5901.pid; '
             'vncserver :1 -geometry 1920x1080 -depth 24 -localhost no; '
             'fi'],
        output='screen'
    )

    # -------------------- Start Websockify to forward VNC to browser
    websockify_process = ExecuteProcess(
        cmd=['bash', '-c',
             'if ss -tlnp 2>/dev/null | grep -q ":9005 "; then '
             'echo "Websockify :9005 already running, skipping"; '
             'else '
             'exec websockify 9005 127.0.0.1:5901; '
             'fi'],
        output='screen'
    )

    # -------------------- foxglove_bridge (replaces rosbridge_websocket on 9004)
    robot_info_path = os.path.join(
        get_package_share_directory('robot_webserver'),
        'config', 'robot_info.yaml')
    foxglove = GroupAction([
        PushRosNamespace(nsp),
        Node(
            name='foxglove_bridge',
            package='foxglove_bridge',
            executable='foxglove_bridge',
            output='screen',
            parameters=[{
                'address': '0.0.0.0',
                'port': 9004,
                'send_buffer_limit': 10000000,
                'use_compression': True,
                'max_qos_depth': 10,
                'min_qos_depth': 1,
                # Derived from robot_info.yaml viewer topics; edit topics there.
                'topic_whitelist': build_topic_whitelist(robot_info_path),
                'capabilities': ['clientPublish'],
            }],
        ),
    ])

    ld = LaunchDescription()

    ld.add_action(node_webserver)
    ld.add_action(foxglove)
    ld.add_action(vncserver_process)
    ld.add_action(websockify_process)

    ld.add_action(node_annotate_server)
    ld.add_action(node_llm_server)
    ld.add_action(node_robot_webrtc)

    return ld
