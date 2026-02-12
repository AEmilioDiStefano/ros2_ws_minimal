#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary
# Enhanced Sprint 1 Launch File with ROS2 Bridge
# Launches all core nodes + rosbridge_server for web interface integration

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Launch all Fleet Orchestrator core nodes + rosbridge_server

    Nodes launched:
    1. intent_gateway_node - Aggregates HMI inputs
    2. intent_translator_node - Translates intent to playbook commands
    3. audit_logger_node - Audit trail logging
    4. rosbridge_server - WebSocket bridge for web interface
    """

    # Declare launch arguments
    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge',
        default_value='true',
        description='Enable rosbridge_server for web interface'
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket port for rosbridge_server'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='ROS logging level'
    )

    # Get launch configuration values
    enable_rosbridge = LaunchConfiguration('enable_rosbridge')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    log_level = LaunchConfiguration('log_level')

    # Fleet Orchestrator Core Nodes

    intent_gateway = Node(
        package='fleet_orchestrator_core',
        executable='intent_gateway',
        name='intent_gateway',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    intent_translator = Node(
        package='fleet_orchestrator_core',
        executable='intent_translator',
        name='intent_translator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    audit_logger = Node(
        package='fleet_orchestrator_core',
        executable='audit_logger',
        name='audit_logger',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # Unit Emulator - Simulates multiple units receiving commands
    unit_emulator = Node(
        package='fleet_orchestrator_core',
        executable='unit_emulator',
        name='unit_emulator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'num_units': 5,  # Number of simulated units
            'publish_rate_hz': 1.0  # Status update rate
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # Rosbridge Server for web interface
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '',
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_rosbridge),
        respawn=True,
        respawn_delay=5.0
    )

    # Log startup message
    startup_log = LogInfo(
        msg=[
            '\n',
            '======================================================================\n',
            'Fleet Orchestrator Core - Sprint 1 Full Stack\n',
            '======================================================================\n',
            'Nodes launched:\n',
            '  - intent_gateway_node\n',
            '  - intent_translator_node\n',
            '  - audit_logger_node\n',
            '  - unit_emulator (5 simulated units with MAC addresses)\n',
            '  - rosbridge_websocket (port: 9090)\n',
            '\n',
            'Topics:\n',
            '  Inputs:  /fo/hmi/intent_text, /fo/hmi/intent_voice, /fo/hmi/intent_gui\n',
            '  Outputs: /fo/task, /fo/audit\n',
            '  Bridge:  ws://localhost:9090\n',
            '\n',
            'Web Interface:\n',
            '  Access: http://localhost:5002/military-ops.html\n',
            '\n',
            '======================================================================\n'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        enable_rosbridge_arg,
        rosbridge_port_arg,
        log_level_arg,

        # Startup message
        startup_log,

        # Core nodes
        intent_gateway,
        intent_translator,
        audit_logger,
        unit_emulator,

        # Rosbridge (conditional)
        rosbridge,
    ])
