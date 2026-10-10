#!/usr/bin/env python

"""
Launch file for the interceptor/target scenario.

Starts the Micro XRCE-DDS agent, the 4 odometry/diagnostic nodes
(two instances of vehicle_odometry_subscriber, one per vehicle, plus
target_tf2_odometry and interceptor_tf2_odometry) and a single guidance
mode, chosen with the argument mode:=pn|pursuit (pn by default).
The old name, modo:=, is accepted for now as an alias of mode.

Usage example:
    ros2 launch interceptor interceptor.launch.py            # same as mode:=pn
    ros2 launch interceptor interceptor.launch.py mode:=pursuit

PX4 is started by hand, each instance in its own terminal (from ~/PX4-Autopilot):
    interceptor (instance 0): make px4_sitl gz_x500
    target (instance 1):
        GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 \
            ./build/px4_sitl_default/bin/px4 -i 1

On Ctrl-C, the launch takes about 5 s to close: on purpose, to give the
guidance mode time to unregister from PX4 before the agent dies.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node

MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='pn',
        choices=['pn', 'pursuit'],
        description=(
            'Guidance mode registered in PX4 (pn = PN mode, pursuit = Pursuit '
            'Intercept); only one is launched. Default: pn.'
        ),
    )

    # Temporary alias for the old argument name. If modo:= is given, its value
    # replaces mode. Remove once modo:= is no longer used anywhere.
    modo_arg = DeclareLaunchArgument(
        'modo',
        default_value='',
        choices=['', 'pn', 'pursuit'],
        description='Deprecated: temporary alias of mode. Use mode:=pn|pursuit.',
    )
    modo_given = UnlessCondition(EqualsSubstitution(LaunchConfiguration('modo'), ''))
    modo_alias = SetLaunchConfiguration('mode', LaunchConfiguration('modo'), condition=modo_given)
    modo_warning = LogInfo(
        msg='The modo:= argument is deprecated; use mode:= instead.',
        condition=modo_given,
    )

    # The agent ignores SIGINT to stay alive after Ctrl-C until the launch escalates to
    # SIGTERM: that gives the guidance mode time to send `Unregistering` to PX4.
    micro_xrce_agent = ExecuteProcess(
        cmd=[["trap '' INT; exec ./build/MicroXRCEAgent udp4 -p 8888"]],
        shell=True,
        cwd=MICRO_XRCE_DDS_AGENT_DIR,
        output='log',
    )

    target_vehicle_odometry_subscriber_node = Node(
        package='interceptor',
        executable='vehicle_odometry_subscriber',
        name='target_vehicle_odometry_subscriber',
        parameters=[{
            'vehicle_name': 'target',
            'odometry_topic': '/px4_1/fmu/out/vehicle_odometry',
        }],
        output='log',
    )

    target_tf2_odometry_node = Node(
        package='interceptor',
        executable='target_tf2_odometry',
        output='screen',
    )

    interceptor_vehicle_odometry_subscriber_node = Node(
        package='interceptor',
        executable='vehicle_odometry_subscriber',
        name='interceptor_vehicle_odometry_subscriber',
        parameters=[{
            'vehicle_name': 'interceptor',
            'odometry_topic': '/fmu/out/vehicle_odometry',
        }],
        output='log',
    )

    interceptor_tf2_odometry_node = Node(
        package='interceptor',
        executable='interceptor_tf2_odometry',
        output='screen',
    )

    # tf2_listener_node = Node(
    #     package='interceptor',
    #     executable='tf2_listener',
    #     output='screen',
    # )

    pursuit_mode_node = Node(
        package='interceptor',
        executable='pursuit_mode',
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'pursuit')),
    )

    PN_mode_node = Node(
        package='interceptor',
        executable='PN_mode',
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'pn')),
    )

    return LaunchDescription([
        mode_arg,
        modo_arg,
        modo_alias,
        modo_warning,
        micro_xrce_agent,
        target_vehicle_odometry_subscriber_node,
        target_tf2_odometry_node,
        interceptor_vehicle_odometry_subscriber_node,
        interceptor_tf2_odometry_node,
        pursuit_mode_node,
        PN_mode_node,
    ])
