#!/usr/bin/env python

"""
Launch file para el escenario interceptor/target.

- Agente Micro-XRCE-DDS (puente uXRCE-DDS <-> ROS 2)
- Los 4 nodos del paquete interceptor
(el dron 0 y el dron 1 se lanzan a mano, cada uno en su propia terminal,
dron 0 (interceptor) make px4_sitl gz_x500
dron1 (target) PX4_SIM_MODEL=gz_x500 /build/px4_sitl_default/bin/px4 -i 1
para tener control manual e interactivo de ambos)
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')


def generate_launch_description():

    micro_xrce_agent = ExecuteProcess(
        cmd=[['./build/MicroXRCEAgent udp4 -p 8888']],
        shell=True,
        cwd=MICRO_XRCE_DDS_AGENT_DIR,
        output='log',
    )

    target_vehicle_odometry_subscriber_node = Node(
        package='interceptor',
        executable='target_vehicle_odometry_subscriber',
        output='log',
    )

    target_tf2_odometry_node = Node(
        package='interceptor',
        executable='target_tf2_odometry',
        output='screen',
    )

    interceptor_vehicle_odometry_subscriber_node = Node(
        package='interceptor',
        executable='interceptor_vehicle_odometry_subscriber',
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
    )

    PN_mode_node = Node(
        package='interceptor',
        executable='PN_mode',
        output='screen',
    )

    return LaunchDescription([
        micro_xrce_agent,
        target_vehicle_odometry_subscriber_node,
        target_tf2_odometry_node,
        interceptor_vehicle_odometry_subscriber_node,
        interceptor_tf2_odometry_node,
        pursuit_mode_node,
        PN_mode_node,
    ])
