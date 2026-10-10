#!/usr/bin/env python

"""
Launch file para el escenario interceptor/target.

Arranca el agente Micro XRCE-DDS, los 4 nodos de odometria/diagnostico
(dos instancias de vehicle_odometry_subscriber, una por vehiculo, mas
target_tf2_odometry e interceptor_tf2_odometry) y un solo modo de
guiado, elegido con el argumento mode:=pn|pursuit (por defecto, pn).
El nombre antiguo, modo:=, se acepta temporalmente como alias de mode.

Ejemplo de uso:
    ros2 launch interceptor interceptor.launch.py            # equivale a mode:=pn
    ros2 launch interceptor interceptor.launch.py mode:=pursuit

PX4 se lanza a mano, cada instancia en su propia terminal (desde ~/PX4-Autopilot):
    interceptor (instancia 0): make px4_sitl gz_x500
    target (instancia 1):
        GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 \
            ./build/px4_sitl_default/bin/px4 -i 1

Al pulsar Ctrl-C, el launch tarda unos 5 s en cerrarse: a proposito, para dar tiempo al
modo de guiado a darse de baja en PX4 antes de que el agente muera.
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
            'Modo de guiado que se registra en PX4 (pn = PN mode, pursuit = Pursuit '
            'Intercept); solo se lanza uno. Por defecto, pn.'
        ),
    )

    # Alias temporal del nombre antiguo del argumento. Si se pasa modo:=, su valor
    # sustituye al de mode. Eliminar cuando ya no se use modo:= en ningun sitio.
    modo_arg = DeclareLaunchArgument(
        'modo',
        default_value='',
        choices=['', 'pn', 'pursuit'],
        description='Obsoleto: alias temporal de mode. Usar mode:=pn|pursuit.',
    )
    modo_given = UnlessCondition(EqualsSubstitution(LaunchConfiguration('modo'), ''))
    modo_alias = SetLaunchConfiguration('mode', LaunchConfiguration('modo'), condition=modo_given)
    modo_warning = LogInfo(
        msg='El argumento modo:= esta obsoleto; usa mode:= en su lugar.',
        condition=modo_given,
    )

    # El agente ignora SIGINT para seguir vivo tras el Ctrl-C hasta que el launch escala a
    # SIGTERM: asi el modo de guiado tiene tiempo de enviar `Unregistering` a PX4.
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
