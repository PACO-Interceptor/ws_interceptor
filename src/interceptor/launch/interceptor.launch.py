#!/usr/bin/env python

"""
Launch file para el escenario interceptor/target.

Arranca el agente Micro XRCE-DDS, los 4 nodos de odometria/diagnostico
(dos instancias de vehicle_odometry_subscriber, una por vehiculo, mas
target_tf2_odometry e interceptor_tf2_odometry) y un solo modo de
guiado, elegido con el argumento modo:=pn|pursuit (por defecto, pn).

Ejemplo de uso:
    ros2 launch interceptor interceptor.launch.py            # equivale a modo:=pn
    ros2 launch interceptor interceptor.launch.py modo:=pursuit

Con use_camera:=true (por defecto) arranca ademas la cadena de percepcion: el puente
de la camara de Gazebo, el detector YOLO y la tf estatica base_link -> camera_link.
Para que el interceptor tenga camara hay que lanzarlo con el airframe correspondiente.

PX4 se lanza a mano, cada instancia en su propia terminal (desde ~/PX4-Autopilot):
    interceptor (instancia 0): make px4_sitl gz_x500_mono_cam
    target (instancia 1):
        GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 \
            ./build/px4_sitl_default/bin/px4 -i 1

Al pulsar Ctrl-C, el launch tarda unos 5 s en cerrarse: a proposito, para dar tiempo al
modo de guiado a darse de baja en PX4 antes de que el agente muera.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')


def generate_launch_description():

    modo_arg = DeclareLaunchArgument(
        'modo',
        default_value='pn',
        choices=['pn', 'pursuit'],
        description=(
            'Modo de guiado que se registra en PX4 (pn = PN mode, pursuit = Pursuit '
            'Intercept); solo se lanza uno. Por defecto, pn.'
        ),
    )

    gz_camera_topic_arg = DeclareLaunchArgument(
        'gz_camera_topic',
        default_value=(
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'
        ),
        description=(
            'Topic de imagen de la camara en Gazebo. Comprobar el real con: '
            'gz topic -l | grep camera'
        ),
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo26n.pt',
        description=(
            'Fichero de pesos YOLO. El defecto trae las 80 clases COCO, entre las que '
            'no hay drones: la cadena funciona pero no detecta al objetivo.'
        ),
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description=(
            'Arranca el puente de camara, el detector y la tf estatica de la camara. '
            'Con false, el sistema vuela como antes de existir la percepcion.'
        ),
    )

    use_target_trajectory_arg = DeclareLaunchArgument(
        'use_target_trajectory',
        default_value='false',
        description=(
            'Manda al objetivo volar un circulo repetible, para poder medir. '
            'Por defecto false: sin el, el objetivo se queda donde este.'
        ),
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
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('modo'), 'pursuit')),
    )

    PN_mode_node = Node(
        package='interceptor',
        executable='PN_mode',
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('modo'), 'pn')),
    )

    gz_camera_topic = LaunchConfiguration('gz_camera_topic')
    use_camera = LaunchConfiguration('use_camera')

    # El topic de camera_info es el de imagen con /image sustituido por /camera_info.
    gz_camera_info_topic = PythonExpression([
        '"', gz_camera_topic, '".replace("/image", "/camera_info")',
    ])

    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [gz_camera_topic, '@sensor_msgs/msg/Image[gz.msgs.Image'],
            [gz_camera_info_topic, '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        ],
        remappings=[
            (gz_camera_topic, '/interceptor/camera/image_raw'),
            (gz_camera_info_topic, '/interceptor/camera/camera_info'),
        ],
        output='screen',
        condition=IfCondition(use_camera),
    )

    target_detector_node = Node(
        package='interceptor',
        executable='target_detector',
        parameters=[{'model_path': LaunchConfiguration('model_path')}],
        output='screen',
        condition=IfCondition(use_camera),
    )

    # Montaje de la camara de x500_mono_cam respecto a base_link (FLU, metros). En el SDF de
    # PX4 la camara esta en (0.12, 0.03, 0.242) respecto al ORIGEN DEL MODELO, y base_link
    # esta 0.24 m por encima de ese origen: respecto a base_link, z = 0.002. Con 0.242 la
    # estimacion salia 0.24 m alta y el interceptor pasaba por encima de la pelota.
    camera_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.12',
            '--y', '0.03',
            '--z', '0.002',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'interceptor/base_link',
            '--child-frame-id', 'interceptor/camera_link',
        ],
        output='screen',
        condition=IfCondition(use_camera),
    )

    target_trajectory_node = Node(
        package='interceptor',
        executable='target_trajectory',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_target_trajectory')),
    )

    return LaunchDescription([
        modo_arg,
        gz_camera_topic_arg,
        model_path_arg,
        use_camera_arg,
        use_target_trajectory_arg,
        micro_xrce_agent,
        target_vehicle_odometry_subscriber_node,
        target_tf2_odometry_node,
        interceptor_vehicle_odometry_subscriber_node,
        interceptor_tf2_odometry_node,
        pursuit_mode_node,
        PN_mode_node,
        camera_bridge_node,
        target_detector_node,
        camera_static_tf_node,
        target_trajectory_node,
    ])
