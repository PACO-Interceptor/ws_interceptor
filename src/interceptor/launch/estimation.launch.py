#!/usr/bin/env python

"""
Launch file para ensayar la estimacion visual del checkpoint movil (tamano desconocido).

Arranca toda la cadena de percepcion y estimacion, mas dos nodos de vuelo en offboard
directo (sin modo de guiado): el objetivo recorre una recta a velocidad constante y el
interceptor ejecuta un perfil de maniobra fijo. La verdad de la simulacion solo la usa
estimation_evaluator, que escribe el error en un CSV.

    ros2 launch interceptor estimation.launch.py profile:=weave
    ros2 launch interceptor estimation.launch.py mode:=guidance

mode:=maneuver (defecto) vuela un perfil fijo (profile): hover, constant, surge, weave.
Con hover y constant la escala no es observable (no deberia converger); con surge y
weave si. mode:=guidance guia al interceptor a traves de la pelota (checkpoint_guidance);
el evaluador registra por cuanto pasa del centro ("PASO por el checkpoint").

PX4 y Gazebo se lanzan antes, a mano, cada uno en su terminal:
    interceptor (instancia 0), desde ~/PX4-Autopilot: make px4_sitl gz_x500_mono_cam
    objetivo con pelota (instancia 1):                 ros2 run interceptor spawn_ball_target

Todos los nodos que comparan instantes de imagen con tf usan el reloj de Gazebo
(/clock, use_sim_time): las imagenes llegan selladas con tiempo de simulacion.

YOLO: si ultralytics esta en un venv aparte, se pasa su site-packages con
yolo_site_packages:=... (o la variable INTERCEPTOR_YOLO_SITE_PACKAGES). Se anade al
PYTHONPATH DETRAS del numpy del sistema, porque cv_bridge esta compilado contra el.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')
GZ_CAMERA_TOPIC = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'
GZ_CAMERA_INFO_TOPIC = GZ_CAMERA_TOPIC.replace('/image', '/camera_info')
SIM_TIME = {'use_sim_time': True}


def float_arg(name):
    """Argumento de launch como parametro float ('30' llegaria como entero)."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def detector_node(context):
    """Crea el detector con el PYTHONPATH del venv de YOLO, si se ha dado."""
    site = LaunchConfiguration('yolo_site_packages').perform(context)
    env = {}
    if site:
        env['PYTHONPATH'] = ':'.join(
            p for p in (os.environ.get('PYTHONPATH', ''), '/usr/lib/python3/dist-packages', site)
            if p)
    return [Node(
        package='interceptor',
        executable='target_detector',
        parameters=[SIM_TIME, {
            'model_path': LaunchConfiguration('model_path'),
            'target_class': 'sports ball',
            # Como texto: '0' (GPU) llegaria como entero y el nodo espera string.
            'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
            'imgsz': 1280,
            'max_rate_hz': 15.0,
        }],
        additional_env=env,
        output='screen',
    )]


def generate_launch_description():
    """Describe la cadena de estimacion y los nodos de vuelo del ensayo."""
    args = [
        DeclareLaunchArgument(
            'mode', default_value='maneuver', choices=['maneuver', 'guidance'],
            description='maneuver = perfil fijo (observer_maneuver); guidance = pasar por el '
                        'checkpoint (checkpoint_guidance).'),
        DeclareLaunchArgument(
            'speed_mps', default_value='3.0',
            description='Velocidad del interceptor en mode:=guidance [m/s].'),
        DeclareLaunchArgument(
            'freeze_time_s', default_value='0.5',
            description='En mode:=guidance, segundos antes del paso en que congela el rumbo.'),
        DeclareLaunchArgument(
            'excitation_amp_mps', default_value='2.0',
            description='Amplitud de la excitacion lateral en mode:=guidance (0 = sin ella).'),
        DeclareLaunchArgument(
            'profile', default_value='weave',
            choices=['hover', 'constant', 'surge', 'weave'],
            description='Perfil de maniobra del interceptor.'),
        DeclareLaunchArgument(
            'duration_s', default_value='30.0',
            description='Duracion de la maniobra del interceptor [s].'),
        DeclareLaunchArgument(
            'amplitude_mps', default_value='2.5',
            description=(
                'Amplitud de la oscilacion de velocidad en surge y weave [m/s]. Con 1.0 la '
                'escala converge con un +12 % de sesgo; con 2.5, a +-5 %.')),
        DeclareLaunchArgument(
            'true_size', default_value='1.0',
            description='Diametro real de la pelota [m], solo para el evaluador '
                        '(spawn_ball_target con BALL_DIAMETER).'),
        DeclareLaunchArgument(
            'size_prior', default_value='0.5',
            description=(
                'Tamano que el estimador supone al principio [m]. A proposito distinto del '
                'real (1.0) y menor: la distancia estimada sale corta, del lado seguro.')),
        DeclareLaunchArgument(
            'model_path', default_value='yolo26n.pt',
            description='Pesos YOLO con las clases COCO (sports ball).'),
        DeclareLaunchArgument(
            'device', default_value='cpu',
            description='Dispositivo de inferencia: cpu, 0 (primera GPU CUDA), ...'),
        DeclareLaunchArgument(
            'yolo_site_packages',
            default_value=EnvironmentVariable('INTERCEPTOR_YOLO_SITE_PACKAGES', default_value=''),
            description='site-packages de un venv con ultralytics (vacio = el del sistema).'),
        DeclareLaunchArgument(
            'csv_path', default_value='',
            description=(
                'CSV de la evaluacion (vacio = ~/.ros/interceptor_estimation/<fecha>.csv).')),
    ]

    # Igual que en interceptor.launch.py: ignora SIGINT para cerrar despues que los nodos.
    micro_xrce_agent = ExecuteProcess(
        cmd=[["trap '' INT; exec ./build/MicroXRCEAgent udp4 -p 8888"]],
        shell=True,
        cwd=MICRO_XRCE_DDS_AGENT_DIR,
        output='log',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            (GZ_CAMERA_TOPIC, '/interceptor/camera/image_raw'),
            (GZ_CAMERA_INFO_TOPIC, '/interceptor/camera/camera_info'),
        ],
        # PX4 arranca Gazebo con GZ_IP=127.0.0.1. Sin la misma variable, el puente ve los
        # topics pero no le llegan los datos.
        additional_env={'GZ_IP': '127.0.0.1'},
        output='screen',
    )

    interceptor_tf2_odometry = Node(
        package='interceptor', executable='interceptor_tf2_odometry',
        parameters=[SIM_TIME], output='screen')
    target_tf2_odometry = Node(
        package='interceptor', executable='target_tf2_odometry',
        parameters=[SIM_TIME], output='screen')

    # Montaje de la camara de x500_mono_cam respecto a base_link (FLU, metros). En el SDF de
    # PX4 la camara esta en (0.12, 0.03, 0.242) respecto al ORIGEN DEL MODELO, y base_link
    # esta 0.24 m por encima de ese origen: respecto a base_link, z = 0.002. Con 0.242 la
    # estimacion salia 0.24 m alta y el interceptor pasaba por encima de la pelota.
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.12', '--y', '0.03', '--z', '0.002',
            '--frame-id', 'interceptor/base_link',
            '--child-frame-id', 'interceptor/camera_link',
        ],
        parameters=[SIM_TIME],
        output='log',
    )

    target_estimator = Node(
        package='interceptor', executable='target_estimator',
        parameters=[SIM_TIME, {
            'size_prior': float_arg('size_prior'),
        }],
        output='screen')

    estimation_evaluator = Node(
        package='interceptor', executable='estimation_evaluator',
        parameters=[SIM_TIME, {
            'true_size': float_arg('true_size'),
            'ball_offset': [0.0, 0.0, 2.5],
            'csv_path': LaunchConfiguration('csv_path'),
        }],
        output='screen')

    # Objetivo: parte 15 m al oeste de su origen y cruza 30 m hacia el este a 1 m/s,
    # a 20 m al norte del interceptor (spawn_ball_target lo pone en y = 20).
    target_trajectory = Node(
        package='interceptor', executable='target_trajectory',
        parameters=[{
            'trajectory_type': 'line',
            'altitude_m': 10.0,
            'center_north_m': 0.0,
            'center_east_m': -15.0,
            'line_heading_deg': 90.0,
            'line_speed_mps': 1.0,
            'line_length_m': 30.0,
            'line_start_delay_s': 10.0,
        }],
        output='screen')

    observer_maneuver = Node(
        package='interceptor', executable='observer_maneuver',
        parameters=[{
            'profile': LaunchConfiguration('profile'),
            'altitude_m': 10.0,
            'heading_deg': 0.0,
            'duration_s': float_arg('duration_s'),
            'amplitude_mps': float_arg('amplitude_mps'),
            'start_delay_s': 10.0,
            'start_north_m': 0.0,
            'start_east_m': 0.0,
        }],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'maneuver')),
        output='screen')

    checkpoint_guidance = Node(
        package='interceptor', executable='checkpoint_guidance',
        parameters=[{
            'altitude_m': 10.0,
            'speed_mps': float_arg('speed_mps'),
            'excitation_amp_mps': float_arg('excitation_amp_mps'),
            'freeze_time_s': float_arg('freeze_time_s'),
            'start_delay_s': 10.0,
            'start_north_m': 0.0,
            'start_east_m': 0.0,
        }],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'guidance')),
        output='screen')

    return LaunchDescription([
        *args,
        micro_xrce_agent,
        gz_bridge,
        interceptor_tf2_odometry,
        target_tf2_odometry,
        camera_static_tf,
        OpaqueFunction(function=detector_node),
        target_estimator,
        estimation_evaluator,
        target_trajectory,
        observer_maneuver,
        checkpoint_guidance,
    ])
