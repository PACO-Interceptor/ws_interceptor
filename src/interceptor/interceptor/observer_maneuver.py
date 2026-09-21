"""
Nodo ROS 2 que vuela el interceptor con un perfil de maniobra fijo, para ensayar el estimador.

La escala del objetivo (tamano, distancia) solo es observable si el interceptor
acelera. Este nodo permite comprobarlo con perfiles controlados, en offboard directo
contra PX4 (como target_trajectory, sin modo de guiado):

- hover:    se queda quieto. La escala NO deberia converger.
- constant: avanza a velocidad constante. Tampoco deberia converger.
- surge:    avanza con velocidad oscilante (acelera en la linea de avance).
- weave:    avanza a velocidad constante mas una oscilacion lateral.

Al empezar la maniobra reinicia target_estimator (servicio target_estimator/reset): el
despegue ya es una aceleracion del observador y, sin reinicio, hover y constant
llegarian a la maniobra con la escala ya estimada.

En todos, el yaw sigue al objetivo con el bearing de la camara (sin usar su posicion
real) para no perderlo del campo de vision. Tras duration_s, o si la estimacion dice
que el objetivo esta a menos de min_range_m, se queda quieto.

AVISO: mientras el nodo corre, fuerza offboard+armado cada 2 s, como target_trajectory.
"""

import math

from interceptor_msgs.msg import TargetEstimate, TargetSighting
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleControlMode, VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

PROFILES = ('hover', 'constant', 'surge', 'weave')
NAN = float('nan')


def profile_velocity(profile, t, speed, amplitude, frequency_hz):
    """
    Velocidad (avance, lateral) del perfil en el instante t de la maniobra [m/s].

    amplitude es la amplitud de la oscilacion de velocidad, en m/s, tanto en surge
    (a lo largo del avance) como en weave (en lateral).
    """
    osc = amplitude * math.sin(2.0 * math.pi * frequency_hz * t)
    if profile == 'hover':
        return 0.0, 0.0
    if profile == 'constant':
        return speed, 0.0
    if profile == 'surge':
        return speed + osc, 0.0
    if profile == 'weave':
        return speed, osc
    raise ValueError(f'Perfil desconocido: {profile}')


def yaw_from_quaternion(q) -> float:
    """Yaw NED de un cuaternio PX4 (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a: float) -> float:
    """Normaliza un angulo a [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class ObserverManeuver(Node):
    """Despega, espera y ejecuta un perfil de maniobra con el yaw siguiendo al objetivo."""

    def __init__(self) -> None:
        """Declara parametros, publicadores y suscripciones."""
        super().__init__('observer_maneuver')

        ns = self.declare_parameter('px4_namespace', '').value.rstrip('/')
        self._target_system = self.declare_parameter('target_system', 1).value
        self._altitude = self.declare_parameter('altitude_m', 10.0).value
        self._profile = self.declare_parameter('profile', 'hover').value
        self._heading = math.radians(self.declare_parameter('heading_deg', 0.0).value)
        self._speed = self.declare_parameter('speed_mps', 0.5).value
        self._amplitude = self.declare_parameter('amplitude_mps', 1.0).value
        self._frequency = self.declare_parameter('frequency_hz', 0.2).value
        self._start_delay = self.declare_parameter('start_delay_s', 5.0).value
        self._duration = self.declare_parameter('duration_s', 30.0).value
        self._min_range = self.declare_parameter('min_range_m', 6.0).value
        self._reset_estimator = self.declare_parameter('reset_estimator', True).value
        # Punto (NED, origen del interceptor) al que sube antes de empezar. NaN = donde este.
        self._start = (
            self.declare_parameter('start_north_m', NAN).value,
            self.declare_parameter('start_east_m', NAN).value,
        )
        if self._profile not in PROFILES:
            raise ValueError(f'profile debe ser uno de {PROFILES}, no "{self._profile}"')

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', 10)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', 10)
        self._command_pub = self.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry', self._odometry_callback, px4_qos)
        self.create_subscription(
            VehicleControlMode, f'{ns}/fmu/out/vehicle_control_mode',
            self._control_mode_callback, px4_qos)
        self.create_subscription(
            TargetSighting, 'interceptor/target_sighting', self._sighting_callback, 10)
        self.create_subscription(
            TargetEstimate, 'interceptor/target_estimate', self._estimate_callback, 10)
        self._reset_client = self.create_client(Trigger, 'target_estimator/reset')

        self._position = None       # NED [m]
        self._yaw = None            # NED [rad]
        self._yaw_setpoint = self._heading
        self._estimated_range = math.inf
        self._armed = False
        self._offboard = False

        self._phase = 'CLIMB'
        self._phase_start = self._now()
        self._hold = None           # posicion NED que se mantiene fuera de la maniobra
        self._tick = 0
        self._last_arm_attempt = -math.inf

        self.create_timer(0.05, self._timer_callback)
        self.get_logger().info(f'Perfil "{self._profile}", rumbo {math.degrees(self._heading)}.')

    def _now(self) -> float:
        """Tiempo del reloj del nodo [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _odometry_callback(self, msg: VehicleOdometry) -> None:
        """Guarda posicion y yaw propios (NED)."""
        self._position = [float(c) for c in msg.position]
        if not math.isnan(msg.q[0]):
            self._yaw = yaw_from_quaternion(msg.q)

    def _control_mode_callback(self, msg: VehicleControlMode) -> None:
        """Guarda si esta armado y en offboard."""
        self._armed = msg.flag_armed
        self._offboard = msg.flag_control_offboard_enabled

    def _sighting_callback(self, msg: TargetSighting) -> None:
        """Apunta el yaw al objetivo: la camara mira al frente del cuerpo (FLU)."""
        if self._yaw is None:
            return
        azimuth = math.atan2(msg.bearing.y, msg.bearing.x)   # FLU: positivo a la izquierda
        self._yaw_setpoint = wrap_pi(self._yaw - azimuth)    # NED: positivo a la derecha

    def _estimate_callback(self, msg: TargetEstimate) -> None:
        """Guarda la distancia estimada al objetivo (para parar antes de chocar)."""
        self._estimated_range = msg.range

    def _set_phase(self, phase: str) -> None:
        """Cambia de fase y lo registra."""
        self._phase = phase
        self._phase_start = self._now()
        self._hold = list(self._position) if self._position is not None else None
        self.get_logger().info(f'Fase {phase}')

    def _timer_callback(self) -> None:
        """Publica el setpoint de la fase actual y mantiene offboard+armado."""
        self._tick += 1
        if self._position is None:
            return

        elapsed = self._now() - self._phase_start
        if self._phase == 'CLIMB':
            if self._hold is None:
                self._hold = list(self._position)
                if not math.isnan(self._start[0]) and not math.isnan(self._start[1]):
                    self._hold[0], self._hold[1] = self._start
            horizontal = math.hypot(
                self._position[0] - self._hold[0], self._position[1] - self._hold[1])
            if abs(self._position[2] + self._altitude) < 0.5 and horizontal < 1.0:
                self._set_phase('SETTLE')
        elif self._phase == 'SETTLE' and elapsed >= self._start_delay:
            self._set_phase('MANEUVER')
            if self._reset_estimator:
                self._estimated_range = math.inf
                if self._reset_client.service_is_ready():
                    self._reset_client.call_async(Trigger.Request())
                else:
                    self.get_logger().warning('target_estimator/reset no disponible.')
        elif self._phase == 'MANEUVER':
            if elapsed >= self._duration:
                self._set_phase('HOLD')
            elif self._estimated_range < self._min_range:
                self.get_logger().info(
                    f'Objetivo estimado a {self._estimated_range:.1f} m: se para.')
                self._set_phase('HOLD')

        sp = TrajectorySetpoint()
        sp.timestamp = int(self._now() * 1e6)
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yawspeed = NAN
        sp.yaw = float(self._yaw_setpoint)

        if self._phase == 'MANEUVER' and self._profile != 'hover':
            forward, lateral = profile_velocity(
                self._profile, elapsed, self._speed, self._amplitude, self._frequency)
            ch, sh = math.cos(self._heading), math.sin(self._heading)
            sp.position = [NAN, NAN, float(-self._altitude)]
            sp.velocity = [forward * ch - lateral * sh, forward * sh + lateral * ch, NAN]
        else:
            sp.position = [float(self._hold[0]), float(self._hold[1]), float(-self._altitude)]
            sp.velocity = [NAN, NAN, NAN]

        mode = OffboardControlMode()
        mode.timestamp = sp.timestamp
        mode.position = True
        mode.velocity = True
        self._offboard_pub.publish(mode)
        self._setpoint_pub.publish(sp)

        # PX4 exige recibir setpoints antes de aceptar offboard: primer intento en el tick 10.
        if self._tick >= 10 and not (self._armed and self._offboard):
            if self._now() - self._last_arm_attempt >= 2.0:
                self._last_arm_attempt = self._now()
                self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        """Envia un VehicleCommand al PX4 del interceptor."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self._target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self._now() * 1e6)
        self._command_pub.publish(msg)


def main(args=None):
    """Ejecuta el nodo observer_maneuver."""
    rclpy.init(args=args)
    node = ObserverManeuver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
