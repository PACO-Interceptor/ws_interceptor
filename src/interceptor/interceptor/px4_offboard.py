"""
Base para los nodos que vuelan el interceptor en offboard directo contra PX4.

Reune lo que comparten observer_maneuver y checkpoint_guidance: odometria propia (NED),
estado armado/offboard, reintento de offboard+armado cada 2 s, publicacion del
setpoint, el yaw que sigue al objetivo con el bearing de la camara y la secuencia de
fases:

    CLIMB   sube a altitude_m sobre (start_north_m, start_east_m)
    SETTLE  espera start_delay_s quieto
    <fase activa>  la subclase decide el setpoint (active_setpoint)
    HOLD    se queda quieto donde este

Al entrar en la fase activa reinicia target_estimator (servicio target_estimator/reset):
el despegue ya es una aceleracion del observador y la estimacion llegaria contaminada.

AVISO: mientras un nodo de estos corre, fuerza offboard+armado cada 2 s.
"""

import math

from interceptor_msgs.msg import TargetSighting
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleControlMode, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

NAN = float('nan')


def yaw_from_quaternion(q) -> float:
    """Yaw NED de un cuaternio PX4 (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a: float) -> float:
    """Normaliza un angulo a [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Px4OffboardNode(Node):
    """Nodo con la fontaneria de offboard de PX4; las subclases deciden el setpoint."""

    def __init__(self, name: str, active_phase: str) -> None:
        """Declara los parametros comunes y crea publicadores, suscripciones y timer."""
        super().__init__(name)
        ns = self.declare_parameter('px4_namespace', '').value.rstrip('/')
        self._target_system = self.declare_parameter('target_system', 1).value
        self.altitude = self.declare_parameter('altitude_m', 10.0).value
        self._start_delay = self.declare_parameter('start_delay_s', 5.0).value
        self._reset_estimator = self.declare_parameter('reset_estimator', True).value
        # Punto (NED, origen del interceptor) al que sube antes de empezar. NaN = donde este.
        self._start = (
            self.declare_parameter('start_north_m', NAN).value,
            self.declare_parameter('start_east_m', NAN).value,
        )
        self._active_phase = active_phase

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

        self.position = None        # NED [m]
        self.velocity = None        # NED [m/s]
        self.yaw = None             # NED [rad]
        self.yaw_setpoint = 0.0
        self._armed = False
        self._offboard = False
        self._tick = 0
        self._last_arm_attempt = -math.inf

        self._reset_client = self.create_client(Trigger, 'target_estimator/reset')
        self.phase = 'CLIMB'
        self._phase_start = self.now()
        self.hold = None           # posicion NED que se mantiene fuera de la fase activa
        self.create_timer(0.05, self._timer_callback)

    # --- A implementar por la subclase ------------------------------------------------

    def on_active_start(self) -> None:
        """Se llama al entrar en la fase activa."""

    def active_setpoint(self, elapsed: float):
        """
        Setpoint (position, velocity) NED de la fase activa, o None para pasar a HOLD.

        elapsed son los segundos desde que empezo la fase activa.
        """
        raise NotImplementedError

    # --- Fases -------------------------------------------------------------------------

    def set_phase(self, phase: str) -> None:
        """Cambia de fase y lo registra; fuera de la fase activa se queda donde esta."""
        self.phase = phase
        self._phase_start = self.now()
        self.hold = list(self.position) if self.position is not None else None
        self.get_logger().info(f'Fase {phase}')

    def _timer_callback(self) -> None:
        """Avanza las fases y publica el setpoint que toca."""
        if self.position is None:
            return
        elapsed = self.now() - self._phase_start

        if self.phase == 'CLIMB':
            if self.hold is None:
                self.hold = list(self.position)
                if not math.isnan(self._start[0]) and not math.isnan(self._start[1]):
                    self.hold[0], self.hold[1] = self._start
            horizontal = math.hypot(
                self.position[0] - self.hold[0], self.position[1] - self.hold[1])
            if abs(self.position[2] + self.altitude) < 0.5 and horizontal < 1.0:
                self.set_phase('SETTLE')
        elif self.phase == 'SETTLE' and elapsed >= self._start_delay:
            self.set_phase(self._active_phase)
            self._request_estimator_reset()
            self.on_active_start()
            elapsed = 0.0

        if self.phase == self._active_phase:
            setpoint = self.active_setpoint(elapsed)
            if setpoint is not None:
                self.publish_setpoint(*setpoint)
                return
            self.set_phase('HOLD')

        self.publish_setpoint([self.hold[0], self.hold[1], -self.altitude], [NAN] * 3)

    def _request_estimator_reset(self) -> None:
        """Pide a target_estimator que olvide lo estimado hasta ahora."""
        if not self._reset_estimator:
            return
        if self._reset_client.service_is_ready():
            self._reset_client.call_async(Trigger.Request())
        else:
            self.get_logger().warning('target_estimator/reset no disponible.')

    def now(self) -> float:
        """Tiempo del reloj del nodo [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _odometry_callback(self, msg: VehicleOdometry) -> None:
        """Guarda posicion, velocidad y yaw propios (NED)."""
        self.position = [float(c) for c in msg.position]
        self.velocity = [float(c) for c in msg.velocity]
        if not math.isnan(msg.q[0]):
            self.yaw = yaw_from_quaternion(msg.q)

    def _control_mode_callback(self, msg: VehicleControlMode) -> None:
        """Guarda si esta armado y en offboard."""
        self._armed = msg.flag_armed
        self._offboard = msg.flag_control_offboard_enabled

    def _sighting_callback(self, msg: TargetSighting) -> None:
        """Apunta el yaw al objetivo: la camara mira al frente del cuerpo (FLU)."""
        if self.yaw is None:
            return
        azimuth = math.atan2(msg.bearing.y, msg.bearing.x)   # FLU: positivo a la izquierda
        self.yaw_setpoint = wrap_pi(self.yaw - azimuth)      # NED: positivo a la derecha

    def publish_setpoint(self, position, velocity) -> None:
        """
        Publica el setpoint (NED; NaN = libre) con el yaw hacia el objetivo.

        Mantiene ademas offboard+armado: PX4 exige recibir setpoints antes de aceptar
        offboard, asi que el primer intento va en la llamada 10 y luego cada 2 s.
        """
        self._tick += 1
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.now() * 1e6)
        sp.position = [float(c) for c in position]
        sp.velocity = [float(c) for c in velocity]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = float(self.yaw_setpoint)
        sp.yawspeed = NAN

        mode = OffboardControlMode()
        mode.timestamp = sp.timestamp
        mode.position = True
        mode.velocity = True
        self._offboard_pub.publish(mode)
        self._setpoint_pub.publish(sp)

        if self._tick >= 10 and not (self._armed and self._offboard):
            if self.now() - self._last_arm_attempt >= 2.0:
                self._last_arm_attempt = self.now()
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
        msg.timestamp = int(self.now() * 1e6)
        self._command_pub.publish(msg)
