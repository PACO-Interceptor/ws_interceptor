"""
Nodo ROS 2 que vuela el interceptor con un perfil de maniobra fijo, para ensayar el estimador.

La escala del objetivo (tamano, distancia) solo es observable si el interceptor
acelera. Este nodo permite comprobarlo con perfiles controlados, en offboard directo
contra PX4 (sin modo de guiado):

- hover:    se queda quieto. La escala NO deberia converger.
- constant: avanza a velocidad constante. Tampoco deberia converger.
- surge:    avanza con velocidad oscilante (acelera en la linea de avance).
- weave:    avanza a velocidad constante mas una oscilacion lateral.

Fases, yaw hacia el objetivo y reinicio del estimador: ver px4_offboard. Tras
duration_s, o si la estimacion dice que el objetivo esta a menos de min_range_m, se
queda quieto.
"""

import math

from interceptor.px4_offboard import NAN, Px4OffboardNode
from interceptor_msgs.msg import TargetEstimate
import rclpy
from rclpy.executors import ExternalShutdownException

PROFILES = ('hover', 'constant', 'surge', 'weave')


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


class ObserverManeuver(Px4OffboardNode):
    """Ejecuta un perfil de maniobra con el yaw siguiendo al objetivo."""

    def __init__(self) -> None:
        """Declara los parametros del perfil."""
        super().__init__('observer_maneuver', 'MANEUVER')
        self._profile = self.declare_parameter('profile', 'hover').value
        self._heading = math.radians(self.declare_parameter('heading_deg', 0.0).value)
        self.yaw_setpoint = self._heading
        self._speed = self.declare_parameter('speed_mps', 0.5).value
        self._amplitude = self.declare_parameter('amplitude_mps', 1.0).value
        self._frequency = self.declare_parameter('frequency_hz', 0.2).value
        self._duration = self.declare_parameter('duration_s', 30.0).value
        self._min_range = self.declare_parameter('min_range_m', 6.0).value
        if self._profile not in PROFILES:
            raise ValueError(f'profile debe ser uno de {PROFILES}, no "{self._profile}"')

        self._estimated_range = math.inf
        self.create_subscription(
            TargetEstimate, 'interceptor/target_estimate', self._estimate_callback, 10)
        self.get_logger().info(f'Perfil "{self._profile}", rumbo {math.degrees(self._heading)}.')

    def _estimate_callback(self, msg: TargetEstimate) -> None:
        """Guarda la distancia estimada al objetivo (para parar antes de chocar)."""
        self._estimated_range = msg.range

    def on_active_start(self) -> None:
        """Olvida la distancia de la estimacion anterior al reinicio."""
        self._estimated_range = math.inf

    def active_setpoint(self, elapsed: float):
        """Velocidad del perfil, a altitud fija; None al acabar o si esta demasiado cerca."""
        if elapsed >= self._duration:
            return None
        if self._estimated_range < self._min_range:
            self.get_logger().info(f'Objetivo estimado a {self._estimated_range:.1f} m: se para.')
            return None
        if self._profile == 'hover':
            return [self.hold[0], self.hold[1], -self.altitude], [NAN] * 3
        forward, lateral = profile_velocity(
            self._profile, elapsed, self._speed, self._amplitude, self._frequency)
        ch, sh = math.cos(self._heading), math.sin(self._heading)
        return ([NAN, NAN, -self.altitude],
                [forward * ch - lateral * sh, forward * sh + lateral * ch, NAN])


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
