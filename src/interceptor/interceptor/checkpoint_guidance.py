"""
Nodo ROS 2 que guia al interceptor a traves del checkpoint movil con su estimacion visual.

Usa la ley de interceptor.guidance (rumbo de colision + excitacion lateral mientras el
tamano no se conoce) con la estimacion de target_estimator, en offboard directo contra
PX4. Fases (ver px4_offboard): CLIMB, SETTLE, PURSUE, HOLD. Dentro de PURSUE:

- Con estimacion reciente: velocidad de guiado.
- El tramo final se decide con el tiempo hasta el paso calculado con la velocidad
  relativa (time_to_closest_approach), que no depende del tamano estimado. El t_go del
  punto de encuentro si depende: con el tamano a la mitad llego a congelar el rumbo 1 s
  antes de tiempo en Gazebo.
- Frena en la aproximacion final (interceptor.guidance.approach_speed): a 3 m/s el
  tramo a ciegas del final mide metro y medio, demasiado para un checkpoint pequeno.
- En el ultimo freeze_time_s antes del paso congela el rumbo: de muy cerca la direccion
  al checkpoint gira deprisa y seguirla desvia el paso. Es corto porque, congelado, el
  error de la velocidad estimada del checkpoint se convierte directamente en fallo.
- Si deja de ver el checkpoint en el tramo final (< terminal_time), o si la
  estimacion dice que ya lo ha dejado atras, sigue recto con la ultima orden durante
  el t_go que quedaba mas coast_s, y pasa a HOLD. Si iba a ciegas y lo vuelve a ver
  delante, retoma el guiado.
- Si lo pierde lejos, se para en el sitio hasta volver a verlo.

La estimacion se extrapola con su velocidad hasta el instante actual: entre la captura
de la imagen y el uso de la estimacion pasan decenas de ms (YOLO, transporte y el lazo
de 20 Hz), y con el checkpoint en movimiento eso es error lateral directo.
"""

import math

from interceptor.guidance import guidance_velocity, GuidanceParams
from interceptor.guidance import should_freeze, time_to_closest_approach
from interceptor.px4_offboard import NAN, Px4OffboardNode
from interceptor_msgs.msg import TargetEstimate
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException


def enu_to_ned(x, y, z) -> np.ndarray:
    """Vector ENU (map) a NED (PX4)."""
    return np.array([y, x, -z])


class CheckpointGuidance(Px4OffboardNode):
    """Persigue la estimacion del checkpoint hasta pasar por el."""

    def __init__(self) -> None:
        """Declara los parametros del guiado."""
        super().__init__('checkpoint_guidance', 'PURSUE')
        defaults = GuidanceParams()
        self._prm = GuidanceParams(
            speed=self.declare_parameter('speed_mps', defaults.speed).value,
            excitation_amp=self.declare_parameter(
                'excitation_amp_mps', defaults.excitation_amp).value,
            excitation_freq=self.declare_parameter(
                'excitation_freq_hz', defaults.excitation_freq).value,
            terminal_time=self.declare_parameter(
                'terminal_time_s', defaults.terminal_time).value,
            terminal_speed=self.declare_parameter(
                'terminal_speed_mps', defaults.terminal_speed).value,
            slowdown_time=self.declare_parameter(
                'slowdown_time_s', defaults.slowdown_time).value,
        )
        self._pass_radius = self.declare_parameter('pass_radius_m', 5.0).value
        self._coast_time = self.declare_parameter('coast_s', 2.0).value
        self._freeze_time = self.declare_parameter('freeze_time_s', 0.3).value
        self._estimate_timeout = self.declare_parameter('estimate_timeout_s', 0.5).value
        self._max_duration = self.declare_parameter('max_duration_s', 60.0).value
        self._min_altitude = self.declare_parameter('min_altitude_m', 3.0).value
        # Retraso entre la captura de la imagen y la llegada de la estimacion [s].
        self._estimate_latency = self.declare_parameter('estimate_latency_s', 0.06).value

        self._estimate = None
        self._estimate_time = -math.inf
        self._last_cmd = None
        self._last_t_go = math.inf
        self._coast_until = None
        self._coast_start = None
        self._coast_resumable = False
        self._last_log = -math.inf
        self.create_subscription(
            TargetEstimate, 'interceptor/target_estimate', self._estimate_callback, 10)

    def _estimate_callback(self, msg: TargetEstimate) -> None:
        """Guarda la ultima estimacion y cuando llego."""
        self._estimate = msg
        self._estimate_time = self.now()

    def on_active_start(self) -> None:
        """Olvida la estimacion previa al reinicio del estimador."""
        self._estimate = None
        self._last_cmd = None
        self._last_t_go = math.inf
        self._coast_until = None

    def active_setpoint(self, elapsed: float):
        """Velocidad de guiado, recta tras el paso o parada si no hay estimacion."""
        now = self.now()
        if elapsed >= self._max_duration:
            self.get_logger().warning('Tiempo maximo de persecucion agotado.')
            return None

        fresh = self._estimate is not None and now - self._estimate_time < self._estimate_timeout
        if self._coast_until is not None:
            if (self._coast_resumable and fresh and self._estimate_time > self._coast_start
                    and self._still_ahead()):
                self.get_logger().info('Vuelve a ver el checkpoint delante: retoma el guiado.')
                self._coast_until = None
            elif now >= self._coast_until:
                return None
            else:
                return [NAN] * 3, self._limit_altitude(self._last_cmd)

        if not fresh:
            if self._last_cmd is not None and self._last_t_go < self._prm.terminal_time:
                self._start_coast(now, 'sin imagen en el tramo final', resumable=True)
                return [NAN] * 3, self._limit_altitude(self._last_cmd)
            return [NAN] * 3, [0.0, 0.0, 0.0]

        e = self._estimate
        v_t = enu_to_ned(e.velocity.x, e.velocity.y, e.velocity.z)
        age = now - self._estimate_time + self._estimate_latency
        r = (enu_to_ned(e.position.x, e.position.y, e.position.z) + v_t * age
             - np.array(self.position))
        rel_sigma = math.sqrt(max(e.covariance[48], 0.0)) / max(e.size, 1e-6)
        t_pass = time_to_closest_approach(r, v_t - np.array(self.velocity))
        v_cmd, _, gain = guidance_velocity(r, v_t, rel_sigma, elapsed, self._prm, t_pass)
        t_go = t_pass
        freeze = should_freeze(t_go, e.angular_range, self._freeze_time, self._prm)
        if freeze and self._last_cmd is not None:
            self._last_t_go = t_go
            self._start_coast(now, 'rumbo congelado en el tramo final')
            return [NAN] * 3, self._limit_altitude(self._last_cmd)
        self._last_cmd, self._last_t_go = v_cmd, t_go

        if float(r @ v_cmd) < 0.0 and float(np.linalg.norm(r)) < self._pass_radius:
            self._start_coast(now, 'checkpoint estimado detras')
        elif now - self._last_log >= 1.0:
            self._last_log = now
            self.get_logger().info(
                f't_go {t_go:4.1f} s  dist {np.linalg.norm(r):5.1f} m  '
                f'tamano {e.size:4.2f} m (sigma {100 * rel_sigma:3.0f} %)  '
                f'excitacion {100 * gain:3.0f} %')
        return [NAN] * 3, self._limit_altitude(v_cmd)

    def _still_ahead(self) -> bool:
        """Indica si la estimacion pone el checkpoint delante de la ultima orden."""
        e = self._estimate
        r = enu_to_ned(e.position.x, e.position.y, e.position.z) - np.array(self.position)
        return float(r @ np.array(self._last_cmd)) > 0.0

    def _start_coast(self, now: float, reason: str, resumable: bool = False) -> None:
        """Sigue recto con la ultima orden el t_go que quedaba mas coast_s."""
        remaining = self._last_t_go if math.isfinite(self._last_t_go) else 0.0
        self._coast_start = now
        self._coast_resumable = resumable
        self._coast_until = now + max(remaining, 0.0) + self._coast_time
        self.get_logger().info(
            f'Paso final ({reason}): recto {self._coast_until - now:.1f} s y parada.')

    def _limit_altitude(self, v_cmd):
        """No deja bajar por debajo de min_altitude_m."""
        v = [float(c) for c in v_cmd]
        if -self.position[2] < self._min_altitude and v[2] > 0.0:
            v[2] = 0.0
        return v


def main(args=None):
    """Ejecuta el nodo checkpoint_guidance."""
    rclpy.init(args=args)
    node = CheckpointGuidance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
