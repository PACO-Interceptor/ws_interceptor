"""
Nodo ROS 2 que compara la estimacion visual del objetivo con la verdad de la simulacion.

Solo para pruebas: la "verdad" es la odometria de PX4 del dron objetivo (tf
map -> target/base_link y el topic target/velocity), mas el desplazamiento fijo del
centro de la pelota respecto a su base_link. En vuelo real no existe.

Escribe una fila por estimacion en un CSV (para tools/plot_estimation.py) y saca un
resumen por consola cada pocos segundos.
"""

import csv
import math
import os
import time

from geometry_msgs.msg import TwistStamped
from interceptor.bearing import rotate_by_quaternion
from interceptor_msgs.msg import TargetEstimate
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

CSV_COLUMNS = [
    'segment', 't', 'update_count',
    'est_x', 'est_y', 'est_z', 'true_x', 'true_y', 'true_z',
    'est_vx', 'est_vy', 'est_vz', 'true_vx', 'true_vy', 'true_vz',
    'est_size', 'size_sigma', 'true_size',
    'est_range', 'true_range', 'est_angular_range', 'true_angular_range',
    'pos_err', 'vel_err', 'pos_sigma',
    'cam_x', 'cam_y', 'cam_z',
]


class EstimationEvaluator(Node):
    """Registra el error de la estimacion frente a la verdad de la simulacion."""

    def __init__(self) -> None:
        """Declara parametros, abre el CSV y se suscribe a la estimacion."""
        super().__init__('estimation_evaluator')

        self._world_frame = self.declare_parameter('world_frame', 'map').value
        self._target_frame = self.declare_parameter('target_frame', 'target/base_link').value
        self._camera_frame = self.declare_parameter(
            'camera_frame', 'interceptor/camera_link').value
        # Centro de la pelota en el base_link del objetivo (FLU) y su diametro real.
        self._ball_offset = tuple(self.declare_parameter(
            'ball_offset', [0.0, 0.0, 2.5]).value)
        self._true_size = self.declare_parameter('true_size', 1.0).value
        self._summary_period = self.declare_parameter('summary_period_s', 2.0).value
        self._interceptor_frame = self.declare_parameter(
            'interceptor_frame', 'interceptor/base_link').value
        csv_path = self.declare_parameter('csv_path', '').value
        if not csv_path:
            csv_dir = os.path.expanduser('~/.ros/interceptor_estimation')
            os.makedirs(csv_dir, exist_ok=True)
            csv_path = os.path.join(csv_dir, time.strftime('%Y%m%d_%H%M%S') + '.csv')

        self._csv_file = open(csv_path, 'w', newline='')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_COLUMNS)
        self.get_logger().info(f'Registrando la evaluacion en {csv_path}')

        self._true_velocity = None
        self._t0 = None
        self._last_summary = None
        self._segment = 0
        self._last_update_count = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(TwistStamped, 'target/velocity', self._velocity_callback, 10)
        # Distancia real interceptor-pelota, para medir por cuanto pasa por el checkpoint.
        self._closest = math.inf
        self._pass_logged = False
        # Cada acercamiento se etiqueta segun si el guiado ya se habia comprometido con el
        # paso final cuando fue la distancia minima. Cuenta el PRIMER ATAQUE; los reataques
        # se reportan aparte y los acercamientos observando no son un intento.
        self._committed = False
        self._committed_at_closest = False
        self._attacks = 0
        self.create_subscription(
            Bool, 'interceptor/guidance_committed', self._committed_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.02, self._closest_approach_callback)
        self.create_subscription(
            TargetEstimate, 'interceptor/target_estimate', self._estimate_callback, 10)

    def _committed_callback(self, msg: Bool) -> None:
        """Guarda si el guiado ya se ha comprometido con el paso final."""
        self._committed = msg.data

    def _velocity_callback(self, msg: TwistStamped) -> None:
        """Guarda la ultima velocidad real del objetivo (ENU)."""
        v = msg.twist.linear
        self._true_velocity = np.array([v.x, v.y, v.z])

    def _lookup(self, frame: str, stamp: Time):
        """Devuelve la tf world <- frame en stamp, o None si no esta disponible."""
        try:
            return self._tf_buffer.lookup_transform(self._world_frame, frame, stamp)
        except TransformException:
            return None

    def _closest_approach_callback(self) -> None:
        """Registra la distancia minima de cada paso cerca de la pelota (< 5 m)."""
        target_tf = self._lookup(self._target_frame, Time())
        interceptor_tf = self._lookup(self._interceptor_frame, Time())
        if target_tf is None or interceptor_tf is None:
            return
        t = target_tf.transform.translation
        r = target_tf.transform.rotation
        ball = np.array([t.x, t.y, t.z]) + np.array(
            rotate_by_quaternion(r.x, r.y, r.z, r.w, self._ball_offset))
        i = interceptor_tf.transform.translation
        dist = float(np.linalg.norm(ball - np.array([i.x, i.y, i.z])))
        if dist > 6.0:
            # Lejos: se rearma para registrar el proximo paso.
            self._closest = math.inf
            self._pass_logged = False
        elif dist < self._closest:
            self._closest = dist
            self._committed_at_closest = self._committed
        elif not self._pass_logged and self._closest < 5.0 and dist > self._closest + 1.0:
            self._pass_logged = True
            if not self._committed_at_closest:
                self.get_logger().info(
                    f'Acercamiento sin atacar (observando): distancia minima '
                    f'{self._closest:.2f} m')
                return
            self._attacks += 1
            if self._attacks == 1:
                self.get_logger().info(
                    f'PASO por el checkpoint (primer ataque): distancia minima al centro '
                    f'{self._closest:.2f} m')
            else:
                self.get_logger().info(
                    f'Reataque {self._attacks - 1}: distancia minima {self._closest:.2f} m')

    def _estimate_callback(self, msg: TargetEstimate) -> None:
        """Compara una estimacion con la verdad y la registra."""
        stamp = Time.from_msg(msg.header.stamp)
        target_tf = self._lookup(self._target_frame, stamp)
        camera_tf = self._lookup(self._camera_frame, stamp)
        if target_tf is None or camera_tf is None or self._true_velocity is None:
            self.get_logger().warning(
                'Sin verdad de simulacion (tf del objetivo/camara o target/velocity).',
                throttle_duration_sec=5.0)
            return

        t = target_tf.transform.translation
        r = target_tf.transform.rotation
        offset = rotate_by_quaternion(r.x, r.y, r.z, r.w, self._ball_offset)
        true_p = np.array([t.x, t.y, t.z]) + np.array(offset)
        c = camera_tf.transform.translation
        cam = np.array([c.x, c.y, c.z])

        est_p = np.array([msg.position.x, msg.position.y, msg.position.z])
        est_v = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        cov = np.array(msg.covariance).reshape(7, 7)
        true_range = float(np.linalg.norm(true_p - cam))
        pos_err = float(np.linalg.norm(est_p - true_p))
        vel_err = float(np.linalg.norm(est_v - self._true_velocity))
        size_sigma = math.sqrt(max(cov[6, 6], 0.0))
        pos_sigma = math.sqrt(max(np.trace(cov[0:3, 0:3]), 0.0))

        now = stamp.nanoseconds * 1e-9
        # Si el contador baja, el estimador se ha reiniciado: empieza un tramo nuevo.
        if msg.update_count < self._last_update_count:
            self._segment += 1
            self._t0 = None
            self.get_logger().info(f'Estimador reiniciado: tramo {self._segment}.')
        self._last_update_count = msg.update_count
        if self._t0 is None:
            self._t0 = now
        rel_t = now - self._t0

        self._csv.writerow([
            self._segment, f'{rel_t:.3f}', msg.update_count,
            *est_p, *true_p, *est_v, *self._true_velocity,
            msg.size, size_sigma, self._true_size,
            msg.range, true_range, msg.angular_range, true_range / self._true_size,
            pos_err, vel_err, pos_sigma,
            *cam,
        ])

        if self._last_summary is None or now - self._last_summary >= self._summary_period:
            self._last_summary = now
            self._csv_file.flush()
            self.get_logger().info(
                f't={rel_t:5.1f}s  err pos {pos_err:5.2f} m (sigma {pos_sigma:5.2f})  '
                f'vel {vel_err:4.2f} m/s  tamano {msg.size:4.2f}+-{size_sigma:4.2f} m '
                f'(real {self._true_size:.2f})  dist {msg.range:5.1f}/{true_range:5.1f} m')

    def destroy_node(self) -> None:
        """Cierra el CSV antes de destruir el nodo."""
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    """Ejecuta el nodo estimation_evaluator."""
    rclpy.init(args=args)
    node = EstimationEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
