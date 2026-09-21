"""
Nodo ROS 2 que estima posicion, velocidad y tamano del objetivo desde la camara.

Entrada: TargetSighting (bearing en el frame de la camara + angulo subtendido), y la
pose de la camara en el mundo via tf2 en el instante de captura de la imagen.
Salida: TargetEstimate en el frame del mundo, y la tf world -> target_estimate para
verla en RViz.

El filtro esta en interceptor.bearing_angle_filter; este nodo solo lo alimenta.
"""

import math

from geometry_msgs.msg import TransformStamped
from interceptor.bearing import rotate_by_quaternion
from interceptor.bearing_angle_filter import BearingAngleFilter, FilterParams
from interceptor_msgs.msg import TargetEstimate, TargetSighting
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class TargetEstimator(Node):
    """Alimenta el filtro bearing-angle con cada TargetSighting y publica la estimacion."""

    def __init__(self) -> None:
        """Declara parametros, crea el filtro y las suscripciones."""
        super().__init__('target_estimator')

        self._world_frame = self.declare_parameter('world_frame', 'map').value
        self._child_frame = self.declare_parameter('estimate_frame', 'target_estimate').value
        self._min_confidence = self.declare_parameter('min_confidence', 0.3).value
        self._lost_timeout = self.declare_parameter('lost_timeout_s', 2.0).value
        self._publish_tf = self.declare_parameter('publish_tf', True).value
        # Si se rechazan tantas observaciones seguidas, el filtro ha divergido (p. ej. el
        # objetivo ha maniobrado fuerte) y ya no se recuperaria solo: se reinicia.
        self._max_consecutive_rejections = self.declare_parameter(
            'max_consecutive_rejections', 15).value

        defaults = FilterParams()
        params = FilterParams(
            sigma_accel=self.declare_parameter('sigma_accel', defaults.sigma_accel).value,
            sigma_bearing=self.declare_parameter(
                'sigma_bearing', defaults.sigma_bearing).value,
            sigma_angle=self.declare_parameter('sigma_angle', defaults.sigma_angle).value,
            size_prior=self.declare_parameter('size_prior', defaults.size_prior).value,
            size_prior_rel_std=self.declare_parameter(
                'size_prior_rel_std', defaults.size_prior_rel_std).value,
            velocity_prior_std=self.declare_parameter(
                'velocity_prior_std', defaults.velocity_prior_std).value,
            huber_k=self.declare_parameter('huber_k', defaults.huber_k).value,
            gate=self.declare_parameter('gate', defaults.gate).value,
        )
        self._filter = BearingAngleFilter(params)
        self._last_stamp = None
        self._rejected = 0
        self._consecutive_rejections = 0

        self._tf_buffer = Buffer()
        # Sin esperar en el callback: cuando llega una observacion, YOLO ya ha tardado
        # decenas de ms y la tf del instante de captura ya esta en el buffer.
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._estimate_pub = self.create_publisher(
            TargetEstimate, 'interceptor/target_estimate', 10)
        self._sighting_sub = self.create_subscription(
            TargetSighting, 'interceptor/target_sighting', self._sighting_callback, 10)
        self.create_service(Trigger, 'target_estimator/reset', self._reset_callback)

    def _reset_callback(self, request, response):
        """Olvida la estimacion: la proxima observacion vuelve a partir del prior."""
        self._filter.reset()
        self._last_stamp = None
        self.get_logger().info('Estimacion reiniciada a peticion.')
        response.success = True
        return response

    def _sighting_callback(self, msg: TargetSighting) -> None:
        """Predice hasta el instante de la imagen y corrige con la observacion."""
        if msg.confidence < self._min_confidence:
            return
        stamp = Time.from_msg(msg.header.stamp)

        if self._last_stamp is not None:
            dt = (stamp - self._last_stamp).nanoseconds * 1e-9
            if dt < 0.0:
                return  # llega desordenada: ya hemos pasado ese instante
            if dt > self._lost_timeout:
                self.get_logger().info(
                    f'Objetivo perdido {dt:.1f} s: se reinicia la estimacion '
                    f'conservando el tamano ({self._filter.size:.2f} m).')
                self._filter.reset(keep_size=True)
        else:
            dt = 0.0

        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, msg.header.frame_id, stamp)
        except TransformException as err:
            self.get_logger().warning(
                f'Sin tf {self._world_frame} <- {msg.header.frame_id}: {err}',
                throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        p_o = np.array([t.x, t.y, t.z])
        g = np.array(rotate_by_quaternion(
            r.x, r.y, r.z, r.w, (msg.bearing.x, msg.bearing.y, msg.bearing.z)))
        theta = msg.subtended_angle

        if not self._filter.initialized:
            if theta <= 0.0:
                return  # hace falta el angulo para colocar la primera estimacion
            self._filter.update(p_o, g, theta)
            self.get_logger().info(
                f'Estimacion inicializada con tamano {self._filter.size:.2f} m '
                f'a {self._filter.distance:.1f} m.')
        else:
            self._filter.predict(dt, p_o)
            info = self._filter.update(p_o, g, theta)
            if not (info.bearing_accepted and (info.angle_accepted or theta <= 0.0)):
                self._rejected += 1
                self._consecutive_rejections += 1
                self.get_logger().warning(
                    f'Observacion atipica descartada (d2 bearing {info.bearing_d2:.1f}, '
                    f'angulo {info.angle_d2:.1f}); van {self._rejected}.',
                    throttle_duration_sec=1.0)
                if self._consecutive_rejections >= self._max_consecutive_rejections:
                    self.get_logger().warning(
                        f'{self._consecutive_rejections} observaciones rechazadas seguidas: '
                        'el filtro ha divergido y se reinicia.')
                    self._filter.reset()
                    self._consecutive_rejections = 0
                    self._last_stamp = None
                    return
            else:
                self._consecutive_rejections = 0

        self._last_stamp = stamp
        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        """Publica la estimacion y, opcionalmente, su tf."""
        f = self._filter
        p = f.position
        v = f.velocity

        out = TargetEstimate()
        out.header.stamp = stamp
        out.header.frame_id = self._world_frame
        out.position.x, out.position.y, out.position.z = (float(c) for c in p)
        out.velocity.x, out.velocity.y, out.velocity.z = (float(c) for c in v)
        out.size = float(f.size)
        out.covariance = [float(c) for c in f.physical_covariance().ravel()]
        out.range = float(f.distance)
        out.angular_range = float(f.angular_range)
        out.update_count = f.update_count
        self._estimate_pub.publish(out)

        if self._publish_tf and all(math.isfinite(c) for c in p):
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self._child_frame
            tf.transform.translation.x = out.position.x
            tf.transform.translation.y = out.position.y
            tf.transform.translation.z = out.position.z
            tf.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    """Ejecuta el nodo target_estimator."""
    rclpy.init(args=args)
    node = TargetEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
