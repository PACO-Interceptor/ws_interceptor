#!/usr/bin/env python3
"""
Reproduce offline el filtro bearing-angle sobre un rosbag grabado en simulacion.

Uso (con el workspace cargado):
    python3 tools/replay_estimation.py <bag> [--offset-ms 0 20 -20] [--size-prior 0.5]

El bag necesita /interceptor/target_sighting, /tf y /tf_static; con /interceptor/
target_estimate se reproduce solo el ultimo tramo (desde el ultimo reinicio del
estimador en vuelo). Compara con la verdad (tf del objetivo + ball_offset).

--offset-ms desplaza el instante en que se consulta la pose de la camara respecto al
sello de la imagen: sirve para ver si un desfase temporal entre imagen y odometria
sesga la escala.
"""

import argparse
import math

from builtin_interfaces.msg import Time as TimeMsg
from interceptor.bearing import rotate_by_quaternion
from interceptor.bearing_angle_filter import BearingAngleFilter, FilterParams
import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer


def read_bag(path):
    """Devuelve la lista de (topic, mensaje) del bag, en orden."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out.append((topic, deserialize_message(data, types[topic])))
    return out


def stamp_plus(stamp, offset_s):
    """Sello de tiempo desplazado offset_s segundos."""
    ns = Time.from_msg(stamp).nanoseconds + int(offset_s * 1e9)
    return Time(nanoseconds=ns)


def main():
    """Lee el bag, reproduce el filtro para cada desfase y resume el resultado."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('bag')
    parser.add_argument('--offset-ms', type=float, nargs='+', default=[0.0])
    parser.add_argument('--size-prior', type=float, default=0.5)
    parser.add_argument('--sigma-bearing', type=float, default=FilterParams().sigma_bearing)
    parser.add_argument('--true-size', type=float, default=1.0)
    parser.add_argument('--ball-offset', type=float, nargs=3, default=[0.0, 0.0, 2.5])
    args = parser.parse_args()

    msgs = read_bag(args.bag)
    buf = Buffer(cache_time=Duration(seconds=3600))
    for topic, m in msgs:
        for tf in getattr(m, 'transforms', []):
            if topic == '/tf_static':
                buf.set_transform_static(tf, 'bag')
            elif topic == '/tf':
                buf.set_transform(tf, 'bag')

    sightings = [m for t, m in msgs if t == '/interceptor/target_sighting']
    # Ultimo reinicio del estimador en vuelo: donde update_count vuelve a 0.
    start = TimeMsg()
    last_count = None
    for t, m in msgs:
        if t == '/interceptor/target_estimate':
            if last_count is not None and m.update_count < last_count:
                start = m.header.stamp
            last_count = m.update_count
    t_start = Time.from_msg(start).nanoseconds
    sightings = [s for s in sightings if Time.from_msg(s.header.stamp).nanoseconds >= t_start]
    print(f'{len(sightings)} observaciones en el tramo reproducido')

    for offset_ms in args.offset_ms:
        filt = BearingAngleFilter(FilterParams(
            size_prior=args.size_prior, sigma_bearing=args.sigma_bearing))
        last = None
        rows = []
        for s in sightings:
            if s.confidence < 0.3:
                continue
            stamp = Time.from_msg(s.header.stamp)
            try:
                cam = buf.lookup_transform('map', s.header.frame_id,
                                           stamp_plus(s.header.stamp, offset_ms / 1000.0))
                tgt = buf.lookup_transform('map', 'target/base_link', stamp)
            except Exception:
                continue
            ct, cr = cam.transform.translation, cam.transform.rotation
            p_o = np.array([ct.x, ct.y, ct.z])
            g = np.array(rotate_by_quaternion(cr.x, cr.y, cr.z, cr.w,
                                              (s.bearing.x, s.bearing.y, s.bearing.z)))
            dt = 0.0 if last is None else (stamp - last).nanoseconds * 1e-9
            if not filt.initialized and s.subtended_angle <= 0.0:
                continue
            if filt.initialized:
                filt.predict(dt, p_o)
            filt.update(p_o, g, s.subtended_angle)
            last = stamp
            tt, tr = tgt.transform.translation, tgt.transform.rotation
            ball = np.array([tt.x, tt.y, tt.z]) + np.array(
                rotate_by_quaternion(tr.x, tr.y, tr.z, tr.w, tuple(args.ball_offset)))
            sigma = math.sqrt(filt.physical_covariance()[6, 6])
            rows.append((stamp.nanoseconds * 1e-9, filt.size, sigma,
                         float(np.linalg.norm(filt.position - ball)),
                         float(np.linalg.norm(ball - p_o))))
        if not rows:
            print(f'desfase {offset_ms:+6.1f} ms: sin datos')
            continue
        t0 = rows[0][0]
        marks = [r for r in rows if r[0] - t0 >= 1.0][:1] + [rows[len(rows) // 2], rows[-1]]
        txt = '  '.join(f't={r[0] - t0:4.1f}s tam {r[1]:.2f}+-{r[2]:.2f} err {r[3]:.2f}m '
                        f'(dist {r[4]:.1f})' for r in marks)
        print(f'desfase {offset_ms:+6.1f} ms: {txt}')


if __name__ == '__main__':
    main()
