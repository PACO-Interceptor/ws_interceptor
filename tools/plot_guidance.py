#!/usr/bin/env python3
"""
Dibuja una pasada de checkpoint_guidance a partir de un rosbag grabado en simulacion.

Uso (con el workspace cargado):
    python3 tools/plot_guidance.py <bag> -o salida [--video cam]

Saca salida.png (resumen: vista cenital, altura, tamano estimado y error) y, con
--video, salida.mp4: la camara del interceptor con las cajas de YOLO junto a la vista
cenital animada. --video es el prefijo de cam.mp4 + cam_stamps.csv (imagenes anotadas
del detector y sus sellos).

El bag necesita /tf y /interceptor/target_estimate. La verdad es la tf del objetivo mas
ball_offset; solo existe en simulacion.
"""

import argparse
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_estimation import read_bag  # noqa: E402

from interceptor.bearing import rotate_by_quaternion  # noqa: E402

C_INT, C_BALL, C_EST = '#1f6fb4', '#e8710a', '#2a9d4b'


def secs(stamp):
    """Segundos de un sello de tiempo."""
    return stamp.sec + stamp.nanosec * 1e-9


def load(bag, ball_offset):
    """Series de la pasada: verdad muestreada a 20 Hz y estimaciones del tramo."""
    msgs = read_bag(bag)
    buf = Buffer(cache_time=Duration(seconds=3600))
    tf_times = []
    for topic, m in msgs:
        for tf in getattr(m, 'transforms', []):
            if topic == '/tf_static':
                buf.set_transform_static(tf, 'bag')
            elif topic == '/tf':
                buf.set_transform(tf, 'bag')
                if tf.child_frame_id == 'interceptor/base_link':
                    tf_times.append(secs(tf.header.stamp))
    est = [m for t, m in msgs if t == '/interceptor/target_estimate']

    def pose(frame, t):
        try:
            tf = buf.lookup_transform('map', frame, Time(nanoseconds=int(t * 1e9)))
        except Exception:
            return None
        return tf.transform

    # Verdad a 20 Hz: interceptor y centro de la pelota.
    truth = []
    for t in np.arange(tf_times[0] + 0.5, tf_times[-1] - 0.5, 0.05):
        i, g = pose('interceptor/base_link', t), pose('target/base_link', t)
        if i is None or g is None:
            continue
        r = g.rotation
        ball = np.array([g.translation.x, g.translation.y, g.translation.z]) + np.array(
            rotate_by_quaternion(r.x, r.y, r.z, r.w, ball_offset))
        truth.append((t, np.array([i.translation.x, i.translation.y, i.translation.z]), ball))
    t_truth = np.array([x[0] for x in truth])
    p_int = np.array([x[1] for x in truth])
    p_ball = np.array([x[2] for x in truth])
    dist = np.linalg.norm(p_int - p_ball, axis=1)
    # Cada paso: minimo local por debajo de 5 m (se rearma al alejarse a mas de 6 m).
    passes, best, armed = [], None, True
    for k, dk in enumerate(dist):
        if dk > 6.0:
            best, armed = None, True
        elif armed and (best is None or dk < dist[best]):
            best = k
        elif armed and best is not None and dist[best] < 5.0 and dk > dist[best] + 1.0:
            passes.append(best)
            armed = False
    if armed and best is not None and dist[best] < 5.0:
        passes.append(best)
    k_ca = passes[-1] if passes else int(np.argmin(dist))
    t_ca = t_truth[k_ca]

    # Inicio de la persecucion: el reinicio del estimador con el interceptor en su punto
    # de partida (el origen).
    t_start, last = secs(est[0].header.stamp), None
    for m in est:
        t = secs(m.header.stamp)
        if last is not None and m.update_count < last and t < t_ca:
            k = int(np.argmin(abs(t_truth - t)))
            if np.hypot(*p_int[k, :2]) < 2.0:
                t_start = t
        last = m.update_count
    t_end = t_ca + 3.0
    seg = [m for m in est if t_start <= secs(m.header.stamp) <= t_end]
    e = {
        't': np.array([secs(m.header.stamp) for m in seg]),
        'p': np.array([[m.position.x, m.position.y, m.position.z] for m in seg]),
        'size': np.array([m.size for m in seg]),
        'size_sigma': np.array([math.sqrt(max(m.covariance[48], 0.0)) for m in seg]),
        'pos_sigma': np.array([math.sqrt(max(m.covariance[0] + m.covariance[8], 0.0))
                               for m in seg]),
    }
    keep = (t_truth >= t_start - 1.0) & (t_truth <= t_end)
    ball_at_est = np.array([p_ball[np.argmin(abs(t_truth - t))] for t in e['t']])
    e['err'] = np.linalg.norm(e['p'] - ball_at_est, axis=1)
    return {
        't0': t_start, 't_ca': t_ca, 'miss': float(dist[k_ca]),
        'passes': [(t_truth[k], float(dist[k])) for k in passes],
        't': t_truth[keep], 'int': p_int[keep], 'ball': p_ball[keep], 'est': e,
    }


def draw_map(ax, d, t_now=None):
    """Vista cenital (este, norte): trayectorias reales y estimaciones."""
    e = d['est']
    m = d['t'] <= (t_now if t_now is not None else math.inf)
    me = e['t'] <= (t_now if t_now is not None else math.inf)
    ax.plot(d['int'][m, 0], d['int'][m, 1], color=C_INT, lw=2, label='interceptor (real)')
    ax.plot(d['ball'][m, 0], d['ball'][m, 1], color=C_BALL, lw=2, label='checkpoint (real)')
    if me.any():
        ax.scatter(e['p'][me, 0], e['p'][me, 1], s=10, color=C_EST, alpha=0.5,
                   label='checkpoint (estimado)')
        if t_now is not None:
            k = np.flatnonzero(me)[-1]
            circ = plt.Circle(e['p'][k, :2], 2 * e['pos_sigma'][k], color=C_EST, fill=False,
                              ls='--', lw=1)
            ax.add_patch(circ)
            ax.plot([d['int'][m, 0][-1], e['p'][k, 0]], [d['int'][m, 1][-1], e['p'][k, 1]],
                    color=C_EST, lw=0.8, alpha=0.6)
    if m.any():
        ax.plot(*d['int'][m, :2][-1], 'o', color=C_INT, ms=8)
        ax.plot(*d['ball'][m, :2][-1], 'o', color=C_BALL, ms=10)
    for n, (t_pass, miss) in enumerate(d['passes'], 1):
        if t_now is not None and t_now < t_pass:
            continue
        k = int(np.argmin(abs(d['t'] - t_pass)))
        label = f'paso {n}: {miss:.2f} m del centro' if len(d['passes']) > 1 else \
            f'paso a {miss:.2f} m del centro'
        ax.annotate(label, d['int'][k, :2], xytext=(12, -22 - 16 * n),
                    textcoords='offset points', fontsize=9,
                    arrowprops={'arrowstyle': '->', 'lw': 0.8})
    ax.set_xlabel('este [m]')
    ax.set_ylabel('norte [m]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    all_xy = np.vstack((d['int'][:, :2], d['ball'][:, :2]))
    lo, hi = all_xy.min(0) - 3, all_xy.max(0) + 3
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])


def draw_size(ax, d, true_size, t_now=None):
    """Tamano estimado con +-2 sigma frente al real."""
    e = d['est']
    t = e['t'] - d['t0']
    ax.fill_between(t, e['size'] - 2 * e['size_sigma'], e['size'] + 2 * e['size_sigma'],
                    color=C_EST, alpha=0.2)
    ax.plot(t, e['size'], color=C_EST, lw=2, label='estimado (+-2 sigma)')
    ax.axhline(true_size, color='0.3', ls='--', lw=1, label='real')
    ax.axvline(d['t_ca'] - d['t0'], color=C_BALL, lw=1, alpha=0.7)
    if t_now is not None:
        ax.axvline(t_now - d['t0'], color='k', lw=1)
    ax.set_ylim(0, max(2.0, true_size * 2))
    ax.set_xlabel('tiempo de persecucion [s]')
    ax.set_ylabel('tamano [m]')
    ax.grid(True, alpha=0.3)


def summary(d, out, true_size, title):
    """Figura resumen de la pasada."""
    fig = plt.figure(figsize=(12, 8))
    ax_map = fig.add_subplot(1, 2, 1)
    ax_alt = fig.add_subplot(3, 2, 2)
    ax_size = fig.add_subplot(3, 2, 4)
    ax_err = fig.add_subplot(3, 2, 6)
    draw_map(ax_map, d)
    ax_map.legend(loc='lower left', fontsize=8)

    t = d['t'] - d['t0']
    ax_alt.plot(t, d['int'][:, 2], color=C_INT, label='interceptor')
    ax_alt.plot(t, d['ball'][:, 2], color=C_BALL, label='checkpoint')
    ax_alt.axvline(d['t_ca'] - d['t0'], color=C_BALL, lw=1, alpha=0.7)
    ax_alt.set_ylabel('altura [m]')
    ax_alt.legend(fontsize=8)
    ax_alt.grid(True, alpha=0.3)

    draw_size(ax_size, d, true_size)
    ax_size.legend(fontsize=8)

    e = d['est']
    ax_err.plot(e['t'] - d['t0'], e['err'], color=C_EST)
    ax_err.axvline(d['t_ca'] - d['t0'], color=C_BALL, lw=1, alpha=0.7)
    ax_err.set_ylabel('error posicion [m]')
    ax_err.set_xlabel('tiempo de persecucion [s] (vertical naranja = paso)')
    ax_err.set_yscale('log')
    ax_err.grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out + '.png', dpi=110)
    plt.close(fig)


def video(d, cam_prefix, out, true_size, title):
    """Camara anotada + vista cenital animada, sincronizadas por sello de tiempo."""
    import cv2
    stamps = np.loadtxt(cam_prefix + '_stamps.csv')
    cap = cv2.VideoCapture(cam_prefix + '.mp4')
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr[:, :, ::-1])
    stamps = stamps[:len(frames)]
    idx = [k for k, s in enumerate(stamps) if d['t0'] - 1.0 <= s <= d['t_ca'] + 3.0]

    fig = plt.figure(figsize=(14, 6.2))
    ax_cam = fig.add_axes([0.01, 0.08, 0.48, 0.82])
    ax_map = fig.add_axes([0.55, 0.36, 0.42, 0.58])
    ax_size = fig.add_axes([0.55, 0.08, 0.42, 0.2])
    writer = FFMpegWriter(fps=11, bitrate=3000)
    with writer.saving(fig, out + '.mp4', dpi=100):
        for k in idx:
            t_now = stamps[k]
            for ax in (ax_cam, ax_map, ax_size):
                ax.clear()
            ax_cam.imshow(frames[k])
            ax_cam.set_axis_off()
            ax_cam.set_title('camara del interceptor (YOLO)', fontsize=10)
            draw_map(ax_map, d, t_now)
            ax_map.legend(loc='lower left', fontsize=7)
            draw_size(ax_size, d, true_size, t_now)
            fig.suptitle(f'{title}   t = {t_now - d["t0"]:5.1f} s', fontsize=11)
            writer.grab_frame()
    plt.close(fig)


def main():
    """Lee el bag y genera la figura y, si se pide, el video."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('bag')
    parser.add_argument('-o', '--output', required=True, help='Prefijo de salida')
    parser.add_argument('--video', help='Prefijo de cam.mp4 + cam_stamps.csv')
    parser.add_argument('--title', default='checkpoint_guidance en Gazebo')
    parser.add_argument('--true-size', type=float, default=1.0)
    parser.add_argument('--ball-offset', type=float, nargs=3, default=[0.0, 0.0, 2.5])
    args = parser.parse_args()

    d = load(args.bag, tuple(args.ball_offset))
    for n, (_, miss) in enumerate(d['passes'], 1):
        print(f'paso {n}: {miss:.2f} m del centro')
    print(f"tamano final {d['est']['size'][-1]:.2f} m")
    summary(d, args.output, args.true_size, args.title)
    print(f'{args.output}.png')
    if args.video:
        video(d, args.video, args.output, args.true_size, args.title)
        print(f'{args.output}.mp4')


if __name__ == '__main__':
    main()
