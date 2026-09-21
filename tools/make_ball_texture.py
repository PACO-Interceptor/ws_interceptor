#!/usr/bin/env python3
"""
Genera la textura de balon de futbol de la pelota del checkpoint (x500_ball).

Proyeccion equirectangular (2048x1024), la que Gazebo usa para una esfera: pentagonos
negros y hexagonos blancos, calculados como el Voronoi esferico de las 32 caras del
icosaedro truncado.

Se eligio midiendo con YOLO (COCO) en Gazebo a 2-24 m: fue la unica textura
reconocida como "sports ball" en casi todo el rango. La naranja lisa salia como
"orange" de cerca, los cuadros blancos y negros como "kite" y la blanca no se
detectaba.

Uso: python3 tools/make_ball_texture.py [salida.png]
"""

import os
import sys

import numpy as np
from PIL import Image

W, H = 2048, 1024
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'interceptor',
                           'models', 'x500_ball', 'materials', 'textures', 'soccer.png')


def main():
    """Calcula y guarda la textura."""
    lon = (np.arange(W) + 0.5) / W * 2 * np.pi - np.pi
    lat = np.pi / 2 - (np.arange(H) + 0.5) / H * np.pi
    lon, lat = np.meshgrid(lon, lat)
    dirs = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], -1)

    phi = (1 + 5 ** 0.5) / 2
    ico = np.array([[0, 1, phi], [0, -1, phi], [0, 1, -phi], [0, -1, -phi],
                    [1, phi, 0], [-1, phi, 0], [1, -phi, 0], [-1, -phi, 0],
                    [phi, 0, 1], [-phi, 0, 1], [phi, 0, -1], [-phi, 0, -1]], float)
    ico /= np.linalg.norm(ico, axis=1, keepdims=True)
    faces = []
    for i in range(12):
        for j in range(i + 1, 12):
            for k in range(j + 1, 12):
                if all(ico[a] @ ico[b] > 0.4 for a, b in ((i, j), (j, k), (i, k))):
                    c = ico[i] + ico[j] + ico[k]
                    faces.append(c / np.linalg.norm(c))
    centers = np.vstack([ico, np.array(faces)])     # 12 pentagonos + 20 hexagonos

    dots = dirs @ centers.T
    order = np.argsort(-dots, axis=-1)
    best = np.take_along_axis(dots, order[..., :1], -1)[..., 0]
    second = np.take_along_axis(dots, order[..., 1:2], -1)[..., 0]
    rgb = np.where((order[..., 0] < 12)[..., None], 25, 245) * np.ones(3)
    rgb[(best - second) < 0.012] = 90                # costuras

    out = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT
    Image.fromarray(rgb.astype(np.uint8)).save(out)
    print(out)


if __name__ == '__main__':
    main()
