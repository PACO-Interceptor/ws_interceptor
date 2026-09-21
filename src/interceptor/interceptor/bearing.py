"""Bearing calculation from pixel coordinates to FLU camera frame."""

import math
from typing import Tuple


def pixel_to_bearing(
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """
    Devuelve (x, y, z), unitario, en el frame FLU de la camara.

    :param u: Coordenada horizontal del pixel (a la derecha).
    :param v: Coordenada vertical del pixel (hacia abajo).
    :param fx: Distancia focal horizontal en pixeles.
    :param fy: Distancia focal vertical en pixeles.
    :param cx: Coordenada horizontal del centro optico en pixeles.
    :param cy: Coordenada vertical del centro optico en pixeles.
    :return: Tupla (x, y, z) con el vector unitario en el frame FLU.
    :raises ValueError: Si fx o fy son cero.
    """
    if fx == 0.0 or fy == 0.0:
        raise ValueError('Focal lengths fx and fy must be non-zero.')

    x_opt = (u - cx) / fx
    y_opt = (v - cy) / fy
    z_opt = 1.0

    # optico -> FLU
    x_flu = z_opt
    y_flu = -x_opt
    z_flu = -y_opt

    norm = math.sqrt(x_flu * x_flu + y_flu * y_flu + z_flu * z_flu)
    return (x_flu / norm, y_flu / norm, z_flu / norm)


def subtended_angle(
    u: float,
    v: float,
    width: float,
    fx: float,
    cx: float,
    cy: float,
) -> float:
    """
    Devuelve el angulo subtendido por el objetivo, en radianes.

    Se calcula por la ley del coseno entre los rayos que van a los puntos medios
    de los lados izquierdo y derecho de la caja. A diferencia del ancho en
    pixeles, el angulo es invariante a la rotacion de la camara: girar la camara
    cambia el ancho de la caja aunque el objetivo no se haya movido.

    El angulo NO da distancia por si solo, porque cumple theta ~= l / r con l el
    tamano fisico del objetivo, que es desconocido. Lo que si da, por su
    derivada temporal, es el tiempo hasta el impacto, que es invariante a la
    escala.

    :param u: Coordenada horizontal del centro de la caja, en pixeles.
    :param v: Coordenada vertical del centro de la caja, en pixeles.
    :param width: Ancho de la caja en pixeles.
    :param fx: Distancia focal horizontal en pixeles.
    :param cx: Coordenada horizontal del centro optico en pixeles.
    :param cy: Coordenada vertical del centro optico en pixeles.
    :return: Angulo subtendido en radianes, en (0, PI).
    :raises ValueError: Si fx es cero o el ancho no es positivo.
    """
    if fx == 0.0:
        raise ValueError('Focal length fx must be non-zero.')
    if width <= 0.0:
        raise ValueError('Bounding box width must be positive.')

    delta_x = abs(u - cx)
    delta_y = abs(v - cy)
    half = width / 2.0

    l_left = math.sqrt(fx * fx + (delta_x - half) ** 2 + delta_y * delta_y)
    l_right = math.sqrt(fx * fx + (delta_x + half) ** 2 + delta_y * delta_y)

    cosine = (l_left * l_left + l_right * l_right - width * width) / (2.0 * l_left * l_right)
    # El redondeo puede sacar el coseno de [-1, 1] con cajas muy pequenas.
    return math.acos(max(-1.0, min(1.0, cosine)))
