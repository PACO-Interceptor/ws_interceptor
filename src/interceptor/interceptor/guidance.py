"""
Guiado para pasar por el checkpoint movil con su estimacion visual.

Ley: rumbo de colision. Suponiendo que el checkpoint sigue a velocidad constante, se
resuelve el instante t_go en que el interceptor, volando a velocidad fija, lo alcanza,
y se manda la velocidad hacia ese punto de encuentro. Es la version a nivel de
velocidad de la navegacion proporcional: con la estimacion perfecta, el bearing al
checkpoint se queda constante.

Acertar NO necesita escala: la condicion de colision (bearing constante) es la misma a
cualquier escala, y en simulacion sintetica el interceptor pasa a < 0.3 m del centro
con el tamano mal estimado. La escala sirve para saber CUANDO se pasa (t_go), para
frenar o esquivar lo que haya alrededor del checkpoint, y para llegar con una
orientacion dada.

Pero un bearing constante con el interceptor a velocidad constante es justo el caso en
que la escala NO es observable (ver bearing_angle_filter). Por eso se suma una
oscilacion lateral (excitacion) cuya amplitud depende de lo mal que se conoce el
tamano: plena por encima de sigma_rel_start, nula por debajo de sigma_rel_done, y nula
en los ultimos terminal_time segundos para no desviar el paso final.

Todo en NED (el frame de PX4), metros y segundos. Sin ROS: el nodo esta en
checkpoint_guidance.py.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

DOWN = np.array([0.0, 0.0, 1.0])


@dataclass
class GuidanceParams:
    """Parametros del guiado."""

    # Velocidad del interceptor hacia el punto de encuentro [m/s].
    speed: float = 3.0
    # Amplitud y frecuencia de la oscilacion lateral de excitacion [m/s], [Hz].
    excitation_amp: float = 2.0
    excitation_freq: float = 0.25
    # Incertidumbre relativa del tamano (sigma/tamano) con excitacion plena y nula.
    sigma_rel_start: float = 0.15
    sigma_rel_done: float = 0.05
    # En los ultimos segundos antes del paso, sin excitacion: con 2.5 s la oscilacion aun
    # no se ha amortiguado al perder la imagen de cerca y el fallo se triplica (sintetico).
    terminal_time: float = 4.0
    # Limite de velocidad vertical [m/s].
    max_vertical_speed: float = 1.5
    # Congelar el rumbo exige ademas ver el checkpoint grande: distancia menor que
    # freeze_max_angular_range tamanos de checkpoint (sin escala). Evita congelar lejos
    # cuando la velocidad estimada aun es ruido (recien reiniciado el estimador).
    freeze_max_angular_range: float = 8.0


def intercept_time(r: np.ndarray, v_t: np.ndarray, speed: float) -> Optional[float]:
    """
    Menor t > 0 con |r + v_t t| = speed * t, o None si no hay encuentro posible.

    r es la posicion del checkpoint relativa al interceptor y v_t su velocidad.
    """
    a = float(v_t @ v_t) - speed * speed
    b = 2.0 * float(r @ v_t)
    c = float(r @ r)
    if abs(a) < 1e-9:
        # Misma velocidad que el checkpoint: ecuacion lineal.
        return -c / b if b < 0.0 else None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    roots = [t for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)) if t > 0.0]
    return min(roots) if roots else None


def time_to_closest_approach(r: np.ndarray, v_rel: np.ndarray) -> float:
    """
    Tiempo hasta la maxima aproximacion con la velocidad relativa actual (inf si se aleja).

    r es la posicion del checkpoint relativa al interceptor y v_rel = v_checkpoint -
    v_interceptor. A diferencia de intercept_time, no mezcla la velocidad del interceptor
    (en metros) con la distancia estimada: r y v_rel escalan con el mismo factor si el
    tamano esta mal estimado, y el cociente no cambia.
    """
    vv = float(v_rel @ v_rel)
    if vv < 1e-9:
        return math.inf
    t = -float(r @ v_rel) / vv
    return t if t > 0.0 else math.inf


def should_freeze(time_to_pass: float, angular_range: float, freeze_time: float,
                  prm: GuidanceParams) -> bool:
    """Indica si toca congelar el rumbo: paso inminente y checkpoint visto de cerca."""
    return time_to_pass < freeze_time and angular_range < prm.freeze_max_angular_range


def excitation_gain(size_rel_sigma: float, t_go: float, prm: GuidanceParams) -> float:
    """Fraccion [0, 1] de la excitacion segun la incertidumbre del tamano y t_go."""
    if t_go <= prm.terminal_time:
        return 0.0
    span = prm.sigma_rel_start - prm.sigma_rel_done
    return min(max((size_rel_sigma - prm.sigma_rel_done) / span, 0.0), 1.0)


def guidance_velocity(
    r: np.ndarray,
    v_t: np.ndarray,
    size_rel_sigma: float,
    tau: float,
    prm: GuidanceParams,
    time_to_pass: Optional[float] = None,
) -> Tuple[np.ndarray, float, float]:
    """
    Velocidad de mando (NED) hacia el punto de encuentro, con la excitacion sumada.

    :param r: Posicion estimada del checkpoint relativa al interceptor [m].
    :param v_t: Velocidad estimada del checkpoint [m/s].
    :param size_rel_sigma: sigma/tamano de la estimacion.
    :param tau: Tiempo [s] que marca la fase de la oscilacion de excitacion.
    :param prm: Parametros.
    :param time_to_pass: Tiempo hasta el paso para apagar la excitacion en el tramo
        final; por defecto, el t_go del punto de encuentro (que depende de la escala).
    :return: (velocidad de mando, t_go, fraccion de excitacion aplicada).
    """
    t_go = intercept_time(r, v_t, prm.speed)
    if t_go is None:
        t_go = float(np.linalg.norm(r)) / prm.speed   # persecucion pura como respaldo
        aim = r
    else:
        aim = r + v_t * t_go
    norm = float(np.linalg.norm(aim))
    direction = aim / norm if norm > 1e-6 else np.zeros(3)
    v_cmd = prm.speed * direction

    gain = excitation_gain(size_rel_sigma, t_go if time_to_pass is None else time_to_pass, prm)
    lateral = np.cross(direction, DOWN)
    lateral_norm = float(np.linalg.norm(lateral))
    if gain > 0.0 and lateral_norm > 1e-6:
        osc = math.sin(2.0 * math.pi * prm.excitation_freq * tau)
        v_cmd = v_cmd + gain * prm.excitation_amp * osc * lateral / lateral_norm

    v_cmd[2] = min(max(v_cmd[2], -prm.max_vertical_speed), prm.max_vertical_speed)
    return v_cmd, t_go, gain
