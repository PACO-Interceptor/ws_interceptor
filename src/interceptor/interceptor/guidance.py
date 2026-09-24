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

No comprometerse hasta saber: mientras la incertidumbre relativa del tamano (sigma del
filtro, nunca la verdad) siga por encima de sigma_rel_commit, el interceptor OBSERVA:
excita y se acerca, pero no deja que el tiempo hasta el contacto (distancia entre
velocidad de cierre) baje de standoff_time
(tramo final + frenado), retrocediendo si hace falta. Solo con la escala conocida
ATACA con el guiado de siempre. Si no puede esperar (el checkpoint se le echa encima
mas rapido de lo que puede apartarse, o la sigma no baja en max_observe_time), se
compromete igualmente y lo dice (CommitGate).

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

    # Velocidad de crucero del interceptor hacia el punto de encuentro [m/s] con un
    # checkpoint que no es lento; con uno lento baja hasta terminal_speed (ver base_speed) y
    # sube solo cuando hace falta para alcanzarlo (ver cruise_speed), hasta max_speed.
    speed: float = 3.0
    # Lo que el dron puede dar [m/s]: debe coincidir con MPC_XY_VEL_MAX de PX4 (12 m/s por
    # defecto), que recorta las consignas de velocidad en offboard.
    max_speed: float = 12.0
    # En los ultimos slowdown_time segundos baja hasta terminal_speed: a 3 m/s el tramo
    # final a ciegas mide metro y medio, demasiado para un checkpoint pequeno. Nunca baja
    # de terminal_speed_margin veces la velocidad ESTIMADA del checkpoint: con poco margen
    # sobre ella, la geometria de intercepcion se vuelve muy sensible. Ese limite sale de
    # la estimacion de a bordo, no de la velocidad real, que en vuelo no se conoce.
    terminal_speed: float = 1.2
    terminal_speed_margin: float = 1.8
    slowdown_time: float = 3.0
    # Amplitud y frecuencia de la oscilacion de excitacion [m/s], [Hz].
    excitation_amp: float = 2.0
    excitation_freq: float = 0.25
    # Direccion de la excitacion al observar: 'lateral' (perpendicular al rumbo) o 'radial'
    # (acelerar y frenar a lo largo de la linea de vision). Con bearing + angulo la escala es
    # observable con cualquier aceleracion, tambien la radial (Ning et al., IJRR 2024,
    # apartado 5.4), y la radial no desvia el punto de mira. En Gazebo el sesgo del tamano
    # nace de errores laterales del bearing (reproduccion con el bearing real: +18 % -> +6 %).
    excitation_mode: str = 'lateral'
    # Incertidumbre relativa del tamano (sigma/tamano) con excitacion plena y nula.
    sigma_rel_start: float = 0.15
    sigma_rel_done: float = 0.05
    # En los ultimos segundos antes del paso, sin excitacion: con 2.5 s la oscilacion aun
    # no se ha amortiguado al perder la imagen de cerca y el fallo se triplica (sintetico).
    terminal_time: float = 4.0
    # Limite de velocidad vertical [m/s].
    max_vertical_speed: float = 1.5
    # Al observar, angulo de elevacion del checkpoint [rad] al que se sube o baja ya a
    # max_vertical_speed: 10 grados, una cuarta parte del semicampo vertical de la camara
    # (+-42 grados con 1280x960 y 1.74 rad de campo horizontal), para que no se salga de la
    # imagen con el cabeceo al acelerar. El piloto no dejara el dron a la altura exacta del
    # checkpoint: con 2.5 m de desnivel se salia del campo vertical a menos de 4 m.
    level_full_elevation: float = math.radians(10.0)
    # Congelar el rumbo exige ademas ver el checkpoint grande: distancia menor que
    # freeze_max_angular_range tamanos de checkpoint (sin escala). Evita congelar lejos
    # cuando la velocidad estimada aun es ruido (recien reiniciado el estimador).
    freeze_max_angular_range: float = 8.0
    # Por debajo de esta sigma/tamano se da la escala por conocida y se ataca. Es el
    # mismo nivel al que la excitacion empieza a reducirse: por encima, el propio guiado
    # ya considera que el tamano no se conoce.
    sigma_rel_commit: float = 0.15
    # Sin escala conocida, compromiso forzado si el tiempo hasta el contacto se queda por
    # debajo de terminal_time durante forced_commit_hold segundos (no se ha podido
    # mantener la distancia), o si se lleva max_observe_time observando.
    forced_commit_hold: float = 0.5
    max_observe_time: float = 25.0
    # Aceleracion horizontal maxima del propio dron [m/s^2]: MPC_ACC_HOR_MAX de PX4 (5 por
    # defecto; en los vuelos de Gazebo el interceptor llega a p99 5 m/s^2). Con ella,
    # reverse_time es lo que tarda en pasar de acercarse a speed a retroceder a speed.
    max_accel: float = 5.0
    # Ganancia de la retencion al observar [m/s por s de error en el tiempo hasta el
    # contacto].
    standoff_gain: float = 0.5
    # Velocidad maxima de acercamiento propio al observar [m/s]. Con el tamano mal
    # estimado, el filtro confunde el avance propio con movimiento del checkpoint y el
    # tiempo hasta el contacto sale optimista (68 s estimados con 7 s reales y el tamano
    # 3 veces sobreestimado, lazo sintetico); la escala la da la excitacion lateral, no
    # el avance.
    observe_speed: float = 1.0
    # ... pero solo cuando el checkpoint ya se ve bien: a menos de
    # observe_max_angular_range tamanos (sin escala; 25 tamanos son 40 mrad, 21 px con la
    # camara de 1280 px y 1.74 rad de campo). Por debajo de ~25 mrad (13 px) el detector
    # pasa mas de la mitad del tiempo sin ver nada (vuelos de Gazebo, cualquier pelota),
    # y observar desde ahi es solo perder detecciones y reiniciar el filtro.
    observe_max_angular_range: float = 25.0

    @property
    def reverse_time(self) -> float:
        """Tiempo para invertir la velocidad propia a lo largo de la linea de vision [s]."""
        return 2.0 * self.speed / self.max_accel

    @property
    def standoff_time(self) -> float:
        """Tiempo hasta el contacto que se guarda al observar: tramo final mas frenado."""
        return self.terminal_time + self.slowdown_time


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


def base_speed(v_t: np.ndarray, size_rel_sigma: float, prm: GuidanceParams) -> float:
    """
    Velocidad de ataque de partida, segun lo rapido que va el checkpoint.

    Es terminal_speed_margin veces su velocidad estimada, entre terminal_speed y speed.
    Se usa la cota ALTA de la velocidad, v (1 + size_rel_sigma): solo se frena cuando se
    esta seguro de que el checkpoint es lento. Con la cota baja, con la escala aun
    incierta iba despacio y lo perdia (lazo sintetico: cruce a 2 m/s de 10/10 a 3/10).

    Con un checkpoint lento no hace falta ir a speed, y el ataque tiene que durar
    terminal_time sin excitacion antes del paso: a 3 m/s eso pide comprometerse a ~12 m y
    el detector solo es fiable hasta ~7.5 m con la pelota de 0.30 m (validacion del 24/09:
    a 0.5 y 1 m/s se comprometia a ~4 m y el ataque duraba ~1.3 s). Sin constantes nuevas.
    """
    v = float(np.linalg.norm(v_t)) * (1.0 + size_rel_sigma)
    return min(max(prm.terminal_speed, prm.terminal_speed_margin * v), prm.speed)


def cruise_speed(r: np.ndarray, v_t: np.ndarray, size_rel_sigma: float,
                 prm: GuidanceParams, slow_attack: bool = False) -> float:
    """
    Velocidad de crucero: la de partida salvo que haga falta mas para alcanzarlo.

    La de partida es speed, o base_speed si slow_attack. slow_attack solo al atacar tras
    comprometerse por escala: justo tras reiniciar el estimador la velocidad estimada es
    cero, y observando (o tras un compromiso forzado) iba despacio y lo perdia o fallaba
    de frente (lazo sintetico).

    Sube a terminal_speed_margin veces la componente de la velocidad ESTIMADA del
    checkpoint que se aleja del interceptor (con 3 m/s fijos, uno a 2 m/s que se aleja
    apenas se dejaba alcanzar), y a ese margen sobre su modulo si ni asi hay punto de
    encuentro (checkpoint rapido cruzando). Nunca pasa de max_speed. Solo lo que hace
    falta: ir mas deprisa de lo necesario empeoraba el paso (lazo sintetico: de frente y
    alejandose a 2 m/s, 6/10 con 1.8 veces el modulo frente a 10/10 con 3 m/s).

    La velocidad estimada escala con el tamano estimado, asi que se toma su cota baja,
    v_t (1 - size_rel_sigma): con la escala aun sin conocer (tamano 3 veces sobreestimado
    al empezar) subir el crucero echaba el interceptor encima del checkpoint al observar.
    """
    base = base_speed(v_t, size_rel_sigma, prm) if slow_attack else prm.speed
    v_t = v_t * max(1.0 - size_rel_sigma, 0.0)
    dist = float(np.linalg.norm(r))
    receding = max(float(r @ v_t) / dist, 0.0) if dist > 1e-6 else 0.0
    speed = max(base, prm.terminal_speed_margin * receding)
    if intercept_time(r, v_t, speed) is None:
        speed = max(speed, prm.terminal_speed_margin * float(np.linalg.norm(v_t)))
    return min(speed, prm.max_speed)


def approach_speed(time_to_pass: float, target_speed: float, prm: GuidanceParams,
                   cruise: Optional[float] = None) -> float:
    """
    Velocidad de aproximacion: la de crucero, bajando hacia terminal_speed al acercarse el paso.

    target_speed es el modulo de la velocidad ESTIMADA del checkpoint; el frenado no baja
    de terminal_speed_margin veces esa velocidad, ni de la velocidad de crucero si el
    checkpoint resultara ser mas rapido que ella. cruise es la velocidad de crucero
    (cruise_speed); por defecto, speed.
    """
    cruise = prm.speed if cruise is None else cruise
    if not math.isfinite(time_to_pass) or time_to_pass >= prm.slowdown_time:
        return cruise
    floor = min(max(prm.terminal_speed, prm.terminal_speed_margin * target_speed), cruise)
    f = max(time_to_pass, 0.0) / prm.slowdown_time
    return floor + f * (cruise - floor)


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
    excite: bool = True,
    slow_attack: bool = False,
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
    :param excite: False para no sumar excitacion aunque el tamano no se conozca.
    :param slow_attack: True al atacar: con un checkpoint lento, crucero mas lento
        (base_speed).
    :return: (velocidad de mando, t_go, fraccion de excitacion aplicada).
    """
    speed = approach_speed(time_to_pass if time_to_pass is not None else math.inf,
                           float(np.linalg.norm(v_t)), prm,
                           cruise_speed(r, v_t, size_rel_sigma, prm, slow_attack))
    t_go = intercept_time(r, v_t, speed)
    if t_go is None:
        t_go = float(np.linalg.norm(r)) / speed       # persecucion pura como respaldo
        aim = r
    else:
        aim = r + v_t * t_go
    norm = float(np.linalg.norm(aim))
    direction = aim / norm if norm > 1e-6 else np.zeros(3)
    v_cmd = speed * direction

    gain = excitation_gain(size_rel_sigma, t_go if time_to_pass is None else time_to_pass,
                           prm) if excite else 0.0
    lateral = np.cross(direction, DOWN)
    lateral_norm = float(np.linalg.norm(lateral))
    if gain > 0.0 and lateral_norm > 1e-6:
        osc = math.sin(2.0 * math.pi * prm.excitation_freq * tau)
        v_cmd = v_cmd + gain * prm.excitation_amp * osc * lateral / lateral_norm

    v_cmd[2] = min(max(v_cmd[2], -prm.max_vertical_speed), prm.max_vertical_speed)
    return v_cmd, t_go, gain


def time_to_contact(r: np.ndarray, v_rel: np.ndarray) -> float:
    """
    Distancia entre velocidad de cierre: cuanto falta para llegar al checkpoint.

    Es infinito si no se cierra distancia.
    A diferencia de time_to_closest_approach, no llega a cero cuando el checkpoint pasa
    de largo a distancia. Tambien es libre de escala.
    """
    dist = float(np.linalg.norm(r))
    closing = -float(r @ v_rel) / dist if dist > 1e-6 else 0.0
    return dist / closing if closing > 1e-6 else math.inf


def contact_rate(r: np.ndarray, v_rel: np.ndarray,
                 cov: Optional[np.ndarray] = None) -> Tuple[float, float]:
    """
    Inversa del tiempo hasta el contacto, k = -r.v_rel / |r|^2 [1/s], y su sigma.

    k > 0 si se cierra distancia (tau = 1/k), <= 0 si se abre; es continua donde tau salta
    a infinito. La sigma sale por propagacion lineal de cov, la covarianza 6x6 de la
    posicion y la velocidad del checkpoint (la propia del dron se da por exacta); 0 sin cov.
    Como k no cambia al escalar r y v_rel juntos, la parte de la sigma que solo es
    incertidumbre del tamano se cancela: la sigma de la velocidad de cierre en metros no
    sirve (con el tamano al 70 % el dron se creia siempre a punto de chocar y retrocedia sin
    parar: lazo sintetico, cruce a 2 m/s de 10/10 a 4/10).
    """
    dist2 = float(r @ r)
    if dist2 < 1e-12:
        return 0.0, 0.0
    k = -float(r @ v_rel) / dist2
    if cov is None:
        return k, 0.0
    grad = np.concatenate((-v_rel / dist2 - 2.0 * k * r / dist2, -r / dist2))
    return k, math.sqrt(max(float(grad @ cov @ grad), 0.0))


def contact_time_bounds(r: np.ndarray, v_rel: np.ndarray,
                        cov: Optional[np.ndarray] = None) -> Tuple[float, float]:
    """Cotas baja y alta del tiempo hasta el contacto: 1/(k + sigma) y 1/(k - sigma)."""
    k, sigma = contact_rate(r, v_rel, cov)
    low = 1.0 / (k + sigma) if k + sigma > 1e-9 else math.inf
    high = 1.0 / (k - sigma) if k - sigma > 1e-9 else math.inf
    return low, high


def can_hold_distance(r: np.ndarray, v_t: np.ndarray, size_rel_sigma: float,
                      contact_time_high: float, prm: GuidanceParams) -> bool:
    """
    Indica si retrocediendo aun se puede mantener la distancia al checkpoint.

    Hace falta que el checkpoint no se acerque mas deprisa que speed (a lo que retrocede
    hold_off como poco), con la cota baja de su velocidad v_t (1 - size_rel_sigma) porque
    la estimada escala con el tamano, y que el contacto no llegue antes de lo que el dron
    tarda en invertir la marcha (reverse_time), con la cota alta del tiempo hasta el
    contacto.
    """
    dist = float(np.linalg.norm(r))
    if dist < 1e-6:
        return False
    approach = max(-float(r @ v_t) / dist, 0.0) * max(1.0 - size_rel_sigma, 0.0)
    return approach < prm.speed and contact_time_high > prm.reverse_time


def hold_off(v_cmd: np.ndarray, r: np.ndarray, v_t: np.ndarray, v_own: np.ndarray,
             angular_range: float, prm: GuidanceParams,
             size_rel_sigma: float = 0.0) -> np.ndarray:
    """
    Limita la velocidad de cierre de la orden para no bajar de standoff_time.

    Recorta la componente de v_cmd a lo largo de la linea de vision (u = r/|r|) a
    observe_speed si el checkpoint ya se ve bien (angular_range, distancia en tamanos de
    checkpoint, por debajo de observe_max_angular_range), y siempre a u.v_own +
    standoff_gain (tiempo hasta el contacto - standoff_time): si el contacto llegaria
    antes de standoff_time, pide cerrar mas despacio que ahora (y retroceder si hace
    falta, como mucho a la velocidad de crucero); si llegaria despues, deja acelerar poco
    a poco. Si la distancia se abre, sube la componente para no perder el checkpoint. La
    componente lateral, y con ella la excitacion, no se toca.

    Se realimenta con el tiempo hasta el contacto, que no depende de la escala, y sobre
    la velocidad propia, que PX4 da en metros: el equilibrio es tiempo hasta el contacto
    = standoff_time aunque el tamano este mal estimado. Un limite calculado con r y v_t
    estimados (u.v_t + |r|/standoff_time) no lo cumple: con el tamano 3 veces
    sobreestimado dejaba cerrar a menos de 4 s en el lazo sintetico.

    Si el tiempo hasta el contacto queda por debajo de terminal_time, retrocede a tope
    (-crucero): la realimentacion suave dejaba al checkpoint acercarse a 2 m/s mientras el
    dron solo llegaba a retroceder a 1.8 m/s (Gazebo, S1v20). Aqui no se usa la
    incertidumbre de la velocidad: recien hecho el traspaso es tan grande que, con la cota
    baja del tiempo hasta el contacto (en la realimentacion o para retroceder a tope), el
    dron huia y perdia el checkpoint, y exigiendo que la distancia se abra con seguridad
    para seguirlo se escapaba el que se aleja (lazo sintetico: cruce a 2 m/s de 10/10 a
    4-5/10; alejandose a 4 m/s de 8/10 a 5/10).
    """
    dist = float(np.linalg.norm(r))
    if dist < 1e-6:
        return v_cmd
    u = r / dist
    own = float(u @ v_own)
    upper = prm.observe_speed if angular_range < prm.observe_max_angular_range else math.inf
    tau = time_to_contact(r, v_t - v_own)
    if math.isfinite(tau):
        upper = min(upper, own + prm.standoff_gain * (tau - prm.standoff_time))
    retreat = -cruise_speed(r, v_t, size_rel_sigma, prm)
    if tau < prm.terminal_time:
        upper = min(upper, retreat)
    # Suelo de distancia con un checkpoint lento (ataque por debajo de speed): el ataque
    # necesita terminal_time a esa velocidad; si el checkpoint esta mas cerca, retroceder
    # aunque no venga hacia el dron (uno lento que cruza no baja el tiempo hasta el
    # contacto). Como mucho a observe_max_angular_range tamanos, el alcance fiable del
    # detector. Con uno rapido no se aplica: el suelo quedaba en ese borde del alcance y
    # el dron perdia detecciones (lazo sintetico: de 10/10 a 5-7/10 en cruce a 2 m/s y a
    # 4 m/s de frente o alejandose).
    base = base_speed(v_t, size_rel_sigma, prm)
    floor = base * prm.terminal_time if base < prm.speed else 0.0
    if angular_range > 1e-6:
        floor = min(floor, dist * prm.observe_max_angular_range / angular_range)
    if dist < floor:
        upper = min(upper, own + prm.standoff_gain * (dist - floor) / base)
    upper = max(upper, retreat)
    # Si la distancia se abre, no dejarlo escapar: al observar sin la escala, el crucero no
    # sube (cota baja de la velocidad) y un checkpoint que se aleja salia del alcance del
    # detector. Tambien se realimenta sobre la velocidad propia: el equilibrio (no se abre
    # distancia) no depende de la escala. Manda sobre el limite superior.
    opening = float(u @ (v_t - v_own))
    lower = own + prm.standoff_gain * opening if opening > 0.0 else -math.inf
    cmd_closing = float(u @ v_cmd)
    target = min(max(cmd_closing, lower), max(upper, lower))
    if target == cmd_closing:
        return v_cmd
    out = v_cmd + (target - cmd_closing) * u
    norm = float(np.linalg.norm(out))
    if norm > prm.max_speed:
        out *= prm.max_speed / norm
    out[2] = min(max(out[2], -prm.max_vertical_speed), prm.max_vertical_speed)
    return out


class CommitGate:
    """
    Decide cuando el interceptor se compromete con la aproximacion final.

    Una vez comprometido no vuelve atras. forced indica si el compromiso fue sin la
    escala conocida, y reason por que.
    """

    def __init__(self, prm: GuidanceParams) -> None:
        """Empieza observando."""
        self._prm = prm
        self.committed = False
        self.forced = False
        self.reason = ''
        self.commit_sigma = math.nan
        self.commit_contact_time = math.nan
        self._observe_time = 0.0
        self._short_time = 0.0

    def update(self, size_rel_sigma: float, contact_time: float, dt: float,
               can_hold: bool = False) -> bool:
        """
        Actualiza con la sigma relativa del tamano y el tiempo hasta el contacto.

        :param contact_time: Tiempo hasta el contacto (su cota alta si se conoce).
        :param can_hold: True si retrocediendo aun se puede mantener la distancia
            (can_hold_distance): entonces un contacto cercano no fuerza el compromiso,
            porque hold_off ya esta retrocediendo. Con 0.5 s de margen se forzaba antes de
            que el dron invirtiera la marcha, casi siempre recien hecho el traspaso y con la
            velocidad aun mal estimada (validacion 2: 6 de 20 forzados, uno con el
            checkpoint quieto).
        :return: True solo en el paso en que se compromete.
        """
        if self.committed:
            return False
        prm = self._prm
        self._observe_time += dt
        short = contact_time < prm.terminal_time and not can_hold
        self._short_time = self._short_time + dt if short else 0.0
        # Con la escala ya conocida pero sin terminal_time por delante, si retrocediendo se
        # puede recuperar tiempo, se espera: comprometerse asi dejaba ~1 s de ataque (S1v20,
        # 0.32 m).
        no_time = contact_time < prm.terminal_time and can_hold
        if size_rel_sigma < prm.sigma_rel_commit and not no_time:
            self.reason = 'escala conocida'
        elif self._short_time >= prm.forced_commit_hold:
            self.forced, self.reason = True, 'no se puede mantener la distancia'
        elif self._observe_time >= prm.max_observe_time:
            self.forced, self.reason = True, 'tiempo maximo de observacion'
        else:
            return False
        self.committed = True
        self.commit_sigma, self.commit_contact_time = size_rel_sigma, contact_time
        return True


def level_velocity(r: np.ndarray, v_t: np.ndarray, prm: GuidanceParams) -> float:
    """
    Velocidad vertical (NED) para dejar el checkpoint a la altura de la camara al observar.

    Proporcional al angulo de elevacion del bearing, que no depende del tamano estimado,
    mas la velocidad vertical estimada del checkpoint; acotada por max_vertical_speed.
    """
    dist = float(np.linalg.norm(r))
    if dist < 1e-6:
        return 0.0
    elevation = math.asin(max(-1.0, min(1.0, -float(r[2]) / dist)))    # NED: arriba > 0
    vz = float(v_t[2]) - prm.max_vertical_speed * elevation / prm.level_full_elevation
    return min(max(vz, -prm.max_vertical_speed), prm.max_vertical_speed)


def gated_velocity(gate: CommitGate, r: np.ndarray, v_t: np.ndarray, v_own: np.ndarray,
                   size_rel_sigma: float, angular_range: float, tau: float, dt: float,
                   prm: GuidanceParams,
                   cov: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float, float]:
    """
    Velocidad de mando observando o atacando, segun decida la puerta de compromiso.

    OBSERVA: excitacion segun la sigma, cierre limitado por hold_off y altura segun
    level_velocity (checkpoint a la altura de la camara). El tiempo que
    apaga la excitacion es el de contacto: el de maxima aproximacion llega a cero cuando
    el checkpoint pasa de largo a distancia, y apagaba la excitacion justo al observar.
    ATACA: guiado de siempre sin excitacion (el ataque es corto y en el lazo sintetico la
    oscilacion aun activa llegaba al paso), con el tiempo hasta el paso para el frenado.

    :param v_own: Velocidad del interceptor [m/s] (NED).
    :param angular_range: Distancia al checkpoint en tamanos de checkpoint (sin escala).
    :param dt: Tiempo desde la llamada anterior [s], para la puerta.
    :param cov: Covarianza 6x6 de la posicion y la velocidad estimadas del checkpoint,
        para la cota alta del tiempo hasta el contacto (None: sin incertidumbre).
    :return: (velocidad de mando, tiempo hasta el paso (atacando) o hasta el contacto
        (observando), excitacion).
    """
    v_rel = v_t - v_own
    t_contact = time_to_contact(r, v_rel)
    t_contact_high = contact_time_bounds(r, v_rel, cov)[1]
    gate.update(size_rel_sigma, t_contact_high, dt,
                can_hold_distance(r, v_t, size_rel_sigma, t_contact_high, prm))
    if not gate.committed:
        if prm.excitation_mode == 'radial':
            # La retencion va antes de sumar la oscilacion: si no, recortaria su mitad de
            # acercamiento y la excitacion dejaria de ser simetrica.
            v_cmd, _, _ = guidance_velocity(r, v_t, size_rel_sigma, tau, prm, t_contact,
                                            excite=False)
            v_cmd = hold_off(v_cmd, r, v_t, v_own, angular_range, prm, size_rel_sigma)
            gain = excitation_gain(size_rel_sigma, t_contact, prm)
            dist = float(np.linalg.norm(r))
            if gain > 0.0 and dist > 1e-6:
                osc = math.sin(2.0 * math.pi * prm.excitation_freq * tau)
                v_cmd = v_cmd + gain * prm.excitation_amp * osc * r / dist
        else:
            v_cmd, _, gain = guidance_velocity(r, v_t, size_rel_sigma, tau, prm, t_contact)
            v_cmd = hold_off(v_cmd, r, v_t, v_own, angular_range, prm, size_rel_sigma)
        v_cmd[2] = level_velocity(r, v_t, prm)
        return v_cmd, t_contact, gain
    t_pass = time_to_closest_approach(r, v_rel)
    v_cmd, _, gain = guidance_velocity(r, v_t, size_rel_sigma, tau, prm, t_pass,
                                       excite=False, slow_attack=not gate.forced)
    return v_cmd, t_pass, gain
