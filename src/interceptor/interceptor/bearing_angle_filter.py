"""
Filtro bearing-angle para estimar el movimiento de un objetivo de tamano desconocido.

Medidas (Ning et al., "A Bearing-Angle Approach for Unknown Target Motion Analysis
Based on Visual Measurements", IJRR 2024, arXiv 2401.17117), por cada deteccion:

- g: vector unitario de la camara al objetivo, ya rotado al mundo.
- theta: angulo subtendido por el objetivo. Para una esfera de diametro l cuyo centro
  esta a distancia r: sin(theta / 2) = (l / 2) / r.

Estado normalizado por el tamano (idea del "normalized depth" de Bearing-Box,
arXiv 2601.06887), en el frame del mundo (map, ENU):

    x = [q (3), w (3), eta (1)]
    q   = (p - p_o) / l   posicion relativa del objetivo, en tamanos de objetivo
    w   = v / l           velocidad del objetivo, en tamanos de objetivo por segundo
    eta = ln(l)           logaritmo del tamano (l en metros)

con p, v la posicion y velocidad del centro del objetivo, p_o la posicion de la camara
y l el tamano (diametro), DESCONOCIDO.

Por que asi y no con [p, v, l]:

- q sale ENTERA de una sola imagen: direccion = g, modulo = 1 / (2 sin(theta/2)). Las
  medidas solo dependen de q, asi que su linealizacion es buena desde el principio.
- La escala entra solo por la dinamica: q' = w - v_o / l, con v_o la velocidad
  (conocida, en metros) de la camara. Si el observador va a velocidad constante,
  v_o / l es indistinguible de un cambio de w, y el filtro deja l en su prior con su
  incertidumbre intacta, en vez de inventarse una escala.
- ln(l) y no l ni 1/l: el tamano es positivo por construccion. Con 1/l (que hace la
  dinamica exactamente lineal) la estimacion cruzaba cero al empezar a maniobrar y el
  tamano se disparaba durante ~1 s en simulacion.
- Probadas antes: la forma pseudo-lineal del paper colapsa la escala a cero con
  ruido (la solucion trivial p = p_o, l = 0 cumple sus ecuaciones), y un EKF en
  [p, v, l] declara una escala que no es observable (sigma de l 5 veces menor que su
  error real con el observador quieto).

Observabilidad: la escala solo es observable si el observador tiene un movimiento de
orden mayor que el objetivo. Con el objetivo a velocidad constante, el observador tiene
que ACELERAR. Sin eso, q y w (direccion, tamano angular, tiempo hasta impacto) si se
estiman bien, pero p, v y l no.

Robustez (Dynamic Bearing-Angle, Sensors 2025): pesos de Huber sobre la innovacion y
rechazo de medidas atipicas por distancia de Mahalanobis.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

# Indices del vector de estado.
Q = slice(0, 3)
W = slice(3, 6)
ETA = 6
STATE_DIM = 7


@dataclass
class FilterParams:
    """Parametros del filtro. Unidades SI; angulos en radianes."""

    # Ruido de proceso: densidad espectral de la aceleracion del objetivo [m/s^2/sqrt(Hz)].
    sigma_accel: float = 0.3
    # Deriva relativa del tamano [1/sqrt(s)]; pequena, solo para errores de modelo.
    sigma_log_size_rate: float = 1e-3
    # Ruido del bearing [rad] (jitter del centro de la caja / focal).
    sigma_bearing: float = 0.005
    # Ruido del angulo subtendido [rad] (jitter del ancho de la caja / focal).
    sigma_angle: float = 0.004
    # Prior del tamano [m]: lo que se supone antes de ver nada. NO es el tamano real.
    size_prior: float = 1.0
    # Incertidumbre relativa del prior de tamano (0.8 = +-80 %).
    size_prior_rel_std: float = 0.8
    # Prior de la velocidad del objetivo [m/s].
    velocity_prior_std: float = 5.0
    # Umbral de Huber sobre la innovacion normalizada (sqrt de Mahalanobis).
    huber_k: float = 2.0
    # Rechazo de un bloque de medida si la Mahalanobis^2 lo supera (0 = sin rechazo).
    gate: float = 30.0
    # Actualizaciones iniciales en las que no se rechaza nada.
    warmup_updates: int = 10
    # Tamanos admisibles [m]: salvaguarda numerica, no deberian alcanzarse.
    min_size: float = 0.01
    max_size: float = 100.0


@dataclass
class UpdateInfo:
    """Resultado de una actualizacion, para diagnostico."""

    bearing_d2: float
    angle_d2: float
    bearing_accepted: bool
    angle_accepted: bool


def perpendicular_basis(g: np.ndarray) -> np.ndarray:
    """Devuelve E (2x3): dos vectores ortonormales perpendiculares al unitario g."""
    # Se parte del eje menos alineado con g para que el producto vectorial no degenere.
    axis = np.eye(3)[int(np.argmin(np.abs(g)))]
    e1 = np.cross(g, axis)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(g, e1)
    return np.vstack((e1, e2))


class BearingAngleFilter:
    """EKF con estado normalizado [q, w, ln l]; ver el docstring del modulo."""

    def __init__(self, params: Optional[FilterParams] = None) -> None:
        """Crea el filtro sin inicializar; se inicializa con la primera medida."""
        self.params = params if params is not None else FilterParams()
        self.x = np.zeros(STATE_DIM)
        self.P = np.eye(STATE_DIM)
        self.p_o = np.zeros(3)
        self.initialized = False
        self.update_count = 0
        # (eta, var_eta) que se conserva tras un reset(keep_size=True).
        self._kept_size = None

    def reset(self, keep_size: bool = False) -> None:
        """
        Olvida el estado; la proxima medida vuelve a inicializar.

        Con keep_size, conserva el tamano estimado y su incertidumbre: sirve cuando se
        pierde el objetivo de vista, porque su tamano no cambia aunque no se le vea.
        """
        self._kept_size = (
            (float(self.x[ETA]), float(self.P[ETA, ETA]))
            if keep_size and self.initialized else None)
        self.initialized = False
        self.update_count = 0

    def initialize(self, p_o: np.ndarray, g: np.ndarray, theta: float) -> None:
        """Inicializa con la primera medida (requiere theta > 0)."""
        prm = self.params
        g = _unit(g)
        s = 2.0 * math.sin(theta / 2.0)
        if s <= 0.0:
            raise ValueError('theta must be positive to initialize.')
        rho = 1.0 / s
        if self._kept_size is not None:
            eta0, var_eta0 = self._kept_size
        else:
            eta0 = math.log(prm.size_prior)
            # Varianza lognormal equivalente a la incertidumbre relativa pedida.
            var_eta0 = math.log(1.0 + prm.size_prior_rel_std ** 2)
        self._kept_size = None
        lam0 = math.exp(-eta0)

        self.x = np.zeros(STATE_DIM)
        self.x[Q] = rho * g
        self.x[ETA] = eta0

        # Incertidumbre de q: lateral por el bearing, radial por el angulo.
        var_rho = (rho * rho * math.cos(theta / 2.0) / 2.0 * prm.sigma_angle) ** 2 * 4.0
        perp = np.eye(3) - np.outer(g, g)
        self.P = np.zeros((STATE_DIM, STATE_DIM))
        self.P[Q, Q] = (rho * prm.sigma_bearing) ** 2 * perp + var_rho * np.outer(g, g)
        self.P[W, W] = (prm.velocity_prior_std * lam0) ** 2 * np.eye(3)
        self.P[ETA, ETA] = var_eta0

        self.p_o = np.asarray(p_o, dtype=float).copy()
        self.initialized = True
        self.update_count = 0

    def predict(self, dt: float, p_o: np.ndarray) -> None:
        """
        Propaga dt segundos hasta que la camara esta en p_o.

        q(k+1) = q + w dt - (p_o(k+1) - p_o(k)) / l: el desplazamiento de la camara,
        conocido en metros, es lo unico que lleva informacion de escala.
        """
        if not self.initialized:
            return
        p_o = np.asarray(p_o, dtype=float)
        dp_o = p_o - self.p_o
        self.p_o = p_o.copy()
        if dt <= 0.0 and not dp_o.any():
            return
        dt = max(dt, 0.0)
        prm = self.params

        lam = math.exp(-self.x[ETA])
        f = np.eye(STATE_DIM)
        f[Q, W] = dt * np.eye(3)
        f[Q, ETA] = lam * dp_o          # d(-dp_o e^-eta)/d eta
        self.x[Q] = self.x[Q] + dt * self.x[W] - lam * dp_o

        # Aceleracion del objetivo a: w' = a / l, asi que su ruido escala con 1/l.
        qa = (prm.sigma_accel * lam) ** 2
        q = np.zeros((STATE_DIM, STATE_DIM))
        q[Q, Q] = qa * dt ** 3 / 3.0 * np.eye(3)
        q[Q, W] = qa * dt ** 2 / 2.0 * np.eye(3)
        q[W, Q] = qa * dt ** 2 / 2.0 * np.eye(3)
        q[W, W] = qa * dt * np.eye(3)
        q[ETA, ETA] = prm.sigma_log_size_rate ** 2 * dt
        self.P = f @ self.P @ f.T + q
        self._clamp_size()

    def update(self, p_o: np.ndarray, g: np.ndarray, theta: float) -> UpdateInfo:
        """
        Corrige con una medida.

        g es el bearing unitario en el mundo y theta el angulo subtendido (<= 0 si no
        hay angulo). p_o es la posicion de la camara en el instante de la medida, y
        tiene que coincidir con la de la ultima predict().

        Si el filtro no esta inicializado, lo inicializa (requiere theta > 0).
        """
        g = _unit(g)
        if not self.initialized:
            self.initialize(p_o, g, theta)
            return UpdateInfo(0.0, 0.0, True, theta > 0.0)

        prm = self.params

        # Bloque de bearing: la componente de u = q/|q| perpendicular al g medido es 0.
        q, rho, u = self._q_polar()
        e = perpendicular_basis(g)
        h_b = np.zeros((2, STATE_DIM))
        h_b[:, Q] = e @ ((np.eye(3) - np.outer(u, u)) / rho)
        y_b = -(e @ u)
        r_b = prm.sigma_bearing ** 2 * np.eye(2)
        d2_b, ok_b = self._robust_update(h_b, y_b, r_b)

        d2_a, ok_a = 0.0, False
        if theta > 0.0:
            # Bloque de angulo: theta = 2 asin(1 / (2 |q|)).
            q, rho, u = self._q_polar()
            ratio = min(1.0 / (2.0 * rho), 0.99)
            c = 1.0 / math.sqrt(1.0 - ratio * ratio)
            h_a = np.zeros((1, STATE_DIM))
            h_a[0, Q] = -c / (rho * rho) * u
            y_a = np.array([theta - 2.0 * math.asin(ratio)])
            r_a = np.array([[prm.sigma_angle ** 2]])
            d2_a, ok_a = self._robust_update(h_a, y_a, r_a)

        self._clamp_size()
        self.update_count += 1
        return UpdateInfo(d2_b, d2_a, ok_b, ok_a)

    def _q_polar(self):
        """Devuelve q, su modulo (acotado para no dividir por cero) y su direccion."""
        q = self.x[Q]
        rho = max(float(np.linalg.norm(q)), 0.6)
        return q, rho, q / rho

    def _clamp_size(self) -> None:
        """Salvaguarda numerica: mantiene el tamano en [min_size, max_size]."""
        prm = self.params
        self.x[ETA] = min(max(self.x[ETA], math.log(prm.min_size)), math.log(prm.max_size))

    def _robust_update(self, h: np.ndarray, y: np.ndarray, r: np.ndarray):
        """Actualizacion de Kalman (innovacion y) con peso de Huber y rechazo."""
        prm = self.params
        s = h @ self.P @ h.T + r
        d2 = float(y @ np.linalg.solve(s, y))

        if prm.gate > 0.0 and self.update_count >= prm.warmup_updates and d2 > prm.gate:
            return d2, False

        # Huber: por encima de k, la medida pesa como si su ruido fuera d/k veces mayor.
        d = math.sqrt(max(d2, 0.0))
        if d > prm.huber_k:
            r = r * (d / prm.huber_k)
            s = h @ self.P @ h.T + r

        k = np.linalg.solve(s, h @ self.P).T
        self.x = self.x + k @ y
        # Forma de Joseph: mantiene P simetrica y definida positiva.
        i_kh = np.eye(STATE_DIM) - k @ h
        self.P = i_kh @ self.P @ i_kh.T + k @ r @ k.T
        return d2, True

    # --- Salidas en unidades fisicas -------------------------------------------------

    @property
    def size(self) -> float:
        """Tamano (diametro) estimado del objetivo [m]."""
        return math.exp(self.x[ETA])

    @property
    def position(self) -> np.ndarray:
        """Posicion estimada del centro del objetivo en el mundo [m]."""
        return self.p_o + self.x[Q] * self.size

    @property
    def velocity(self) -> np.ndarray:
        """Velocidad estimada del objetivo en el mundo [m/s]."""
        return self.x[W] * self.size

    @property
    def distance(self) -> float:
        """Distancia estimada de la camara al centro del objetivo [m]."""
        return float(np.linalg.norm(self.x[Q])) * self.size

    @property
    def angular_range(self) -> float:
        """|q|: distancia en tamanos de objetivo. Observable siempre, sin escala."""
        return float(np.linalg.norm(self.x[Q]))

    def physical_covariance(self) -> np.ndarray:
        """Covarianza 7x7 de [p (3), v (3), l], propagada al primer orden."""
        size = self.size
        j = np.zeros((STATE_DIM, STATE_DIM))
        j[0:3, Q] = size * np.eye(3)
        j[0:3, ETA] = size * self.x[Q]
        j[3:6, W] = size * np.eye(3)
        j[3:6, ETA] = size * self.x[W]
        j[6, ETA] = size
        return j @ self.P @ j.T


def _unit(v: np.ndarray) -> np.ndarray:
    """Normaliza v; falla si es nulo."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n == 0.0:
        raise ValueError('Zero-length vector.')
    return v / n
