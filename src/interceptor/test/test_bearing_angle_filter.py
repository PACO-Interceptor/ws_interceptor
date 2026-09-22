"""
Tests del filtro bearing-angle con escenarios sinteticos.

Comprueban la teoria de observabilidad, no solo que el codigo corre: con el objetivo
a velocidad constante, la escala (tamano, distancia) solo converge si el observador
ACELERA; si no, el filtro tiene que quedarse en el prior SIN declararse seguro.
"""

import math

from interceptor.bearing_angle_filter import BearingAngleFilter, FilterParams
import numpy as np
import pytest

RATE_HZ = 15.0
DURATION_S = 20.0
TRUE_SIZE = 0.6          # distinto del prior (1.0) a proposito
TARGET_P0 = np.array([-10.0, 20.0, 10.0])
TARGET_V = np.array([1.0, 0.0, 0.0])
SURGE_W = 2.0 * math.pi * 0.2


def observer_position(profile: str, t: float) -> np.ndarray:
    """Posicion de la camara en cada escenario."""
    if profile == 'hover':
        return np.array([0.0, 0.0, 10.0])
    if profile == 'constant':
        return np.array([0.0, 0.5 * t, 10.0])
    if profile == 'surge':  # velocidad 0.5 + 0.5 sin(w t) hacia delante
        return np.array([0.0, 0.5 * t + 0.5 / SURGE_W * (1.0 - math.cos(SURGE_W * t)), 10.0])
    if profile == 'weave':  # avance constante + oscilacion lateral
        return np.array([1.5 * math.sin(SURGE_W * t), 0.5 * t, 10.0])
    raise ValueError(profile)


def run(profile: str, seed: int = 0, outlier_rate: float = 0.0) -> BearingAngleFilter:
    """Simula un escenario con ruido de medida y devuelve el filtro al final."""
    rng = np.random.default_rng(seed)
    filt = BearingAngleFilter(FilterParams())
    dt = 1.0 / RATE_HZ
    for k in range(int(DURATION_S * RATE_HZ)):
        t = k * dt
        p_t = TARGET_P0 + TARGET_V * t
        p_o = observer_position(profile, t)
        rel = p_t - p_o
        r = np.linalg.norm(rel)
        g = rel / r + rng.normal(0.0, 0.005, 3)
        theta = 2.0 * math.asin(TRUE_SIZE / 2.0 / r) + rng.normal(0.0, 0.004)
        if outlier_rate and rng.random() < outlier_rate:
            theta *= 3.0
        filt.predict(dt if k else 0.0, p_o)
        filt.update(p_o, g, theta)
    return filt


def final_target_position() -> np.ndarray:
    """Posicion real del objetivo al final de la simulacion."""
    return TARGET_P0 + TARGET_V * (DURATION_S - 1.0 / RATE_HZ)


def size_sigma(filt: BearingAngleFilter) -> float:
    """Desviacion tipica que el filtro declara para el tamano."""
    return math.sqrt(filt.physical_covariance()[6, 6])


@pytest.mark.parametrize('profile', ['hover', 'constant'])
def test_scale_unobservable_without_observer_acceleration(profile):
    """Sin aceleracion del observador, el tamano se queda en el prior y lo reconoce."""
    filt = run(profile)
    assert filt.size == pytest.approx(1.0, abs=0.05)
    # No se declara seguro: la sigma sigue siendo la del prior.
    assert size_sigma(filt) > 0.5


@pytest.mark.parametrize('profile', ['surge', 'weave'])
def test_scale_converges_with_observer_acceleration(profile):
    """Acelerando (en la linea de vision o en lateral), converge tamano y posicion."""
    errors = []
    for seed in range(5):
        filt = run(profile, seed)
        errors.append(abs(filt.size - TRUE_SIZE))
        assert np.linalg.norm(filt.position - final_target_position()) < 4.0
        assert np.linalg.norm(filt.velocity - TARGET_V) < 0.6
    assert np.mean(errors) < 0.1


def test_declared_uncertainty_is_consistent():
    """El error real del tamano no supera 3 sigmas de lo que declara el filtro."""
    for profile in ('hover', 'surge', 'weave'):
        for seed in range(5):
            filt = run(profile, seed)
            assert abs(filt.size - TRUE_SIZE) < 3.0 * size_sigma(filt)


def test_direction_and_angular_range_observable_while_hovering():
    """Aun sin escala, |q| (distancia en tamanos de objetivo) es observable."""
    filt = run('hover')
    true_rho = np.linalg.norm(final_target_position() - observer_position('hover', 0.0))
    true_rho /= TRUE_SIZE
    assert filt.angular_range == pytest.approx(true_rho, rel=0.1)


def test_outliers_do_not_break_convergence():
    """Un 5 % de angulos atipicos (x3) no impide converger."""
    filt = run('weave', outlier_rate=0.05)
    assert abs(filt.size - TRUE_SIZE) < 0.15


def test_initialization_uses_size_prior():
    """La primera medida coloca el objetivo a la distancia que implica el prior."""
    filt = BearingAngleFilter(FilterParams(size_prior=2.0))
    theta = 2.0 * math.asin(0.5 / 10.0)  # esfera de 1 m a 10 m
    filt.update(np.zeros(3), np.array([1.0, 0.0, 0.0]), theta)
    assert filt.size == pytest.approx(2.0)
    assert filt.distance == pytest.approx(20.0, rel=1e-6)
    assert filt.position[1] == pytest.approx(0.0)


def test_reset_keep_size_reuses_estimated_size():
    """Tras perder el objetivo, se conserva el tamano ya estimado, no el prior."""
    filt = run('weave')
    size, var = filt.size, filt.physical_covariance()[6, 6]
    filt.reset(keep_size=True)
    theta = 2.0 * math.asin(TRUE_SIZE / 2.0 / 15.0)
    filt.update(np.zeros(3), np.array([0.0, 1.0, 0.0]), theta)
    assert filt.size == pytest.approx(size)
    assert filt.physical_covariance()[6, 6] == pytest.approx(var, rel=1e-6)
    assert filt.distance == pytest.approx(15.0 * size / TRUE_SIZE, rel=0.01)


def test_plain_reset_goes_back_to_prior():
    """Un reset normal vuelve al prior de tamano."""
    filt = run('weave')
    filt.reset()
    filt.update(np.zeros(3), np.array([0.0, 1.0, 0.0]), 0.05)
    assert filt.size == pytest.approx(FilterParams().size_prior)


def test_no_size_blowup_when_maneuver_starts():
    """
    Al empezar a maniobrar desde parado, el tamano no se dispara.

    Reproduce el transitorio visto en simulacion con el estado en 1/l: prior 0.5 m,
    real 1.0 m, objetivo a 25 m y oscilacion lateral de 2.5 m/s desde el reposo.
    """
    rng = np.random.default_rng(1)
    filt = BearingAngleFilter(FilterParams(size_prior=0.5))
    rate, w = 11.0, 2.0 * math.pi * 0.2
    target_p0, target_v = np.array([-15.0, 25.0, 10.0]), np.array([1.0, 0.0, 0.0])
    max_size = 0.0
    for k in range(int(10 * rate)):
        t = k / rate
        p_o = np.array([2.5 / w * (1.0 - math.cos(w * t)), 0.0, 10.0])
        rel = target_p0 + target_v * t - p_o
        r = np.linalg.norm(rel)
        g = rel / r + rng.normal(0.0, 0.003, 3)
        theta = 2.0 * math.asin(0.5 / r) + rng.normal(0.0, 0.002)
        filt.predict(1.0 / rate if k else 0.0, p_o)
        filt.update(p_o, g, theta)
        max_size = max(max_size, filt.size)
    assert max_size < 3.0
    assert filt.size == pytest.approx(1.0, abs=0.25)


def test_larger_bearing_sigma_moves_estimate_less():
    """Un bearing con sigma grande (actitud poco fiable) corrige menos el estado."""
    moves = []
    for sigma in (None, 0.2):
        filt = BearingAngleFilter(FilterParams())
        filt.update(np.zeros(3), np.array([1.0, 0.0, 0.0]), 0.1)
        before = filt.position
        filt.predict(0.1, np.zeros(3))
        filt.update(np.zeros(3), np.array([1.0, 0.2, 0.0]), 0.1, sigma)
        moves.append(np.linalg.norm(filt.position - before))
    assert moves[1] < 0.5 * moves[0]
