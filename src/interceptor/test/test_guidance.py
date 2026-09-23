"""Tests del guiado al checkpoint, incluido un bucle cerrado sintetico con el filtro real."""

import math

from interceptor.bearing_angle_filter import BearingAngleFilter, FilterParams
from interceptor.guidance import approach_speed, excitation_gain, guidance_velocity
from interceptor.guidance import GuidanceParams
from interceptor.guidance import intercept_time, should_freeze, time_to_closest_approach
import numpy as np
import pytest


def test_intercept_time_static_target():
    """Checkpoint quieto a 30 m y velocidad 3 m/s: 10 s."""
    assert intercept_time(np.array([30.0, 0.0, 0.0]), np.zeros(3), 3.0) == pytest.approx(10.0)


def test_intercept_time_crossing_target():
    """Cruce perpendicular: |r + v t| = V t con r = (20, 0), v = (0, 1), V = 3."""
    t = intercept_time(np.array([20.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), 3.0)
    assert math.hypot(20.0, t) == pytest.approx(3.0 * t)


def test_intercept_time_unreachable():
    """Si el checkpoint se aleja mas rapido de lo que vuela el interceptor, no hay encuentro."""
    assert intercept_time(np.array([10.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]), 3.0) is None


def test_excitation_gain_limits():
    """Plena con mucha incertidumbre, nula cuando ya se conoce y en el tramo final."""
    prm = GuidanceParams()
    assert excitation_gain(0.5, 10.0, prm) == 1.0
    assert excitation_gain(0.01, 10.0, prm) == 0.0
    assert excitation_gain(0.5, prm.terminal_time - 0.1, prm) == 0.0


def test_guidance_points_to_intercept_point():
    """Sin excitacion, la orden va hacia r + v t_go con modulo speed."""
    prm = GuidanceParams()
    r, v = np.array([20.0, -15.0, -2.5]), np.array([0.0, 1.0, 0.0])
    v_cmd, t_go, gain = guidance_velocity(r, v, 0.0, 0.0, prm)
    aim = r + v * t_go
    assert gain == 0.0
    assert np.linalg.norm(v_cmd) == pytest.approx(prm.speed)
    assert v_cmd / prm.speed == pytest.approx(aim / np.linalg.norm(aim))


def fly(seed, excitation_amp, blind_range=3.0, rate=11.0):
    """
    Simula el bucle cerrado y devuelve (distancia minima, tamano estimado).

    Camara con ruido, filtro, guiado y un interceptor con respuesta de primer orden
    (0.6 s, 4 m/s^2). Por debajo de blind_range deja de ver el checkpoint y sigue con la
    ultima orden.
    """
    rng = np.random.default_rng(seed)
    filt = BearingAngleFilter(FilterParams(size_prior=0.5))
    prm = GuidanceParams(excitation_amp=excitation_amp)
    dt = 1.0 / rate
    p_i, v_i, v_cmd = np.array([0.0, 0.0, -10.0]), np.zeros(3), np.zeros(3)
    p_t, v_t = np.array([20.0, -15.0, -12.5]), np.array([0.0, 1.0, 0.0])
    min_dist, seen = math.inf, True
    for k in range(int(25.0 * rate)):
        rel = p_t - p_i
        dist = float(np.linalg.norm(rel))
        min_dist = min(min_dist, dist)
        seen = seen and dist > blind_range
        if seen:
            g = rel / dist + rng.normal(0.0, 0.003, 3)
            theta = 2.0 * math.asin(0.5 / dist) + rng.normal(0.0, 0.002)
            filt.predict(dt if k else 0.0, p_i)
            filt.update(p_i, g, theta)
            rel_sigma = math.sqrt(filt.physical_covariance()[6, 6]) / filt.size
            v_cmd, _, _ = guidance_velocity(filt.position - p_i, filt.velocity, rel_sigma,
                                            k * dt, prm)
        a = (v_cmd - v_i) / 0.6
        if np.linalg.norm(a) > 4.0:
            a *= 4.0 / np.linalg.norm(a)
        v_i = v_i + a * dt
        p_i = p_i + v_i * dt
        p_t = p_t + v_t * dt
    return min_dist, filt.size


def test_closed_loop_passes_through_checkpoint():
    """Con excitacion: pasa a menos de 0.3 m del centro y el tamano converge (real 1.0)."""
    for seed in range(5):
        min_dist, size = fly(seed, excitation_amp=2.0)
        assert min_dist < 0.3
        assert size == pytest.approx(1.0, abs=0.15)


def test_hitting_does_not_need_scale():
    """
    Sin excitacion tambien acierta, aunque la escala no se conozca.

    La condicion de colision (bearing constante) no depende de la escala: la escala
    sirve para saber CUANDO se pasa, no para acertar. Si este test deja de cumplirse,
    el guiado ha empezado a depender de la escala.
    """
    for seed in range(5):
        min_dist, _ = fly(seed, excitation_amp=0.0)
        assert min_dist < 0.3


def test_time_to_closest_approach_head_on():
    """A 12 m acercandose a 3 m/s: 4 s. Si se aleja, infinito."""
    r = np.array([12.0, 0.0, 0.0])
    assert time_to_closest_approach(r, np.array([-3.0, 0.0, 0.0])) == pytest.approx(4.0)
    assert time_to_closest_approach(r, np.array([3.0, 0.0, 0.0])) == math.inf


def test_time_to_closest_approach_is_scale_free():
    """
    No cambia si el tamano esta mal estimado.

    Con un factor de escala k, el filtro da r y v_rel multiplicados por k (v_rel = l q').
    intercept_time, que mezcla la velocidad del interceptor en metros, si cambia.
    """
    r, v_t, v_own = np.array([3.0, 14.0, 2.5]), np.array([1.0, 0.0, 0.0]), np.array([0, 3.0, 0])
    v_rel = v_t - v_own
    t_true = time_to_closest_approach(r, v_rel)
    for k in (0.4, 2.0):
        assert time_to_closest_approach(k * r, k * v_rel) == pytest.approx(t_true)
    # El t_go del punto de encuentro si cambia con la escala.
    for k in (0.5, 2.0):
        t_k = intercept_time(k * r, k * v_rel + v_own, 3.0)
        assert abs(t_k - intercept_time(r, v_t, 3.0)) > 0.4


def test_should_freeze_needs_close_checkpoint():
    """Un t_go corto por ruido con el checkpoint lejos (12 tamanos) no congela."""
    prm = GuidanceParams()
    assert should_freeze(0.3, 2.0, 0.5, prm)
    assert not should_freeze(0.3, 12.0, 0.5, prm)
    assert not should_freeze(2.0, 2.0, 0.5, prm)


def test_approach_speed_slows_down_near_the_pass():
    """Lejos va a speed; al llegar el paso, a terminal_speed; en medio, intermedio."""
    prm = GuidanceParams()
    assert approach_speed(math.inf, 0.0, prm) == prm.speed
    assert approach_speed(prm.slowdown_time + 1.0, 0.0, prm) == prm.speed
    assert approach_speed(0.0, 0.0, prm) == pytest.approx(prm.terminal_speed)
    medio = approach_speed(prm.slowdown_time / 2, 0.0, prm)
    assert prm.terminal_speed < medio < prm.speed


def test_guidance_velocity_uses_the_slower_speed_at_the_end():
    """Con el paso inminente frena: a terminal_speed si el checkpoint esta quieto."""
    prm = GuidanceParams()
    r = np.array([3.0, 0.0, 0.0])
    v_cmd, _, _ = guidance_velocity(r, np.zeros(3), 0.0, 0.0, prm, time_to_pass=0.0)
    assert np.linalg.norm(v_cmd) == pytest.approx(prm.terminal_speed)
    # Con el checkpoint estimado en movimiento, guarda el margen sobre su velocidad.
    v_cmd, _, _ = guidance_velocity(r, np.array([0.0, 1.0, 0.0]), 0.0, 0.0, prm, time_to_pass=0.0)
    assert np.linalg.norm(v_cmd) == pytest.approx(prm.terminal_speed_margin * 1.0)


def test_approach_speed_keeps_margin_over_the_estimated_target_speed():
    """
    No frena por debajo del margen sobre la velocidad ESTIMADA del checkpoint.

    Con poco margen la geometria de intercepcion se vuelve muy sensible, y la velocidad
    real del checkpoint no se conoce en vuelo: solo se usa la que estima el filtro.
    """
    prm = GuidanceParams()
    rapido = 1.4                      # estimacion de a bordo, no verdad del simulador
    assert approach_speed(0.0, rapido, prm) == pytest.approx(prm.terminal_speed_margin * rapido)
    # Nunca por encima de la velocidad de crucero, aunque el checkpoint parezca muy rapido.
    assert approach_speed(0.0, 100.0, prm) == pytest.approx(prm.speed)
