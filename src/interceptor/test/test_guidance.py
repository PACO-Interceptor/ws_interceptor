"""Tests del guiado al checkpoint, incluido un bucle cerrado sintetico con el filtro real."""

import math

from interceptor.bearing_angle_filter import BearingAngleFilter, FilterParams
from interceptor.guidance import approach_speed, base_speed, can_hold_distance
from interceptor.guidance import CommitGate, contact_rate, contact_time_bounds, cruise_speed
from interceptor.guidance import excitation_gain
from interceptor.guidance import gated_velocity, GuidanceParams, hold_off
from interceptor.guidance import guidance_velocity, level_velocity
from interceptor.guidance import intercept_time, should_freeze, time_to_closest_approach
from interceptor.guidance import time_to_contact
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
    speed = cruise_speed(r, v, 0.0, prm)
    assert np.linalg.norm(v_cmd) == pytest.approx(speed)
    assert v_cmd / speed == pytest.approx(aim / np.linalg.norm(aim))


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


def test_cruise_speed_only_rises_when_needed_to_catch_the_checkpoint():
    """
    Crucero: sube solo si hace falta para alcanzar el checkpoint.

    speed si el checkpoint cruza o viene de frente; mas si se aleja o si no hay punto de
    encuentro; nunca por encima de max_speed. Con 3 m/s fijos, un checkpoint a 2 m/s que
    se aleja apenas se deja alcanzar (Gazebo). Solo se usa la velocidad ESTIMADA.
    """
    prm = GuidanceParams()
    r = np.array([10.0, 0.0, 0.0])
    assert cruise_speed(r, np.array([0.0, 2.0, 0.0]), 0.0, prm) == prm.speed     # cruza
    assert cruise_speed(r, np.array([-2.0, 0.0, 0.0]), 0.0, prm) == prm.speed    # de frente
    away = cruise_speed(r, np.array([2.0, 0.0, 0.0]), 0.0, prm)
    assert away == pytest.approx(prm.terminal_speed_margin * 2.0)
    fast_crossing = np.array([0.0, 5.0, 0.0])
    assert intercept_time(r, fast_crossing, prm.speed) is None
    assert cruise_speed(r, fast_crossing, 0.0, prm) == pytest.approx(
        prm.terminal_speed_margin * 5.0)
    assert cruise_speed(r, np.array([50.0, 0.0, 0.0]), 0.0, prm) == prm.max_speed
    v_cmd, _, _ = guidance_velocity(r, np.array([2.0, 0.0, 0.0]), 0.0, 0.0, prm)
    assert np.linalg.norm(v_cmd) == pytest.approx(away)


def test_cruise_speed_uses_the_low_bound_of_the_estimated_speed():
    """
    Con la escala sin conocer no acelera por una velocidad que puede estar inflada.

    La velocidad estimada escala con el tamano estimado: se usa v (1 - sigma relativa).
    """
    prm = GuidanceParams()
    r, away = np.array([10.0, 0.0, 0.0]), np.array([4.0, 0.0, 0.0])
    assert cruise_speed(r, away, 0.7, prm) == prm.speed
    assert cruise_speed(r, away, 0.1, prm) == pytest.approx(
        prm.terminal_speed_margin * 4.0 * 0.9)


def test_cruise_speed_is_slow_only_when_attacking_a_slow_checkpoint():
    """Al atacar un checkpoint quieto, velocidad terminal; observando, speed."""
    prm = GuidanceParams()
    r = np.array([6.0, 0.0, 0.0])
    assert cruise_speed(r, np.zeros(3), 0.1, prm, slow_attack=True) == prm.terminal_speed
    assert cruise_speed(r, np.zeros(3), 0.1, prm) == prm.speed


def test_time_to_contact_is_infinite_when_the_checkpoint_passes_by():
    """
    De frente: distancia entre cierre. Cruzando lejos por delante: infinito.

    El tiempo hasta la maxima aproximacion, en cambio, llega a cero al cruzar, aunque
    el checkpoint pase a 20 m: en Gazebo apagaba la excitacion desde el principio.
    """
    r = np.array([12.0, 0.0, 0.0])
    assert time_to_contact(r, np.array([-3.0, 0.0, 0.0])) == pytest.approx(4.0)
    abeam, crossing = np.array([20.0, 0.0, 0.0]), np.array([0.0, 2.0, 0.0])
    assert time_to_contact(abeam, crossing) == math.inf
    assert time_to_closest_approach(abeam, crossing) == math.inf
    just_before = np.array([20.0, -0.5, 0.0])
    assert time_to_closest_approach(just_before, crossing) < 0.5
    assert time_to_contact(just_before, crossing) > 100.0


def test_hold_off_caps_closing_and_keeps_the_lateral_part():
    """Observando no avanza hacia el checkpoint a mas de observe_speed; lo lateral queda."""
    prm = GuidanceParams()
    r, v_cmd = np.array([30.0, 0.0, 0.0]), np.array([3.0, 1.5, 0.0])
    out = hold_off(v_cmd, r, np.zeros(3), np.zeros(3), 10.0, prm)
    assert out[0] == pytest.approx(prm.observe_speed)
    assert out[1] == pytest.approx(1.5)


def test_hold_off_lets_it_approach_until_the_checkpoint_is_seen_well():
    """
    Lejos (mas de observe_max_angular_range tamanos) no limita a observe_speed.

    Desde ahi el detector apenas ve la pelota: en Gazebo el interceptor se quedo
    observando a 30 m, perdio el checkpoint cada pocos segundos y no llego a comprometerse.
    """
    prm = GuidanceParams()
    r, v_cmd = np.array([30.0, 0.0, 0.0]), np.array([3.0, 0.0, 0.0])
    far = prm.observe_max_angular_range + 50.0
    out = hold_off(v_cmd, r, np.zeros(3), np.zeros(3), far, prm)
    assert out == pytest.approx(v_cmd)


def test_hold_off_backs_away_when_contact_is_too_close():
    """Con el contacto antes de standoff_time, pide cerrar mas despacio que ahora."""
    prm = GuidanceParams()
    r, v_t, v_own = np.array([10.0, 0.0, 0.0]), np.array([-2.0, 0.0, 0.0]), np.zeros(3)
    assert time_to_contact(r, v_t - v_own) < prm.standoff_time
    out = hold_off(np.array([3.0, 0.0, 0.0]), r, v_t, v_own, 100.0, prm)
    assert out[0] < 0.0
    assert out[0] >= -prm.speed


def test_hold_off_retreats_at_full_speed_inside_terminal_time():
    """Con el contacto antes de terminal_time retrocede a la velocidad de crucero."""
    prm = GuidanceParams()
    r, v_t, v_own = np.array([5.0, 0.0, 0.0]), np.array([-2.0, 0.0, 0.0]), np.zeros(3)
    assert time_to_contact(r, v_t - v_own) < prm.terminal_time
    out = hold_off(np.array([3.0, 1.0, 0.0]), r, v_t, v_own, 20.0, prm)
    assert out[0] == pytest.approx(-prm.speed)
    assert out[1] == pytest.approx(1.0)


def test_hold_off_equilibrium_does_not_depend_on_scale():
    """
    La retencion solo usa el tiempo hasta el contacto y la velocidad propia.

    Con el tamano mal estimado por k, r y v_rel salen por k y la orden no cambia.
    """
    prm = GuidanceParams()
    r, v_t, v_own = np.array([12.0, 3.0, 0.0]), np.array([-1.0, 0.5, 0.0]), np.array([1.0, 0, 0])
    v_cmd = np.array([3.0, 0.0, 0.0])
    ref = hold_off(v_cmd, r, v_t, v_own, 10.0, prm)
    for k in (0.4, 2.5):
        v_t_k = k * (v_t - v_own) + v_own      # v_rel escala con k; v_own es de PX4
        assert hold_off(v_cmd, k * r, v_t_k, v_own, 10.0, prm) == pytest.approx(ref)


def test_commit_gate_waits_for_the_size_sigma():
    """Con la sigma alta sigue observando; por debajo del umbral se compromete y no vuelve."""
    prm = GuidanceParams()
    gate = CommitGate(prm)
    assert not gate.update(0.58, 20.0, 0.1)
    assert not gate.committed
    assert gate.update(prm.sigma_rel_commit - 0.01, 20.0, 0.1)
    assert gate.committed and not gate.forced
    assert not gate.update(0.9, 20.0, 0.1)
    assert gate.committed


def test_commit_gate_forces_when_it_cannot_keep_the_distance():
    """Contacto por debajo de terminal_time durante forced_commit_hold: compromiso forzado."""
    prm = GuidanceParams()
    gate = CommitGate(prm)
    short = prm.terminal_time - 1.0
    assert not gate.update(0.5, short, prm.forced_commit_hold / 2)
    assert not gate.update(0.5, 20.0, prm.forced_commit_hold / 2)     # se recupera
    assert not gate.update(0.5, short, prm.forced_commit_hold / 2)
    assert gate.update(0.5, short, prm.forced_commit_hold / 2 + 0.01)
    assert gate.forced


def test_commit_gate_does_not_force_while_it_can_back_off():
    """Si retrocediendo aun puede mantener la distancia, un contacto cercano no fuerza."""
    prm = GuidanceParams()
    gate = CommitGate(prm)
    short = prm.terminal_time - 1.0
    for _ in range(20):
        assert not gate.update(0.5, short, prm.forced_commit_hold, can_hold=True)
    assert gate.update(0.5, short, prm.forced_commit_hold + 0.01, can_hold=False)
    assert gate.forced


def test_can_hold_distance_needs_a_slower_target_and_time_to_reverse():
    """Retroceder vale si el checkpoint se acerca mas despacio que speed y queda tiempo."""
    prm = GuidanceParams()
    r = np.array([6.0, 0.0, 0.0])
    slow = np.array([-2.0, 0.0, 0.0])
    fast = np.array([-2.0 * prm.speed, 0.0, 0.0])
    assert can_hold_distance(r, slow, 0.0, 3.0, prm)
    assert not can_hold_distance(r, fast, 0.0, 3.0, prm)
    assert not can_hold_distance(r, slow, 0.0, 0.9 * prm.reverse_time, prm)
    # Con la escala sin conocer se toma la cota baja de la velocidad estimada.
    assert can_hold_distance(r, fast, 0.6, 3.0, prm)
    assert prm.reverse_time == pytest.approx(2.0 * prm.speed / prm.max_accel)


def test_contact_rate_sigma_matches_numerical_propagation():
    """La sigma de 1/tau coincide con el gradiente numerico y no depende de la escala."""
    rng = np.random.default_rng(3)
    r = np.array([5.0, 2.0, -1.0])
    v_rel = np.array([-2.0, -0.3, 0.2])
    a = rng.normal(size=(6, 6))
    cov = 0.05 * a @ a.T

    def rate(x):
        return -float(x[:3] @ x[3:]) / float(x[:3] @ x[:3])

    eps = 1e-6
    x0 = np.concatenate((r, v_rel))
    grad = np.array([(rate(x0 + eps * e) - rate(x0 - eps * e)) / (2 * eps) for e in np.eye(6)])
    k, sigma = contact_rate(r, v_rel, cov)
    assert k == pytest.approx(1.0 / time_to_contact(r, v_rel))
    assert sigma == pytest.approx(math.sqrt(grad @ cov @ grad), rel=1e-4)
    assert contact_rate(r, v_rel)[1] == 0.0
    # Incertidumbre solo de escala (r y v_rel juntos): no cuenta.
    x0 = x0.reshape(6, 1)
    assert contact_rate(r, v_rel, 0.3 * x0 @ x0.T)[1] == pytest.approx(0.0, abs=1e-6)


def test_contact_time_bounds_bracket_the_estimate():
    """Las cotas del tiempo hasta el contacto rodean al valor central."""
    r, v_rel = np.array([6.0, 0.0, 0.0]), np.array([-2.0, 0.0, 0.0])
    cov = np.diag([0.0, 0.0, 0.0, 0.25, 0.0, 0.0])                     # sigma 0.5 m/s
    low, high = contact_time_bounds(r, v_rel, cov)
    assert low == pytest.approx(6.0 / 2.5)
    assert high == pytest.approx(6.0 / 1.5)
    big = np.diag([0.0, 0.0, 0.0, 9.0, 0.0, 0.0])                       # puede abrirse
    assert contact_time_bounds(r, v_rel, big)[1] == math.inf


def test_commit_gate_forces_after_max_observe_time():
    """Si la sigma no baja nunca, se compromete igualmente al agotar max_observe_time."""
    prm = GuidanceParams()
    gate = CommitGate(prm)
    assert not gate.update(0.5, 20.0, prm.max_observe_time - 1.0)
    assert gate.update(0.5, 20.0, 1.0)
    assert gate.forced


def fly_gated(seed, v_t, p_t, size=0.3, prior=1.0, rate=11.0, duration=40.0):
    """
    Bucle cerrado con puerta de compromiso: devuelve (distancia minima, puerta).

    Como fly(), pero con el checkpoint pequeno y el prior de tamano 3 veces mas grande,
    y el guiado de gated_velocity. Deja de ver el checkpoint por debajo de 1 m.
    """
    rng = np.random.default_rng(seed)
    filt = BearingAngleFilter(FilterParams(size_prior=prior))
    prm = GuidanceParams()
    gate = CommitGate(prm)
    dt = 1.0 / rate
    p_i, v_i, v_cmd = np.array([0.0, 0.0, -10.0]), np.zeros(3), np.zeros(3)
    p_t, v_t = np.array(p_t, dtype=float), np.array(v_t, dtype=float)
    min_dist, seen = math.inf, True
    for k in range(int(duration * rate)):
        rel = p_t - p_i
        dist = float(np.linalg.norm(rel))
        min_dist = min(min_dist, dist)
        seen = seen and dist > 1.0
        if seen:
            g = rel / dist + rng.normal(0.0, 0.01, 3)
            theta = 2.0 * math.asin(0.5 * size / dist) + rng.normal(0.0, 0.002)
            filt.predict(dt if k else 0.0, p_i)
            filt.update(p_i, g, theta)
            rel_sigma = math.sqrt(filt.physical_covariance()[6, 6]) / filt.size
            v_cmd, _, _ = gated_velocity(gate, filt.position - p_i, filt.velocity, v_i,
                                         rel_sigma, filt.angular_range, k * dt, dt, prm)
        a = (v_cmd - v_i) / 0.6
        if np.linalg.norm(a) > 4.0:
            a *= 4.0 / np.linalg.norm(a)
        v_i = v_i + a * dt
        p_i = p_i + v_i * dt
        p_t = p_t + v_t * dt
    return min_dist, gate


def test_level_velocity_climbs_toward_a_checkpoint_above():
    """
    Con el checkpoint por encima sube, por debajo baja, y a 10 grados ya va al maximo.

    No depende del tamano estimado: r escalado da la misma orden.
    """
    prm = GuidanceParams()
    above = np.array([10.0, 0.0, -2.5])                  # NED: z negativo = mas alto
    vz = level_velocity(above, np.zeros(3), prm)
    assert vz == pytest.approx(-prm.max_vertical_speed)
    below = np.array([10.0, 0.0, 2.5])
    assert level_velocity(below, np.zeros(3), prm) == pytest.approx(prm.max_vertical_speed)
    small = np.array([10.0, 0.0, -10.0 * math.tan(math.radians(5.0))])
    assert level_velocity(small, np.zeros(3), prm) == pytest.approx(
        -prm.max_vertical_speed * 0.5, rel=1e-3)
    assert level_velocity(3.0 * small, np.zeros(3), prm) == pytest.approx(
        level_velocity(small, np.zeros(3), prm))
    assert level_velocity(np.array([10.0, 0.0, 0.0]), np.zeros(3), prm) == 0.0


def test_observing_uses_the_level_velocity():
    """Al observar, la componente vertical de la orden es la de level_velocity."""
    prm = GuidanceParams()
    r = np.array([12.0, 3.0, -2.0])
    v_cmd, _, _ = gated_velocity(CommitGate(prm), r, np.zeros(3), np.zeros(3), 0.7, 40.0, 0.3,
                                 0.1, prm)
    assert v_cmd[2] == pytest.approx(level_velocity(r, np.zeros(3), prm))


def test_base_speed_is_slow_for_a_slow_checkpoint():
    """Checkpoint quieto: velocidad terminal; rapido: speed; en medio, el margen."""
    prm = GuidanceParams()
    assert base_speed(np.zeros(3), 0.0, prm) == prm.terminal_speed
    assert base_speed(np.array([0.0, 1.0, 0.0]), 0.0, prm) == pytest.approx(
        prm.terminal_speed_margin * 1.0)
    assert base_speed(np.array([0.0, 4.0, 0.0]), 0.0, prm) == prm.speed
    # Cota alta: con la escala sin conocer no frena.
    assert base_speed(np.array([0.0, 1.0, 0.0]), 0.9, prm) == prm.speed


def test_hold_off_backs_away_from_a_close_crossing_checkpoint():
    """
    Retrocede si el checkpoint esta demasiado cerca aunque solo cruce.

    El tiempo hasta el contacto es infinito, pero esta mas cerca que base_speed *
    terminal_time.
    """
    prm = GuidanceParams()
    r, v_t = np.array([3.0, 0.0, 0.0]), np.array([0.0, 0.5, 0.0])     # a 3 m, cruzando
    assert time_to_contact(r, v_t) == math.inf
    out = hold_off(np.array([1.0, 0.0, 0.0]), r, v_t, np.zeros(3), 10.0, prm)
    assert out[0] < 0.0
    far = np.array([6.0, 0.0, 0.0])                                    # mas alla del suelo
    assert hold_off(np.array([1.0, 0.0, 0.0]), far, v_t, np.zeros(3), 20.0, prm)[0] > 0.0


def test_radial_excitation_moves_along_the_line_of_sight():
    """En modo radial la oscilacion va a lo largo de la linea de vision, no de lado."""
    prm = GuidanceParams(excitation_mode='radial')
    r, v_t = np.array([20.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    tau = 1.0 / (4.0 * prm.excitation_freq)                    # maximo de la oscilacion
    with_exc, _, gain = gated_velocity(CommitGate(prm), r, v_t, np.zeros(3), 0.7, 100.0, tau,
                                       0.1, prm)
    base, _, _ = gated_velocity(CommitGate(prm), r, v_t, np.zeros(3), 0.7, 100.0, 0.0, 0.1, prm)
    assert gain == 1.0
    delta = with_exc - base
    assert delta[0] == pytest.approx(prm.excitation_amp)
    assert abs(delta[1]) < 1e-9


@pytest.mark.parametrize('v_t, p_t', [
    ((0.0, 2.0, 0.0), (20.0, -15.0, -12.5)),     # cruza a 2 m/s, como en Gazebo
    ((0.0, 1.0, 0.0), (20.0, -15.0, -12.5)),     # cruza a 1 m/s
    ((0.0, 0.0, 0.0), (20.0, -5.0, -12.5)),      # quieto
])
def test_closed_loop_commits_only_with_the_scale_known(v_t, p_t):
    """
    Con la puerta, se compromete por sigma (no a la fuerza) y pasa por el checkpoint.

    Pelota de 0.30 m con prior de 1.0 m: sin la puerta, a 2 m/s el tramo final
    empezaba con sigma del 60-70 %.
    """
    prm = GuidanceParams()
    for seed in range(3):
        min_dist, gate = fly_gated(seed, v_t, p_t)
        assert gate.committed and not gate.forced
        assert gate.commit_sigma < prm.sigma_rel_commit
        assert min_dist < 0.15
