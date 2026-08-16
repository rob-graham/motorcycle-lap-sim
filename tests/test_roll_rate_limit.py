import math
from dataclasses import replace

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.motorcycle.roll import (
    curvature_transition_roll_rate_radps,
    roll_rate_speed_limit_mps,
)
from motorcycle_lap_sim.path import SampledPath, curvature_gradient_1pm2, from_sampled_track
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import CircularArc, Pose, Track, sample_track


def synthetic_sine_path(samples=120):
    length = 120.0
    q = np.arange(samples, dtype=float) * length / samples
    curvature = 0.045 * np.sin(2 * np.pi * q / length)
    return SampledPath(q, np.zeros(samples), np.zeros(samples), curvature, length)


def test_zero_curvature_gradient_never_creates_roll_speed_limit():
    sampled = sample_track(
        Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 5, True), 1.5)
    path = from_sampled_track(sampled)
    gradient = curvature_gradient_1pm2(path)
    assert np.array_equal(gradient, np.zeros_like(gradient))
    cap = np.full(len(path.q_m), 100.0)
    assert np.all(np.isinf(roll_rate_speed_limit_mps(
        path.curvature_1pm, gradient, cap, 0.6)))


def test_roll_limit_solution_respects_constant_rate_and_only_slows():
    path = synthetic_sine_path()
    bike = load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")
    disabled = solve_speed_profile(path, bike)
    limit = 0.35
    enabled = solve_speed_profile(
        path, replace(bike, handling=HandlingConfig(max_roll_rate_radps=limit)))

    assert enabled.lap_time_s >= disabled.lap_time_s
    assert np.all(enabled.speed_mps <= disabled.speed_mps + 1e-12)
    assert np.count_nonzero(np.isfinite(enabled.speed_limit_roll_rate_mps)) > 0
    assert np.max(np.abs(enabled.demanded_roll_rate_radps)) <= limit * (1 + 1e-10)


def test_local_roll_formula_matches_closed_form_at_zero_curvature():
    speed = np.asarray([12.0])
    curvature = np.asarray([0.0])
    gradient = np.asarray([0.002])
    gravity = 9.80665
    actual = curvature_transition_roll_rate_radps(
        speed, curvature, gradient, gravity_mps2=gravity)[0]
    expected = speed[0] ** 3 * gradient[0] / gravity
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-15)
