import math
from dataclasses import replace

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.path import (SampledPath, curvature_gradient_1pm2,
                                     curvature_transient_speed_limit_mps, from_sampled_track)
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import CircularArc, Pose, Track, sample_track


def synthetic_sine(samples):
    length = 100.0
    # Deterministic nonuniform periodic spacing exercises the general formula.
    u = np.arange(samples, dtype=float) / samples
    q = length * (u + 0.12 / (2 * np.pi) * np.sin(2 * np.pi * u))
    amplitude = 0.04
    curvature = amplitude * np.sin(2 * np.pi * q / length)
    path = SampledPath(q, np.zeros(samples), np.zeros(samples), curvature, length)
    exact = amplitude * 2 * np.pi / length * np.cos(2 * np.pi * q / length)
    return path, exact


def test_periodic_nonuniform_sinusoid_converges_and_wraps():
    coarse, coarse_exact = synthetic_sine(40)
    fine, fine_exact = synthetic_sine(80)
    coarse_error = np.max(np.abs(curvature_gradient_1pm2(coarse) - coarse_exact))
    fine_gradient = curvature_gradient_1pm2(fine)
    fine_error = np.max(np.abs(fine_gradient - fine_exact))
    assert fine_error < coarse_error / 3.5
    # Endpoint errors remain comparable to interior errors: no seam artifact.
    errors = np.abs(fine_gradient - fine_exact)
    assert max(errors[0], errors[-1]) <= errors.max()
    assert max(errors[0], errors[-1]) < 1.5 * np.percentile(errors, 95)


def test_constant_circle_has_infinite_ceiling_and_unchanged_solution():
    sampled = sample_track(Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0),
                                 5, 5, True), 1.7)
    path = from_sampled_track(sampled)
    gradient = curvature_gradient_1pm2(path)
    assert np.array_equal(gradient, np.zeros_like(gradient))
    assert np.all(np.isinf(curvature_transient_speed_limit_mps(path, 0.8)))
    bike = load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")
    disabled = solve_speed_profile(path, bike)
    enabled = solve_speed_profile(path, replace(bike, handling=HandlingConfig(0.8)))
    assert disabled.lap_time_s == enabled.lap_time_s
    assert np.array_equal(disabled.speed_mps, enabled.speed_mps)


def test_active_limit_and_actual_rate_hold_at_every_sample():
    path, _ = synthetic_sine(80)
    bike = load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")
    result = solve_speed_profile(path, replace(bike, handling=HandlingConfig(0.4)))
    applicable = result.curvature_gradient_1pm2 != 0
    assert np.all(result.speed_mps[applicable]
                  <= result.speed_limit_curvature_transient_mps[applicable])
    assert np.max(np.abs(result.curvature_rate_1pmps)) <= 0.4 * (1 + 1e-12)


def test_disabled_diagnostics_are_nonrestrictive_and_regression_exact():
    path, _ = synthetic_sine(50)
    bike = load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")
    first = solve_speed_profile(path, bike)
    second = solve_speed_profile(path, replace(bike, handling=None))
    assert first.lap_time_s == second.lap_time_s
    assert np.array_equal(first.speed_mps, second.speed_mps)
    assert np.all(np.isinf(first.speed_limit_curvature_transient_mps))
