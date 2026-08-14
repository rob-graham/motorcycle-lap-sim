import math
from dataclasses import replace
import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (OptimisationConfig, PeriodicCubicParameterisation,
    evaluate_racing_line, optimise_racing_line)
from motorcycle_lap_sim.racing_line import build_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import CircularArc, Pose, Track, sample_track


def circle(spacing=2.0):
    return sample_track(Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 3, True), spacing)


def bike(): return load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")


def test_periodic_parameterisation_zero_direction_boundaries_and_smoothness():
    samples = circle(); p = PeriodicCubicParameterisation(8)
    zero = p.offsets(np.zeros(8), samples, 0.25)
    assert zero.shape == samples.s_m.shape and np.array_equal(zero, np.zeros_like(zero))
    positive = p.offsets(np.ones(8), samples, 0.25)
    negative = p.offsets(-np.ones(8), samples, 0.25)
    assert np.all(positive > 0) and np.all(negative < 0) and np.all(np.isfinite(positive))
    assert np.all(positive < samples.width_left_m - .25)
    assert np.all(negative > -(samples.width_right_m - .25))
    latent = p.latent_values([1, 0, -1, 0, 1, 0, -1, 0], samples)
    wrapped_second_difference = np.diff(np.r_[latent[-1], latent, latent[0]], 2)
    assert np.max(np.abs(wrapped_second_difference)) < .02


def test_controls_configuration_and_margin_validation():
    p = PeriodicCubicParameterisation(8); samples = circle()
    for controls in (np.zeros(7), np.r_[np.zeros(7), np.nan], np.r_[np.zeros(7), np.inf]):
        with pytest.raises(ValueError): p.offsets(controls, samples, 0)
    for kwargs in ({"control_count": 3}, {"control_bound": 0}, {"step_reduction": 1},
                   {"max_sweeps": 0}, {"max_evaluations": 0}, {"boundary_margin_m": -1}):
        with pytest.raises(ValueError): OptimisationConfig(**kwargs)
    with pytest.raises(ValueError, match="margin"): p.offsets(np.zeros(8), samples, 3.1)


def test_zero_objective_is_phase4_zero_path_lap_time():
    samples = circle(); motorcycle = bike(); p = PeriodicCubicParameterisation(8)
    evaluation = evaluate_racing_line(np.zeros(8), samples, motorcycle, p, 0)
    ordinary = solve_speed_profile(build_racing_line_path(samples, np.zeros(len(samples.s_m))), motorcycle)
    assert evaluation.feasible and evaluation.lap_time_s == ordinary.lap_time_s


def test_pattern_search_improves_circle_is_deterministic_and_interoperable():
    samples = circle(2); config = OptimisationConfig(control_count=4, max_sweeps=8,
        max_evaluations=80, minimum_step=.2, boundary_margin_m=.1)
    first = optimise_racing_line(samples, bike(), config)
    second = optimise_racing_line(samples, bike(), config)
    assert first.best_lap_time_s < first.initial_lap_time_s
    assert first.best_lap_time_s == second.best_lap_time_s
    assert np.array_equal(first.best_controls, second.best_controls)
    assert first.evaluations == second.evaluations and first.termination_reason == second.termination_reason
    assert first.speed_profile.converged and np.all(np.isfinite(first.speed_profile.speed_mps))
    assert np.all(np.isfinite(first.sampled_path.curvature_1pm))
    assert np.all(first.dense_offset_m <= samples.width_left_m - .1)
    assert np.all(first.dense_offset_m >= -(samples.width_right_m - .1))
    with pytest.raises(ValueError): first.best_controls[0] = 2


def test_coarse_circle_improvement_survives_fine_resolution():
    track = Track((CircularArc(30, 2 * math.pi),), Pose(0, 0, 0), 5, 5, True)
    motorcycle = bike(); config = OptimisationConfig(control_count=4, max_sweeps=5,
        max_evaluations=50, minimum_step=.4, boundary_margin_m=.25)
    coarse = optimise_racing_line(sample_track(track, 1), motorcycle, config)
    fine = sample_track(track, .5); p = PeriodicCubicParameterisation(4)
    zero = evaluate_racing_line(np.zeros(4), fine, motorcycle, p, .25)
    best = evaluate_racing_line(coarse.best_controls, fine, motorcycle, p, .25)
    assert best.lap_time_s < zero.lap_time_s
