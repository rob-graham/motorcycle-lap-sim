"""Strict equivalence checks for the explicitly optional speed backend."""

from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip("numba")

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.path import SampledPath, from_sampled_track
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY, generate_planar_control_stations,
)
from motorcycle_lap_sim.racing_line import build_smooth_racing_line_path
from motorcycle_lap_sim.speed_solver import (
    SolverConfig, braking_capability, forward_acceleration_capability,
    solve_speed_profile,
)
from motorcycle_lap_sim.speed_solver.numba_backend import (
    braking_deceleration_numba, forward_acceleration_numba,
    solve_speed_profile_numba,
)
from motorcycle_lap_sim.track import Track, sample_track


@pytest.mark.parametrize("filename", [
    "examples/motorcycles/test_motorcycle.yaml",
    "examples/motorcycles/r6_2017plus_reference.yaml",
])
def test_scalar_capabilities_match_reference(filename):
    bike = load_motorcycle_config(filename)
    for speed in (1.0, 5.0, 10.0, 30.0, 60.0, 100.0):
        for curvature in (0.0, 0.005, -0.02, 0.04, -0.08):
            expected = forward_acceleration_capability(speed, curvature, bike)
            acceleration, gear, rpm = forward_acceleration_numba(speed, curvature, bike)
            assert acceleration == pytest.approx(expected.acceleration_mps2, rel=0, abs=2e-13)
            assert gear == expected.gear_number
            assert rpm == pytest.approx(expected.engine_rpm, rel=0, abs=2e-12)
            assert braking_deceleration_numba(speed, curvature, bike) == pytest.approx(
                braking_capability(speed, curvature, bike).deceleration_mps2,
                rel=0, abs=2e-13)


def _corner_path():
    q = np.arange(0.0, 240.0, 5.0)
    curvature = np.zeros(q.size)
    curvature[(q >= 100.0) & (q < 140.0)] = 0.04
    return SampledPath(q, np.zeros(q.size), np.zeros(q.size), curvature, 240.0)


@pytest.mark.parametrize("path", [
    _corner_path(),
    from_sampled_track(sample_track(Track.from_yaml("examples/tracks/test_oval.yaml"), 1.0)),
    from_sampled_track(sample_track(Track.from_yaml("examples/tracks/mallala_reference.yaml"), 1.0)),
])
def test_complete_profiles_match_reference(path):
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    expected = solve_speed_profile(path, bike)
    actual = solve_speed_profile_numba(path, bike)
    assert actual.iterations == expected.iterations
    assert np.array_equal(actual.gear_number, expected.gear_number)
    assert np.allclose(actual.speed_mps, expected.speed_mps, rtol=0, atol=1e-10)
    assert actual.lap_time_s == pytest.approx(expected.lap_time_s, rel=0, abs=1e-12)
    for array in (actual.speed_mps, actual.gear_number, actual.engine_rpm):
        assert not array.flags.writeable


def test_roll_limited_profile_matches_reference():
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    bike = replace(bike, handling=HandlingConfig(max_roll_rate_radps=0.8))
    path = _corner_path()
    expected = solve_speed_profile(path, bike)
    actual = solve_speed_profile_numba(path, bike)

    assert np.count_nonzero(np.isfinite(expected.speed_limit_roll_rate_mps)) > 0
    assert actual.iterations == expected.iterations
    assert np.array_equal(actual.gear_number, expected.gear_number)
    assert np.allclose(actual.speed_mps, expected.speed_mps, rtol=0, atol=1e-10)
    assert np.allclose(
        actual.speed_limit_roll_rate_mps, expected.speed_limit_roll_rate_mps,
        rtol=0, atol=1e-12, equal_nan=False)
    assert np.allclose(
        actual.demanded_roll_rate_radps, expected.demanded_roll_rate_radps,
        rtol=0, atol=1e-12)
    assert actual.lap_time_s == pytest.approx(expected.lap_time_s, rel=0, abs=1e-12)


def test_nonconvergence_failure_matches_reference():
    bike = load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")
    config = SolverConfig(max_iterations=1)
    with pytest.raises(RuntimeError, match="did not converge in 1 iterations"):
        solve_speed_profile(_corner_path(), bike, config)
    with pytest.raises(RuntimeError, match="did not converge in 1 iterations"):
        solve_speed_profile_numba(_corner_path(), bike, config)


def test_mallala_zero_control_planar_profile_matches_reference():
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    track = Track.from_yaml("examples/tracks/mallala_reference.yaml")
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    path = build_smooth_racing_line_path(
        track, np.zeros(stations.size), guide_s_m=stations,
        sample_spacing_m=1.0, boundary_margin_m=0.25,
        boundary_check_spacing_m=0.25).sampled_path
    expected = solve_speed_profile(path, bike)
    actual = solve_speed_profile_numba(path, bike)
    assert actual.iterations == expected.iterations
    assert np.array_equal(actual.gear_number, expected.gear_number)
    assert np.allclose(actual.speed_mps, expected.speed_mps, rtol=0, atol=1e-10)
    assert actual.lap_time_s == pytest.approx(expected.lap_time_s, rel=0, abs=1e-12)
