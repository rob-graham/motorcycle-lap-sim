import math
from dataclasses import replace

import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.racing_line import LateralOffsetProfile, build_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import CircularArc, Pose, SampledTrack, Track, sample_track


def circle_samples(spacing_m=1.0, radius_m=30.0):
    return sample_track(Track((CircularArc(radius_m, 2 * math.pi),), Pose(0.0, 0.0, 0.0), 5.0, 5.0, True), spacing_m)


def test_zero_offset_coordinates_closed_wrap_and_length_convergence():
    errors = []
    for spacing in (2.0, 1.0, 0.5):
        samples = circle_samples(spacing)
        path = build_racing_line_path(samples, np.zeros(len(samples.s_m)))
        assert np.array_equal(path.x_m, samples.x_m)
        assert np.array_equal(path.y_m, samples.y_m)
        assert path.closed and path.segment_lengths_m[-1] > 0
        assert np.allclose(path.curvature_1pm, 1 / 30.0, rtol=1e-10)
        errors.append(abs(path.total_length_m - 2 * math.pi * 30.0))
    assert errors[2] < errors[1] < errors[0]


@pytest.mark.parametrize(("offset", "expected_radius"), [(2.0, 28.0), (-2.0, 32.0)])
def test_constant_circle_offset_sign_length_and_curvature(offset, expected_radius):
    samples = circle_samples(0.5)
    path = build_racing_line_path(samples, np.full(len(samples.s_m), offset))
    assert path.total_length_m == pytest.approx(2 * math.pi * expected_radius, rel=2e-5)
    assert path.curvature_1pm == pytest.approx(np.full(len(path.q_m), 1 / expected_radius), rel=1e-9)
    assert np.hypot(path.x_m, path.y_m - 30.0) == pytest.approx(
        np.full(len(path.q_m), expected_radius), rel=1e-12)


def test_boundaries_and_margin_are_validated_without_clipping():
    samples = circle_samples(2.0)
    count = len(samples.s_m)
    assert np.all(LateralOffsetProfile(samples, np.full(count, 5.0)).offset_m == 5.0)
    assert np.all(LateralOffsetProfile(samples, np.full(count, -5.0)).offset_m == -5.0)
    for value in (5.01, -5.01):
        with pytest.raises(ValueError, match="outside"):
            LateralOffsetProfile(samples, np.full(count, value))
    LateralOffsetProfile(samples, np.full(count, 4.0), 1.0)
    with pytest.raises(ValueError, match="outside"):
        LateralOffsetProfile(samples, np.full(count, 4.01), 1.0)
    with pytest.raises(ValueError, match="exceeds"):
        LateralOffsetProfile(samples, np.zeros(count), 5.01)


def test_profile_input_validation_and_immutability():
    samples = circle_samples(2.0); count = len(samples.s_m)
    for invalid in (np.zeros((count, 1)), np.zeros(count - 1), np.r_[np.zeros(count - 1), np.nan],
                    np.r_[np.zeros(count - 1), np.inf]):
        with pytest.raises(ValueError):
            LateralOffsetProfile(samples, invalid)
    with pytest.raises(ValueError, match="closed"):
        LateralOffsetProfile(SampledTrack(samples.s_m, samples.x_m, samples.y_m,
            samples.heading_rad, samples.tangent_x, samples.tangent_y, samples.normal_x,
            samples.normal_y, samples.curvature_1pm, samples.width_left_m,
            samples.width_right_m, samples.total_length_m, False), np.zeros(count))
    profile = LateralOffsetProfile(samples, np.zeros(count))
    with pytest.raises(ValueError):
        profile.offset_m[0] = 1.0


def test_degenerate_generated_neighbours_are_rejected():
    original = circle_samples(10.0)
    samples = replace(original, width_left_m=np.full(len(original.s_m), 31.0))
    # Moving every point to the circle centre makes all generated samples coincide.
    with pytest.raises(ValueError, match="degenerate"):
        build_racing_line_path(samples, np.full(len(samples.s_m), 30.0))


def test_periodic_endpoint_curvature_uses_wrapped_neighbours():
    samples = circle_samples(1.0)
    path = build_racing_line_path(samples, np.full(len(samples.s_m), 2.0))
    assert path.curvature_1pm[0] == pytest.approx(1 / 28.0, rel=1e-10)
    assert path.curvature_1pm[-1] == pytest.approx(1 / 28.0, rel=1e-10)
    assert path.total_length_m > path.q_m[-1]


def test_test_oval_paths_feed_generic_periodic_speed_solver():
    samples = sample_track(Track.from_yaml("examples/tracks/test_oval.yaml"), 2.0)
    bike = load_motorcycle_config("examples/motorcycles/test_motorcycle.yaml")
    for offset in (0.0, 2.0):
        path = build_racing_line_path(samples, np.full(len(samples.s_m), offset))
        result = solve_speed_profile(path, bike)
        assert result.converged and result.lap_time_s > 0
        assert np.all(np.isfinite(result.speed_mps)) and result.speed_mps[0] > 0
        assert np.all(result.speed_mps <= result.speed_limit_lateral_mps + 1e-10)
