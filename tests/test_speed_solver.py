"""Analytically understandable validation of the Phase 3 fixed-path solver."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.speed_solver.capability import (NumericalConfig,
    acceleration_capability_mps2, best_gear, braking_capability_mps2)
from motorcycle_lap_sim.speed_solver.solver import segment_lengths_m, solve_fixed_path
from motorcycle_lap_sim.track.sampling import sample_track
from motorcycle_lap_sim.track.track import Track


BIKE = load_motorcycle_config(Path("examples/motorcycles/test_motorcycle.yaml"))
OVAL = Track.from_yaml(Path("examples/tracks/test_oval.yaml"))


def solve_oval(spacing_m: float = 1.0):
    sampled = sample_track(OVAL, spacing_m)
    return sampled, solve_fixed_path(sampled, BIKE)


def test_best_gear_is_deterministic_and_no_gear_is_explicit():
    selections = [best_gear(10.0, BIKE) for _ in range(10)]
    assert selections == [selections[0]] * 10
    assert selections[0] is not None and selections[0].gear_number == 1
    # At 100 m/s even top gear is beyond its 10,000 rpm limit.
    assert best_gear(100.0, BIKE) is None


def test_analytic_straight_line_capabilities():
    # Equal 0.7 m CG height/offsets make both geometric tip limits exactly 1 g.
    assert acceleration_capability_mps2(10.0, 0.0, BIKE) == pytest.approx(9.81, abs=1e-6)
    assert braking_capability_mps2(10.0, 0.0, BIKE) == pytest.approx(9.81, abs=1e-6)
    # Phase 2 torque interpolation, gearing, drag and rolling resistance give
    # 7.398 m/s^2 (rounded to the precision meaningful for this analytic bike).
    assert acceleration_capability_mps2(30.0, 0.0, BIKE) == pytest.approx(7.398, abs=0.002)


def test_corner_braking_and_exit_acceleration_are_present():
    track, profile = solve_oval()
    first_corner = np.flatnonzero(track.curvature_1pm)[0]
    assert profile.speed_mps[first_corner - 20] > profile.speed_mps[first_corner - 1]
    second_straight = np.flatnonzero((track.s_m > 195) & (track.curvature_1pm == 0))
    assert profile.speed_mps[second_straight[10]] > profile.speed_mps[second_straight[0]]


def test_every_point_respects_lateral_and_rev_ceilings():
    _, profile = solve_oval()
    assert np.all(profile.speed_mps <= profile.lateral_speed_ceiling_mps + 1e-10)
    assert np.all(profile.speed_mps <= profile.powertrain_speed_ceiling_mps + 1e-10)


def test_every_segment_including_wrap_respects_longitudinal_constraints():
    track, profile = solve_oval()
    ds = segment_lengths_m(track)
    checked = []
    for i in range(len(profile.speed_mps)):
        j = (i + 1) % len(profile.speed_mps)
        vi, vj = profile.speed_mps[i], profile.speed_mps[j]
        forward = acceleration_capability_mps2(
            vi, vi**2 * abs(track.curvature_1pm[i]), BIKE)
        braking = braking_capability_mps2(
            vj, vj**2 * abs(track.curvature_1pm[j]), BIKE)
        actual = (vj**2 - vi**2) / (2 * ds[i])
        assert actual <= forward + 2e-6
        assert actual >= -braking - 2e-6
        checked.append((i, j))
    assert checked[-1] == (len(profile.speed_mps) - 1, 0)


def test_periodic_solution_does_not_assume_zero_start_speed():
    _, profile = solve_oval()
    assert profile.speed_mps[0] > 1.0


def test_cyclic_start_index_does_not_materially_change_lap_time():
    track, original = solve_oval()
    # Sampling is uniform for a closed track.  Rolling every point field models
    # placing start/finish 73 samples later on exactly the same periodic path.
    shift = 73
    shifted = replace(track,
        x_m=np.roll(track.x_m, -shift), y_m=np.roll(track.y_m, -shift),
        heading_rad=np.roll(track.heading_rad, -shift),
        tangent_x=np.roll(track.tangent_x, -shift), tangent_y=np.roll(track.tangent_y, -shift),
        normal_x=np.roll(track.normal_x, -shift), normal_y=np.roll(track.normal_y, -shift),
        curvature_1pm=np.roll(track.curvature_1pm, -shift))
    moved = solve_fixed_path(shifted, BIKE)
    assert moved.lap_time_s == pytest.approx(original.lap_time_s, abs=1e-9)
    assert moved.speed_mps == pytest.approx(np.roll(original.speed_mps, -shift), abs=1e-7)


def test_oval_spacing_convergence_has_documented_tolerance():
    # The piecewise-constant curvature discretisation places primitive changes
    # within one sample.  Halving spacing from 1.0 to 0.5 m must therefore alter
    # lap time by under 0.05 s; 2.0 m is retained as a coarser trend check.
    times = [solve_oval(spacing)[1].lap_time_s for spacing in (2.0, 1.0, 0.5)]
    assert abs(times[2] - times[1]) < 0.05
    assert abs(times[2] - times[1]) < abs(times[1] - times[0])


def test_numerical_speed_tolerance_is_validated():
    with pytest.raises(ValueError):
        NumericalConfig(speed_tolerance_mps=0.0)
