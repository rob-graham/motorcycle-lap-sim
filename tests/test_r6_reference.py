"""Regression and integration checks for the provisional R6 reference data."""

from math import isfinite
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.motorcycle.engine import available_engine_torque_nm
from motorcycle_lap_sim.motorcycle.limits import (
    stoppie_deceleration_mps2,
    wheelie_acceleration_mps2,
)
from motorcycle_lap_sim.motorcycle.powertrain import engine_speed_rpm, overall_ratio
from motorcycle_lap_sim.path import from_sampled_track
from motorcycle_lap_sim.speed_solver import (
    maximum_rev_limited_speed_mps,
    road_speed_at_rpm_mps,
    solve_speed_profile,
)
from motorcycle_lap_sim.track import Track, sample_track


R6_PATH = Path("examples/motorcycles/r6_2017plus_reference.yaml")


@pytest.fixture(scope="module")
def r6():
    return load_motorcycle_config(R6_PATH)


def test_r6_reference_loads_with_six_gears_and_ordered_torque_curve(r6):
    assert len(r6.powertrain.gear_ratios) == 6
    rpms = [point.rpm for point in r6.powertrain.torque_curve]
    assert all(left < right for left, right in zip(rpms, rpms[1:]))


def test_r6_torque_interpolation_reproduces_every_listed_point(r6):
    for point in r6.powertrain.torque_curve:
        assert available_engine_torque_nm(point.rpm, r6.powertrain) == pytest.approx(
            point.torque_nm
        )
    assert available_engine_torque_nm(r6.powertrain.idle_rpm - 1, r6.powertrain) == 0
    assert available_engine_torque_nm(r6.powertrain.rev_limit_rpm + 1, r6.powertrain) == 0


def test_r6_gearing_and_rev_limited_speed_are_finite_and_sensible(r6):
    speeds = []
    for gear in range(1, 7):
        ratio = overall_ratio(r6.powertrain, gear)
        rpm = engine_speed_rpm(30.0, r6.motorcycle.wheel_radius_m, ratio)
        speed = road_speed_at_rpm_mps(r6.powertrain.rev_limit_rpm, gear, r6)
        assert isfinite(rpm) and rpm > 0
        assert isfinite(speed) and speed > 0
        speeds.append(speed)
    assert speeds == sorted(speeds)
    assert maximum_rev_limited_speed_mps(r6) == pytest.approx(speeds[-1])


def test_r6_baseline_geometry_reproduces_legacy_longitudinal_limits(r6):
    bike, gravity = r6.motorcycle, r6.environment.gravity_mps2
    assert wheelie_acceleration_mps2(
        gravity, bike.cg_from_rear_m, bike.cg_height_m
    ) / gravity == pytest.approx(1.0)
    assert stoppie_deceleration_mps2(
        gravity, bike.wheelbase_m, bike.cg_from_rear_m, bike.cg_height_m
    ) / gravity == pytest.approx(1.2)


def test_r6_fixed_path_test_oval_converges_with_finite_positive_lap_time(r6):
    track = Track.from_yaml("examples/tracks/test_oval.yaml")
    path = from_sampled_track(sample_track(track, 1.0))
    result = solve_speed_profile(path, r6)
    assert result.converged
    assert isfinite(result.lap_time_s) and result.lap_time_s > 0
    assert np.all(np.isfinite(result.speed_mps))
