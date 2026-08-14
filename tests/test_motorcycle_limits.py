from math import radians, sqrt, tan

import pytest

from motorcycle_lap_sim.motorcycle.forces import axle_normal_loads_n
from motorcycle_lap_sim.motorcycle.limits import (effective_lateral_acceleration_mps2,
    friction_ellipse_utilisation, lean_lateral_acceleration_mps2,
    maximum_longitudinal_force_n, stoppie_deceleration_mps2,
    tyre_lateral_acceleration_mps2, wheelie_acceleration_mps2)


def test_tip_limits_make_corresponding_axle_load_zero():
    wheelie = wheelie_acceleration_mps2(9.81, 0.7, 0.7)
    stoppie = stoppie_deceleration_mps2(9.81, 1.4, 0.7, 0.7)
    assert wheelie == pytest.approx(9.81)
    assert stoppie == pytest.approx(9.81)
    assert axle_normal_loads_n(200, 9.81, 1.4, 0.7, 0.7, wheelie).front_n == pytest.approx(0)
    assert axle_normal_loads_n(200, 9.81, 1.4, 0.7, 0.7, -stoppie).rear_n == pytest.approx(0)


def test_independent_and_effective_lateral_limits():
    tyre = tyre_lateral_acceleration_mps2(1.2, 9.81)
    lean = lean_lateral_acceleration_mps2(radians(50), 9.81)
    assert tyre == pytest.approx(1.2 * 9.81)
    assert lean == pytest.approx(9.81 * tan(radians(50)))
    assert effective_lateral_acceleration_mps2(1.2, radians(50), 9.81) == min(tyre, lean)


def test_friction_ellipse_longitudinal_capacity_cases():
    assert maximum_longitudinal_force_n(0, 1000, 1.2, 1.0) == pytest.approx(1200)
    assert maximum_longitudinal_force_n(1000, 1000, 1.2, 1.0) == 0
    assert maximum_longitudinal_force_n(500, 1000, 1.2, 1.0) == pytest.approx(1200 * sqrt(0.75))


def test_friction_ellipse_utilisation_and_zero_load_are_explicit():
    assert friction_ellipse_utilisation(600, 500, 1000, 1.2, 1.0) == pytest.approx(0.5)
    assert friction_ellipse_utilisation(0, 0, 0, 1.2, 1.0) == 0
    assert friction_ellipse_utilisation(1, 0, 0, 1.2, 1.0) == float("inf")
    assert maximum_longitudinal_force_n(0, 0, 1.2, 1.0) == 0
