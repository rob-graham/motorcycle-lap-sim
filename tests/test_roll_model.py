import math

import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.roll import (
    demanded_lean_rad, demanded_roll_rate_radps, roll_rate_excess_radps)


def test_demanded_lean_is_zero_on_straight_and_signed_with_curvature():
    speed = np.array([20.0, 20.0, 20.0])
    curvature = np.array([0.0, 0.02, -0.02])
    lean = demanded_lean_rad(speed, curvature)

    assert lean[0] == 0.0
    assert lean[1] == pytest.approx(-lean[2])
    assert lean[1] == pytest.approx(math.atan(20.0**2 * 0.02 / 9.80665))


def test_open_roll_rate_recovers_linear_lean_rate():
    distance = np.arange(5.0)
    speed = np.full(5, 10.0)
    lean = 0.03 * distance

    rate = demanded_roll_rate_radps(distance, speed, lean, closed=False)

    assert np.allclose(rate, 0.3, rtol=0.0, atol=1e-12)


def test_closed_roll_rate_is_periodic_for_sinusoidal_lean():
    distance = np.arange(0.0, 100.0, 1.0)
    speed = np.full_like(distance, 10.0)
    lean = np.sin(2.0 * math.pi * distance / 100.0)

    rate = demanded_roll_rate_radps(distance, speed, lean)
    expected = 10.0 * 2.0 * math.pi / 100.0 * np.cos(2.0 * math.pi * distance / 100.0)

    assert np.max(np.abs(rate - expected)) < 5e-4
    assert rate[0] == pytest.approx(rate[-1], abs=0.003)


def test_roll_rate_excess_is_symmetric_and_non_negative():
    excess = roll_rate_excess_radps(np.array([-4.0, -2.0, 0.0, 3.0]), 2.5)
    assert np.array_equal(excess, [1.5, 0.0, 0.0, 0.5])
