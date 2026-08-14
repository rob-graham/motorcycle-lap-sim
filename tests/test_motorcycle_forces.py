import pytest

from motorcycle_lap_sim.motorcycle.forces import (aerodynamic_drag_n, axle_normal_loads_n,
                                                   rolling_resistance_n)


def loads(acceleration):
    return axle_normal_loads_n(200, 9.81, 1.4, 0.7, 0.7, acceleration)


def test_drag_is_zero_and_quadratic():
    assert aerodynamic_drag_n(0, 1.225, 0.4) == 0
    assert aerodynamic_drag_n(20, 1.225, 0.4) == pytest.approx(
        4 * aerodynamic_drag_n(10, 1.225, 0.4))


def test_rolling_resistance():
    assert rolling_resistance_n(0.015, 200, 9.81) == pytest.approx(29.43)


def test_static_symmetric_loads_sum_to_weight():
    static = loads(0)
    assert static.front_n + static.rear_n == pytest.approx(200 * 9.81)
    assert static.front_n == pytest.approx(static.rear_n)


def test_acceleration_transfers_rearward_and_braking_forward():
    static, accelerating, braking = loads(0), loads(2), loads(-2)
    assert accelerating.front_n < static.front_n
    assert accelerating.rear_n > static.rear_n
    assert braking.front_n > static.front_n
    assert braking.rear_n < static.rear_n
