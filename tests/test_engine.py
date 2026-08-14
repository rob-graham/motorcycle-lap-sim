from pathlib import Path

import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.motorcycle.engine import available_engine_torque_nm


POWER = load_motorcycle_config(Path("examples/motorcycles/test_motorcycle.yaml")).powertrain


@pytest.mark.parametrize(("rpm", "torque"), [(1000, 50), (5000, 100), (10000, 50),
                                               (3000, 75), (7500, 75)])
def test_torque_table_and_linear_interpolation(rpm, torque):
    assert available_engine_torque_nm(rpm, POWER) == pytest.approx(torque)


def test_torque_is_zero_outside_operating_range():
    assert available_engine_torque_nm(999, POWER) == 0
    assert available_engine_torque_nm(10001, POWER) == 0
