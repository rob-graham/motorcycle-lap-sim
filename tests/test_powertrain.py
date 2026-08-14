from math import pi
from pathlib import Path

import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.motorcycle.powertrain import (engine_speed_rpm, overall_ratio,
                                                       rear_wheel_drive_force_n)


CONFIG = load_motorcycle_config(Path("examples/motorcycles/test_motorcycle.yaml"))


def test_overall_ratio_and_one_based_gears():
    assert overall_ratio(CONFIG.powertrain, 1) == 12
    assert overall_ratio(CONFIG.powertrain, 2) == 9
    with pytest.raises(ValueError):
        overall_ratio(CONFIG.powertrain, 0)


def test_engine_speed_from_vehicle_speed():
    speed = 0.3 * 2 * pi  # wheel rotates at 1 revolution/s
    assert engine_speed_rpm(speed, 0.3, 12) == pytest.approx(720)


def test_known_wheel_drive_force():
    assert rear_wheel_drive_force_n(50, 12, 0.9, 0.3) == pytest.approx(1800)
