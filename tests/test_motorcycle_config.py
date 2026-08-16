from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from motorcycle_lap_sim.motorcycle.config import motorcycle_config_from_dict, load_motorcycle_config


PATH = Path("examples/motorcycles/test_motorcycle.yaml")


def data():
    return yaml.safe_load(PATH.read_text())


def test_valid_configuration_loading():
    config = load_motorcycle_config(PATH)
    assert config.motorcycle.mass_kg == 200
    assert config.powertrain.gear_ratios == (2.0, 1.5)
    assert config.powertrain.torque_curve[-1].rpm == 10000
    assert config.handling is None


def test_optional_handling_configuration_and_validation():
    raw = data(); raw["handling"] = {"max_path_curvature_rate_1pmps": 0.8}
    handling = motorcycle_config_from_dict(raw).handling
    assert handling.max_path_curvature_rate_1pmps == 0.8
    assert handling.max_roll_rate_radps is None

    raw = data(); raw["handling"] = {"max_roll_rate_radps": 0.7}
    handling = motorcycle_config_from_dict(raw).handling
    assert handling.max_path_curvature_rate_1pmps is None
    assert handling.max_roll_rate_radps == 0.7

    raw = data(); raw["handling"] = {
        "max_path_curvature_rate_1pmps": 0.8,
        "max_roll_rate_radps": 0.7,
    }
    handling = motorcycle_config_from_dict(raw).handling
    assert handling.max_path_curvature_rate_1pmps == 0.8
    assert handling.max_roll_rate_radps == 0.7

    invalid = (
        None, 0.8, {},
        {"max_path_curvature_rate_1pmps": 0},
        {"max_path_curvature_rate_1pmps": -1},
        {"max_path_curvature_rate_1pmps": float("nan")},
        {"max_path_curvature_rate_1pmps": float("inf")},
        {"max_roll_rate_radps": 0},
        {"max_roll_rate_radps": -1},
        {"max_roll_rate_radps": float("nan")},
        {"max_roll_rate_radps": float("inf")},
    )
    for handling_raw in invalid:
        raw = data(); raw["handling"] = handling_raw
        with pytest.raises(ValueError):
            motorcycle_config_from_dict(raw)


@pytest.mark.parametrize(("section", "field", "value"), [
    ("motorcycle", "mass_kg", 0), ("motorcycle", "wheelbase_m", -1),
    ("motorcycle", "cg_height_m", 0), ("motorcycle", "cg_from_rear_m", 1.4),
    ("motorcycle", "wheel_radius_m", 0), ("tyres", "mu_lateral", -0.1),
    ("tyres", "mu_longitudinal", -0.1), ("tyres", "max_lean_angle_deg", 90),
    ("powertrain", "primary_ratio", 0), ("powertrain", "final_drive_ratio", -1),
    ("powertrain", "driveline_efficiency", 1.1),
    ("environment", "gravity_mps2", 0), ("environment", "air_density_kgpm3", -1),
])
def test_invalid_scalar_values_rejected(section, field, value):
    raw = deepcopy(data())
    raw[section][field] = value
    with pytest.raises(ValueError):
        motorcycle_config_from_dict(raw)


def test_bad_gears_and_rpm_data_rejected():
    raw = data()
    raw["powertrain"]["gear_ratios"] = [2, 0]
    with pytest.raises(ValueError):
        motorcycle_config_from_dict(raw)
    raw = data()
    raw["powertrain"]["torque_curve"][1]["rpm"] = 1000
    with pytest.raises(ValueError):
        motorcycle_config_from_dict(raw)
    raw = data()
    raw["powertrain"]["torque_curve"][-1]["rpm"] = 9000
    with pytest.raises(ValueError, match="cover"):
        motorcycle_config_from_dict(raw)
