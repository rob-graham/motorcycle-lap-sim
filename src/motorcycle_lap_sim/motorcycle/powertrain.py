"""Pure powertrain kinematic and force formulas."""

from math import pi

from .config import PowertrainConfig


def overall_ratio(powertrain: PowertrainConfig, gear_number: int) -> float:
    """Return total reduction for a user-facing, one-based gear number."""
    if isinstance(gear_number, bool) or not isinstance(gear_number, int):
        raise TypeError("gear_number must be an integer")
    if not 1 <= gear_number <= len(powertrain.gear_ratios):
        raise ValueError(f"gear_number must be between 1 and {len(powertrain.gear_ratios)}")
    return (powertrain.primary_ratio * powertrain.gear_ratios[gear_number - 1]
            * powertrain.final_drive_ratio)


def wheel_angular_speed_radps(vehicle_speed_mps: float, wheel_radius_m: float) -> float:
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be positive")
    return vehicle_speed_mps / wheel_radius_m


def engine_speed_rpm(vehicle_speed_mps: float, wheel_radius_m: float,
                     ratio: float) -> float:
    return wheel_angular_speed_radps(vehicle_speed_mps, wheel_radius_m) * ratio * 60 / (2 * pi)


def rear_wheel_drive_force_n(engine_torque_nm: float, ratio: float,
                             driveline_efficiency: float, wheel_radius_m: float) -> float:
    """Return unconstrained rear-wheel force before any tyre limit."""
    if ratio <= 0 or wheel_radius_m <= 0 or not 0 < driveline_efficiency <= 1:
        raise ValueError("ratio, radius and efficiency must be physically valid")
    return engine_torque_nm * ratio * driveline_efficiency / wheel_radius_m
