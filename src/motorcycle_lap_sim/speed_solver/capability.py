"""Longitudinal capability calculations using the Phase 2 physical formulas.

Assumptions are a flat road, no wind, no engine braking and ideal use of the
combined-tyre envelope.  Tip-over limits apply to the *net* vehicle
acceleration; drag and rolling resistance are included in the force balance.
"""

from dataclasses import dataclass
from math import isfinite

from motorcycle_lap_sim.motorcycle.config import MotorcycleConfig
from motorcycle_lap_sim.motorcycle.engine import available_engine_torque_nm
from motorcycle_lap_sim.motorcycle.forces import aerodynamic_drag_n, rolling_resistance_n
from motorcycle_lap_sim.motorcycle.limits import (maximum_longitudinal_force_n,
    stoppie_deceleration_mps2, wheelie_acceleration_mps2)
from motorcycle_lap_sim.motorcycle.powertrain import (engine_speed_rpm, overall_ratio,
    rear_wheel_drive_force_n)


@dataclass(frozen=True)
class NumericalConfig:
    """Numerical controls for periodic speed-profile convergence."""

    speed_tolerance_mps: float = 1e-7
    max_iterations: int = 10_000

    def __post_init__(self) -> None:
        if self.speed_tolerance_mps <= 0:
            raise ValueError("numerical tolerances must be positive")
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")


@dataclass(frozen=True)
class GearSelection:
    gear_number: int
    engine_speed_rpm: float
    drive_force_n: float


def best_gear(speed_mps: float, config: MotorcycleConfig) -> GearSelection | None:
    """Return the usable gear producing most wheel force (lowest gear wins ties)."""
    if not isfinite(speed_mps) or speed_mps < 0:
        raise ValueError("speed must be finite and non-negative")
    candidates: list[GearSelection] = []
    for gear in range(1, len(config.powertrain.gear_ratios) + 1):
        ratio = overall_ratio(config.powertrain, gear)
        rpm = engine_speed_rpm(speed_mps, config.motorcycle.wheel_radius_m, ratio)
        torque = available_engine_torque_nm(rpm, config.powertrain)
        if torque > 0:
            force = rear_wheel_drive_force_n(torque, ratio,
                                             config.powertrain.driveline_efficiency,
                                             config.motorcycle.wheel_radius_m)
            candidates.append(GearSelection(gear, rpm, force))
    return max(candidates, key=lambda item: (item.drive_force_n, -item.gear_number), default=None)


def _resistance_n(speed_mps: float, config: MotorcycleConfig) -> float:
    return (aerodynamic_drag_n(speed_mps, config.environment.air_density_kgpm3,
                               config.aerodynamics.cda_m2)
            + rolling_resistance_n(config.rolling_resistance.crr,
                                   config.motorcycle.mass_kg,
                                   config.environment.gravity_mps2))


def _tyre_force_n(lateral_acceleration_mps2: float, config: MotorcycleConfig) -> float:
    mass = config.motorcycle.mass_kg
    return maximum_longitudinal_force_n(
        mass * abs(lateral_acceleration_mps2), mass * config.environment.gravity_mps2,
        config.tyres.mu_longitudinal, config.tyres.mu_lateral)


def acceleration_capability_mps2(speed_mps: float, lateral_acceleration_mps2: float,
                                 config: MotorcycleConfig) -> float:
    """Return maximum net forward acceleration."""
    selection = best_gear(speed_mps, config)
    drive = 0.0 if selection is None else selection.drive_force_n
    mass = config.motorcycle.mass_kg
    return min(wheelie_acceleration_mps2(config.environment.gravity_mps2,
                                         config.motorcycle.cg_from_rear_m,
                                         config.motorcycle.cg_height_m),
               max(0.0, (min(drive, _tyre_force_n(lateral_acceleration_mps2, config))
                         - _resistance_n(speed_mps, config)) / mass))


def braking_capability_mps2(speed_mps: float, lateral_acceleration_mps2: float,
                            config: MotorcycleConfig) -> float:
    """Return maximum positive deceleration magnitude."""
    mass = config.motorcycle.mass_kg
    force_deceleration = (_tyre_force_n(lateral_acceleration_mps2, config)
                          + _resistance_n(speed_mps, config)) / mass
    return min(stoppie_deceleration_mps2(config.environment.gravity_mps2,
                                         config.motorcycle.wheelbase_m,
                                         config.motorcycle.cg_from_rear_m,
                                         config.motorcycle.cg_height_m),
               force_deceleration)


def powertrain_speed_ceiling_mps(config: MotorcycleConfig) -> float:
    """Highest vehicle speed possible in top gear at the rev limit."""
    from math import pi
    ratio = overall_ratio(config.powertrain, len(config.powertrain.gear_ratios))
    return (config.powertrain.rev_limit_rpm * 2 * pi / 60
            * config.motorcycle.wheel_radius_m / ratio)
