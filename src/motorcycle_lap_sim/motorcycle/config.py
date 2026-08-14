"""Immutable, validated motorcycle configuration loaded from YAML."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, radians
from pathlib import Path
from typing import Any, Mapping

import yaml


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _positive(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result < 0:
        raise ValueError(f"{name} must be non-negative")
    return result


@dataclass(frozen=True)
class EnvironmentConfig:
    gravity_mps2: float
    air_density_kgpm3: float


@dataclass(frozen=True)
class MotorcycleGeometry:
    name: str
    mass_kg: float
    wheelbase_m: float
    cg_height_m: float
    cg_from_rear_m: float
    wheel_radius_m: float


@dataclass(frozen=True)
class AerodynamicsConfig:
    cda_m2: float


@dataclass(frozen=True)
class RollingResistanceConfig:
    crr: float


@dataclass(frozen=True)
class TyreConfig:
    mu_longitudinal: float
    mu_lateral: float
    max_lean_angle_rad: float


@dataclass(frozen=True)
class TorquePoint:
    rpm: float
    torque_nm: float


@dataclass(frozen=True)
class PowertrainConfig:
    primary_ratio: float
    gear_ratios: tuple[float, ...]
    final_drive_ratio: float
    driveline_efficiency: float
    idle_rpm: float
    rev_limit_rpm: float
    torque_curve: tuple[TorquePoint, ...]


@dataclass(frozen=True)
class MotorcycleConfig:
    environment: EnvironmentConfig
    motorcycle: MotorcycleGeometry
    aerodynamics: AerodynamicsConfig
    rolling_resistance: RollingResistanceConfig
    tyres: TyreConfig
    powertrain: PowertrainConfig


def _section(data: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = data.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def motorcycle_config_from_dict(data: Mapping[str, Any]) -> MotorcycleConfig:
    """Validate and construct configuration; invalid values are never repaired."""
    env, bike = _section(data, "environment"), _section(data, "motorcycle")
    aero, rolling = _section(data, "aerodynamics"), _section(data, "rolling_resistance")
    tyres, power = _section(data, "tyres"), _section(data, "powertrain")

    environment = EnvironmentConfig(
        _positive(env.get("gravity_mps2"), "gravity_mps2"),
        _positive(env.get("air_density_kgpm3"), "air_density_kgpm3"),
    )
    wheelbase = _positive(bike.get("wheelbase_m"), "wheelbase_m")
    cg_from_rear = _positive(bike.get("cg_from_rear_m"), "cg_from_rear_m")
    if cg_from_rear >= wheelbase:
        raise ValueError("cg_from_rear_m must be less than wheelbase_m")
    name = bike.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("name must be a non-empty string")
    geometry = MotorcycleGeometry(
        name=name,
        mass_kg=_positive(bike.get("mass_kg"), "mass_kg"),
        wheelbase_m=wheelbase,
        cg_height_m=_positive(bike.get("cg_height_m"), "cg_height_m"),
        cg_from_rear_m=cg_from_rear,
        wheel_radius_m=_positive(bike.get("wheel_radius_m"), "wheel_radius_m"),
    )
    angle_deg = _finite(tyres.get("max_lean_angle_deg"), "max_lean_angle_deg")
    if not 0 <= angle_deg < 90:
        raise ValueError("max_lean_angle_deg must be in [0, 90)")
    tyre_config = TyreConfig(
        _nonnegative(tyres.get("mu_longitudinal"), "mu_longitudinal"),
        _nonnegative(tyres.get("mu_lateral"), "mu_lateral"),
        radians(angle_deg),
    )
    ratios_raw = power.get("gear_ratios")
    if not isinstance(ratios_raw, (list, tuple)) or not ratios_raw:
        raise ValueError("gear_ratios must be a non-empty ordered sequence")
    ratios = tuple(_positive(x, "gear ratio") for x in ratios_raw)
    idle = _positive(power.get("idle_rpm"), "idle_rpm")
    rev_limit = _positive(power.get("rev_limit_rpm"), "rev_limit_rpm")
    if idle >= rev_limit:
        raise ValueError("idle_rpm must be less than rev_limit_rpm")
    curve_raw = power.get("torque_curve")
    if not isinstance(curve_raw, (list, tuple)) or len(curve_raw) < 2:
        raise ValueError("torque_curve must contain at least two points")
    points: list[TorquePoint] = []
    for item in curve_raw:
        if not isinstance(item, Mapping):
            raise ValueError("each torque_curve point must be a mapping")
        points.append(TorquePoint(_positive(item.get("rpm"), "torque curve rpm"),
                                  _finite(item.get("torque_nm"), "torque_nm")))
    if any(right.rpm <= left.rpm for left, right in zip(points, points[1:])):
        raise ValueError("torque_curve RPM values must be strictly increasing")
    if points[0].rpm > idle or points[-1].rpm < rev_limit:
        raise ValueError("torque_curve must cover idle_rpm through rev_limit_rpm")
    efficiency = _positive(power.get("driveline_efficiency"), "driveline_efficiency")
    if efficiency > 1:
        raise ValueError("driveline_efficiency must not exceed 1")
    powertrain = PowertrainConfig(
        _positive(power.get("primary_ratio"), "primary_ratio"), ratios,
        _positive(power.get("final_drive_ratio"), "final_drive_ratio"), efficiency,
        idle, rev_limit, tuple(points),
    )
    return MotorcycleConfig(
        environment, geometry,
        AerodynamicsConfig(_nonnegative(aero.get("cda_m2"), "cda_m2")),
        RollingResistanceConfig(_nonnegative(rolling.get("crr"), "crr")),
        tyre_config, powertrain,
    )


def load_motorcycle_config(path: str | Path) -> MotorcycleConfig:
    """Load a UTF-8 YAML motorcycle configuration."""
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, Mapping):
        raise ValueError("configuration root must be a mapping")
    return motorcycle_config_from_dict(data)
