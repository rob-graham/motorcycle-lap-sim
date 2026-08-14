"""Geometric, lateral, and generic combined-tyre limits."""

from math import sqrt, tan


def wheelie_acceleration_mps2(gravity_mps2: float, cg_from_rear_m: float,
                              cg_height_m: float) -> float:
    return gravity_mps2 * cg_from_rear_m / cg_height_m


def stoppie_deceleration_mps2(gravity_mps2: float, wheelbase_m: float,
                              cg_from_rear_m: float, cg_height_m: float) -> float:
    """Return positive braking-deceleration magnitude at zero rear load."""
    return gravity_mps2 * (wheelbase_m - cg_from_rear_m) / cg_height_m


def tyre_lateral_acceleration_mps2(mu_lateral: float, gravity_mps2: float) -> float:
    return mu_lateral * gravity_mps2


def lean_lateral_acceleration_mps2(max_lean_angle_rad: float,
                                   gravity_mps2: float) -> float:
    return gravity_mps2 * tan(max_lean_angle_rad)


def effective_lateral_acceleration_mps2(mu_lateral: float, max_lean_angle_rad: float,
                                        gravity_mps2: float) -> float:
    return min(tyre_lateral_acceleration_mps2(mu_lateral, gravity_mps2),
               lean_lateral_acceleration_mps2(max_lean_angle_rad, gravity_mps2))


def _normalised_square(force: float, coefficient: float, normal_load_n: float) -> float:
    capacity = coefficient * normal_load_n
    if capacity == 0:
        return 0.0 if force == 0 else float("inf")
    return (force / capacity) ** 2


def friction_ellipse_utilisation(fx_n: float, fy_n: float, normal_load_n: float,
                                 mu_longitudinal: float, mu_lateral: float) -> float:
    """Return squared ellipse utilisation (feasible when no greater than one)."""
    if normal_load_n < 0 or mu_longitudinal < 0 or mu_lateral < 0:
        raise ValueError("normal load and friction coefficients must be non-negative")
    return (_normalised_square(fx_n, mu_longitudinal, normal_load_n)
            + _normalised_square(fy_n, mu_lateral, normal_load_n))


def maximum_longitudinal_force_n(fy_n: float, normal_load_n: float,
                                 mu_longitudinal: float, mu_lateral: float) -> float:
    """Return available positive Fx magnitude; zero at/beyond lateral capacity."""
    if normal_load_n < 0 or mu_longitudinal < 0 or mu_lateral < 0:
        raise ValueError("normal load and friction coefficients must be non-negative")
    if normal_load_n == 0 or mu_longitudinal == 0:
        return 0.0
    lateral_capacity = mu_lateral * normal_load_n
    if lateral_capacity == 0 or abs(fy_n) >= lateral_capacity:
        return 0.0
    lateral_utilisation = abs(fy_n) / lateral_capacity
    return mu_longitudinal * normal_load_n * sqrt(1 - lateral_utilisation**2)
