"""Optional Numba implementation of the fixed-path propagation hot loop.

Importing this module intentionally requires the ``accelerated`` extra.  The
ordinary package and authoritative Python solver never import it.
"""

from math import inf, pi, sqrt

import numpy as np
from numba import njit

from motorcycle_lap_sim.motorcycle.roll import (
    curvature_transition_roll_rate_radps,
    demanded_lean_rad,
)
from motorcycle_lap_sim.path import (
    curvature_gradient_1pm2,
    curvature_transient_speed_limit_mps,
)
from .capabilities import lateral_speed_limit_mps, maximum_rev_limited_speed_mps
from .results import SpeedProfileResult
from .solver import SolverConfig, lap_time_seconds


def _parameters(bike):
    geometry, environment = bike.motorcycle, bike.environment
    powertrain = bike.powertrain
    return (
        geometry.mass_kg, environment.gravity_mps2, geometry.wheelbase_m,
        geometry.cg_height_m, geometry.cg_from_rear_m, geometry.wheel_radius_m,
        environment.air_density_kgpm3, bike.aerodynamics.cda_m2,
        bike.rolling_resistance.crr, bike.tyres.mu_longitudinal,
        bike.tyres.mu_lateral, powertrain.primary_ratio,
        np.asarray(powertrain.gear_ratios, dtype=np.float64),
        powertrain.final_drive_ratio, powertrain.driveline_efficiency,
        powertrain.idle_rpm, powertrain.rev_limit_rpm,
        np.asarray([point.rpm for point in powertrain.torque_curve], dtype=np.float64),
        np.asarray([point.torque_nm for point in powertrain.torque_curve], dtype=np.float64),
    )


@njit(cache=True, fastmath=False)
def _torque(rpm, idle_rpm, rev_limit_rpm, curve_rpm, curve_torque):
    if rpm < idle_rpm or rpm > rev_limit_rpm:
        return 0.0
    for index in range(curve_rpm.size - 1):
        if rpm <= curve_rpm[index + 1]:
            fraction = ((rpm - curve_rpm[index])
                        / (curve_rpm[index + 1] - curve_rpm[index]))
            return (curve_torque[index]
                    + fraction * (curve_torque[index + 1] - curve_torque[index]))
    return 0.0


@njit(cache=True, fastmath=False)
def _best_gear(speed, wheel_radius, primary, gears, final_drive, efficiency,
               idle_rpm, rev_limit_rpm, curve_rpm, curve_torque):
    best_gear, best_rpm, best_force = 0, 0.0, 0.0
    for index in range(gears.size):
        ratio = primary * gears[index] * final_drive
        rpm = speed / wheel_radius * ratio * 60 / (2 * pi)
        if abs(rpm - rev_limit_rpm) <= 1e-9 * rev_limit_rpm:
            rpm = rev_limit_rpm
        torque = _torque(rpm, idle_rpm, rev_limit_rpm, curve_rpm, curve_torque)
        force = torque * ratio * efficiency / wheel_radius if torque > 0 else 0.0
        if force > best_force:
            best_gear, best_rpm, best_force = index + 1, rpm, force
    return best_gear, best_rpm, best_force


@njit(cache=True, fastmath=False)
def _maximum_longitudinal_force(fy, normal, mu_longitudinal, mu_lateral):
    if normal == 0 or mu_longitudinal == 0:
        return 0.0
    lateral_capacity = mu_lateral * normal
    if lateral_capacity == 0 or abs(fy) >= lateral_capacity:
        return 0.0
    utilisation = abs(fy) / lateral_capacity
    return mu_longitudinal * normal * sqrt(1 - utilisation ** 2)


@njit(cache=True, fastmath=False)
def _axle(speed, curvature, acceleration, mass, gravity, wheelbase,
          cg_height, cg_from_rear):
    front = ((mass * gravity * cg_from_rear - mass * acceleration * cg_height)
             / wheelbase)
    rear = mass * gravity - front
    if front < 0 or rear < 0:
        return front, rear, 0.0, 0.0
    lateral = mass * speed ** 2 * abs(curvature)
    return front, rear, lateral * front / (mass * gravity), lateral * rear / (mass * gravity)


@njit(cache=True, fastmath=False)
def _forward(speed, curvature, p):
    (mass, gravity, wheelbase, cg_height, cg_from_rear, wheel_radius,
     air_density, cda, crr, mu_longitudinal, mu_lateral, primary, gears,
     final_drive, efficiency, idle_rpm, rev_limit_rpm, curve_rpm, curve_torque) = p
    drag = 0.5 * air_density * cda * speed ** 2
    rolling = crr * mass * gravity
    gear, rpm, drive = _best_gear(speed, wheel_radius, primary, gears, final_drive,
                                  efficiency, idle_rpm, rev_limit_rpm,
                                  curve_rpm, curve_torque)
    upper = gravity * cg_from_rear / cg_height
    low = -(drag + rolling) / mass
    front, rear, _, fyr = _axle(speed, curvature, low, mass, gravity, wheelbase,
                                cg_height, cg_from_rear)
    margin = -inf
    if front >= 0 and rear >= 0:
        tyre = _maximum_longitudinal_force(fyr, rear, mu_longitudinal, mu_lateral)
        margin = min(drive, tyre) - (mass * low + drag + rolling)
    if margin < -1e-7 * mass:
        low = -gravity * 10
    high = upper
    for _ in range(60):
        middle = (low + high) / 2
        front, rear, _, fyr = _axle(speed, curvature, middle, mass, gravity, wheelbase,
                                    cg_height, cg_from_rear)
        margin = -inf
        if front >= 0 and rear >= 0:
            tyre = _maximum_longitudinal_force(fyr, rear, mu_longitudinal, mu_lateral)
            margin = min(drive, tyre) - (mass * middle + drag + rolling)
        if margin >= 0:
            low = middle
        else:
            high = middle
    return low, gear, rpm


@njit(cache=True, fastmath=False)
def _braking(speed, curvature, p):
    (mass, gravity, wheelbase, cg_height, cg_from_rear, _, air_density,
     cda, crr, mu_longitudinal, mu_lateral, _, _, _, _, _, _, _, _) = p
    drag = 0.5 * air_density * cda * speed ** 2
    rolling = crr * mass * gravity
    low = 0.0
    high = gravity * (wheelbase - cg_from_rear) / cg_height
    for _ in range(60):
        middle = (low + high) / 2
        front, rear, fyf, fyr = _axle(speed, curvature, -middle, mass, gravity, wheelbase,
                                     cg_height, cg_from_rear)
        feasible = False
        if front >= 0 and rear >= 0:
            capacity = (_maximum_longitudinal_force(fyf, front, mu_longitudinal, mu_lateral)
                        + _maximum_longitudinal_force(fyr, rear, mu_longitudinal, mu_lateral))
            feasible = max(0.0, mass * middle - drag - rolling) <= capacity
        if feasible:
            low = middle
        else:
            high = middle
    return low


@njit(cache=True, fastmath=False)
def _roll_rate_demand(speed, curvature, curvature_gradient, gravity):
    lean_ratio = speed * speed * curvature / gravity
    return (speed ** 3 * curvature_gradient / gravity) / (1.0 + lean_ratio * lean_ratio)


@njit(cache=True, fastmath=False)
def _roll_rate_speed_limits(curvature, gradient, cap, max_roll_rate, gravity,
                            bisection_iterations):
    limit = np.full(curvature.shape, np.inf)
    for index in range(curvature.size):
        kappa = curvature[index]
        kappa_gradient = gradient[index]
        upper_cap = cap[index]
        if kappa_gradient == 0.0 or upper_cap == 0.0:
            continue
        if kappa == 0.0:
            search_upper = upper_cap
        else:
            sixty_degree_speed = sqrt(sqrt(3.0) * gravity / abs(kappa))
            search_upper = min(upper_cap, sixty_degree_speed)
        upper_rate = abs(_roll_rate_demand(search_upper, kappa, kappa_gradient, gravity))
        if upper_rate <= max_roll_rate:
            continue
        lower = 0.0
        upper = search_upper
        for _ in range(bisection_iterations):
            middle = 0.5 * (lower + upper)
            middle_rate = abs(_roll_rate_demand(middle, kappa, kappa_gradient, gravity))
            if middle_rate <= max_roll_rate:
                lower = middle
            else:
                upper = middle
        limit[index] = upper
    return limit


def roll_rate_speed_limit_numba(curvature_1pm, curvature_gradient_1pm2,
                                speed_cap_mps, max_roll_rate_radps, *,
                                gravity_mps2=9.80665, bisection_iterations=64):
    """Numba equivalent of the authoritative Level-1 roll-rate speed ceiling."""
    curvature = np.asarray(curvature_1pm, dtype=float)
    gradient = np.asarray(curvature_gradient_1pm2, dtype=float)
    cap = np.asarray(speed_cap_mps, dtype=float)
    if curvature.shape != gradient.shape or curvature.shape != cap.shape:
        raise ValueError("curvature, curvature gradient and speed cap must have identical shapes")
    if (not np.isfinite(max_roll_rate_radps) or max_roll_rate_radps <= 0
            or isinstance(max_roll_rate_radps, bool)):
        raise ValueError("maximum roll rate must be finite and positive")
    if not np.isfinite(gravity_mps2) or gravity_mps2 <= 0:
        raise ValueError("gravity must be finite and positive")
    if (isinstance(bisection_iterations, bool)
            or not isinstance(bisection_iterations, int)
            or bisection_iterations <= 0):
        raise ValueError("bisection_iterations must be a positive integer")
    if (not np.all(np.isfinite(curvature)) or not np.all(np.isfinite(gradient))
            or not np.all(np.isfinite(cap)) or np.any(cap < 0)):
        raise ValueError("roll-rate speed-limit inputs must be finite with non-negative speed caps")
    limit = _roll_rate_speed_limits(
        curvature, gradient, cap, float(max_roll_rate_radps), float(gravity_mps2),
        bisection_iterations)
    if np.any(np.isnan(limit)) or np.any(limit <= 0):
        raise ValueError("roll-rate speed limits must be positive or infinity")
    limit.setflags(write=False)
    return limit


# Compatibility hook used by the merged Phase 11 runtime profiler. Within this
# optional backend the unqualified name intentionally denotes the accelerated
# implementation; the authoritative Python function remains in motorcycle.roll.
roll_rate_speed_limit_mps = roll_rate_speed_limit_numba


@njit(cache=True, fastmath=False)
def _propagate(curvature, segment_lengths, initial_speed, tolerance, max_iterations, p):
    speed = initial_speed.copy()
    count = speed.size
    for iteration in range(1, max_iterations + 1):
        old = speed.copy()
        for index in range(count):
            following = (index + 1) % count
            acceleration, _, _ = _forward(speed[index], curvature[index], p)
            candidate = sqrt(max(0.0, speed[index] ** 2
                                 + 2 * acceleration * segment_lengths[index]))
            speed[following] = min(speed[following], candidate)
        for index in range(count - 1, -1, -1):
            following = (index + 1) % count
            deceleration = _braking(speed[following], curvature[following], p)
            candidate = sqrt(max(0.0, speed[following] ** 2
                                 + 2 * deceleration * segment_lengths[index]))
            speed[index] = min(speed[index], candidate)
        difference = 0.0
        for index in range(count):
            difference = max(difference, abs(old[index] - speed[index]))
        if difference < tolerance:
            return speed, iteration, True
    return speed, max_iterations, False


def forward_acceleration_numba(speed_mps, curvature_1pm, bike):
    return _forward(float(speed_mps), float(curvature_1pm), _parameters(bike))


def braking_deceleration_numba(speed_mps, curvature_1pm, bike):
    return float(_braking(float(speed_mps), float(curvature_1pm), _parameters(bike)))


def solve_speed_profile_numba(path, bike, config=SolverConfig()):
    count = len(path.q_m)
    lateral = np.array([lateral_speed_limit_mps(k, bike) for k in path.curvature_1pm])
    power = np.full(count, maximum_rev_limited_speed_mps(bike))
    gradient = curvature_gradient_1pm2(path)
    handling = bike.handling
    curvature_limit = (
        np.full(count, np.inf)
        if handling is None or handling.max_path_curvature_rate_1pmps is None
        else curvature_transient_speed_limit_mps(
            path, handling.max_path_curvature_rate_1pmps))
    pre_roll_cap = np.minimum(np.minimum(lateral, power), curvature_limit)
    roll_limit = (
        np.full(count, np.inf)
        if handling is None or handling.max_roll_rate_radps is None
        else roll_rate_speed_limit_numba(
            path.curvature_1pm, gradient, pre_roll_cap,
            handling.max_roll_rate_radps,
            gravity_mps2=bike.environment.gravity_mps2))
    initial = np.minimum(pre_roll_cap, roll_limit)
    parameters = _parameters(bike)
    speed, iteration, converged = _propagate(
        np.asarray(path.curvature_1pm), np.asarray(path.segment_lengths_m), initial,
        config.speed_tolerance_mps, config.max_iterations, parameters)
    if not converged:
        raise RuntimeError(
            f"periodic speed solver did not converge in {config.max_iterations} iterations")
    following = np.roll(speed, -1)
    longitudinal = (following ** 2 - speed ** 2) / (2 * path.segment_lengths_m)
    lateral_acceleration = speed ** 2 * np.abs(path.curvature_1pm)
    gears = np.empty(count, dtype=int)
    rpms = np.empty(count)
    for index, value in enumerate(speed):
        gears[index], rpms[index], _ = _best_gear(value, *parameters[5:6], parameters[11],
                                                   parameters[12], parameters[13], parameters[14],
                                                   parameters[15], parameters[16], parameters[17],
                                                   parameters[18])
    curvature_rate = speed * gradient
    lean = demanded_lean_rad(
        speed, path.curvature_1pm, gravity_mps2=bike.environment.gravity_mps2)
    roll_rate = curvature_transition_roll_rate_radps(
        speed, path.curvature_1pm, gradient,
        gravity_mps2=bike.environment.gravity_mps2)
    arrays = [speed, lateral, power, gradient, curvature_rate, curvature_limit,
              roll_limit, lean, roll_rate, lateral_acceleration, longitudinal,
              gears, rpms]
    for array in arrays:
        array.setflags(write=False)
    return SpeedProfileResult(
        path.q_m, speed, lateral, power, gradient, curvature_rate, curvature_limit,
        roll_limit, lean, roll_rate, lateral_acceleration, longitudinal, gears, rpms,
        lap_time_seconds(path, speed), iteration, True)
