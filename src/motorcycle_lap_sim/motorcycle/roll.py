"""Level-1 demanded-lean and roll-rate calculations for planar validation.

The finite-roll model deliberately remains simple and trajectory-driven.  It
uses steady planar lean demand and one constant maximum roll-rate parameter; it
does not infer rider intent or make the limit depend on throttle, braking,
rider strength, or measured telemetry.
"""

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]


def demanded_lean_rad(speed_mps: ArrayLike, curvature_1pm: ArrayLike,
                       *, gravity_mps2: float = 9.80665) -> FloatArray:
    """Return signed steady planar lean demand atan(v^2*kappa/g)."""
    speed = np.asarray(speed_mps, dtype=float)
    curvature = np.asarray(curvature_1pm, dtype=float)
    if speed.shape != curvature.shape:
        raise ValueError("speed and curvature must have identical shapes")
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0:
        raise ValueError("gravity must be finite and positive")
    if np.any(speed < 0) or not np.all(np.isfinite(speed)) or not np.all(np.isfinite(curvature)):
        raise ValueError("speed must be finite/non-negative and curvature finite")
    return np.arctan(speed * speed * curvature / gravity_mps2)


def demanded_roll_rate_radps(distance_m: ArrayLike, speed_mps: ArrayLike,
                              lean_rad: ArrayLike, *, closed: bool = True) -> FloatArray:
    """Return demanded roll rate v*d(phi)/ds on a sampled path.

    Closed paths use a periodic centred difference with the end-to-start spacing
    inferred from the median sample spacing. Open paths use NumPy's second-order
    gradient. This diagnostic includes whatever speed variation is present in
    the supplied lean array; it is not a filtered IMU channel.
    """
    distance = np.asarray(distance_m, dtype=float)
    speed = np.asarray(speed_mps, dtype=float)
    lean = np.asarray(lean_rad, dtype=float)
    if distance.ndim != 1 or len(distance) < 3 or speed.shape != distance.shape or lean.shape != distance.shape:
        raise ValueError("distance, speed and lean must be equal 1D arrays with at least three samples")
    if (not np.all(np.isfinite(distance)) or not np.all(np.isfinite(speed))
            or not np.all(np.isfinite(lean)) or np.any(speed < 0) or np.any(np.diff(distance) <= 0)):
        raise ValueError("distance must increase and all roll-model inputs must be finite")

    if not closed:
        derivative = np.gradient(lean, distance, edge_order=2)
        return speed * derivative

    spacing = np.diff(distance)
    nominal = float(np.median(spacing))
    if not math.isfinite(nominal) or nominal <= 0:
        raise ValueError("path spacing must be finite and positive")
    lap_length = float(distance[-1] + nominal - distance[0])
    previous_s = np.roll(distance, 1)
    next_s = np.roll(distance, -1)
    previous_s[0] -= lap_length
    next_s[-1] += lap_length
    derivative = (np.roll(lean, -1) - np.roll(lean, 1)) / (next_s - previous_s)
    return speed * derivative


def curvature_transition_roll_rate_radps(
        speed_mps: ArrayLike, curvature_1pm: ArrayLike,
        curvature_gradient_1pm2: ArrayLike, *, gravity_mps2: float = 9.80665
        ) -> FloatArray:
    """Return the Level-1 roll demand caused by changing path curvature.

    The local speed is held constant while differentiating the steady lean
    relation.  Therefore this intentionally omits the additional lean-rate term
    caused by longitudinal acceleration or braking.  That keeps the first
    finite-roll constraint independent of a rider/braking model while retaining
    direct dependence on the chosen trajectory.
    """
    speed = np.asarray(speed_mps, dtype=float)
    curvature = np.asarray(curvature_1pm, dtype=float)
    gradient = np.asarray(curvature_gradient_1pm2, dtype=float)
    if speed.shape != curvature.shape or speed.shape != gradient.shape:
        raise ValueError("speed, curvature and curvature gradient must have identical shapes")
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0:
        raise ValueError("gravity must be finite and positive")
    if (np.any(speed < 0) or not np.all(np.isfinite(speed))
            or not np.all(np.isfinite(curvature)) or not np.all(np.isfinite(gradient))):
        raise ValueError("finite roll inputs require finite curvature/gradient and non-negative speed")
    lean_ratio = speed * speed * curvature / gravity_mps2
    return (speed ** 3 * gradient / gravity_mps2) / (1.0 + lean_ratio * lean_ratio)


def roll_rate_speed_limit_mps(
        curvature_1pm: ArrayLike, curvature_gradient_1pm2: ArrayLike,
        speed_cap_mps: ArrayLike, max_roll_rate_radps: float,
        *, gravity_mps2: float = 9.80665, bisection_iterations: int = 64
        ) -> FloatArray:
    """Return a local speed ceiling for a constant maximum Level-1 roll rate.

    ``speed_cap_mps`` is the already-established lateral/power/other-handling
    ceiling.  It is used only to decide whether the roll constraint can bind.
    Non-binding samples return infinity so applying this result cannot alter an
    otherwise-unconstrained solution.

    For nonzero curvature, the constant-speed roll-demand expression reaches
    its first maximum at 60 degrees of steady lean.  If a roll limit is crossed,
    the first crossing is found by deterministic bisection below that maximum.
    This avoids selecting a spurious high-speed branch of the local formula.
    """
    curvature = np.asarray(curvature_1pm, dtype=float)
    gradient = np.asarray(curvature_gradient_1pm2, dtype=float)
    cap = np.asarray(speed_cap_mps, dtype=float)
    if curvature.shape != gradient.shape or curvature.shape != cap.shape:
        raise ValueError("curvature, curvature gradient and speed cap must have identical shapes")
    if (not math.isfinite(max_roll_rate_radps) or max_roll_rate_radps <= 0
            or isinstance(max_roll_rate_radps, bool)):
        raise ValueError("maximum roll rate must be finite and positive")
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0:
        raise ValueError("gravity must be finite and positive")
    if isinstance(bisection_iterations, bool) or not isinstance(bisection_iterations, int) or bisection_iterations <= 0:
        raise ValueError("bisection_iterations must be a positive integer")
    if (not np.all(np.isfinite(curvature)) or not np.all(np.isfinite(gradient))
            or not np.all(np.isfinite(cap)) or np.any(cap < 0)):
        raise ValueError("roll-rate speed-limit inputs must be finite with non-negative speed caps")

    limit = np.full(curvature.shape, np.inf, dtype=float)
    flat_curvature = curvature.ravel()
    flat_gradient = gradient.ravel()
    flat_cap = cap.ravel()
    flat_limit = limit.ravel()

    for index in range(flat_curvature.size):
        kappa = float(flat_curvature[index])
        kappa_gradient = float(flat_gradient[index])
        upper_cap = float(flat_cap[index])
        if kappa_gradient == 0.0 or upper_cap == 0.0:
            continue

        if kappa == 0.0:
            search_upper = upper_cap
        else:
            sixty_degree_speed = math.sqrt(math.sqrt(3.0) * gravity_mps2 / abs(kappa))
            search_upper = min(upper_cap, sixty_degree_speed)

        upper_rate = abs(float(curvature_transition_roll_rate_radps(
            np.asarray([search_upper]), np.asarray([kappa]), np.asarray([kappa_gradient]),
            gravity_mps2=gravity_mps2)[0]))
        if upper_rate <= max_roll_rate_radps:
            continue

        lower = 0.0
        upper = search_upper
        for _ in range(bisection_iterations):
            middle = 0.5 * (lower + upper)
            middle_rate = abs(float(curvature_transition_roll_rate_radps(
                np.asarray([middle]), np.asarray([kappa]), np.asarray([kappa_gradient]),
                gravity_mps2=gravity_mps2)[0]))
            if middle_rate <= max_roll_rate_radps:
                lower = middle
            else:
                upper = middle
        flat_limit[index] = upper

    if np.any(np.isnan(limit)) or np.any(limit <= 0):
        raise ValueError("roll-rate speed limits must be positive or infinity")
    limit.setflags(write=False)
    return limit


def roll_rate_excess_radps(demanded_rate_radps: ArrayLike, max_roll_rate_radps: float) -> FloatArray:
    """Return non-negative magnitude by which demanded roll rate exceeds a limit."""
    rate = np.asarray(demanded_rate_radps, dtype=float)
    if not np.all(np.isfinite(rate)):
        raise ValueError("demanded roll rate must be finite")
    if not math.isfinite(max_roll_rate_radps) or max_roll_rate_radps <= 0:
        raise ValueError("maximum roll rate must be finite and positive")
    return np.maximum(np.abs(rate) - max_roll_rate_radps, 0.0)
