"""Level-1 demanded-lean and roll-rate calculations for planar validation.

This module begins the Phase 10 roll-response work without replacing the
existing fixed-path solver or optimiser.  It provides physically interpretable
quantities that can first be compared with Mallala telemetry and then used by a
switchable solver constraint in a later Phase 10 change.
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
    gradient. This is a diagnostic/model quantity, not a filtered IMU channel.
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


def roll_rate_excess_radps(demanded_rate_radps: ArrayLike, max_roll_rate_radps: float) -> FloatArray:
    """Return non-negative magnitude by which demanded roll rate exceeds a limit."""
    rate = np.asarray(demanded_rate_radps, dtype=float)
    if not np.all(np.isfinite(rate)):
        raise ValueError("demanded roll rate must be finite")
    if not math.isfinite(max_roll_rate_radps) or max_roll_rate_radps <= 0:
        raise ValueError("maximum roll rate must be finite and positive")
    return np.maximum(np.abs(rate) - max_roll_rate_radps, 0.0)
