"""Deterministic differential geometry calculations for closed sampled paths."""

import math
import numpy as np


def curvature_gradient_1pm2(path) -> np.ndarray:
    """Return periodic d(kappa)/dq using a nonuniform centred three-point formula.

    ``SampledPath`` omits the duplicate endpoint.  Consequently the previous
    and next distances at start/finish are obtained using ``total_length_m``.
    The difference form used here gives exactly zero for exactly constant input.
    """
    q = path.q_m
    previous_q = np.roll(q, 1).copy()
    previous_q[0] -= path.total_length_m
    next_q = np.roll(q, -1).copy()
    next_q[-1] += path.total_length_m
    h_previous = q - previous_q
    h_next = next_q - q
    curvature = path.curvature_1pm
    previous_curvature = np.roll(curvature, 1)
    next_curvature = np.roll(curvature, -1)
    gradient = (h_previous**2 * (next_curvature - curvature)
                + h_next**2 * (curvature - previous_curvature)) / (
                    h_previous * h_next * (h_previous + h_next))
    if gradient.shape != curvature.shape or not np.all(np.isfinite(gradient)):
        raise ValueError("curvature gradient must contain one finite value per path sample")
    gradient.setflags(write=False)
    return gradient


def curvature_transient_speed_limit_mps(path, max_path_curvature_rate_1pmps: float) -> np.ndarray:
    """Return the local speed ceiling implied by the path-curvature-rate proxy."""
    if (isinstance(max_path_curvature_rate_1pmps, bool)
            or not isinstance(max_path_curvature_rate_1pmps, (int, float))
            or not math.isfinite(max_path_curvature_rate_1pmps)
            or max_path_curvature_rate_1pmps <= 0):
        raise ValueError("max_path_curvature_rate_1pmps must be finite and positive")
    gradient = curvature_gradient_1pm2(path)
    limit = np.full(gradient.shape, np.inf)
    nonzero = gradient != 0.0
    limit[nonzero] = max_path_curvature_rate_1pmps / np.abs(gradient[nonzero])
    if np.any(np.isnan(limit)) or np.any(limit <= 0):
        raise ValueError("curvature-transient speed limits must be positive or infinity")
    limit.setflags(write=False)
    return limit
