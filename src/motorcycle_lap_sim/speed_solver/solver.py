"""Deterministic forward/backward solver for a sampled closed centreline."""

from dataclasses import dataclass
from math import inf, sqrt

import numpy as np
from numpy.typing import NDArray

from motorcycle_lap_sim.motorcycle.config import MotorcycleConfig
from motorcycle_lap_sim.motorcycle.limits import effective_lateral_acceleration_mps2
from motorcycle_lap_sim.track.sampling import SampledTrack
from .capability import (NumericalConfig, acceleration_capability_mps2,
                         braking_capability_mps2, powertrain_speed_ceiling_mps)


@dataclass(frozen=True)
class SpeedProfile:
    speed_mps: NDArray[np.float64]
    lateral_speed_ceiling_mps: NDArray[np.float64]
    powertrain_speed_ceiling_mps: float
    lap_time_s: float
    iterations: int


def segment_lengths_m(track: SampledTrack) -> NDArray[np.float64]:
    """Distances to the next point, including the last-to-first wrap segment."""
    return np.diff(np.append(track.s_m, track.total_length_m))


def solve_fixed_path(track: SampledTrack, config: MotorcycleConfig,
                     numerical: NumericalConfig = NumericalConfig()) -> SpeedProfile:
    """Solve a periodic profile; no sample is privileged as a zero-speed start."""
    if len(track.s_m) < 2:
        raise ValueError("a closed speed profile requires at least two samples")
    if segment_lengths_m(track).min() <= 0:
        raise ValueError("sampled closed track must omit its duplicate endpoint")
    lateral_limit = effective_lateral_acceleration_mps2(
        config.tyres.mu_lateral, config.tyres.max_lean_angle_rad,
        config.environment.gravity_mps2)
    lateral_ceiling = np.array([sqrt(lateral_limit / abs(k)) if k else inf
                                for k in track.curvature_1pm])
    rev_ceiling = powertrain_speed_ceiling_mps(config)
    speed = np.minimum(lateral_ceiling, rev_ceiling)
    ds = segment_lengths_m(track)
    for iteration in range(1, numerical.max_iterations + 1):
        previous = speed.copy()
        for i in range(len(speed)):
            j = (i + 1) % len(speed)
            ay = speed[i] ** 2 * abs(track.curvature_1pm[i])
            accel = acceleration_capability_mps2(speed[i], ay, config)
            speed[j] = min(speed[j], sqrt(max(0.0, speed[i] ** 2 + 2 * accel * ds[i])))
        for i in range(len(speed) - 1, -1, -1):
            j = (i + 1) % len(speed)
            ay = speed[j] ** 2 * abs(track.curvature_1pm[j])
            braking = braking_capability_mps2(speed[j], ay, config)
            speed[i] = min(speed[i], sqrt(max(0.0, speed[j] ** 2 + 2 * braking * ds[i])))
        if np.max(np.abs(speed - previous)) <= numerical.speed_tolerance_mps:
            break
    else:
        raise RuntimeError("periodic speed solver did not converge")
    lap_time = float(np.sum(2 * ds / (speed + np.roll(speed, -1))))
    return SpeedProfile(speed, lateral_ceiling, rev_ceiling, lap_time, iteration)
