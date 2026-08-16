"""Measured-versus-simulated speed comparison on closed-track chainage.

The functions in this module are numerical diagnostics only.  They do not tune
motorcycle parameters, rescale track geometry, or infer that a discrepancy has
a unique physical cause.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]
IntArray = NDArray[np.int64]


def _immutable(values, *, dtype=float):
    result = np.asarray(values, dtype=dtype).copy()
    result.setflags(write=False)
    return result


def uniform_closed_parameter_grid(total_length_m: float, sample_count: int) -> FloatArray:
    """Return the periodic parameter stations used by a uniform closed-path sample.

    ``PeriodicPlanarSpline.sampled_path`` samples its centreline-``s`` parameter
    uniformly over ``[0, period)`` and omits the duplicated endpoint.  Keeping
    this relation explicit lets validation compare a simulated racing-line speed
    profile against telemetry binned on nominal track chainage.
    """
    if not math.isfinite(total_length_m) or total_length_m <= 0.0:
        raise ValueError("total_length_m must be finite and positive")
    if not isinstance(sample_count, (int, np.integer)) or sample_count < 2:
        raise ValueError("sample_count must be an integer of at least two")
    return _immutable(np.arange(sample_count, dtype=float) * total_length_m / sample_count)


def periodic_interpolate(
        source_chainage_m: ArrayLike,
        source_values: ArrayLike,
        query_chainage_m: ArrayLike,
        total_length_m: float,
        ) -> FloatArray:
    """Linearly interpolate a periodic scalar signal on closed-track chainage."""
    source_s = np.asarray(source_chainage_m, dtype=float)
    source_v = np.asarray(source_values, dtype=float)
    query = np.asarray(query_chainage_m, dtype=float)
    if source_s.ndim != 1 or source_v.shape != source_s.shape or len(source_s) < 2:
        raise ValueError("source chainage and values must be equal one-dimensional arrays")
    if query.ndim != 1:
        raise ValueError("query chainage must be one-dimensional")
    if not math.isfinite(total_length_m) or total_length_m <= 0.0:
        raise ValueError("total_length_m must be finite and positive")
    if (not np.all(np.isfinite(source_s)) or not np.all(np.isfinite(source_v))
            or not np.all(np.isfinite(query))):
        raise ValueError("periodic interpolation inputs must be finite")
    if np.any(source_s < 0.0) or np.any(source_s >= total_length_m):
        raise ValueError("source chainage must lie in [0, total_length_m)")
    if np.any(np.diff(source_s) <= 0.0):
        raise ValueError("source chainage must be strictly increasing")

    wrapped = np.mod(query, total_length_m)
    extended_s = np.r_[source_s[-1] - total_length_m, source_s,
                       source_s[0] + total_length_m]
    extended_v = np.r_[source_v[-1], source_v, source_v[0]]
    return _immutable(np.interp(wrapped, extended_s, extended_v))


@dataclass(frozen=True)
class SpeedEnvelopeComparison:
    """Common-chainage comparison between a measured envelope and simulation."""

    chainage_m: FloatArray
    measured_median_mps: FloatArray
    measured_p10_mps: FloatArray
    measured_p90_mps: FloatArray
    measured_lap_count: IntArray
    simulated_mps: FloatArray
    eligible_mask: BoolArray
    sim_minus_median_mps: FloatArray

    def __post_init__(self) -> None:
        arrays = (
            np.asarray(self.chainage_m), np.asarray(self.measured_median_mps),
            np.asarray(self.measured_p10_mps), np.asarray(self.measured_p90_mps),
            np.asarray(self.measured_lap_count), np.asarray(self.simulated_mps),
            np.asarray(self.eligible_mask), np.asarray(self.sim_minus_median_mps),
        )
        if any(array.ndim != 1 for array in arrays):
            raise ValueError("speed comparison arrays must be one-dimensional")
        if len(arrays[0]) == 0 or any(len(array) != len(arrays[0]) for array in arrays[1:]):
            raise ValueError("speed comparison arrays must be non-empty and equal length")
        for name in ("chainage_m", "measured_median_mps", "measured_p10_mps",
                     "measured_p90_mps", "simulated_mps", "sim_minus_median_mps"):
            object.__setattr__(self, name, _immutable(getattr(self, name)))
        object.__setattr__(self, "measured_lap_count",
                           _immutable(self.measured_lap_count, dtype=np.int64))
        object.__setattr__(self, "eligible_mask",
                           _immutable(self.eligible_mask, dtype=bool))


@dataclass(frozen=True)
class SpeedComparisonSummary:
    eligible_bins: int
    mean_bias_mps: float
    median_bias_mps: float
    mean_absolute_error_mps: float
    rms_error_mps: float
    p95_absolute_error_mps: float
    maximum_absolute_error_mps: float
    maximum_absolute_error_chainage_m: float
    within_p10_p90_bins: int
    above_p90_bins: int
    below_p10_bins: int


def compare_speed_envelope(
        chainage_m: ArrayLike,
        measured_median_mps: ArrayLike,
        measured_p10_mps: ArrayLike,
        measured_p90_mps: ArrayLike,
        measured_lap_count: ArrayLike,
        simulated_chainage_m: ArrayLike,
        simulated_speed_mps: ArrayLike,
        total_length_m: float,
        *,
        required_lap_count: int,
        ) -> SpeedEnvelopeComparison:
    """Interpolate simulation to measured bins and identify complete evidence.

    A bin is eligible only when its measured lap count exactly equals
    ``required_lap_count`` and measured median/p10/p90 are finite.  Incomplete
    bins remain in the returned arrays but their signed error is NaN.
    """
    chainage = np.asarray(chainage_m, dtype=float)
    median = np.asarray(measured_median_mps, dtype=float)
    p10 = np.asarray(measured_p10_mps, dtype=float)
    p90 = np.asarray(measured_p90_mps, dtype=float)
    lap_count_raw = np.asarray(measured_lap_count)
    if any(values.ndim != 1 for values in (chainage, median, p10, p90, lap_count_raw)):
        raise ValueError("measured speed-envelope channels must be one-dimensional")
    if len(chainage) == 0 or any(len(values) != len(chainage)
                                for values in (median, p10, p90, lap_count_raw)):
        raise ValueError("measured speed-envelope channels must be non-empty and equal length")
    if not isinstance(required_lap_count, (int, np.integer)) or required_lap_count <= 0:
        raise ValueError("required_lap_count must be a positive integer")
    if not np.all(np.isfinite(chainage)) or np.any(np.diff(chainage) <= 0.0):
        raise ValueError("measured chainage must be finite and strictly increasing")
    if np.any(chainage < 0.0) or np.any(chainage >= total_length_m):
        raise ValueError("measured chainage must lie in [0, total_length_m)")
    if not np.all(np.isfinite(lap_count_raw)) or np.any(lap_count_raw < 0):
        raise ValueError("measured lap counts must be finite and non-negative")
    if not np.all(lap_count_raw == np.floor(lap_count_raw)):
        raise ValueError("measured lap counts must be integers")
    lap_count = lap_count_raw.astype(np.int64)

    simulated = periodic_interpolate(
        simulated_chainage_m, simulated_speed_mps, chainage, total_length_m)
    finite_measured = np.isfinite(median) & np.isfinite(p10) & np.isfinite(p90)
    ordered_percentiles = (~finite_measured) | ((p10 <= median) & (median <= p90))
    if not np.all(ordered_percentiles):
        raise ValueError("finite measured speed percentiles must satisfy p10 <= median <= p90")
    eligible = (lap_count == required_lap_count) & finite_measured
    if not np.any(eligible):
        raise ValueError("no speed-envelope bins contain the required complete-lap evidence")

    delta = np.full(len(chainage), np.nan, dtype=float)
    delta[eligible] = simulated[eligible] - median[eligible]
    return SpeedEnvelopeComparison(
        chainage, median, p10, p90, lap_count, simulated, eligible, delta)


def summarize_speed_comparison(comparison: SpeedEnvelopeComparison) -> SpeedComparisonSummary:
    """Return scalar discrepancy metrics for eligible bins only."""
    eligible = np.asarray(comparison.eligible_mask, dtype=bool)
    delta = comparison.sim_minus_median_mps[eligible]
    absolute = np.abs(delta)
    simulated = comparison.simulated_mps[eligible]
    p10 = comparison.measured_p10_mps[eligible]
    p90 = comparison.measured_p90_mps[eligible]
    worst_local = int(np.argmax(absolute))
    eligible_indices = np.flatnonzero(eligible)
    worst_index = int(eligible_indices[worst_local])
    return SpeedComparisonSummary(
        eligible_bins=int(len(delta)),
        mean_bias_mps=float(np.mean(delta)),
        median_bias_mps=float(np.median(delta)),
        mean_absolute_error_mps=float(np.mean(absolute)),
        rms_error_mps=float(np.sqrt(np.mean(delta ** 2))),
        p95_absolute_error_mps=float(np.percentile(absolute, 95.0)),
        maximum_absolute_error_mps=float(absolute[worst_local]),
        maximum_absolute_error_chainage_m=float(comparison.chainage_m[worst_index]),
        within_p10_p90_bins=int(np.count_nonzero((simulated >= p10) & (simulated <= p90))),
        above_p90_bins=int(np.count_nonzero(simulated > p90)),
        below_p10_bins=int(np.count_nonzero(simulated < p10)),
    )
