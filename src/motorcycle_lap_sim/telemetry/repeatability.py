"""Closed-track chainage diagnostics and multi-lap repeatability summaries.

These helpers deliberately separate map-matching quality from rider line
variation.  Small local backtracking in nearest-track chainage is preserved and
reported rather than silently forced monotonic; only the start/finish wrap is
unwrapped geometrically.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class ChainageProgressDiagnostics:
    """Forward-progress diagnostics for one map-matched lap."""

    wrap_count: int
    backward_step_count: int
    total_backward_m: float
    largest_backward_step_m: float
    net_progress_m: float


@dataclass(frozen=True)
class CrossLapEnvelope:
    """Per-chainage summary across independently binned laps."""

    chainage_m: FloatArray
    median: FloatArray
    p10: FloatArray
    p90: FloatArray
    minimum: FloatArray
    maximum: FloatArray
    lap_count: IntArray
    per_lap_values: FloatArray


def unwrap_closed_chainage(chainage_m, total_length_m: float) -> tuple[FloatArray, int]:
    """Remove start/finish discontinuities while preserving local backtracking.

    A jump larger than half a lap is interpreted as crossing the closed-track
    start/finish seam.  Smaller negative steps remain untouched so GPS or
    map-matching reversals stay visible to diagnostics.
    """
    chainage = np.asarray(chainage_m, dtype=float)
    if chainage.ndim != 1 or len(chainage) == 0 or not np.all(np.isfinite(chainage)):
        raise ValueError("chainage must be a non-empty finite one-dimensional array")
    if not math.isfinite(total_length_m) or total_length_m <= 0:
        raise ValueError("total_length_m must be finite and positive")
    if np.any(chainage < 0.0) or np.any(chainage >= total_length_m):
        raise ValueError("closed-track chainage samples must lie in [0, total_length_m)")

    unwrapped = np.empty_like(chainage)
    unwrapped[0] = chainage[0]
    offset = 0.0
    wraps = 0
    half_length = 0.5 * total_length_m
    for index in range(1, len(chainage)):
        raw_delta = chainage[index] - chainage[index - 1]
        if raw_delta < -half_length:
            offset += total_length_m
            wraps += 1
        elif raw_delta > half_length:
            offset -= total_length_m
            wraps -= 1
        unwrapped[index] = chainage[index] + offset
    return unwrapped, wraps


def chainage_progress_diagnostics(
        chainage_m,
        total_length_m: float,
        *,
        backward_tolerance_m: float = 0.25,
        ) -> ChainageProgressDiagnostics:
    """Quantify non-forward map-matched chainage without modifying the samples."""
    if not math.isfinite(backward_tolerance_m) or backward_tolerance_m < 0:
        raise ValueError("backward_tolerance_m must be finite and non-negative")
    unwrapped, wraps = unwrap_closed_chainage(chainage_m, total_length_m)
    if len(unwrapped) < 2:
        return ChainageProgressDiagnostics(wraps, 0, 0.0, 0.0, 0.0)

    delta = np.diff(unwrapped)
    backward = delta < -backward_tolerance_m
    backward_magnitudes = -delta[backward]
    total_backward = float(np.sum(backward_magnitudes)) if len(backward_magnitudes) else 0.0
    largest_backward = float(np.max(backward_magnitudes)) if len(backward_magnitudes) else 0.0
    return ChainageProgressDiagnostics(
        wrap_count=wraps,
        backward_step_count=int(np.count_nonzero(backward)),
        total_backward_m=total_backward,
        largest_backward_step_m=largest_backward,
        net_progress_m=float(unwrapped[-1] - unwrapped[0]),
    )


def _bin_one_lap(chainage, values, total_length_m: float, edges: FloatArray) -> FloatArray:
    chainage = np.asarray(chainage, dtype=float)
    values = np.asarray(values, dtype=float)
    if chainage.ndim != 1 or values.shape != chainage.shape:
        raise ValueError("lap chainage and values must be equal one-dimensional arrays")
    valid = np.isfinite(chainage) & np.isfinite(values)
    if not np.any(valid):
        return np.full(len(edges) - 1, np.nan, dtype=float)
    if np.any(chainage[valid] < 0.0) or np.any(chainage[valid] >= total_length_m):
        raise ValueError("closed-track chainage samples must lie in [0, total_length_m)")

    bin_index = np.searchsorted(edges, chainage[valid], side="right") - 1
    bin_index = np.clip(bin_index, 0, len(edges) - 2)
    values_valid = values[valid]
    result = np.full(len(edges) - 1, np.nan, dtype=float)
    for index in np.unique(bin_index):
        result[index] = float(np.median(values_valid[bin_index == index]))
    return result


def cross_lap_envelope(
        lap_chainage_m,
        lap_values,
        total_length_m: float,
        *,
        bin_width_m: float = 10.0,
        ) -> CrossLapEnvelope:
    """Bin each lap independently, then summarize between-lap variation.

    Binning each lap before forming the envelope avoids overweighting a slower
    lap merely because it contributes more time samples to a chainage interval.
    """
    if not math.isfinite(total_length_m) or total_length_m <= 0:
        raise ValueError("total_length_m must be finite and positive")
    if not math.isfinite(bin_width_m) or bin_width_m <= 0:
        raise ValueError("bin_width_m must be finite and positive")
    if len(lap_chainage_m) == 0 or len(lap_chainage_m) != len(lap_values):
        raise ValueError("lap_chainage_m and lap_values must contain the same non-zero number of laps")

    bin_count = max(1, int(math.ceil(total_length_m / bin_width_m)))
    edges = np.linspace(0.0, total_length_m, bin_count + 1, dtype=float)
    centres = 0.5 * (edges[:-1] + edges[1:])
    per_lap = np.vstack([
        _bin_one_lap(chainage, values, total_length_m, edges)
        for chainage, values in zip(lap_chainage_m, lap_values)
    ])
    lap_count = np.sum(np.isfinite(per_lap), axis=0, dtype=np.int64)

    def percentile(q: float) -> FloatArray:
        result = np.full(bin_count, np.nan, dtype=float)
        populated = lap_count > 0
        if np.any(populated):
            result[populated] = np.nanpercentile(per_lap[:, populated], q, axis=0)
        return result

    return CrossLapEnvelope(
        chainage_m=centres,
        median=percentile(50.0),
        p10=percentile(10.0),
        p90=percentile(90.0),
        minimum=percentile(0.0),
        maximum=percentile(100.0),
        lap_count=lap_count,
        per_lap_values=per_lap,
    )
