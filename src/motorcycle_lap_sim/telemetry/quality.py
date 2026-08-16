"""GPS-quality channels and explicit quality masking for telemetry validation.

Quality indicators are evidence, not automatic truth labels.  The supplied R6
workbook can show repeatable-looking quality-channel values even where position
traces disagree between laps, so callers must choose any thresholds explicitly
and should combine them with cross-lap/track-residual evidence.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class GPSQualitySeries:
    """Raw AiM GPS quality indicators converted to canonical units."""

    time_s: FloatArray
    satellites: FloatArray
    position_accuracy_m: FloatArray
    speed_accuracy_mps: FloatArray
    source_sheet: str

    def __post_init__(self):
        names = ("time_s", "satellites", "position_accuracy_m", "speed_accuracy_mps")
        arrays = []
        for name in names:
            values = np.asarray(getattr(self, name), dtype=float)
            if values.ndim != 1:
                raise ValueError("GPS quality channels must be one-dimensional")
            object.__setattr__(self, name, values)
            arrays.append(values)

        time_s, satellites, position_accuracy_m, speed_accuracy_mps = arrays
        count = len(time_s)
        channels = (satellites, position_accuracy_m, speed_accuracy_mps)
        if count == 0 or any(values.shape != time_s.shape for values in channels):
            raise ValueError("GPS quality channels must be non-empty and have identical shapes")
        if not np.all(np.isfinite(time_s)) or np.any(np.diff(time_s) <= 0):
            raise ValueError("GPS quality time must be finite and strictly increasing")


def require_time_alignment(reference_time_s, quality: GPSQualitySeries, *, atol_s: float = 1e-9) -> None:
    """Require sample-for-sample time alignment before combining quality data."""
    reference = np.asarray(reference_time_s, dtype=float)
    if reference.ndim != 1 or len(reference) != len(quality.time_s):
        raise ValueError("GPS quality series must have the same sample count as telemetry")
    if not math.isfinite(atol_s) or atol_s < 0:
        raise ValueError("alignment tolerance must be finite and non-negative")
    if not np.allclose(reference, quality.time_s, rtol=0.0, atol=atol_s, equal_nan=False):
        raise ValueError("GPS quality time is not sample-aligned with telemetry")


def gps_quality_mask(
        quality: GPSQualitySeries,
        *,
        min_satellites: float | None = None,
        max_position_accuracy_m: float | None = None,
        max_speed_accuracy_mps: float | None = None,
        ) -> BoolArray:
    """Build an explicit mask from caller-selected GPS quality thresholds.

    No thresholds are imposed by default.  Missing/non-finite quality values are
    rejected whenever their corresponding channel is used as a criterion.
    """
    mask = np.ones(len(quality.time_s), dtype=bool)

    if min_satellites is not None:
        if not math.isfinite(min_satellites) or min_satellites < 0:
            raise ValueError("min_satellites must be finite and non-negative")
        values = np.asarray(quality.satellites, dtype=float)
        mask &= np.isfinite(values) & (values >= min_satellites)

    if max_position_accuracy_m is not None:
        if not math.isfinite(max_position_accuracy_m) or max_position_accuracy_m < 0:
            raise ValueError("max_position_accuracy_m must be finite and non-negative")
        values = np.asarray(quality.position_accuracy_m, dtype=float)
        mask &= np.isfinite(values) & (values <= max_position_accuracy_m)

    if max_speed_accuracy_mps is not None:
        if not math.isfinite(max_speed_accuracy_mps) or max_speed_accuracy_mps < 0:
            raise ValueError("max_speed_accuracy_mps must be finite and non-negative")
        values = np.asarray(quality.speed_accuracy_mps, dtype=float)
        mask &= np.isfinite(values) & (values <= max_speed_accuracy_mps)

    return mask
