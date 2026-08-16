"""Canonical SI telemetry containers used by validation code."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class TelemetrySession:
    """A cleaned telemetry table in SI units, without hidden simulator coupling."""

    time_s: FloatArray
    distance_m: FloatArray
    east_m: FloatArray
    north_m: FloatArray
    speed_mps: FloatArray
    lateral_acceleration_mps2: FloatArray
    longitudinal_acceleration_mps2: FloatArray
    slope_rad: FloatArray
    heading_rad: FloatArray
    gps_gyro_radps: FloatArray
    latitude_deg: FloatArray
    longitude_deg: FloatArray
    roll_rate_radps: FloatArray
    pitch_rate_radps: FloatArray
    yaw_rate_radps: FloatArray
    engine_rpm: FloatArray
    # Preserve the raw numeric AiM gear signal. The supplied workbook contains
    # interpolated fractional values during shifts, so integer classification is
    # a later validation/cleaning operation rather than an import assumption.
    gear_number: FloatArray
    ecu_throttle_rad: FloatArray
    hand_throttle_fraction: FloatArray
    distance_from_start_m: FloatArray
    lap_id: IntArray
    marker: tuple[str | None, ...]
    source_sheet: str

    def __post_init__(self):
        count = len(self.time_s)
        numeric = (
            self.distance_m, self.east_m, self.north_m, self.speed_mps,
            self.lateral_acceleration_mps2, self.longitudinal_acceleration_mps2,
            self.slope_rad, self.heading_rad, self.gps_gyro_radps,
            self.latitude_deg, self.longitude_deg, self.roll_rate_radps,
            self.pitch_rate_radps, self.yaw_rate_radps, self.engine_rpm,
            self.gear_number, self.ecu_throttle_rad, self.hand_throttle_fraction,
            self.distance_from_start_m, self.lap_id)
        if count == 0 or any(len(values) != count for values in numeric) or len(self.marker) != count:
            raise ValueError("telemetry channels must be non-empty and have identical lengths")
        if not np.all(np.isfinite(self.time_s)):
            raise ValueError("telemetry time must be finite")
        if np.any(np.diff(self.time_s) <= 0):
            raise ValueError("telemetry time must be strictly increasing")


@dataclass(frozen=True)
class TelemetryLap:
    """A contiguous positive lap identifier within a telemetry session."""

    lap_id: int
    start_index: int
    stop_index: int
    duration_s: float


def lap_slices(session: TelemetrySession) -> tuple[TelemetryLap, ...]:
    """Return contiguous positive lap-ID runs without claiming they are complete laps."""
    ids = np.asarray(session.lap_id, dtype=np.int64)
    changes = np.r_[True, ids[1:] != ids[:-1], True]
    boundaries = np.flatnonzero(changes)
    laps = []
    for start, stop in zip(boundaries[:-1], boundaries[1:]):
        lap_id = int(ids[start])
        if lap_id <= 0:
            continue
        duration = float(session.time_s[stop - 1] - session.time_s[start])
        laps.append(TelemetryLap(lap_id, int(start), int(stop), duration))
    return tuple(laps)
