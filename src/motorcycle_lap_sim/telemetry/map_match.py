"""Two-dimensional telemetry coordinate transforms and reference-track matching."""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from motorcycle_lap_sim.track.sampling import SampledTrack

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class Rigid2DTransform:
    """Explicit world/local rigid transform; no hidden georeferencing assumptions."""

    origin_east_m: float
    origin_north_m: float
    local_x_bearing_rad: float

    def world_to_local(self, east_m, north_m) -> tuple[FloatArray, FloatArray]:
        east = np.asarray(east_m, dtype=float) - self.origin_east_m
        north = np.asarray(north_m, dtype=float) - self.origin_north_m
        if east.shape != north.shape:
            raise ValueError("east and north arrays must have identical shapes")
        # Bearing is clockwise from north.  Local x points along the bearing and
        # local y is left of local x, matching the simulator normal convention.
        sine, cosine = math.sin(self.local_x_bearing_rad), math.cos(self.local_x_bearing_rad)
        x = east * sine + north * cosine
        y = -east * cosine + north * sine
        return x, y


@dataclass(frozen=True)
class TrackMatchResult:
    """Nearest sampled reference station and signed lateral offset."""

    chainage_m: FloatArray
    lateral_offset_m: FloatArray
    reference_distance_m: FloatArray
    sample_index: NDArray[np.int64]


def map_match_nearest(x_m, y_m, sampled_track: SampledTrack, *, chunk_size=1024) -> TrackMatchResult:
    """Map points already in the simulator frame to the nearest sampled centreline.

    This deliberately performs only transparent nearest-sample matching.  A
    sufficiently fine reference grid should be supplied by the caller; later
    interpolation can improve precision without changing the coordinate contract.
    """
    x = np.asarray(x_m, dtype=float)
    y = np.asarray(y_m, dtype=float)
    if x.ndim != 1 or y.shape != x.shape or len(x) == 0:
        raise ValueError("map-matching coordinates must be equal non-empty 1D arrays")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("map-matching coordinates must be finite")
    if not isinstance(chunk_size, int) or chunk_size <= 0:
        raise ValueError("chunk_size must be a positive integer")

    indices = np.empty(len(x), dtype=np.int64)
    distances = np.empty(len(x), dtype=float)
    offsets = np.empty(len(x), dtype=float)
    for start in range(0, len(x), chunk_size):
        stop = min(start + chunk_size, len(x))
        dx = x[start:stop, None] - sampled_track.x_m[None, :]
        dy = y[start:stop, None] - sampled_track.y_m[None, :]
        squared = dx * dx + dy * dy
        local_indices = np.argmin(squared, axis=1)
        rows = np.arange(stop - start)
        indices[start:stop] = local_indices
        distances[start:stop] = np.sqrt(squared[rows, local_indices])
        offsets[start:stop] = (
            dx[rows, local_indices] * sampled_track.normal_x[local_indices]
            + dy[rows, local_indices] * sampled_track.normal_y[local_indices])

    return TrackMatchResult(sampled_track.s_m[indices], offsets, distances, indices)
