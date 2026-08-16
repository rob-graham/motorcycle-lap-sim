"""Cross-lap trajectory consistency metrics independent of simulator geometry."""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PeerTrajectoryDeviation:
    """Nearest-trajectory deviation for each lap against all peer laps.

    ``median_nearest_distance_m[i]`` has one value per point of lap ``i``.  Each
    value is the median of that point's nearest Euclidean distance to every
    other lap trajectory.  This deliberately does not require chainage or a
    track model, making it useful for detecting one-lap spatial disagreement
    before registration.  Real rider line variation is still present and must
    not automatically be labelled GPS error.
    """

    median_nearest_distance_m: tuple[FloatArray, ...]


def _validate_laps(lap_x_m, lap_y_m):
    if len(lap_x_m) < 2 or len(lap_x_m) != len(lap_y_m):
        raise ValueError("peer deviation requires at least two laps with matching x/y lists")
    validated = []
    for x_values, y_values in zip(lap_x_m, lap_y_m):
        x = np.asarray(x_values, dtype=float)
        y = np.asarray(y_values, dtype=float)
        if x.ndim != 1 or y.shape != x.shape or len(x) == 0:
            raise ValueError("each peer lap must contain equal non-empty one-dimensional x/y arrays")
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
            raise ValueError("peer lap coordinates must be finite")
        validated.append((x, y))
    return tuple(validated)


def _nearest_distance_to_polyline_samples(x, y, peer_x, peer_y, chunk_size: int) -> FloatArray:
    result = np.empty(len(x), dtype=float)
    for start in range(0, len(x), chunk_size):
        stop = min(start + chunk_size, len(x))
        dx = x[start:stop, None] - peer_x[None, :]
        dy = y[start:stop, None] - peer_y[None, :]
        squared = dx * dx + dy * dy
        result[start:stop] = np.sqrt(np.min(squared, axis=1))
    return result


def peer_trajectory_deviation(lap_x_m, lap_y_m, *, chunk_size: int = 512) -> PeerTrajectoryDeviation:
    """Measure each lap point against the nearest samples of every other lap.

    The result is intentionally sample-based rather than interpolated.  At the
    supplied 20 Hz R6 logging rate the spatial sample spacing is small enough for
    a first quality diagnostic, while keeping the method transparent and free of
    extra dependencies.  A later refinement can interpolate peer segments if
    that materially changes the conclusions.
    """
    if not isinstance(chunk_size, int) or chunk_size <= 0:
        raise ValueError("chunk_size must be a positive integer")
    laps = _validate_laps(lap_x_m, lap_y_m)
    output = []
    for index, (x, y) in enumerate(laps):
        distances = []
        for peer_index, (peer_x, peer_y) in enumerate(laps):
            if peer_index == index:
                continue
            distances.append(_nearest_distance_to_polyline_samples(
                x, y, peer_x, peer_y, chunk_size))
        output.append(np.median(np.vstack(distances), axis=0))
    return PeerTrajectoryDeviation(tuple(output))
