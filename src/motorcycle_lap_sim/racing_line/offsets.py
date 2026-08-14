"""Immutable dense lateral-offset profiles and boundary validation."""

from dataclasses import dataclass
import math
import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True, init=False)
class LateralOffsetProfile:
    """Offsets at every centreline sample; positive values are left of travel."""

    offset_m: NDArray[np.float64]
    boundary_margin_m: float

    def __init__(self, sampled_track, offsets: ArrayLike, boundary_margin_m: float = 0.0) -> None:
        values = np.asarray(offsets, dtype=float).copy()
        if not getattr(sampled_track, "closed", False):
            raise ValueError("racing-line profiles require a closed sampled track")
        if values.ndim != 1:
            raise ValueError("offsets must be one-dimensional")
        if len(values) != len(sampled_track.s_m):
            raise ValueError("offset count must equal sampled-track count")
        if not np.all(np.isfinite(values)):
            raise ValueError("offsets must be finite")
        if not math.isfinite(boundary_margin_m) or boundary_margin_m < 0:
            raise ValueError("boundary margin must be finite and non-negative")
        left = np.asarray(sampled_track.width_left_m, dtype=float)
        right = np.asarray(sampled_track.width_right_m, dtype=float)
        if np.any(boundary_margin_m > left) or np.any(boundary_margin_m > right):
            raise ValueError("boundary margin exceeds the available track width")
        lower, upper = -(right - boundary_margin_m), left - boundary_margin_m
        if np.any(values < lower) or np.any(values > upper):
            raise ValueError("racing-line offset lies outside the usable track width")
        values.setflags(write=False)
        object.__setattr__(self, "offset_m", values)
        object.__setattr__(self, "boundary_margin_m", float(boundary_margin_m))

    def minimum_boundary_clearance_m(self, sampled_track) -> float:
        """Return minimum geometric distance from the reference point to either edge."""
        return float(np.min(np.minimum(sampled_track.width_left_m - self.offset_m,
                                       sampled_track.width_right_m + self.offset_m)))
