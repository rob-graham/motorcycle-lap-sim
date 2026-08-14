"""Immutable geometry-independent sampled path representation."""
from dataclasses import dataclass
import math
import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]

@dataclass(frozen=True)
class SampledPath:
    q_m: FloatArray
    x_m: FloatArray
    y_m: FloatArray
    curvature_1pm: FloatArray
    total_length_m: float
    closed: bool = True

    def __post_init__(self) -> None:
        arrays = tuple(np.asarray(a, dtype=float).copy() for a in
                       (self.q_m, self.x_m, self.y_m, self.curvature_1pm))
        if any(a.ndim != 1 for a in arrays) or len({len(a) for a in arrays}) != 1:
            raise ValueError("path arrays must be one-dimensional and have equal lengths")
        if len(arrays[0]) < 3:
            raise ValueError("a closed path requires at least three samples")
        if any(not np.all(np.isfinite(a)) for a in arrays):
            raise ValueError("path samples must be finite")
        if arrays[0][0] != 0 or np.any(np.diff(arrays[0]) <= 0):
            raise ValueError("q_m must start at zero and be strictly increasing")
        if not self.closed:
            raise ValueError("Phase 3 supports closed paths only")
        if not math.isfinite(self.total_length_m) or self.total_length_m <= arrays[0][-1]:
            raise ValueError("total length must exceed the final omitted-endpoint sample")
        for name, array in zip(("q_m", "x_m", "y_m", "curvature_1pm"), arrays):
            array.setflags(write=False); object.__setattr__(self, name, array)

    @property
    def segment_lengths_m(self) -> FloatArray:
        return np.diff(np.r_[self.q_m, self.total_length_m])

def from_sampled_track(track) -> SampledPath:
    """Adapt a Phase 1 centreline sample; q equals centreline arc length."""
    return SampledPath(track.s_m, track.x_m, track.y_m, track.curvature_1pm,
                       track.total_length_m, True)
