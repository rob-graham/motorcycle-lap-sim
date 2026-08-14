"""Track boundary offsets; positive normal is left of travel."""

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

from .sampling import SampledTrack


@dataclass(frozen=True)
class TrackBoundaries:
    left_x_m: NDArray[np.float64]
    left_y_m: NDArray[np.float64]
    right_x_m: NDArray[np.float64]
    right_y_m: NDArray[np.float64]


def calculate_boundaries(samples: SampledTrack) -> TrackBoundaries:
    return TrackBoundaries(
        samples.x_m + samples.width_left_m * samples.normal_x,
        samples.y_m + samples.width_left_m * samples.normal_y,
        samples.x_m - samples.width_right_m * samples.normal_x,
        samples.y_m - samples.width_right_m * samples.normal_y,
    )
