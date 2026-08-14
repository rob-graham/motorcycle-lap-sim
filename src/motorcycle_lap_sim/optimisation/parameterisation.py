"""Smooth, periodic, boundary-safe racing-line parameterisation."""

from dataclasses import dataclass
import math
import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True)
class PeriodicCubicParameterisation:
    """Uniform periodic cubic B-spline evaluated in centreline arc length."""

    control_count: int

    def __post_init__(self) -> None:
        if isinstance(self.control_count, bool) or not isinstance(self.control_count, int) or self.control_count < 4:
            raise ValueError("control_count must be an integer of at least four")

    def latent_values(self, controls: ArrayLike, sampled_track) -> NDArray[np.float64]:
        values = np.asarray(controls, dtype=float)
        if values.shape != (self.control_count,):
            raise ValueError(f"controls must have shape ({self.control_count},)")
        if not np.all(np.isfinite(values)):
            raise ValueError("controls must be finite")
        phase = np.asarray(sampled_track.s_m) / sampled_track.total_length_m * self.control_count
        index = np.floor(phase).astype(int)
        t = phase - index
        # Cardinal cubic B-spline weights.  Wrapping indices makes value and its
        # first two derivatives continuous at start/finish.
        weights = ((1 - t) ** 3 / 6,
                   (3 * t**3 - 6 * t**2 + 4) / 6,
                   (-3 * t**3 + 3 * t**2 + 3 * t + 1) / 6,
                   t**3 / 6)
        latent = sum(w * values[(index + shift) % self.control_count]
                     for w, shift in zip(weights, (-1, 0, 1, 2)))
        return np.asarray(latent, dtype=float)

    def offsets(self, controls: ArrayLike, sampled_track,
                boundary_margin_m: float) -> NDArray[np.float64]:
        """Map latent values smoothly inside asymmetric pointwise boundaries."""
        if not math.isfinite(boundary_margin_m) or boundary_margin_m < 0:
            raise ValueError("boundary margin must be finite and non-negative")
        left = np.asarray(sampled_track.width_left_m, dtype=float) - boundary_margin_m
        right = np.asarray(sampled_track.width_right_m, dtype=float) - boundary_margin_m
        if np.any(left < 0) or np.any(right < 0):
            raise ValueError("boundary margin exceeds the available track width")
        if np.any(left == 0) or np.any(right == 0):
            raise ValueError("boundary margin must leave positive usable width on both sides")
        latent = self.latent_values(controls, sampled_track)
        # sigmoid(log(right/left)) = right/(left+right), hence u=0 maps
        # algebraically to zero even for asymmetric widths.
        z = latent + np.log(right / left)
        sigmoid = np.where(z >= 0, 1 / (1 + np.exp(-z)), np.exp(z) / (1 + np.exp(z)))
        offsets = -right + (left + right) * sigmoid
        offsets[np.asarray(latent) == 0.0] = 0.0
        return offsets
