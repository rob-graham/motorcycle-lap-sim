"""Numerical construction of a generic path from supplied lateral offsets."""

import numpy as np
from numpy.typing import ArrayLike, NDArray

from motorcycle_lap_sim.path import SampledPath
from .offsets import LateralOffsetProfile


def periodic_three_point_curvature(x_m: ArrayLike, y_m: ArrayLike) -> NDArray[np.float64]:
    """Signed circumcircle curvature using periodic previous/current/next points.

    The result is exact for non-degenerate points on a circle, zero for collinear
    triples, and otherwise depends on sampling resolution. Coincident neighbours
    are rejected because their tangent and circumcircle are undefined.
    """
    x, y = np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
    if x.ndim != 1 or y.ndim != 1 or x.shape != y.shape or len(x) < 3:
        raise ValueError("curvature coordinates must be equal one-dimensional arrays of length >= 3")
    previous = np.column_stack((x - np.roll(x, 1), y - np.roll(y, 1)))
    following = np.column_stack((np.roll(x, -1) - x, np.roll(y, -1) - y))
    chord = previous + following
    lengths = np.linalg.norm(previous, axis=1) * np.linalg.norm(following, axis=1) * np.linalg.norm(chord, axis=1)
    if np.any(lengths == 0.0):
        raise ValueError("generated racing line contains coincident neighbouring points")
    cross = previous[:, 0] * following[:, 1] - previous[:, 1] * following[:, 0]
    curvature = 2.0 * cross / lengths
    if not np.all(np.isfinite(curvature)):
        raise ValueError("generated racing-line curvature is not finite")
    return curvature


def build_racing_line_path(sampled_track, offsets: LateralOffsetProfile | ArrayLike,
                           *, boundary_margin_m: float | None = None) -> SampledPath:
    """Displace centreline samples along their left normals and build actual geometry.

    Existing profiles retain their margin unless ``boundary_margin_m`` overrides
    it.  Constructing a new profile here deliberately revalidates every input
    against this call's sampled track; sample-count equality is not sufficient.
    """
    if isinstance(offsets, LateralOffsetProfile):
        margin = offsets.boundary_margin_m if boundary_margin_m is None else boundary_margin_m
        profile = LateralOffsetProfile(sampled_track, offsets.offset_m, margin)
    else:
        margin = 0.0 if boundary_margin_m is None else boundary_margin_m
        profile = LateralOffsetProfile(sampled_track, offsets, margin)
    x = sampled_track.x_m + profile.offset_m * sampled_track.normal_x
    y = sampled_track.y_m + profile.offset_m * sampled_track.normal_y
    segment_lengths = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)
    if np.any(segment_lengths <= 0.0) or not np.all(np.isfinite(segment_lengths)):
        raise ValueError("generated racing line contains degenerate neighbouring points")
    q = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))
    return SampledPath(q, x, y, periodic_three_point_curvature(x, y),
                       float(np.sum(segment_lengths)), closed=True)
