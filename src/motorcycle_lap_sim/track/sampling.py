"""Approximately uniform arc-length sampling of analytic tracks."""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from .track import Track

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SampledTrack:
    s_m: FloatArray
    x_m: FloatArray
    y_m: FloatArray
    heading_rad: FloatArray
    tangent_x: FloatArray
    tangent_y: FloatArray
    normal_x: FloatArray
    normal_y: FloatArray
    curvature_1pm: FloatArray
    width_left_m: FloatArray
    width_right_m: FloatArray
    total_length_m: float
    closed: bool = True


def sample_track(track: Track, spacing_m: float, *, include_endpoint: bool | None = None) -> SampledTrack:
    """Sample a track; closed tracks omit their duplicate endpoint by default."""
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("sampling spacing must be finite and positive")
    include_endpoint = (not track.closed) if include_endpoint is None else include_endpoint
    intervals = max(1, math.ceil(track.total_length_m / spacing_m))
    s = np.linspace(0.0, track.total_length_m, intervals + 1, dtype=float)
    if not include_endpoint:
        s = s[:-1]

    return sample_track_stations(track, s, include_endpoint=include_endpoint)


def sample_track_stations(track: Track, s_m, *, include_endpoint: bool = False) -> SampledTrack:
    """Evaluate analytic track primitives at explicit centreline stations.

    Closed-track callers normally supply stations in ``[0, L)``.  The endpoint
    is accepted only when ``include_endpoint=True``, preserving
    :func:`sample_track`'s explicit duplicate-endpoint option.
    """
    s = np.asarray(s_m, dtype=float)
    if s.ndim != 1 or len(s) == 0 or not np.all(np.isfinite(s)):
        raise ValueError("track stations must be a non-empty finite one-dimensional array")
    upper_ok = s <= track.total_length_m if include_endpoint else s < track.total_length_m
    if np.any(s < 0.0) or np.any(~upper_ok):
        interval = "[0, total_length]" if include_endpoint else "[0, total_length)"
        raise ValueError(f"track stations must lie in {interval}")
    starts_s = track.primitive_start_s_m
    start_poses = [track.start_pose]
    for primitive in track.primitives[:-1]:
        start_poses.append(primitive.end_pose(start_poses[-1]))
    x = np.empty_like(s); y = np.empty_like(s); heading = np.empty_like(s); curvature = np.empty_like(s)
    primitive_indices = np.empty(len(s), dtype=int)
    for j, distance in enumerate(s):
        # A station exactly at a join belongs to the next primitive.  This is
        # the established ``searchsorted(..., side="right")`` convention.
        index = min(int(np.searchsorted(starts_s[1:], distance, side="right")), len(track.primitives) - 1)
        primitive_indices[j] = index
        local_s = min(float(distance - starts_s[index]), track.primitives[index].length_m)
        pose = track.primitives[index].pose_at(start_poses[index], local_s)
        x[j], y[j], heading[j] = pose.x_m, pose.y_m, pose.heading_rad
        curvature[j] = track.primitives[index].curvature_1pm
    tx, ty = np.cos(heading), np.sin(heading)
    # A +90 degree rotation gives the left-of-travel unit normal.
    nx, ny = -ty, tx
    return SampledTrack(s, x, y, heading, tx, ty, nx, ny, curvature,
                        np.asarray(track.primitive_width_left_m)[primitive_indices],
                        np.asarray(track.primitive_width_right_m)[primitive_indices],
                        track.total_length_m, track.closed)
