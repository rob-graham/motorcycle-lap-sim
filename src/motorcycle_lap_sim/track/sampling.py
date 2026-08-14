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


def sample_track(track: Track, spacing_m: float, *, include_endpoint: bool | None = None) -> SampledTrack:
    """Sample a track; closed tracks omit their duplicate endpoint by default."""
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("sampling spacing must be finite and positive")
    include_endpoint = (not track.closed) if include_endpoint is None else include_endpoint
    intervals = max(1, math.ceil(track.total_length_m / spacing_m))
    s = np.linspace(0.0, track.total_length_m, intervals + 1, dtype=float)
    if not include_endpoint:
        s = s[:-1]

    starts_s = track.primitive_start_s_m
    start_poses = [track.start_pose]
    for primitive in track.primitives[:-1]:
        start_poses.append(primitive.end_pose(start_poses[-1]))
    x = np.empty_like(s); y = np.empty_like(s); heading = np.empty_like(s); curvature = np.empty_like(s)
    for j, distance in enumerate(s):
        index = min(int(np.searchsorted(starts_s[1:], distance, side="right")), len(track.primitives) - 1)
        local_s = min(float(distance - starts_s[index]), track.primitives[index].length_m)
        pose = track.primitives[index].pose_at(start_poses[index], local_s)
        x[j], y[j], heading[j] = pose.x_m, pose.y_m, pose.heading_rad
        curvature[j] = track.primitives[index].curvature_1pm
    tx, ty = np.cos(heading), np.sin(heading)
    # A +90 degree rotation gives the left-of-travel unit normal.
    nx, ny = -ty, tx
    return SampledTrack(s, x, y, heading, tx, ty, nx, ny, curvature,
                        np.full_like(s, track.width_left_m),
                        np.full_like(s, track.width_right_m), track.total_length_m)
