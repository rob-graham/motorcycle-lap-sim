"""Robust rigid registration of telemetry positions to sampled track geometry.

The registration is deliberately limited to rotation and translation.  No scale
factor is fitted: persistent length/shape disagreement remains visible as a
validation residual rather than being hidden by georeferencing calibration.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from motorcycle_lap_sim.track.sampling import SampledTrack

from .map_match import Rigid2DTransform, TrackMatchResult, map_match_nearest

FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class RigidRegistrationResult:
    """Result of trimmed iterative closest-point rigid registration."""

    transform: Rigid2DTransform
    match: TrackMatchResult
    inlier_mask: BoolArray
    iterations: int
    rms_residual_m: float
    median_residual_m: float
    p95_residual_m: float


def _normalise_bearing(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _fit_transform_from_correspondences(east, north, target_x, target_y) -> Rigid2DTransform:
    """Least-squares world->local rotation/translation for paired 2D points."""
    east = np.asarray(east, dtype=float)
    north = np.asarray(north, dtype=float)
    target_x = np.asarray(target_x, dtype=float)
    target_y = np.asarray(target_y, dtype=float)
    if not (east.shape == north.shape == target_x.shape == target_y.shape):
        raise ValueError("registration correspondence arrays must have identical shapes")
    if east.ndim != 1 or len(east) < 2:
        raise ValueError("at least two registration correspondences are required")

    world = np.column_stack((east, north))
    local = np.column_stack((target_x, target_y))
    world_centroid = np.mean(world, axis=0)
    local_centroid = np.mean(local, axis=0)
    p = world - world_centroid
    q = local - local_centroid

    dot = float(np.sum(p[:, 0] * q[:, 0] + p[:, 1] * q[:, 1]))
    cross = float(np.sum(p[:, 0] * q[:, 1] - p[:, 1] * q[:, 0]))
    if abs(dot) + abs(cross) <= np.finfo(float).eps:
        raise ValueError("registration correspondences do not determine a rotation")

    theta = math.atan2(cross, dot)
    c, s = math.cos(theta), math.sin(theta)
    rotation = np.array([[c, -s], [s, c]])
    origin = world_centroid - rotation.T @ local_centroid
    bearing = _normalise_bearing(theta + math.pi / 2.0)
    return Rigid2DTransform(float(origin[0]), float(origin[1]), bearing)


def fit_rigid_registration(
        east_m,
        north_m,
        sampled_track: SampledTrack,
        initial_transform: Rigid2DTransform,
        *,
        valid_mask=None,
        trim_fraction: float = 0.85,
        max_iterations: int = 30,
        convergence_translation_m: float = 1e-4,
        convergence_bearing_rad: float = 1e-7,
        chunk_size: int = 1024,
        ) -> RigidRegistrationResult:
    """Fit rotation + translation using trimmed iterative nearest-track matching.

    ``valid_mask`` is intended for independently identified telemetry-quality
    exclusions.  ``trim_fraction`` then provides additional robustness against
    transient position errors and imperfect track/reference correspondence.
    Neither mechanism changes scale or silently repairs the source trajectory.
    """
    east = np.asarray(east_m, dtype=float)
    north = np.asarray(north_m, dtype=float)
    if east.ndim != 1 or north.shape != east.shape or len(east) < 3:
        raise ValueError("registration coordinates must be equal 1D arrays with at least three points")
    finite = np.isfinite(east) & np.isfinite(north)
    if valid_mask is None:
        valid = finite
    else:
        supplied = np.asarray(valid_mask, dtype=bool)
        if supplied.shape != east.shape:
            raise ValueError("valid_mask must have the same shape as registration coordinates")
        valid = finite & supplied
    if np.count_nonzero(valid) < 3:
        raise ValueError("registration requires at least three valid coordinate samples")
    if not math.isfinite(trim_fraction) or not 0.5 <= trim_fraction <= 1.0:
        raise ValueError("trim_fraction must be finite and between 0.5 and 1.0")
    if not isinstance(max_iterations, int) or max_iterations <= 0:
        raise ValueError("max_iterations must be a positive integer")
    if convergence_translation_m <= 0 or convergence_bearing_rad <= 0:
        raise ValueError("registration convergence tolerances must be positive")

    transform = initial_transform
    final_match = None
    final_inliers = None
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        x, y = transform.world_to_local(east, north)
        match = map_match_nearest(x, y, sampled_track, chunk_size=chunk_size)

        candidate_indices = np.flatnonzero(valid)
        candidate_residuals = match.reference_distance_m[candidate_indices]
        if trim_fraction < 1.0:
            keep_count = max(3, int(math.ceil(trim_fraction * len(candidate_indices))))
            order = np.argsort(candidate_residuals, kind="stable")
            inlier_indices = candidate_indices[order[:keep_count]]
        else:
            inlier_indices = candidate_indices

        inliers = np.zeros(len(east), dtype=bool)
        inliers[inlier_indices] = True
        target_indices = match.sample_index[inlier_indices]
        updated = _fit_transform_from_correspondences(
            east[inlier_indices], north[inlier_indices],
            sampled_track.x_m[target_indices], sampled_track.y_m[target_indices])

        translation_delta = math.hypot(
            updated.origin_east_m - transform.origin_east_m,
            updated.origin_north_m - transform.origin_north_m)
        bearing_delta = abs(_normalise_bearing(
            updated.local_x_bearing_rad - transform.local_x_bearing_rad))
        transform = updated
        final_match = match
        final_inliers = inliers
        if (translation_delta <= convergence_translation_m
                and bearing_delta <= convergence_bearing_rad):
            break

    x, y = transform.world_to_local(east, north)
    final_match = map_match_nearest(x, y, sampled_track, chunk_size=chunk_size)
    candidate_indices = np.flatnonzero(valid)
    candidate_residuals = final_match.reference_distance_m[candidate_indices]
    if trim_fraction < 1.0:
        keep_count = max(3, int(math.ceil(trim_fraction * len(candidate_indices))))
        order = np.argsort(candidate_residuals, kind="stable")
        inlier_indices = candidate_indices[order[:keep_count]]
    else:
        inlier_indices = candidate_indices
    final_inliers = np.zeros(len(east), dtype=bool)
    final_inliers[inlier_indices] = True

    residuals = final_match.reference_distance_m[inlier_indices]
    rms = float(np.sqrt(np.mean(residuals * residuals)))
    return RigidRegistrationResult(
        transform=transform,
        match=final_match,
        inlier_mask=final_inliers,
        iterations=iterations,
        rms_residual_m=rms,
        median_residual_m=float(np.median(residuals)),
        p95_residual_m=float(np.percentile(residuals, 95.0)),
    )
