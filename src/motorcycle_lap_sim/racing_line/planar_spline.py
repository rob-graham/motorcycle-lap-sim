"""C2-periodic Cartesian racing-line geometry, independent of optimisation."""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from motorcycle_lap_sim.path import SampledPath
from motorcycle_lap_sim.track import Track, sample_track_stations

FloatArray = NDArray[np.float64]
_GL_X, _GL_W = np.polynomial.legendre.leggauss(5)


def _immutable(values: ArrayLike) -> FloatArray:
    result = np.asarray(values, dtype=float).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class PeriodicPlanarSpline:
    """Uniform periodic interpolating cubic parameterised by centreline ``s``.

    The knot second derivatives solve the cyclic system
    ``M[i-1] + 4 M[i] + M[i+1] = 6/h**2 (G[i+1]-2G[i]+G[i-1])``.
    Standard second-derivative cubic pieces then guarantee C2 continuity.
    """

    guide_s_m: FloatArray
    guide_x_m: FloatArray
    guide_y_m: FloatArray
    period_m: float

    def __post_init__(self) -> None:
        s, x, y = (np.asarray(v, dtype=float) for v in
                   (self.guide_s_m, self.guide_x_m, self.guide_y_m))
        if any(v.ndim != 1 for v in (s, x, y)) or not (len(s) == len(x) == len(y)) or len(s) < 4:
            raise ValueError("periodic spline requires at least four equal one-dimensional guide arrays")
        if not math.isfinite(self.period_m) or self.period_m <= 0 or not all(np.all(np.isfinite(v)) for v in (s, x, y)):
            raise ValueError("period and guide points must be finite and period must be positive")
        h = self.period_m / len(s)
        if not np.allclose(s, np.arange(len(s)) * h, rtol=0.0, atol=1e-10 * max(1.0, self.period_m)):
            raise ValueError("guide stations must be unique, uniform, begin at zero, and omit the periodic endpoint")
        matrix = np.eye(len(s)) * 4.0
        indices = np.arange(len(s))
        matrix[indices, (indices - 1) % len(s)] = 1.0
        matrix[indices, (indices + 1) % len(s)] = 1.0
        points = np.column_stack((x, y))
        rhs = 6.0 / h**2 * (np.roll(points, -1, axis=0) - 2 * points + np.roll(points, 1, axis=0))
        second = np.linalg.solve(matrix, rhs)
        for name, value in (("guide_s_m", s), ("guide_x_m", x), ("guide_y_m", y),
                            ("_points", points), ("_second", second)):
            object.__setattr__(self, name, _immutable(value))
        object.__setattr__(self, "_h_m", h)

    def evaluate(self, s_m: ArrayLike) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
        """Return x, y and their first and second analytic derivatives."""
        s = np.asarray(s_m, dtype=float)
        if not np.all(np.isfinite(s)):
            raise ValueError("spline evaluation stations must be finite")
        wrapped = np.mod(s, self.period_m)
        i = np.floor(wrapped / self._h_m).astype(int)
        u = (wrapped - i * self._h_m) / self._h_m
        j = (i + 1) % len(self.guide_s_m)
        a, b = 1.0 - u, u
        p0, p1, m0, m1 = self._points[i], self._points[j], self._second[i], self._second[j]
        point = a[..., None] * p0 + b[..., None] * p1 + self._h_m**2 / 6 * (((a**3-a)[..., None])*m0 + ((b**3-b)[..., None])*m1)
        first = (p1-p0) / self._h_m + self._h_m / 6 * (((1-3*a**2)[..., None])*m0 + ((3*b**2-1)[..., None])*m1)
        second = a[..., None] * m0 + b[..., None] * m1
        return point[..., 0], point[..., 1], first[..., 0], first[..., 1], second[..., 0], second[..., 1]

    def _integrate(self, start: float, end: float) -> float:
        total = 0.0
        cursor = start
        while cursor < end - 1e-14:
            boundary = min(end, (math.floor(cursor / self._h_m + 1e-12) + 1) * self._h_m)
            if boundary <= cursor:
                boundary = end
            mid, half = (cursor + boundary) / 2, (boundary - cursor) / 2
            _, _, dx, dy, _, _ = self.evaluate(mid + half * _GL_X)
            total += half * float(np.dot(_GL_W, np.hypot(dx, dy)))
            cursor = boundary
        return total

    @property
    def total_length_m(self) -> float:
        return self._integrate(0.0, self.period_m)

    def sampled_path(self, sample_spacing_m: float) -> SampledPath:
        if not math.isfinite(sample_spacing_m) or sample_spacing_m <= 0:
            raise ValueError("sample spacing must be finite and positive")
        count = max(3, math.ceil(self.period_m / sample_spacing_m))
        stations = np.arange(count, dtype=float) * self.period_m / count
        x, y, dx, dy, ddx, ddy = self.evaluate(stations)
        speed = np.hypot(dx, dy)
        if np.any(speed <= 1e-10) or not np.all(np.isfinite(speed)):
            raise ValueError("periodic spline has zero or near-zero tangent magnitude")
        curvature = (dx * ddy - dy * ddx) / speed**3
        if not np.all(np.isfinite(curvature)):
            raise ValueError("periodic spline curvature is not finite")
        segment_q = np.array([self._integrate(float(a), float(b)) for a, b in zip(stations[:-1], stations[1:])])
        q = np.r_[0.0, np.cumsum(segment_q)]
        return SampledPath(q, x, y, curvature, self.total_length_m, closed=True)


@dataclass(frozen=True)
class SmoothRacingLineResult:
    sampled_path: SampledPath
    spline: PeriodicPlanarSpline
    guide_s_m: FloatArray
    guide_offset_m: FloatArray
    guide_x_m: FloatArray
    guide_y_m: FloatArray
    evaluated_track_s_m: FloatArray
    projected_offset_m: FloatArray
    tangent_deviation_m: FloatArray
    minimum_boundary_clearance_m: float

    def __post_init__(self) -> None:
        for name in ("guide_s_m", "guide_offset_m", "guide_x_m", "guide_y_m",
                     "evaluated_track_s_m", "projected_offset_m", "tangent_deviation_m"):
            object.__setattr__(self, name, _immutable(getattr(self, name)))


def build_smooth_racing_line_path(track: Track, guide_offsets_m: ArrayLike, *,
                                  sample_spacing_m: float,
                                  boundary_margin_m: float = 0.0,
                                  boundary_check_spacing_m: float = 0.25) -> SmoothRacingLineResult:
    """Build and corridor-check a planar spline using common track parameter ``s``."""
    if not track.closed:
        raise ValueError("periodic planar racing-line geometry requires a closed track")
    offsets = np.asarray(guide_offsets_m, dtype=float)
    if offsets.ndim != 1 or len(offsets) < 4 or not np.all(np.isfinite(offsets)):
        raise ValueError("guide offsets must be a finite one-dimensional array of length at least four")
    if not math.isfinite(boundary_margin_m) or boundary_margin_m < 0:
        raise ValueError("boundary margin must be finite and non-negative")
    if not math.isfinite(boundary_check_spacing_m) or boundary_check_spacing_m <= 0:
        raise ValueError("boundary-check spacing must be finite and positive")
    guide_s = np.arange(len(offsets), dtype=float) * track.total_length_m / len(offsets)
    guide_track = sample_track_stations(track, guide_s)
    left = guide_track.width_left_m - boundary_margin_m
    right = guide_track.width_right_m - boundary_margin_m
    if np.any(left <= 0) or np.any(right <= 0) or np.any(offsets > left) or np.any(offsets < -right):
        raise ValueError("guide offset violates track boundary margin")
    gx = guide_track.x_m + offsets * guide_track.normal_x
    gy = guide_track.y_m + offsets * guide_track.normal_y
    spline = PeriodicPlanarSpline(guide_s, gx, gy, track.total_length_m)
    check_count = max(4, math.ceil(track.total_length_m / boundary_check_spacing_m))
    check_s = np.arange(check_count, dtype=float) * track.total_length_m / check_count
    checked_track = sample_track_stations(track, check_s)
    px, py, *_ = spline.evaluate(check_s)
    delta_x, delta_y = px - checked_track.x_m, py - checked_track.y_m
    projected = delta_x * checked_track.normal_x + delta_y * checked_track.normal_y
    tangent = delta_x * checked_track.tangent_x + delta_y * checked_track.tangent_y
    clearance = np.minimum(checked_track.width_left_m - boundary_margin_m - projected,
                           checked_track.width_right_m - boundary_margin_m + projected)
    if np.min(clearance) < -1e-10:
        raise ValueError(f"planar spline overshoots track boundary between guides (minimum clearance {np.min(clearance):.6g} m)")
    path = spline.sampled_path(sample_spacing_m)
    return SmoothRacingLineResult(path, spline, guide_s, offsets, gx, gy, check_s,
                                  projected, tangent, float(np.min(clearance)))
