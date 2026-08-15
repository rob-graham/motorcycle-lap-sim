"""Phase 8 direct smooth-planar racing-line optimisation.

Controls are physical lateral offsets at geometry-derived stations.  This
module deliberately does not use the Phase 5 dense offset parameterisation.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from motorcycle_lap_sim.racing_line import SmoothRacingLineResult, build_smooth_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import CircularArc, Track, sample_track_stations
from .results import immutable_array


@dataclass(frozen=True)
class PlanarControlStationPolicy:
    """Maximum centreline distance and arc heading change between controls."""

    max_spacing_m: float
    max_arc_heading_change_rad: float

    def __post_init__(self) -> None:
        if (not math.isfinite(self.max_spacing_m) or self.max_spacing_m <= 0
                or not math.isfinite(self.max_arc_heading_change_rad)
                or self.max_arc_heading_change_rad <= 0):
            raise ValueError("control-station policy limits must be finite and positive")


COARSE_PLANAR_CONTROL_POLICY = PlanarControlStationPolicy(150.0, math.radians(60.0))
REFERENCE_PLANAR_CONTROL_POLICY = PlanarControlStationPolicy(100.0, math.radians(45.0))
FINE_PLANAR_CONTROL_POLICY = PlanarControlStationPolicy(75.0, math.radians(30.0))


def generate_planar_control_stations(track: Track,
                                     policy: PlanarControlStationPolicy) -> NDArray[np.float64]:
    """Generate starts of primitive-local subdivisions, omitting closed endpoint."""
    stations: list[float] = []
    start = 0.0
    for primitive in track.primitives:
        subdivisions = max(1, math.ceil(primitive.length_m / policy.max_spacing_m))
        if isinstance(primitive, CircularArc):
            subdivisions = max(subdivisions, math.ceil(
                abs(primitive.turn_angle_rad) / policy.max_arc_heading_change_rad))
        stations.extend(start + j * primitive.length_m / subdivisions
                        for j in range(subdivisions))
        start += primitive.length_m
    return immutable_array(stations)


def planar_control_bounds(track: Track, control_s_m: ArrayLike,
                          boundary_margin_m: float = 0.25
                          ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    if not math.isfinite(boundary_margin_m) or boundary_margin_m < 0:
        raise ValueError("boundary margin must be finite and non-negative")
    sampled = sample_track_stations(track, control_s_m)
    lower = -(sampled.width_right_m - boundary_margin_m)
    upper = sampled.width_left_m - boundary_margin_m
    if np.any(lower >= upper) or np.any(upper <= 0) or np.any(lower >= 0):
        raise ValueError("boundary margin leaves no usable track corridor")
    return immutable_array(lower), immutable_array(upper)


@dataclass(frozen=True)
class PlanarObjectiveEvaluation:
    feasible: bool
    lap_time_s: float
    smooth_line: SmoothRacingLineResult | None = None
    speed_profile: object | None = None
    failure_reason: str | None = None


def evaluate_planar_racing_line(controls_m: ArrayLike, track: Track, motorcycle,
                                control_s_m: ArrayLike, *, sample_spacing_m: float = 1.0,
                                boundary_margin_m: float = 0.25,
                                boundary_check_spacing_m: float = 0.25
                                ) -> PlanarObjectiveEvaluation:
    """Evaluate a direct planar candidate; known geometric invalidity is infeasible."""
    try:
        smooth = build_smooth_racing_line_path(
            track, controls_m, guide_s_m=control_s_m,
            sample_spacing_m=sample_spacing_m, boundary_margin_m=boundary_margin_m,
            boundary_check_spacing_m=boundary_check_spacing_m)
        speed = solve_speed_profile(smooth.sampled_path, motorcycle)
    except (ValueError, RuntimeError, FloatingPointError) as error:
        return PlanarObjectiveEvaluation(False, math.inf,
                                         failure_reason=f"{type(error).__name__}: {error}")
    return PlanarObjectiveEvaluation(True, speed.lap_time_s, smooth, speed)


@dataclass(frozen=True)
class PlanarOptimisationConfig:
    initial_step_m: float = 1.0
    minimum_step_m: float = 0.0625
    step_reduction: float = 0.5
    lap_time_improvement_tolerance_s: float = 1e-6
    max_sweeps: int = 30
    max_evaluations: int = 1500
    boundary_margin_m: float = 0.25
    boundary_check_spacing_m: float = 0.25
    optimisation_sample_spacing_m: float = 1.0

    def __post_init__(self) -> None:
        positive = (self.initial_step_m, self.minimum_step_m,
                    self.boundary_check_spacing_m, self.optimisation_sample_spacing_m)
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("Phase 8 step and spacing values must be finite and positive")
        if not math.isfinite(self.step_reduction) or not 0 < self.step_reduction < 1:
            raise ValueError("step reduction must be between zero and one")
        if (not math.isfinite(self.lap_time_improvement_tolerance_s)
                or self.lap_time_improvement_tolerance_s < 0):
            raise ValueError("improvement tolerance must be finite and non-negative")
        if not math.isfinite(self.boundary_margin_m) or self.boundary_margin_m < 0:
            raise ValueError("boundary margin must be finite and non-negative")
        if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
               for value in (self.max_sweeps, self.max_evaluations)):
            raise ValueError("sweep and evaluation limits must be positive integers")


@dataclass(frozen=True)
class PlanarOptimisationResult:
    control_s_m: NDArray[np.float64]
    initial_controls_m: NDArray[np.float64]
    best_controls_m: NDArray[np.float64]
    lower_bounds_m: NDArray[np.float64]
    upper_bounds_m: NDArray[np.float64]
    initial_lap_time_s: float
    best_lap_time_s: float
    improvement_s: float
    improvement_percent: float
    smooth_line: SmoothRacingLineResult
    sampled_path: object
    speed_profile: object
    evaluations: int
    sweeps: int
    final_step_m: float
    termination_reason: str
    minimum_boundary_clearance_m: float
    minimum_forward_progress: float

    def __post_init__(self) -> None:
        for name in ("control_s_m", "initial_controls_m", "best_controls_m",
                     "lower_bounds_m", "upper_bounds_m"):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))


def optimise_planar_racing_line(track: Track, motorcycle,
                                policy: PlanarControlStationPolicy = REFERENCE_PLANAR_CONTROL_POLICY,
                                config: PlanarOptimisationConfig = PlanarOptimisationConfig()
                                ) -> PlanarOptimisationResult:
    control_s = generate_planar_control_stations(track, policy)
    lower, upper = planar_control_bounds(track, control_s, config.boundary_margin_m)
    controls = np.zeros(len(control_s))
    kwargs = dict(sample_spacing_m=config.optimisation_sample_spacing_m,
                  boundary_margin_m=config.boundary_margin_m,
                  boundary_check_spacing_m=config.boundary_check_spacing_m)
    initial = evaluate_planar_racing_line(controls, track, motorcycle, control_s, **kwargs)
    if not initial.feasible:
        raise ValueError(f"zero-control planar baseline is infeasible: {initial.failure_reason}")
    best = initial
    evaluations, sweeps, step = 1, 0, config.initial_step_m
    reason = "maximum sweeps reached"
    while sweeps < config.max_sweeps:
        if evaluations >= config.max_evaluations:
            reason = "maximum evaluations reached"
            break
        accepted = False
        for index in range(len(controls)):
            candidates = []
            for order, direction in enumerate((1.0, -1.0)):
                candidate = controls.copy()
                candidate[index] = np.clip(candidate[index] + direction * step,
                                           lower[index], upper[index])
                if candidate[index] == controls[index]:
                    continue
                if evaluations >= config.max_evaluations:
                    break
                evaluation = evaluate_planar_racing_line(candidate, track, motorcycle,
                                                         control_s, **kwargs)
                evaluations += 1
                candidates.append((evaluation.lap_time_s, order, candidate, evaluation))
            if candidates:
                _, _, candidate, evaluation = min(candidates, key=lambda item: (item[0], item[1]))
                if (evaluation.feasible and evaluation.lap_time_s
                        < best.lap_time_s - config.lap_time_improvement_tolerance_s):
                    controls, best, accepted = candidate, evaluation, True
            if evaluations >= config.max_evaluations:
                reason = "maximum evaluations reached"
                break
        sweeps += 1
        if evaluations >= config.max_evaluations:
            break
        if not accepted:
            step *= config.step_reduction
            if step < config.minimum_step_m:
                reason = "minimum step reached"
                break
    improvement = initial.lap_time_s - best.lap_time_s
    assert best.smooth_line is not None and best.speed_profile is not None
    return PlanarOptimisationResult(control_s, np.zeros(len(control_s)), controls, lower, upper,
        initial.lap_time_s, best.lap_time_s, improvement, 100 * improvement / initial.lap_time_s,
        best.smooth_line, best.smooth_line.sampled_path, best.speed_profile, evaluations, sweeps,
        step, reason, best.smooth_line.minimum_boundary_clearance_m,
        best.smooth_line.minimum_forward_progress)


def resample_planar_result(result: PlanarOptimisationResult, motorcycle,
                           sample_spacing_m: float):
    """Solve a new output grid on the exact saved spline (without refitting)."""
    path = result.smooth_line.spline.sampled_path(sample_spacing_m)
    return path, solve_speed_profile(path, motorcycle)
