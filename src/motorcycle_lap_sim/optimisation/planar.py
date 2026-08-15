"""Phase 8 direct smooth-planar racing-line optimisation.

Controls are physical lateral offsets at geometry-derived stations.  This
module deliberately does not use the Phase 5 dense offset parameterisation.
"""

from dataclasses import dataclass
from concurrent.futures import ProcessPoolExecutor
import math
import multiprocessing
from typing import Callable, Iterable

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


@dataclass(frozen=True)
class _PlanarWorkerContext:
    """Immutable per-process data installed once by the pool initializer."""

    track: Track
    motorcycle: object
    control_s_m: NDArray[np.float64]
    sample_spacing_m: float
    boundary_margin_m: float
    boundary_check_spacing_m: float


_planar_worker_context: _PlanarWorkerContext | None = None


def _initialise_planar_worker(track: Track, motorcycle, control_s_m: NDArray[np.float64],
                              sample_spacing_m: float, boundary_margin_m: float,
                              boundary_check_spacing_m: float) -> None:
    """Install evaluation data in a spawned worker instead of sending it per task."""
    global _planar_worker_context
    _planar_worker_context = _PlanarWorkerContext(
        track, motorcycle, control_s_m, sample_spacing_m, boundary_margin_m,
        boundary_check_spacing_m)


def _evaluate_planar_worker(controls_m: NDArray[np.float64]) -> PlanarObjectiveEvaluation:
    """Spawn-picklable candidate evaluator used by :class:`ProcessPoolExecutor`."""
    context = _planar_worker_context
    if context is None:
        raise RuntimeError("planar worker evaluation context was not initialized")
    return evaluate_planar_racing_line(
        controls_m, context.track, context.motorcycle, context.control_s_m,
        sample_spacing_m=context.sample_spacing_m,
        boundary_margin_m=context.boundary_margin_m,
        boundary_check_spacing_m=context.boundary_check_spacing_m)


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
    parallel_workers: int = 1

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
               for value in (self.max_sweeps, self.max_evaluations,
                              self.parallel_workers)):
            raise ValueError("sweep, evaluation, and worker limits must be positive integers")


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


def _periodic_search_directions(control_count: int) -> tuple[NDArray[np.float64], ...]:
    """Return deterministic unit-peak coordinate and smooth periodic directions."""
    directions: list[NDArray[np.float64]] = []
    for centre in range(control_count):
        coordinate = np.zeros(control_count)
        coordinate[centre] = 1.0
        directions.append(coordinate)
    for weights in ((0.5, 1.0, 0.5), (0.25, 0.5, 1.0, 0.5, 0.25)):
        radius = len(weights) // 2
        for centre in range(control_count):
            bump = np.zeros(control_count)
            # Addition, rather than assignment, also defines small periodic
            # control sets where a kernel wraps onto the same control twice.
            for offset, weight in enumerate(weights, start=-radius):
                bump[(centre + offset) % control_count] += weight
            bump /= np.max(bump)
            directions.append(bump)
    return tuple(directions)


def _best_improvement_pattern_search(
        initial_controls: NDArray[np.float64], lower: NDArray[np.float64],
        upper: NDArray[np.float64], initial_evaluation, evaluate: Callable,
        config: PlanarOptimisationConfig,
        evaluate_candidates: Callable[[Iterable[NDArray[np.float64]]],
                                      Iterable[PlanarObjectiveEvaluation]] | None = None):
    """Poll all coordinate/coupled moves and accept only the best candidate.

    Direction and sign order provide deterministic tie-breaking.  A single
    same-direction pattern move is tried after each successful full poll.
    """
    controls = initial_controls.copy()
    best = initial_evaluation
    evaluations, polls, step = 1, 0, config.initial_step_m
    reason = "maximum sweeps reached"
    directions = _periodic_search_directions(len(controls))
    while polls < config.max_sweeps:
        if evaluations >= config.max_evaluations:
            reason = "maximum evaluations reached"
            break
        candidate_controls = []
        for direction_order, direction in enumerate(directions):
            for sign_order, sign in enumerate((1.0, -1.0)):
                candidate = np.clip(controls + sign * step * direction, lower, upper)
                if np.array_equal(candidate, controls):
                    continue
                candidate_controls.append(
                    (direction_order, sign_order, candidate, sign * direction))
        # Do not evaluate or choose a search-order-dependent prefix of a poll.
        if evaluations + len(candidate_controls) > config.max_evaluations:
            reason = "maximum evaluations reached"
            break
        if evaluate_candidates is None:
            poll_evaluations = [evaluate(item[2]) for item in candidate_controls]
        else:
            # Executor.map returns in input order.  Materialising it also makes
            # worker exceptions propagate before deterministic selection.
            poll_evaluations = list(evaluate_candidates(
                item[2] for item in candidate_controls))
        if len(poll_evaluations) != len(candidate_controls):
            raise RuntimeError("candidate evaluator returned an unexpected result count")
        evaluations += len(candidate_controls)
        candidates = []
        for ((direction_order, sign_order, candidate, signed_direction),
             evaluation) in zip(candidate_controls, poll_evaluations):
            candidates.append((evaluation.lap_time_s, direction_order, sign_order,
                               candidate, evaluation, signed_direction))
        polls += 1
        improving = [item for item in candidates if item[4].feasible and
                     item[0] < best.lap_time_s - config.lap_time_improvement_tolerance_s]
        if improving:
            _, _, _, controls, best, accepted_direction = min(
                improving, key=lambda item: (item[0], item[1], item[2]))
            # A deliberately modest pattern move: clipping is component-wise,
            # and a fully clipped/no-change move consumes no evaluation.
            pattern = np.clip(controls + step * accepted_direction, lower, upper)
            if not np.array_equal(pattern, controls) and evaluations < config.max_evaluations:
                pattern_evaluation = evaluate(pattern)
                evaluations += 1
                if (pattern_evaluation.feasible and pattern_evaluation.lap_time_s
                        < best.lap_time_s - config.lap_time_improvement_tolerance_s):
                    controls, best = pattern, pattern_evaluation
        else:
            step *= config.step_reduction
            if step < config.minimum_step_m:
                reason = "minimum step reached"
                break
        if evaluations >= config.max_evaluations:
            reason = "maximum evaluations reached"
            break
    return controls, best, evaluations, polls, step, reason


def optimise_planar_racing_line(track: Track, motorcycle,
                                policy: PlanarControlStationPolicy = REFERENCE_PLANAR_CONTROL_POLICY,
                                config: PlanarOptimisationConfig = PlanarOptimisationConfig(),
                                initial_controls_m: ArrayLike | None = None,
                                ) -> PlanarOptimisationResult:
    """Optimise physical controls, optionally resuming from a supplied candidate.

    The default initial candidate remains the centreline (all zero controls).
    Supplied controls must be finite, have one value per generated station, and
    lie inside the local usable-track bounds.
    """
    control_s = generate_planar_control_stations(track, policy)
    lower, upper = planar_control_bounds(track, control_s, config.boundary_margin_m)
    if initial_controls_m is None:
        controls = np.zeros(len(control_s))
    else:
        controls = np.asarray(initial_controls_m, dtype=float)
        if controls.shape != (len(control_s),):
            raise ValueError("initial controls must have one value per control station")
        if not np.all(np.isfinite(controls)):
            raise ValueError("initial controls must be finite")
        if np.any(controls < lower) or np.any(controls > upper):
            raise ValueError("initial controls must lie within their local bounds")
        controls = controls.copy()
    initial_controls = controls.copy()
    kwargs = dict(sample_spacing_m=config.optimisation_sample_spacing_m,
                  boundary_margin_m=config.boundary_margin_m,
                  boundary_check_spacing_m=config.boundary_check_spacing_m)
    initial = evaluate_planar_racing_line(controls, track, motorcycle, control_s, **kwargs)
    if not initial.feasible:
        raise ValueError(f"initial planar candidate is infeasible: {initial.failure_reason}")
    def evaluate(candidate):
        return evaluate_planar_racing_line(candidate, track, motorcycle, control_s, **kwargs)

    if config.parallel_workers == 1:
        controls, best, evaluations, sweeps, step, reason = _best_improvement_pattern_search(
            controls, lower, upper, initial, evaluate, config)
    else:
        # One persistent pool serves every complete poll.  Pattern moves remain
        # single parent-process evaluations because they are not parallel work.
        with ProcessPoolExecutor(
                max_workers=config.parallel_workers,
                mp_context=multiprocessing.get_context("spawn"),
                initializer=_initialise_planar_worker,
                initargs=(track, motorcycle, control_s,
                          config.optimisation_sample_spacing_m,
                          config.boundary_margin_m,
                          config.boundary_check_spacing_m)) as executor:
            controls, best, evaluations, sweeps, step, reason = (
                _best_improvement_pattern_search(
                    controls, lower, upper, initial, evaluate, config,
                    lambda candidates: executor.map(
                        _evaluate_planar_worker, candidates)))
    improvement = initial.lap_time_s - best.lap_time_s
    assert best.smooth_line is not None and best.speed_profile is not None
    return PlanarOptimisationResult(control_s, initial_controls, controls, lower, upper,
        initial.lap_time_s, best.lap_time_s, improvement, 100 * improvement / initial.lap_time_s,
        best.smooth_line, best.smooth_line.sampled_path, best.speed_profile, evaluations, sweeps,
        step, reason, best.smooth_line.minimum_boundary_clearance_m,
        best.smooth_line.minimum_forward_progress)


def resample_planar_result(result: PlanarOptimisationResult, motorcycle,
                           sample_spacing_m: float):
    """Solve a new output grid on the exact saved spline (without refitting)."""
    path = result.smooth_line.spline.sampled_path(sample_spacing_m)
    return path, solve_speed_profile(path, motorcycle)
