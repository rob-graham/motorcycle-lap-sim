"""Deterministic bounded coordinate pattern search."""

from dataclasses import dataclass
import math
import numpy as np
from .objective import evaluate_racing_line
from .parameterisation import PeriodicCubicParameterisation
from .results import OptimisationResult


@dataclass(frozen=True)
class OptimisationConfig:
    control_count: int = 12
    control_bound: float = 4.0
    initial_step: float = 1.0
    minimum_step: float = 0.0625
    step_reduction: float = 0.5
    lap_time_improvement_tolerance_s: float = 1e-6
    max_sweeps: int = 30
    max_evaluations: int = 500
    boundary_margin_m: float = 0.25

    def __post_init__(self) -> None:
        PeriodicCubicParameterisation(self.control_count)
        positive = ("control_bound", "initial_step", "minimum_step")
        if any(not math.isfinite(getattr(self, n)) or getattr(self, n) <= 0 for n in positive):
            raise ValueError("control bound and step sizes must be finite and positive")
        if self.initial_step > 2 * self.control_bound:
            raise ValueError("initial_step must not exceed the full bounded interval")
        if not math.isfinite(self.step_reduction) or not 0 < self.step_reduction < 1:
            raise ValueError("step_reduction must be between zero and one")
        if (not math.isfinite(self.lap_time_improvement_tolerance_s)
                or self.lap_time_improvement_tolerance_s < 0):
            raise ValueError("improvement tolerance must be finite and non-negative")
        if (isinstance(self.max_sweeps, bool) or not isinstance(self.max_sweeps, int)
                or self.max_sweeps <= 0):
            raise ValueError("max_sweeps must be a positive integer")
        if (isinstance(self.max_evaluations, bool) or not isinstance(self.max_evaluations, int)
                or self.max_evaluations <= 0):
            raise ValueError("max_evaluations must be a positive integer")
        if not math.isfinite(self.boundary_margin_m) or self.boundary_margin_m < 0:
            raise ValueError("boundary margin must be finite and non-negative")


def optimise_racing_line(sampled_track, motorcycle,
                         config: OptimisationConfig = OptimisationConfig()) -> OptimisationResult:
    parameterisation = PeriodicCubicParameterisation(config.control_count)
    controls = np.zeros(config.control_count)
    initial = evaluate_racing_line(controls, sampled_track, motorcycle, parameterisation,
                                   config.boundary_margin_m)
    if not initial.feasible:
        raise ValueError(f"zero-control baseline is infeasible: {initial.failure_reason}")
    best = initial
    evaluations, sweeps, step = 1, 0, config.initial_step
    reason = "maximum sweeps reached"
    while sweeps < config.max_sweeps:
        if evaluations >= config.max_evaluations:
            reason = "maximum evaluations reached"; break
        accepted = False
        for index in range(config.control_count):
            candidates = []
            for direction in (1.0, -1.0):
                candidate_controls = controls.copy()
                candidate_controls[index] = np.clip(candidate_controls[index] + direction * step,
                                                     -config.control_bound, config.control_bound)
                if candidate_controls[index] == controls[index]:
                    continue
                if evaluations >= config.max_evaluations:
                    break
                evaluation = evaluate_racing_line(candidate_controls, sampled_track, motorcycle,
                                                  parameterisation, config.boundary_margin_m)
                evaluations += 1
                candidates.append((evaluation.lap_time_s, 0 if direction > 0 else 1,
                                   candidate_controls, evaluation))
            if candidates:
                _, _, candidate_controls, evaluation = min(candidates, key=lambda item: (item[0], item[1]))
                if evaluation.feasible and evaluation.lap_time_s < best.lap_time_s - config.lap_time_improvement_tolerance_s:
                    controls, best, accepted = candidate_controls, evaluation, True
            if evaluations >= config.max_evaluations:
                reason = "maximum evaluations reached"; break
        sweeps += 1
        if evaluations >= config.max_evaluations:
            break
        if not accepted:
            step *= config.step_reduction
            if step < config.minimum_step:
                reason = "minimum step reached"; break
    improvement = initial.lap_time_s - best.lap_time_s
    return OptimisationResult(np.zeros(config.control_count), controls, initial.lap_time_s,
        best.lap_time_s, improvement, 100 * improvement / initial.lap_time_s,
        best.dense_offset_m, best.sampled_path, best.speed_profile, evaluations, sweeps, step,
        reason == "minimum step reached", reason)
