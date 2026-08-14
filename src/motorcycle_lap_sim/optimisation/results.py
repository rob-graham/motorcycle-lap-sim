"""Immutable objective and optimisation result records."""

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray


def immutable_array(values) -> NDArray[np.float64]:
    result = np.asarray(values, dtype=float).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ObjectiveEvaluation:
    feasible: bool
    lap_time_s: float
    dense_offset_m: NDArray[np.float64] | None = None
    sampled_path: object | None = None
    speed_profile: object | None = None
    failure_reason: str | None = None


@dataclass(frozen=True)
class OptimisationResult:
    initial_controls: NDArray[np.float64]
    best_controls: NDArray[np.float64]
    initial_lap_time_s: float
    best_lap_time_s: float
    improvement_s: float
    improvement_percent: float
    dense_offset_m: NDArray[np.float64]
    sampled_path: object
    speed_profile: object
    evaluations: int
    sweeps: int
    final_step: float
    converged: bool
    termination_reason: str

    def __post_init__(self) -> None:
        for name in ("initial_controls", "best_controls", "dense_offset_m"):
            object.__setattr__(self, name, immutable_array(getattr(self, name)))
