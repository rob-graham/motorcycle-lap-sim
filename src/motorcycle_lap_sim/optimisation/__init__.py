"""Numerically optimised minimum-lap-time racing lines."""

from .objective import evaluate_racing_line
from .optimiser import OptimisationConfig, optimise_racing_line
from .parameterisation import PeriodicCubicParameterisation
from .results import ObjectiveEvaluation, OptimisationResult
from .planar import (COARSE_PLANAR_CONTROL_POLICY, FINE_PLANAR_CONTROL_POLICY,
                     REFERENCE_PLANAR_CONTROL_POLICY, PlanarControlStationPolicy,
                     PlanarObjectiveEvaluation, PlanarOptimisationConfig,
                     PlanarOptimisationProgress,
                     PlanarOptimisationResult, evaluate_planar_racing_line,
                     generate_planar_control_stations, optimise_planar_racing_line,
                     planar_control_bounds, resample_planar_result)

__all__ = ["ObjectiveEvaluation", "OptimisationConfig", "OptimisationResult",
           "PeriodicCubicParameterisation", "evaluate_racing_line", "optimise_racing_line"]

__all__ += ["COARSE_PLANAR_CONTROL_POLICY", "FINE_PLANAR_CONTROL_POLICY",
            "REFERENCE_PLANAR_CONTROL_POLICY", "PlanarControlStationPolicy",
            "PlanarObjectiveEvaluation", "PlanarOptimisationConfig", "PlanarOptimisationProgress",
            "PlanarOptimisationResult",
            "evaluate_planar_racing_line", "generate_planar_control_stations",
            "optimise_planar_racing_line", "planar_control_bounds", "resample_planar_result"]
