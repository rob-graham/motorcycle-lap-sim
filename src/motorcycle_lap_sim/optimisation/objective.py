"""Pure lap-time objective composition for racing-line candidates."""

import math
from numpy.typing import ArrayLike
from motorcycle_lap_sim.racing_line import LateralOffsetProfile, build_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from .results import ObjectiveEvaluation, immutable_array


def evaluate_racing_line(controls: ArrayLike, sampled_track, motorcycle,
                         parameterisation, boundary_margin_m: float) -> ObjectiveEvaluation:
    """Build and solve one candidate; known numerical invalidity is infeasible."""
    try:
        offsets = parameterisation.offsets(controls, sampled_track, boundary_margin_m)
        profile = LateralOffsetProfile(sampled_track, offsets, boundary_margin_m)
        path = build_racing_line_path(sampled_track, profile)
        speed = solve_speed_profile(path, motorcycle)
    except (ValueError, RuntimeError, FloatingPointError) as error:
        return ObjectiveEvaluation(False, math.inf, failure_reason=f"{type(error).__name__}: {error}")
    return ObjectiveEvaluation(True, speed.lap_time_s, immutable_array(offsets), path, speed)
