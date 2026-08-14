"""Fixed-path periodic minimum-time speed solver."""
from .capabilities import (NumericalConfig, best_gear, braking_capability,
                           forward_acceleration_capability, lateral_speed_limit_mps,
                           maximum_rev_limited_speed_mps, road_speed_at_rpm_mps)
from .solver import SolverConfig, solve_speed_profile
from .results import SpeedProfileResult

__all__ = ["NumericalConfig", "SolverConfig", "SpeedProfileResult", "best_gear",
           "braking_capability", "forward_acceleration_capability", "lateral_speed_limit_mps",
           "maximum_rev_limited_speed_mps", "road_speed_at_rpm_mps", "solve_speed_profile"]
