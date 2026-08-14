"""Fixed-centreline, periodic speed-profile calculation."""

from .capability import GearSelection, NumericalConfig, best_gear
from .solver import SpeedProfile, solve_fixed_path

__all__ = ["GearSelection", "NumericalConfig", "SpeedProfile", "best_gear", "solve_fixed_path"]
