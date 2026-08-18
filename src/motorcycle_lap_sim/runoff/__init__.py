"""Versioned simulator-to-run-off engineering interface.

This package exports simulator-derived trajectory facts and traceable candidate
departure seeds.  It deliberately does not implement run-off physics or turn
coaching landmarks into safety criteria.
"""

from .interface import (
    RUNOFF_INTERFACE_VERSION,
    DepartureSeed,
    RunoffInputPackage,
    build_departure_seeds,
    build_runoff_input_package,
    derive_closed_path_heading_rad,
)
from .export import (
    DEPARTURE_SEED_FIELDS,
    RUNOFF_BUNDLE_VERSION,
    serialize_runoff_bundle,
    write_runoff_bundle,
)

__all__ = [
    "RUNOFF_INTERFACE_VERSION",
    "DepartureSeed",
    "RunoffInputPackage",
    "build_departure_seeds",
    "build_runoff_input_package",
    "derive_closed_path_heading_rad",
    "DEPARTURE_SEED_FIELDS",
    "RUNOFF_BUNDLE_VERSION",
    "serialize_runoff_bundle",
    "write_runoff_bundle",
]
