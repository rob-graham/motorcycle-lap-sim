"""Independent flat-road force and longitudinal load-transfer formulas."""

from dataclasses import dataclass


@dataclass(frozen=True)
class AxleLoads:
    front_n: float
    rear_n: float


def aerodynamic_drag_n(speed_mps: float, air_density_kgpm3: float, cda_m2: float) -> float:
    """Return positive drag magnitude (direction is left to a future solver)."""
    return 0.5 * air_density_kgpm3 * cda_m2 * speed_mps**2


def rolling_resistance_n(crr: float, mass_kg: float, gravity_mps2: float) -> float:
    """Return positive, speed-independent rolling resistance magnitude."""
    return crr * mass_kg * gravity_mps2


def axle_normal_loads_n(mass_kg: float, gravity_mps2: float, wheelbase_m: float,
                        cg_height_m: float, cg_from_rear_m: float,
                        longitudinal_acceleration_mps2: float) -> AxleLoads:
    """Loads for rear x=0/front x=L; positive acceleration is forward.

    Negative loads are deliberately retained as tip-over diagnostics.
    """
    front = (mass_kg * gravity_mps2 * cg_from_rear_m
             - mass_kg * longitudinal_acceleration_mps2 * cg_height_m) / wheelbase_m
    return AxleLoads(front_n=front, rear_n=mass_kg * gravity_mps2 - front)
