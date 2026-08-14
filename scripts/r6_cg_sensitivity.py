"""Reproduce the deterministic R6 CG sensitivity table."""

from dataclasses import replace
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.motorcycle.limits import (
    stoppie_deceleration_mps2,
    wheelie_acceleration_mps2,
)
from motorcycle_lap_sim.path.samples import from_sampled_track
from motorcycle_lap_sim.speed_solver.solver import solve_speed_profile
from motorcycle_lap_sim.track.sampling import sample_track
from motorcycle_lap_sim.track.track import Track


CASES = (
    ("Baseline", 0.625, 0.625),
    ("Lower CG", 0.550, 0.625),
    ("Higher CG", 0.700, 0.625),
    ("Rearward CG", 0.625, 0.575),
    ("Forward CG", 0.625, 0.675),
)


def main() -> None:
    """Load the reference inputs and print the five documented cases."""
    bike = load_motorcycle_config(
        ROOT / "examples/motorcycles/r6_2017plus_reference.yaml"
    )
    sampled_track = sample_track(
        Track.from_yaml(ROOT / "examples/tracks/test_oval.yaml"), spacing_m=1.0
    )
    path = from_sampled_track(sampled_track)

    print(
        "Case           CG height  CG from rear  Wheelie limit       "
        "Stoppie limit       Lap time    Iterations  Converged"
    )
    for name, cg_height_m, cg_from_rear_m in CASES:
        geometry = replace(
            bike.motorcycle,
            cg_height_m=cg_height_m,
            cg_from_rear_m=cg_from_rear_m,
        )
        case_bike = replace(bike, motorcycle=geometry)
        gravity = case_bike.environment.gravity_mps2
        wheelie = wheelie_acceleration_mps2(
            gravity, geometry.cg_from_rear_m, geometry.cg_height_m
        )
        stoppie = stoppie_deceleration_mps2(
            gravity,
            geometry.wheelbase_m,
            geometry.cg_from_rear_m,
            geometry.cg_height_m,
        )
        result = solve_speed_profile(path, case_bike)
        print(
            f"{name:<14} {cg_height_m:>9.3f} m {cg_from_rear_m:>11.3f} m  "
            f"{wheelie:>6.3f} m/s² ({wheelie / gravity:.3f} g)  "
            f"{stoppie:>6.3f} m/s² ({stoppie / gravity:.3f} g)  "
            f"{result.lap_time_s:>9.6f} s  {result.iterations:>10}  "
            f"{result.converged}"
        )


if __name__ == "__main__":
    main()
