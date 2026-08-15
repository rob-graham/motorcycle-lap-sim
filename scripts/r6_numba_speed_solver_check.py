"""Time and compare Python and warmed Numba on a Phase 8-like fixed path."""

import argparse
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY, generate_planar_control_stations,
)
from motorcycle_lap_sim.racing_line import build_smooth_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.speed_solver.numba_backend import solve_speed_profile_numba
from motorcycle_lap_sim.track import Track


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args(argv)
    if args.repeats <= 0:
        parser.error("--repeats must be positive")
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    track = Track.from_yaml("examples/tracks/mallala_reference.yaml")
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    path = build_smooth_racing_line_path(
        track, np.zeros(stations.size), guide_s_m=stations,
        sample_spacing_m=1.0, boundary_margin_m=0.25,
        boundary_check_spacing_m=0.25).sampled_path

    started = perf_counter()
    python_result = solve_speed_profile(path, bike)
    python_first_elapsed = perf_counter() - started
    started = perf_counter()
    numba_result = solve_speed_profile_numba(path, bike)
    cold_elapsed = perf_counter() - started

    started = perf_counter()
    for _ in range(args.repeats):
        python_result = solve_speed_profile(path, bike)
    python_elapsed = (perf_counter() - started) / args.repeats
    started = perf_counter()
    for _ in range(args.repeats):
        numba_result = solve_speed_profile_numba(path, bike)
    numba_elapsed = (perf_counter() - started) / args.repeats

    print(f"python_lap_s={python_result.lap_time_s:.12f}")
    print(f"numba_lap_s={numba_result.lap_time_s:.12f}")
    print(f"lap_difference_s={numba_result.lap_time_s-python_result.lap_time_s:.15g}")
    print(f"max_abs_speed_difference_mps={np.max(np.abs(numba_result.speed_mps-python_result.speed_mps)):.15g}")
    print(f"python_iterations={python_result.iterations}")
    print(f"numba_iterations={numba_result.iterations}")
    print(f"cold_numba_elapsed_s={cold_elapsed:.6f}")
    print(f"python_first_elapsed_s={python_first_elapsed:.6f}")
    print(f"python_warm_elapsed_s={python_elapsed:.6f}")
    print(f"numba_warm_elapsed_s={numba_elapsed:.6f}")
    print(f"warm_speedup={python_elapsed/numba_elapsed:.3f}")


if __name__ == "__main__":
    main()
