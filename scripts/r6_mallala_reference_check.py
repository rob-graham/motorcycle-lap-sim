"""Reproducible Mallala v0.3 centreline/R6 diagnostic (no line optimisation)."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.path import from_sampled_track
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import Track, sample_track
from motorcycle_lap_sim.track.boundaries import calculate_boundaries

TRACK_PATH = Path("examples/tracks/mallala_reference.yaml")
BIKE_PATH = Path("examples/motorcycles/r6_2017plus_reference.yaml")


def main() -> None:
    track = Track.from_yaml(TRACK_PATH)
    bike = load_motorcycle_config(BIKE_PATH)
    closure = track.closure_diagnostic()
    print(f"track_length_m={track.total_length_m:.9f}")
    print(f"closure_position_error_m={closure.position_error_m:.12e}")
    print(f"closure_heading_error_deg={np.degrees(closure.heading_error_rad):.12e}")
    print("spacing_m,samples,lap_time_s,min_speed_mps,max_speed_mps,mean_speed_mps,"
          "min_curvature_1pm,max_curvature_1pm,gears,min_rpm,max_rpm,"
          "max_lateral_acceleration_mps2,max_forward_acceleration_mps2,"
          "max_braking_deceleration_mps2")
    lap_times = []
    for spacing in (2.0, 1.0, 0.5):
        samples = sample_track(track, spacing)
        result = solve_speed_profile(from_sampled_track(samples), bike)
        lap_times.append(result.lap_time_s)
        gears = "/".join(str(value) for value in np.unique(result.gear_number))
        print(f"{spacing:.1f},{len(samples.s_m)},{result.lap_time_s:.9f},"
              f"{np.min(result.speed_mps):.9f},{np.max(result.speed_mps):.9f},"
              f"{np.mean(result.speed_mps):.9f},{np.min(samples.curvature_1pm):.9f},"
              f"{np.max(samples.curvature_1pm):.9f},{gears},"
              f"{np.min(result.engine_rpm):.3f},{np.max(result.engine_rpm):.3f},"
              f"{np.max(np.abs(result.lateral_acceleration_mps2)):.9f},"
              f"{np.max(result.longitudinal_acceleration_mps2):.9f},"
              f"{-np.min(result.longitudinal_acceleration_mps2):.9f}")
    print(f"lap_delta_1.0_minus_2.0_s={lap_times[1] - lap_times[0]:.9f}")
    print(f"lap_delta_0.5_minus_1.0_s={lap_times[2] - lap_times[1]:.9f}")

    samples = sample_track(track, 0.5)
    boundaries = calculate_boundaries(samples)
    fig, ax = plt.subplots()
    ax.plot(samples.x_m, samples.y_m, "--", label="centreline")
    ax.plot(boundaries.left_x_m, boundaries.left_y_m, label="left boundary")
    ax.plot(boundaries.right_x_m, boundaries.right_y_m, label="right boundary")
    ax.plot(samples.x_m[0], samples.y_m[0], "o", label="start/finish")
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlabel="x [m]", ylabel="y [m]", title="Mallala reference centreline v0.3")
    ax.legend()
    output = Path("mallala_reference.png")
    fig.savefig(output, dpi=160, bbox_inches="tight")
    print(f"plot={output}")


if __name__ == "__main__":
    main()
