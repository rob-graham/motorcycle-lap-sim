"""Reproducible Phase 7 comparisons; this script does not optimise controls."""

from dataclasses import replace
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import PeriodicCubicParameterisation, evaluate_racing_line
from motorcycle_lap_sim.optimisation.reference import PHASE5_DETERMINISTIC_LOCAL_REFERENCE
from motorcycle_lap_sim.racing_line import build_smooth_racing_line_path
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import Track, sample_track, sample_track_stations
from motorcycle_lap_sim.track.boundaries import calculate_boundaries


def _guide_offsets(track: Track, count: int) -> np.ndarray:
    stations = np.arange(count) * track.total_length_m / count
    samples = sample_track_stations(track, stations)
    return PeriodicCubicParameterisation(12).offsets(
        PHASE5_DETERMINISTIC_LOCAL_REFERENCE, samples, 0.25)


def _metrics(path, profile) -> tuple[float, ...]:
    return (path.total_length_m, profile.lap_time_s, np.min(path.curvature_1pm),
            np.max(path.curvature_1pm), np.max(np.abs(profile.curvature_gradient_1pm2)),
            np.max(np.abs(profile.curvature_rate_1pmps)))


def main() -> None:
    track = Track.from_yaml("examples/tracks/test_oval.yaml")
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    print("Phase 5 deterministic local reference")
    print("guide_count,clearance_m,max_tangent_m,length_m,kappa_min,kappa_max,max_dk_dq,lap_s")
    results = {}
    for count in (24, 36, 48):
        result = build_smooth_racing_line_path(track, _guide_offsets(track, count),
                                               sample_spacing_m=0.5,
                                               boundary_margin_m=0.25,
                                               boundary_check_spacing_m=0.25)
        profile = solve_speed_profile(result.sampled_path, bike)
        m = _metrics(result.sampled_path, profile)
        print(f"{count},{result.minimum_boundary_clearance_m:.9f},"
              f"{np.max(np.abs(result.tangent_deviation_m)):.9f},"
              f"{m[0]:.9f},{m[2]:.9f},{m[3]:.9f},{m[4]:.9f},{m[1]:.9f}")
        results[count] = result

    spline = results[48].spline  # Constructed once; only output evaluation changes below.
    print("\nmode,spacing_m,length_m,lap_s,kappa_min,kappa_max,max_dk_dq,max_dk_dt")
    for label, candidate in (("disabled", bike),
                             ("experimental_0.8", replace(bike, handling=HandlingConfig(0.8)))):
        for spacing in (1.0, 0.5, 0.25):
            path = spline.sampled_path(spacing)
            profile = solve_speed_profile(path, candidate)
            print(label, spacing, *[f"{v:.9f}" for v in _metrics(path, profile)], sep=",")

    print("\nold_vs_smooth,spacing_m,old_lap_s,smooth_lap_s,delta_s,delta_percent,old_length_m,smooth_length_m,old_peak_kappa,smooth_peak_kappa,old_peak_dk_dq,smooth_peak_dk_dq")
    for spacing in (1.0, 0.5, 0.25):
        old = evaluate_racing_line(PHASE5_DETERMINISTIC_LOCAL_REFERENCE,
                                   sample_track(track, spacing), bike,
                                   PeriodicCubicParameterisation(12), 0.25)
        smooth_path = spline.sampled_path(spacing)
        smooth_profile = solve_speed_profile(smooth_path, bike)
        delta = smooth_profile.lap_time_s - old.lap_time_s
        print("comparison", spacing, old.lap_time_s, smooth_profile.lap_time_s, delta,
              100*delta/old.lap_time_s, old.sampled_path.total_length_m,
              smooth_path.total_length_m, np.max(np.abs(old.sampled_path.curvature_1pm)),
              np.max(np.abs(smooth_path.curvature_1pm)),
              np.max(np.abs(old.speed_profile.curvature_gradient_1pm2)),
              np.max(np.abs(smooth_profile.curvature_gradient_1pm2)), sep=",")

    sampled = sample_track(track, 0.5)
    old = evaluate_racing_line(PHASE5_DETERMINISTIC_LOCAL_REFERENCE, sampled, bike,
                               PeriodicCubicParameterisation(12), 0.25)
    edges = calculate_boundaries(sampled)
    fig, ax = plt.subplots()
    ax.plot(edges.left_x_m, edges.left_y_m, label="left boundary")
    ax.plot(edges.right_x_m, edges.right_y_m, label="right boundary")
    ax.plot(sampled.x_m, sampled.y_m, "--", label="centreline")
    ax.plot(old.sampled_path.x_m, old.sampled_path.y_m, label="existing offset-sampled")
    ax.plot(results[48].sampled_path.x_m, results[48].sampled_path.y_m, label="smooth planar")
    ax.plot(results[48].guide_x_m, results[48].guide_y_m, "o", ms=3, label="planar guides")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend()
    output = Path("phase7_planar_geometry.png")
    fig.savefig(output, dpi=160, bbox_inches="tight")
    print(f"\nwrote {output}")


if __name__ == "__main__":
    main()
