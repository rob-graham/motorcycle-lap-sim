"""CLI diagnostics for deterministic local racing-line optimisation."""

import argparse
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.racing_line import build_racing_line_path
from motorcycle_lap_sim.track import Track, sample_track
from motorcycle_lap_sim.track.boundaries import calculate_boundaries
from .objective import evaluate_racing_line
from .optimiser import OptimisationConfig, optimise_racing_line
from .parameterisation import PeriodicCubicParameterisation


def write_csv(filename, samples, evaluation):
    with filename.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("centreline_s_m", "offset_m", "x_m", "y_m", "racing_line_q_m",
                         "curvature_1pm", "speed_mps", "gear", "engine_rpm"))
        writer.writerows(zip(samples.s_m, evaluation.dense_offset_m,
            evaluation.sampled_path.x_m, evaluation.sampled_path.y_m,
            evaluation.sampled_path.q_m, evaluation.sampled_path.curvature_1pm,
            evaluation.speed_profile.speed_mps, evaluation.speed_profile.gear_number,
            evaluation.speed_profile.engine_rpm))


def plot_result(samples, baseline, optimised, filename):
    boundaries = calculate_boundaries(samples)
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(boundaries.left_x_m, boundaries.left_y_m, "k--", label="left boundary")
    ax.plot(boundaries.right_x_m, boundaries.right_y_m, "k--", label="right boundary")
    ax.plot(samples.x_m, samples.y_m, color="0.6", label="centreline")
    ax.plot(baseline.sampled_path.x_m, baseline.sampled_path.y_m, label="zero-offset baseline")
    ax.plot(optimised.sampled_path.x_m, optimised.sampled_path.y_m,
            label="locally optimised racing line")
    ax.axis("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend()
    fig.savefig(filename, dpi=150, bbox_inches="tight"); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track", type=Path); parser.add_argument("motorcycle", type=Path)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--validation-spacing", type=float, default=0.5)
    parser.add_argument("--controls", type=int, default=12)
    parser.add_argument("--control-bound", type=float, default=4.0)
    parser.add_argument("--initial-step", type=float, default=1.0)
    parser.add_argument("--minimum-step", type=float, default=0.0625)
    parser.add_argument("--max-sweeps", type=int, default=30)
    parser.add_argument("--max-evaluations", type=int, default=500)
    parser.add_argument("--boundary-margin", type=float, default=0.25)
    parser.add_argument("--output-csv", type=Path); parser.add_argument("--output-png", type=Path)
    args = parser.parse_args()
    track, bike = Track.from_yaml(args.track), load_motorcycle_config(args.motorcycle)
    samples = sample_track(track, args.spacing)
    config = OptimisationConfig(args.controls, args.control_bound, args.initial_step,
        args.minimum_step, 0.5, 1e-6, args.max_sweeps, args.max_evaluations,
        args.boundary_margin)
    result = optimise_racing_line(samples, bike, config)
    fine = sample_track(track, args.validation_spacing)
    parameterisation = PeriodicCubicParameterisation(args.controls)
    fine_zero = evaluate_racing_line(np.zeros(args.controls), fine, bike, parameterisation,
                                     args.boundary_margin)
    fine_best = evaluate_racing_line(result.best_controls, fine, bike, parameterisation,
                                     args.boundary_margin)
    clearance = np.min(np.minimum(fine.width_left_m - fine_best.dense_offset_m,
                                  fine.width_right_m + fine_best.dense_offset_m))
    print(f"optimisation sample spacing: {args.spacing:.3f} m")
    print(f"validation sample spacing: {args.validation_spacing:.3f} m")
    print(f"control count: {args.controls}"); print(f"boundary margin: {args.boundary_margin:.3f} m")
    print(f"initial lap time: {result.initial_lap_time_s:.9f} s")
    print(f"optimised lap time: {result.best_lap_time_s:.9f} s")
    print(f"improvement: {result.improvement_s:.9f} s ({result.improvement_percent:.6f}%)")
    print(f"evaluations: {result.evaluations}"); print(f"sweeps: {result.sweeps}")
    print(f"final step: {result.final_step:g}"); print(f"converged: {result.converged}")
    print(f"termination reason: {result.termination_reason}")
    print(f"offset range: {fine_best.dense_offset_m.min():.6f} to {fine_best.dense_offset_m.max():.6f} m")
    print(f"minimum boundary clearance: {clearance:.6f} m")
    print(f"path length: {fine_best.sampled_path.total_length_m:.6f} m")
    print(f"speed range: {fine_best.speed_profile.speed_mps.min():.6f} to {fine_best.speed_profile.speed_mps.max():.6f} m/s")
    print(f"curvature range: {fine_best.sampled_path.curvature_1pm.min():.9f} to {fine_best.sampled_path.curvature_1pm.max():.9f} 1/m")
    print(f"zero-control validation lap time: {fine_zero.lap_time_s:.9f} s")
    print(f"optimised-control validation lap time: {fine_best.lap_time_s:.9f} s")
    print(f"validation improvement: {fine_zero.lap_time_s - fine_best.lap_time_s:.9f} s")
    if args.output_csv: write_csv(args.output_csv, fine, fine_best)
    if args.output_png: plot_result(fine, fine_zero, fine_best, args.output_png)


if __name__ == "__main__":
    main()
