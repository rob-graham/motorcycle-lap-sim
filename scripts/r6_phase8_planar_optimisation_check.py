"""Reproducible Phase 8 path-model and fixed-spline resolution diagnostics."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    COARSE_PLANAR_CONTROL_POLICY,
    FINE_PLANAR_CONTROL_POLICY,
    REFERENCE_PLANAR_CONTROL_POLICY,
    PlanarControlStationPolicy,
    PlanarOptimisationConfig,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    optimise_planar_racing_line,
    resample_planar_result,
)
from motorcycle_lap_sim.path import from_sampled_track
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import Track, sample_track, sample_track_stations
from motorcycle_lap_sim.track.boundaries import calculate_boundaries

POLICIES = (
    ("coarse", COARSE_PLANAR_CONTROL_POLICY),
    ("reference", REFERENCE_PLANAR_CONTROL_POLICY),
    ("fine", FINE_PLANAR_CONTROL_POLICY),
)
# This is deliberately local to the diagnostic: it is not a Phase 8 default.
EXTRA_FINE_POLICY = PlanarControlStationPolicy(50.0, math.radians(20.0))
TRACKS = (
    ("oval", "examples/tracks/test_oval.yaml"),
    ("mallala", "examples/tracks/mallala_reference.yaml"),
)


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-evaluations", type=int, default=1500)
    parser.add_argument("--track", choices=("oval", "mallala", "both"), default="both")
    parser.add_argument("--policy", choices=("coarse", "reference", "fine", "all"),
                        default="all")
    return parser


def selected_tracks(selection):
    """Return track definitions selected by the command line."""
    return TRACKS if selection == "both" else tuple(item for item in TRACKS if item[0] == selection)


def selected_policies(selection):
    """Return named optimisation policies selected by the command line."""
    return POLICIES if selection == "all" else tuple(item for item in POLICIES if item[0] == selection)


def metrics(label, result):
    path = result.sampled_path
    speed = result.speed_profile
    print(f"{label}: controls={len(result.control_s_m)} zero_lap_s={result.initial_lap_time_s:.9f} "
          f"optimised_lap_s={result.best_lap_time_s:.9f} improvement_s={result.improvement_s:.9f} "
          f"evaluations={result.evaluations} sweeps={result.sweeps} length_m={path.total_length_m:.9f} "
          f"clearance_m={result.minimum_boundary_clearance_m:.9f} forward_min={result.minimum_forward_progress:.9f} "
          f"curvature_minmax={np.min(path.curvature_1pm):.9f}/{np.max(path.curvature_1pm):.9f} "
          f"max_abs_dk_dq={np.max(np.abs(speed.curvature_gradient_1pm2)):.9f} "
          f"max_abs_dk_dt={np.max(np.abs(speed.curvature_rate_1pmps)):.9f} "
          f"controls_minmax_m={np.min(result.best_controls_m):.6f}/{np.max(result.best_controls_m):.6f} "
          f"termination={result.termination_reason!r}")


def continuous_clearance_data(track, smooth_line, boundary_margin_m):
    """Calculate signed corridor limits and clearances on the spline check grid."""
    checked = sample_track_stations(track, smooth_line.evaluated_track_s_m)
    left_limit = checked.width_left_m - boundary_margin_m
    right_limit = -(checked.width_right_m - boundary_margin_m)
    left_clearance = left_limit - smooth_line.projected_offset_m
    right_clearance = smooth_line.projected_offset_m - right_limit
    return left_limit, right_limit, left_clearance, right_clearance


def print_continuous_details(policy_name, track, result, boundary_margin_m):
    smooth = result.smooth_line
    _, _, left, right = continuous_clearance_data(track, smooth, boundary_margin_m)
    overall = np.minimum(left, right)
    left_index, right_index, overall_index = np.argmin(left), np.argmin(right), np.argmin(overall)
    stations = smooth.evaluated_track_s_m
    print(f"{policy_name} continuous detail:")
    print(f"  projected_offset_min_m={np.min(smooth.projected_offset_m):.9f}")
    print(f"  projected_offset_max_m={np.max(smooth.projected_offset_m):.9f}")
    print(f"  minimum_left_clearance_m={left[left_index]:.9f}")
    print(f"  station_of_minimum_left_clearance_m={stations[left_index]:.9f}")
    print(f"  minimum_right_clearance_m={right[right_index]:.9f}")
    print(f"  station_of_minimum_right_clearance_m={stations[right_index]:.9f}")
    print(f"  minimum_overall_clearance_m={overall[overall_index]:.9f}")
    print(f"  station_of_minimum_overall_clearance_m={stations[overall_index]:.9f}")
    print(f"  final_step_m={result.final_step_m:.9f}")


def plot_plan_view(track, zero, result, policy_name, output_stem):
    sampled = sample_track(track, 0.5)
    edges = calculate_boundaries(sampled)
    fig, ax = plt.subplots(figsize=(16, 10))
    ax.plot(edges.left_x_m, edges.left_y_m, linewidth=0.8, label="left boundary")
    ax.plot(edges.right_x_m, edges.right_y_m, linewidth=0.8, label="right boundary")
    ax.plot(sampled.x_m, sampled.y_m, "--", linewidth=0.7, label="analytic centreline")
    ax.plot(zero.sampled_path.x_m, zero.sampled_path.y_m, linewidth=0.9,
            label="zero-control planar")
    ax.plot(result.sampled_path.x_m, result.sampled_path.y_m, linewidth=1.0,
            label="optimised planar")
    ax.plot(result.smooth_line.guide_x_m, result.smooth_line.guide_y_m, "o", ms=2,
            label="control guides")
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlabel="x [m]", ylabel="y [m]", title=f"{policy_name} policy")
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
    fig.savefig(output_stem.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(output_stem.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)


def plot_offsets(track, zero, result, policy_name, boundary_margin_m, output):
    smooth = result.smooth_line
    left, right, _, _ = continuous_clearance_data(track, smooth, boundary_margin_m)
    fig, ax = plt.subplots(figsize=(14, 7))
    ax.plot(smooth.evaluated_track_s_m, smooth.projected_offset_m, linewidth=1.0,
            label="optimised projected offset")
    ax.plot(zero.evaluated_track_s_m, zero.projected_offset_m, linewidth=0.9,
            label="zero-control projected offset")
    ax.plot(smooth.evaluated_track_s_m, left, linewidth=0.8, label="usable left limit")
    ax.plot(smooth.evaluated_track_s_m, right, linewidth=0.8, label="usable right limit")
    ax.plot(result.control_s_m, result.best_controls_m, "o", ms=3,
            label="best physical controls")
    ax.set(xlabel="centreline station s [m]", ylabel="lateral offset [m]",
           title=f"{policy_name} policy continuous lateral offsets")
    ax.grid(True, linewidth=0.3)
    ax.legend(loc="best")
    fig.savefig(output, dpi=300, bbox_inches="tight")
    plt.close(fig)


def export_controls(result, output):
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("index", "control_s_m", "best_offset_m", "lower_bound_m", "upper_bound_m"))
        for index, values in enumerate(zip(result.control_s_m, result.best_controls_m,
                                           result.lower_bounds_m, result.upper_bounds_m)):
            writer.writerow((index, *values))


def report_extra_fine(track, bike, centre_lap_s):
    stations = generate_planar_control_stations(track, EXTRA_FINE_POLICY)
    zero = evaluate_planar_racing_line(np.zeros(len(stations)), track, bike, stations)
    if not zero.feasible:
        print(f"extra-fine: controls={len(stations)} zero_feasible=false reason={zero.failure_reason}")
        return
    smooth = zero.smooth_line
    assert smooth is not None
    print(f"extra-fine: controls={len(stations)} zero_feasible=true "
          f"zero_length_m={smooth.sampled_path.total_length_m:.9f} zero_lap_s={zero.lap_time_s:.9f} "
          f"analytic_lap_difference_s={zero.lap_time_s - centre_lap_s:.9f} "
          f"clearance_m={smooth.minimum_boundary_clearance_m:.9f}")


def main():
    args = build_parser().parse_args()
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    config = PlanarOptimisationConfig(max_evaluations=args.max_evaluations)
    for name, filename in selected_tracks(args.track):
        track = Track.from_yaml(filename)
        centre = solve_speed_profile(from_sampled_track(sample_track(track, 1.0)), bike)
        print(f"\n{name.upper()} analytic_centreline_lap_s={centre.lap_time_s:.9f}")
        saved = {}
        for policy_name, policy in selected_policies(args.policy):
            stations = generate_planar_control_stations(track, policy)
            zero = evaluate_planar_racing_line(np.zeros(len(stations)), track, bike, stations)
            if not zero.feasible:
                print(f"{policy_name}: controls={len(stations)} zero_feasible=false reason={zero.failure_reason}")
                continue
            smooth = zero.smooth_line
            assert smooth is not None
            print(f"{policy_name}: controls={len(stations)} zero_feasible=true "
                  f"zero_lap_s={zero.lap_time_s:.9f} length_m={smooth.sampled_path.total_length_m:.9f} "
                  f"analytic_lap_difference_s={zero.lap_time_s - centre.lap_time_s:.9f} "
                  f"clearance_m={smooth.minimum_boundary_clearance_m:.9f}")
            # Preserve the complete diagnostic, while a targeted policy explicitly optimises that policy.
            if args.policy != "all" or name == "oval" or policy_name == "reference":
                result = optimise_planar_racing_line(track, bike, policy, config)
                metrics(policy_name, result)
                saved[policy_name] = (smooth, result)

        report_extra_fine(track, bike, centre.lap_time_s)
        preferred = (args.policy if args.policy != "all" else "reference")
        if preferred not in saved and args.policy == "all" and "fine" in saved:
            print("reference policy is geometrically infeasible; selected saved policy=fine")
            preferred = "fine"
        if preferred not in saved:
            continue
        zero, result = saved[preferred]
        speed = result.speed_profile
        print(f"{preferred} detail: "
              f"speed_minmax_mps={np.min(speed.speed_mps):.6f}/{np.max(speed.speed_mps):.6f} "
              f"gears={'/'.join(map(str, np.unique(speed.gear_number)))} "
              f"rpm_minmax={np.min(speed.engine_rpm):.3f}/{np.max(speed.engine_rpm):.3f} "
              f"lateral_max_mps2={np.max(np.abs(speed.lateral_acceleration_mps2)):.6f} "
              f"forward_max_mps2={np.max(speed.longitudinal_acceleration_mps2):.6f} "
              f"braking_max_mps2={-np.min(speed.longitudinal_acceleration_mps2):.6f}")
        print_continuous_details(preferred, track, result, config.boundary_margin_m)
        print(f"{preferred} fixed-spline output-resolution sensitivity (same geometry)")
        for spacing in (1.0, 0.5, 0.25):
            path, profile = resample_planar_result(result, bike, spacing)
            print(f"spacing_m={spacing:.2f} length_m={path.total_length_m:.9f} lap_s={profile.lap_time_s:.9f}")
        stem = Path(f"phase8_{name}_planar")
        plot_plan_view(track, zero, result, preferred, stem)
        plot_offsets(track, zero, result, preferred, config.boundary_margin_m,
                     Path(f"phase8_{name}_offsets.png"))
        export_controls(result, Path(f"phase8_{name}_controls.csv"))

    print("\nControl-policy comparisons are different path-model orders; fixed-spline comparisons are output resolution only.")
    print("A fastest policy is not automatically most accurate; material changes mean path-model sensitivity NOT CONVERGED.")


if __name__ == "__main__":
    main()
