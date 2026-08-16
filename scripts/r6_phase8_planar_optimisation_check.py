"""Reproducible Phase 8 path-model and fixed-spline resolution diagnostics."""

import argparse
import csv
import math
import os
from pathlib import Path
import tempfile
import time

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
    planar_control_bounds,
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
NAMED_POLICIES = POLICIES + (("extra-fine", EXTRA_FINE_POLICY),)
TRACKS = (
    ("oval", "examples/tracks/test_oval.yaml"),
    ("mallala", "examples/tracks/mallala_reference.yaml"),
)


class RestartArgumentParser(argparse.ArgumentParser):
    """Reject a restart CSV before the diagnostic performs any track work."""

    def parse_args(self, args=None, namespace=None):
        parsed = super().parse_args(args, namespace)
        if parsed.initial_controls_csv is not None or parsed.checkpoint_controls_csv is not None:
            option = ("--initial-controls-csv" if parsed.initial_controls_csv is not None
                      else "--checkpoint-controls-csv")
            if parsed.track == "both":
                self.error(f"{option} requires --track oval or mallala, not both")
            if parsed.policy == "all":
                if parsed.initial_controls_csv is not None:
                    self.error("--initial-controls-csv requires --policy coarse, reference, or fine, not all")
                self.error("--checkpoint-controls-csv requires a single --policy, not all")
        return parsed


def build_parser():
    parser = RestartArgumentParser()
    parser.add_argument("--max-evaluations", type=int, default=1500)
    parser.add_argument("--max-sweeps", type=int, default=30)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument("--initial-step-m", type=float, default=1.0)
    parser.add_argument("--track", choices=("oval", "mallala", "both"), default="both")
    parser.add_argument("--policy", choices=("coarse", "reference", "fine", "extra-fine", "all"),
                        default="all")
    parser.add_argument("--initial-controls-csv", type=Path, default=None)
    parser.add_argument("--checkpoint-controls-csv", type=Path, default=None)
    return parser


def optimisation_config(args):
    """Build the Phase 8 configuration from diagnostic CLI limits."""
    return PlanarOptimisationConfig(initial_step_m=args.initial_step_m,
                                    max_sweeps=args.max_sweeps,
                                    max_evaluations=args.max_evaluations,
                                    parallel_workers=args.workers,
                                    speed_backend=args.speed_backend)


def half_lap_symmetry_differences(control_s_m, controls_m, lap_length_m):
    """Return paired control differences when stations repeat after half a lap.

    The absolute station tolerance only accommodates floating-point arithmetic
    used to accumulate otherwise identical primitive subdivisions.
    """
    stations = np.asarray(control_s_m, dtype=float)
    controls = np.asarray(controls_m, dtype=float)
    if (stations.ndim != 1 or controls.shape != stations.shape
            or len(stations) == 0 or len(stations) % 2):
        return None
    half = len(stations) // 2
    if not np.allclose(stations[half:] - 0.5 * lap_length_m, stations[:half],
                       rtol=0.0, atol=1e-10):
        return None
    return controls[:half] - controls[half:]


def timed_optimisation(track, bike, policy, config, initial_controls_m=None,
                       progress_callback=None):
    """Run and report wall-clock timing without affecting optimisation logic."""
    started = time.perf_counter()
    result = optimise_planar_racing_line(
        track, bike, policy, config, initial_controls_m=initial_controls_m,
        progress_callback=progress_callback)
    elapsed = time.perf_counter() - started
    print(f"workers={config.parallel_workers}")
    print(f"speed_backend={config.speed_backend}")
    print(f"optimisation_elapsed_s={elapsed:.6f}")
    print(f"seconds_per_evaluation={elapsed / result.evaluations:.9f}")
    print(f"evaluations_per_wall_second={result.evaluations / elapsed:.6f}")
    return result


def load_initial_controls_csv(path, control_s_m, lower_bounds_m, upper_bounds_m):
    """Load controls from an export, requiring an exact current layout identity."""
    required = ("index", "control_s_m", "best_offset_m", "lower_bound_m", "upper_bound_m")
    path = Path(path)
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            missing = [field for field in required if field not in (reader.fieldnames or ())]
            if missing:
                raise ValueError(f"initial controls CSV is missing required columns: {', '.join(missing)}")
            rows = list(reader)
    except OSError as exc:
        raise ValueError(f"initial controls CSV {path} cannot be read: {exc}") from exc

    stations = np.asarray(control_s_m, dtype=float)
    lower = np.asarray(lower_bounds_m, dtype=float)
    upper = np.asarray(upper_bounds_m, dtype=float)
    expected_count = len(stations)
    if len(rows) != expected_count:
        raise ValueError(f"initial controls CSV row count {len(rows)} does not match "
                         f"generated control-station count {expected_count}")

    parsed = np.empty((expected_count, 4), dtype=float)
    for expected_index, row in enumerate(rows):
        try:
            index = int(row["index"])
            if str(index) != row["index"].strip():
                raise ValueError
        except (TypeError, ValueError) as exc:
            raise ValueError(f"initial controls CSV row {expected_index + 2} has an invalid index") from exc
        if index != expected_index:
            raise ValueError(f"initial controls CSV index {index} at row {expected_index + 2} "
                             f"does not match expected sequential index {expected_index}")
        try:
            parsed[expected_index] = [float(row[field]) for field in required[1:]]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"initial controls CSV row {expected_index + 2} contains "
                             "a non-numeric value") from exc
        if not np.all(np.isfinite(parsed[expected_index])):
            raise ValueError(f"initial controls CSV row {expected_index + 2} contains "
                             "a non-finite numeric value")

    saved_stations, controls, saved_lower, saved_upper = parsed.T
    tolerance = dict(rtol=0.0, atol=1e-9)
    if not np.allclose(saved_stations, stations, **tolerance):
        raise ValueError("initial controls CSV control_s_m does not match generated control stations")
    if not np.allclose(saved_lower, lower, **tolerance):
        raise ValueError("initial controls CSV lower_bound_m does not match current control bounds")
    if not np.allclose(saved_upper, upper, **tolerance):
        raise ValueError("initial controls CSV upper_bound_m does not match current control bounds")
    outside = (controls < lower) | (controls > upper)
    if np.any(outside):
        index = int(np.flatnonzero(outside)[0])
        raise ValueError(f"initial controls CSV best_offset_m at index {index} is outside current bounds")
    return controls


def atomic_write_controls_csv(path, stations, controls, lower, upper):
    """Atomically write the strict same-policy warm-start controls format."""
    path = Path(path)
    temporary = None
    try:
        fd, temporary_name = tempfile.mkstemp(
            dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
        temporary = Path(temporary_name)
        with os.fdopen(fd, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("index", "control_s_m", "best_offset_m",
                             "lower_bound_m", "upper_bound_m"))
            for index, values in enumerate(zip(stations, controls, lower, upper)):
                writer.writerow((index, *values))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def checkpoint_callback(path, stations, lower, upper):
    """Create the parent-process per-poll warm-start checkpoint callback."""
    def checkpoint(progress):
        atomic_write_controls_csv(path, stations, progress.best_controls_m, lower, upper)
        print(f"checkpoint_controls_csv={path} checkpoint_lap_s={progress.lap_time_s:.9f} "
              f"checkpoint_evaluations={progress.evaluations} "
              f"checkpoint_sweeps={progress.sweeps} checkpoint_step_m={progress.step_m:.9f}")
    return checkpoint


def report_oval_symmetry(track, bike, policy, result, config):
    """Evaluate half-lap transforms of the fine-policy oval result."""
    differences = half_lap_symmetry_differences(
        result.control_s_m, result.best_controls_m, track.total_length_m)
    if differences is None:
        return
    print(f"symmetry_max_control_difference_m={np.max(np.abs(differences)):.9f}")
    print(f"symmetry_rms_control_difference_m={np.sqrt(np.mean(differences ** 2)):.9f}")

    half = len(result.best_controls_m) // 2
    original = result.best_controls_m
    candidates = {
        "shifted": np.roll(original, half),
        "symmetric_a": np.tile(original[:half], 2),
        "symmetric_b": np.tile(original[half:], 2),
        "symmetric_c": np.tile(0.5 * (original[:half] + original[half:]), 2),
    }
    kwargs = dict(sample_spacing_m=config.optimisation_sample_spacing_m,
                  boundary_margin_m=config.boundary_margin_m,
                  boundary_check_spacing_m=config.boundary_check_spacing_m,
                  speed_backend=config.speed_backend)
    evaluations = {
        name: evaluate_planar_racing_line(controls, track, bike, result.control_s_m, **kwargs)
        for name, controls in candidates.items()
    }
    shifted_lap = evaluations["shifted"].lap_time_s
    delta = shifted_lap - result.best_lap_time_s
    print(f"original_best_lap_s={result.best_lap_time_s:.9f}")
    print(f"shifted_best_lap_s={shifted_lap:.9f}")
    print(f"shifted_minus_original_s={delta:.12f}")
    resolution_deltas = []
    for spacing in (1.0, 0.5, 0.25):
        original_at_spacing = evaluate_planar_racing_line(
            original, track, bike, result.control_s_m, sample_spacing_m=spacing,
            boundary_margin_m=config.boundary_margin_m,
            boundary_check_spacing_m=config.boundary_check_spacing_m,
            speed_backend=config.speed_backend)
        shifted_at_spacing = evaluate_planar_racing_line(
            candidates["shifted"], track, bike, result.control_s_m, sample_spacing_m=spacing,
            boundary_margin_m=config.boundary_margin_m,
            boundary_check_spacing_m=config.boundary_check_spacing_m,
            speed_backend=config.speed_backend)
        spacing_delta = shifted_at_spacing.lap_time_s - original_at_spacing.lap_time_s
        resolution_deltas.append(spacing_delta)
        print(f"shift_symmetry_spacing_m={spacing:.2f} "
              f"shifted_minus_original_s={spacing_delta:.12f}")
    if (all(math.isfinite(value) for value in resolution_deltas)
            and abs(resolution_deltas[-1]) < 0.5 * abs(resolution_deltas[0])):
        print("shift_symmetry_assessment=numerical sampling-phase sensitivity")
    elif not math.isclose(resolution_deltas[-1], 0.0, rel_tol=0.0, abs_tol=1e-9):
        print("shift_symmetry_assessment=objective symmetry unresolved")
    for name in ("symmetric_a", "symmetric_b", "symmetric_c"):
        print(f"{name}_lap_s={evaluations[name].lap_time_s:.9f}")
    if any(evaluations[name].lap_time_s < result.best_lap_time_s
           for name in ("symmetric_a", "symmetric_b", "symmetric_c")):
        print("CURRENT COORDINATE-SEARCH RESULT IS NOT LOCALLY CONVINCING")
    reference_lap_s = 15.525622213
    quality = "reaches_or_improves" if result.best_lap_time_s <= reference_lap_s else "slower"
    print(f"symmetric_a_reference_quality={quality} reference_lap_s={reference_lap_s:.9f}")

    symmetric_a = candidates["symmetric_a"]
    restart = timed_optimisation(track, bike, policy, config, symmetric_a)
    restart_differences = half_lap_symmetry_differences(
        restart.control_s_m, restart.best_controls_m, track.total_length_m)
    print(f"symmetric_a_restart_initial_lap_s={restart.initial_lap_time_s:.9f}")
    print(f"symmetric_a_restart_final_lap_s={restart.best_lap_time_s:.9f}")
    print(f"symmetric_a_restart_symmetry_max_control_difference_m="
          f"{np.max(np.abs(restart_differences)):.9f}")
    print(f"symmetric_a_restart_symmetry_rms_control_difference_m="
          f"{np.sqrt(np.mean(restart_differences ** 2)):.9f}")
    print(f"symmetric_a_restart_evaluations={restart.evaluations} "
          f"polls={restart.sweeps} termination={restart.termination_reason!r}")
    print(f"symmetric_a_restart_controls_m={restart.best_controls_m.tolist()}")


def selected_tracks(selection):
    """Return track definitions selected by the command line."""
    return TRACKS if selection == "both" else tuple(item for item in TRACKS if item[0] == selection)


def selected_policies(selection):
    """Return named optimisation policies selected by the command line."""
    # ``all`` intentionally retains the original three-policy diagnostic.
    return POLICIES if selection == "all" else tuple(
        item for item in NAMED_POLICIES if item[0] == selection)


def metrics(label, result, restarted=False):
    path = result.sampled_path
    speed = result.speed_profile
    initial_label = "initial_lap_s" if restarted else "zero_lap_s"
    print(f"{label}: controls={len(result.control_s_m)} {initial_label}={result.initial_lap_time_s:.9f} "
          f"optimised_lap_s={result.best_lap_time_s:.9f} improvement_s={result.improvement_s:.9f} "
          f"evaluations={result.evaluations} sweeps={result.sweeps} length_m={path.total_length_m:.9f} "
          f"clearance_m={result.minimum_boundary_clearance_m:.9f} forward_min={result.minimum_forward_progress:.9f} "
          f"curvature_minmax={np.min(path.curvature_1pm):.9f}/{np.max(path.curvature_1pm):.9f} "
          f"max_abs_dk_dq={np.max(np.abs(speed.curvature_gradient_1pm2)):.9f} "
          f"max_abs_dk_dt={np.max(np.abs(speed.curvature_rate_1pmps)):.9f} "
          f"controls_minmax_m={np.min(result.best_controls_m):.6f}/{np.max(result.best_controls_m):.6f} "
          f"termination={result.termination_reason!r}")


def run_selected_optimisation(parser, args, track, bike, policy_name, policy,
                              config, stations):
    """Load an optional restart vector, then run and report one optimisation."""
    initial_controls = None
    if args.initial_controls_csv is not None:
        lower, upper = planar_control_bounds(track, stations, config.boundary_margin_m)
        try:
            initial_controls = load_initial_controls_csv(
                args.initial_controls_csv, stations, lower, upper)
        except ValueError as exc:
            parser.error(str(exc))
        print(f"initial_controls_csv={args.initial_controls_csv}")
        print(f"initial_controls_count={len(initial_controls)}")
    optional_callback = {}
    if args.checkpoint_controls_csv is not None:
        lower, upper = planar_control_bounds(track, stations, config.boundary_margin_m)
        optional_callback["progress_callback"] = checkpoint_callback(
            args.checkpoint_controls_csv, stations, lower, upper)
    result = timed_optimisation(
        track, bike, policy, config, initial_controls_m=initial_controls,
        **optional_callback)
    metrics(policy_name, result, restarted=initial_controls is not None)
    return result


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


def main(argv=None):
    total_started = time.perf_counter()
    parser = build_parser()
    args = parser.parse_args(argv)
    bike = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    config = optimisation_config(args)
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
                result = run_selected_optimisation(
                    parser, args, track, bike, policy_name, policy, config, stations)
                if name == "oval" and policy_name == "fine":
                    report_oval_symmetry(track, bike, policy, result, config)
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
    total_elapsed = time.perf_counter() - total_started
    print(f"total_elapsed_s={total_elapsed:.6f}")
    print(f"total_elapsed_min={total_elapsed / 60.0:.6f}")


if __name__ == "__main__":
    main()
