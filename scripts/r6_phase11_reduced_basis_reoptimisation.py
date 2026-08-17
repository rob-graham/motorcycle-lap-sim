"""Re-optimise a reduced fixed-station Phase 11 racing-line control basis.

This diagnostic removes one selected control from a retained feasible line,
then re-optimises the remaining lateral offsets with the existing deterministic
best-improvement pattern search.  Station relocation is deliberately excluded
so deletion and relocation effects remain separable.
"""

import argparse
import csv
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import importlib.util
import math
import multiprocessing
from pathlib import Path
import time

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    PlanarOptimisationConfig,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.optimisation.planar import _best_improvement_pattern_search
from motorcycle_lap_sim.track import Track, sample_track_stations


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_reduced_basis")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_reduced_basis")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_reduced_basis")
phase11screen = _load_sibling("r6_phase11_control_deletion_screen.py", "phase11_deletion_screen_base")

DEFAULT_DELETE_INDEX = 26
DEFAULT_MARGIN_M = 0.25
DEFAULT_MAX_ROLL_RATE_RADPS = 0.8
DEFAULT_INITIAL_STEP_M = 0.125
DEFAULT_MINIMUM_STEP_M = 0.0625
DEFAULT_MAX_SWEEPS = 12
DEFAULT_MAX_EVALUATIONS = 4000
DEFAULT_WORKERS = 16
DEFAULT_OPTIMISATION_SPACING_M = 1.0
DEFAULT_COMMON_SPACING_M = 0.125
DEFAULT_BOUNDARY_CHECK_SPACING_M = 0.125
DEFAULT_PLOT_DPI = 400

_WORKER_CONTEXT = None


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def _nonnegative_float(text):
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return value


def _positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def _nonnegative_int(text):
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("start_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--delete-index", type=_nonnegative_int, default=DEFAULT_DELETE_INDEX)
    parser.add_argument("--margin-m", type=_nonnegative_float, default=DEFAULT_MARGIN_M)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float,
                        default=DEFAULT_MAX_ROLL_RATE_RADPS)
    parser.add_argument("--initial-step-m", type=_positive_float, default=DEFAULT_INITIAL_STEP_M)
    parser.add_argument("--minimum-step-m", type=_positive_float, default=DEFAULT_MINIMUM_STEP_M)
    parser.add_argument("--max-sweeps", type=_positive_int, default=DEFAULT_MAX_SWEEPS)
    parser.add_argument("--max-evaluations", type=_positive_int,
                        default=DEFAULT_MAX_EVALUATIONS)
    parser.add_argument("--workers", type=_positive_int, default=DEFAULT_WORKERS)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="numba")
    parser.add_argument("--optimisation-spacing-m", type=_positive_float,
                        default=DEFAULT_OPTIMISATION_SPACING_M)
    parser.add_argument("--common-spacing-m", type=_positive_float,
                        default=DEFAULT_COMMON_SPACING_M)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float,
                        default=DEFAULT_BOUNDARY_CHECK_SPACING_M)
    parser.add_argument("--plot-dpi", type=_positive_int, default=DEFAULT_PLOT_DPI)
    return parser


def reduced_basis(control_s_m, controls_m, delete_index):
    return phase11screen.deletion_arrays(control_s_m, controls_m, delete_index)


def _compact(evaluation):
    return phase11screen.phase8.PlanarObjectiveEvaluation(
        evaluation.feasible, evaluation.lap_time_s,
        failure_reason=evaluation.failure_reason)


def _init_worker(track, bike, stations, sample_spacing, margin, check_spacing, backend):
    global _WORKER_CONTEXT
    _WORKER_CONTEXT = (track, bike, stations, sample_spacing, margin, check_spacing, backend)


def _worker_evaluate(controls):
    if _WORKER_CONTEXT is None:
        raise RuntimeError("reduced-basis worker context was not initialized")
    track, bike, stations, sample_spacing, margin, check_spacing, backend = _WORKER_CONTEXT
    evaluation = evaluate_planar_racing_line(
        controls, track, bike, stations,
        sample_spacing_m=sample_spacing,
        boundary_margin_m=margin,
        boundary_check_spacing_m=check_spacing,
        speed_backend=backend,
    )
    return _compact(evaluation)


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


def _minimum_forward(track, evaluation):
    checked_s = evaluation.smooth_line.evaluated_track_s_m
    checked_track = sample_track_stations(track, checked_s)
    _, _, dx, dy, *_ = evaluation.smooth_line.spline.evaluate(checked_s)
    return float(np.min(dx * checked_track.tangent_x + dy * checked_track.tangent_y))


def _start_finish_segment(track):
    sampled = sample_track_stations(track, np.array([0.0]))
    cx = float(sampled.x_m[0])
    cy = float(sampled.y_m[0])
    nx = float(sampled.normal_x[0])
    ny = float(sampled.normal_y[0])
    left = (cx + float(sampled.width_left_m[0]) * nx,
            cy + float(sampled.width_left_m[0]) * ny)
    right = (cx - float(sampled.width_right_m[0]) * nx,
             cy - float(sampled.width_right_m[0]) * ny)
    return left, right


def _write_racing_line_csv(path, track, baseline, reduced):
    count = len(reduced.smooth_line.sampled_path.q_m)
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    sampled_track = sample_track_stations(track, track_s)
    base_x, base_y, *_ = baseline.smooth_line.spline.evaluate(track_s)
    reduced_x, reduced_y, *_ = reduced.smooth_line.spline.evaluate(track_s)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("track_s_m", "track_center_x_m", "track_center_y_m",
                         "baseline_x_m", "baseline_y_m", "reduced_x_m", "reduced_y_m"))
        writer.writerows(zip(track_s, sampled_track.x_m, sampled_track.y_m,
                             base_x, base_y, reduced_x, reduced_y))


def _write_racing_line_png(path, track, baseline, reduced, deleted_index, *, margin_m, dpi):
    checked_s = reduced.smooth_line.evaluated_track_s_m
    checked_track = sample_track_stations(track, checked_s)
    left_x = checked_track.x_m + checked_track.width_left_m * checked_track.normal_x
    left_y = checked_track.y_m + checked_track.width_left_m * checked_track.normal_y
    right_x = checked_track.x_m - checked_track.width_right_m * checked_track.normal_x
    right_y = checked_track.y_m - checked_track.width_right_m * checked_track.normal_y
    usable_left = checked_track.width_left_m - margin_m
    usable_right = checked_track.width_right_m - margin_m
    margin_left_x = checked_track.x_m + usable_left * checked_track.normal_x
    margin_left_y = checked_track.y_m + usable_left * checked_track.normal_y
    margin_right_x = checked_track.x_m - usable_right * checked_track.normal_x
    margin_right_y = checked_track.y_m - usable_right * checked_track.normal_y
    base_x, base_y, *_ = baseline.smooth_line.spline.evaluate(checked_s)
    reduced_x, reduced_y, *_ = reduced.smooth_line.spline.evaluate(checked_s)

    deleted_x = baseline.smooth_line.guide_x_m[deleted_index]
    deleted_y = baseline.smooth_line.guide_y_m[deleted_index]
    sf_left, sf_right = _start_finish_segment(track)

    figure, axis = plt.subplots(figsize=(12, 9))
    axis.plot(left_x, left_y, linewidth=0.35, label="Left track edge")
    axis.plot(right_x, right_y, linewidth=0.35, label="Right track edge")
    axis.plot(margin_left_x, margin_left_y, linewidth=0.3, linestyle="--", label="Margin corridor")
    axis.plot(margin_right_x, margin_right_y, linewidth=0.3, linestyle="--")
    axis.plot(checked_track.x_m, checked_track.y_m, linewidth=0.18, linestyle=":",
              label="Centreline")
    axis.plot(base_x, base_y, linewidth=0.5, linestyle="--", label="52-control baseline")
    axis.plot(reduced_x, reduced_y, linewidth=0.8, label="51-control re-optimised line")
    axis.scatter(reduced.smooth_line.guide_x_m, reduced.smooth_line.guide_y_m,
                 s=7, marker="o", linewidths=0.3, label="Retained control points", zorder=5)
    axis.scatter([deleted_x], [deleted_y], s=22, marker="x", linewidths=0.7,
                 label=f"Deleted control {deleted_index}", zorder=6)
    axis.plot([sf_left[0], sf_right[0]], [sf_left[1], sf_right[1]],
              linewidth=1.0, label="Start / finish", zorder=7)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(f"Mallala reduced-basis re-optimisation - {margin_m:.3f} m edge margin")
    axis.legend(fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def _write_summary(path, row):
    fields = tuple(row.keys())
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.minimum_step_m > args.initial_step_m:
        raise ValueError("minimum step must not exceed initial step")
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    full_stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    full_lower, full_upper = planar_control_bounds(track, full_stations, args.margin_m)
    full_controls = phase8.load_initial_controls_csv(
        args.start_controls_csv, full_stations, full_lower, full_upper)
    reduced_stations, reduced_controls = reduced_basis(
        full_stations, full_controls, args.delete_index)
    lower, upper = planar_control_bounds(track, reduced_stations, args.margin_m)

    kwargs = dict(
        sample_spacing_m=args.optimisation_spacing_m,
        boundary_margin_m=args.margin_m,
        boundary_check_spacing_m=args.boundary_check_spacing_m,
        speed_backend=args.speed_backend,
    )
    initial = _require_feasible(
        evaluate_planar_racing_line(reduced_controls, track, bike, reduced_stations, **kwargs),
        "reduced-basis initial line",
    )

    config = PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m,
        minimum_step_m=args.minimum_step_m,
        max_sweeps=args.max_sweeps,
        max_evaluations=args.max_evaluations,
        boundary_margin_m=args.margin_m,
        boundary_check_spacing_m=args.boundary_check_spacing_m,
        optimisation_sample_spacing_m=args.optimisation_spacing_m,
        parallel_workers=args.workers,
        speed_backend=args.speed_backend,
    )

    def evaluate(candidate):
        return evaluate_planar_racing_line(candidate, track, bike, reduced_stations, **kwargs)

    def materialise(candidate, worker_evaluation):
        regenerated = evaluate(candidate)
        if not regenerated.feasible:
            raise RuntimeError("parallel reduced-basis winner failed parent materialisation")
        if abs(regenerated.lap_time_s - worker_evaluation.lap_time_s) > 1e-9:
            raise RuntimeError("parallel reduced-basis winner disagreed with parent evaluation")
        return regenerated

    started = time.perf_counter()
    if args.workers == 1:
        best_controls, best, evaluations, sweeps, final_step, reason = (
            _best_improvement_pattern_search(
                reduced_controls, lower, upper, initial, evaluate, config))
    else:
        with ProcessPoolExecutor(
            max_workers=args.workers,
            mp_context=multiprocessing.get_context("spawn"),
            initializer=_init_worker,
            initargs=(track, bike, reduced_stations, args.optimisation_spacing_m,
                      args.margin_m, args.boundary_check_spacing_m, args.speed_backend),
        ) as executor:
            best_controls, best, evaluations, sweeps, final_step, reason = (
                _best_improvement_pattern_search(
                    reduced_controls, lower, upper, initial, evaluate, config,
                    lambda candidates: executor.map(_worker_evaluate, candidates),
                    materialise))
    optimisation_elapsed = time.perf_counter() - started

    baseline_common = _require_feasible(
        evaluate_planar_racing_line(
            full_controls, track, bike, full_stations,
            sample_spacing_m=args.common_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend="python"),
        "52-control common-grid baseline",
    )
    reduced_common = _require_feasible(
        evaluate_planar_racing_line(
            best_controls, track, bike, reduced_stations,
            sample_spacing_m=args.common_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend="python"),
        "51-control common-grid result",
    )
    max_disp, rms_disp = phase11screen.geometry_displacement(
        baseline_common.smooth_line.spline,
        reduced_common.smooth_line.spline,
        track.total_length_m,
        args.boundary_check_spacing_m,
    )
    min_forward = _minimum_forward(track, reduced_common)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    controls_csv = args.output_dir / "reduced_51_final_controls.csv"
    phase8.atomic_write_controls_csv(controls_csv, reduced_stations, best_controls, lower, upper)
    racing_csv = args.output_dir / "reduced_51_racing_line.csv"
    racing_png = args.output_dir / "reduced_51_racing_line.png"
    _write_racing_line_csv(racing_csv, track, baseline_common, reduced_common)
    _write_racing_line_png(
        racing_png, track, baseline_common, reduced_common, args.delete_index,
        margin_m=args.margin_m, dpi=args.plot_dpi)

    row = {
        "deleted_original_index": args.delete_index,
        "deleted_station_m": float(full_stations[args.delete_index]),
        "full_control_count": len(full_stations),
        "reduced_control_count": len(reduced_stations),
        "margin_m": args.margin_m,
        "initial_reduced_optimisation_lap_s": float(initial.lap_time_s),
        "final_reduced_optimisation_lap_s": float(best.lap_time_s),
        "baseline_common_grid_lap_s": float(baseline_common.lap_time_s),
        "reduced_common_grid_lap_s": float(reduced_common.lap_time_s),
        "delta_reduced_minus_baseline_common_s": float(
            reduced_common.lap_time_s - baseline_common.lap_time_s),
        "minimum_usable_clearance_m": float(
            reduced_common.smooth_line.minimum_boundary_clearance_m),
        "minimum_track_edge_clearance_m": float(
            reduced_common.smooth_line.minimum_boundary_clearance_m + args.margin_m),
        "minimum_forward_progress": min_forward,
        "maximum_line_displacement_m": max_disp,
        "rms_line_displacement_m": rms_disp,
        "evaluations": evaluations,
        "sweeps": sweeps,
        "final_step_m": final_step,
        "termination_reason": reason,
        "workers": args.workers,
        "speed_backend": args.speed_backend,
        "optimisation_spacing_m": args.optimisation_spacing_m,
        "common_spacing_m": args.common_spacing_m,
        "boundary_check_spacing_m": args.boundary_check_spacing_m,
        "optimisation_elapsed_s": optimisation_elapsed,
        "controls_csv": str(controls_csv),
        "racing_line_csv": str(racing_csv),
        "racing_line_png": str(racing_png),
    }
    summary_csv = args.output_dir / "phase11_reduced_basis_reoptimisation_summary.csv"
    _write_summary(summary_csv, row)

    for key, value in row.items():
        print(f"{key}={value}")
    print(f"summary_csv={summary_csv}")
    return row


if __name__ == "__main__":
    main()
