"""Phase 11 control-basis deletion sensitivity screen for Mallala.

This diagnostic asks a deliberately limited question before introducing movable
control stations: which existing interior spline controls have little direct
influence on the retained racing line when removed one at a time?

Primitive-boundary controls are protected. Every other control is deleted in
isolation without relocating neighbouring stations or re-optimising offsets.
Candidates are screened on the 1 m optimisation grid with the requested dense
boundary checker. The most promising feasible deletions are then re-evaluated
on the authoritative Python common grid.

A low direct deletion penalty identifies a candidate for a later reduced-basis
re-optimisation; it is not evidence that the control is unnecessary after all
other controls are allowed to move.
"""

import argparse
import csv
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import time

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track, sample_track_stations


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_control_deletion_screen")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_control_deletion_screen")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_control_deletion_screen")

DEFAULT_MARGIN_M = 0.25
DEFAULT_MAX_ROLL_RATE_RADPS = 0.8
DEFAULT_OPTIMISATION_SPACING_M = 1.0
DEFAULT_COMMON_SPACING_M = 0.125
DEFAULT_BOUNDARY_CHECK_SPACING_M = 0.125
DEFAULT_COMMON_GRID_TOP = 8
DEFAULT_PLOT_DPI = 400


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


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("start_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--margin-m", type=_nonnegative_float, default=DEFAULT_MARGIN_M)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float,
                        default=DEFAULT_MAX_ROLL_RATE_RADPS)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="numba")
    parser.add_argument("--optimisation-spacing-m", type=_positive_float,
                        default=DEFAULT_OPTIMISATION_SPACING_M)
    parser.add_argument("--common-spacing-m", type=_positive_float,
                        default=DEFAULT_COMMON_SPACING_M)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float,
                        default=DEFAULT_BOUNDARY_CHECK_SPACING_M)
    parser.add_argument("--common-grid-top", type=_positive_int, default=DEFAULT_COMMON_GRID_TOP)
    parser.add_argument("--plot-dpi", type=_positive_int, default=DEFAULT_PLOT_DPI)
    return parser


def primitive_boundary_stations(track):
    """Return primitive starts, including s=0 and excluding the closed endpoint."""
    starts = []
    station = 0.0
    for primitive in track.primitives:
        starts.append(station)
        station += primitive.length_m
    return np.asarray(starts, dtype=float)


def protected_control_mask(track, control_s_m, *, atol=1e-9):
    """Protect controls that coincide with analytic primitive boundaries."""
    stations = np.asarray(control_s_m, dtype=float)
    boundaries = primitive_boundary_stations(track)
    return np.array([
        bool(np.any(np.isclose(value, boundaries, rtol=0.0, atol=atol)))
        for value in stations
    ], dtype=bool)


def deletion_arrays(control_s_m, controls_m, index):
    """Delete exactly one station/control pair after validating the request."""
    stations = np.asarray(control_s_m, dtype=float)
    controls = np.asarray(controls_m, dtype=float)
    if stations.ndim != 1 or controls.shape != stations.shape:
        raise ValueError("stations and controls must be matching one-dimensional arrays")
    if len(stations) <= 4:
        raise ValueError("at least five controls are required for a deletion screen")
    if isinstance(index, bool) or not isinstance(index, (int, np.integer)):
        raise ValueError("deletion index must be an integer")
    if index < 0 or index >= len(stations):
        raise ValueError("deletion index is outside the control array")
    return np.delete(stations, index), np.delete(controls, index)


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


def geometry_displacement(reference_spline, candidate_spline, track_length_m, spacing_m):
    """Compare two splines at identical analytic track stations."""
    count = max(4, math.ceil(track_length_m / spacing_m))
    stations = np.arange(count, dtype=float) * track_length_m / count
    ref_x, ref_y, *_ = reference_spline.evaluate(stations)
    can_x, can_y, *_ = candidate_spline.evaluate(stations)
    displacement = np.hypot(can_x - ref_x, can_y - ref_y)
    return float(np.max(displacement)), float(np.sqrt(np.mean(displacement ** 2)))


def _write_summary(path, rows):
    fields = (
        "original_index",
        "control_s_m",
        "start_offset_m",
        "protected",
        "protected_reason",
        "screen_feasible",
        "screen_lap_s",
        "screen_delta_vs_baseline_s",
        "maximum_line_displacement_m",
        "rms_line_displacement_m",
        "minimum_usable_clearance_m",
        "common_grid_evaluated",
        "common_grid_lap_s",
        "common_grid_delta_vs_baseline_s",
        "common_grid_minimum_usable_clearance_m",
        "failure_reason",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _sampled_track_s(track_length_m, sample_count):
    return np.arange(sample_count, dtype=float) * track_length_m / sample_count


def write_racing_line_csv(path, track, baseline, candidate):
    """Write baseline and best deletion line on the same candidate output grid."""
    sampled = candidate.smooth_line.sampled_path
    count = len(sampled.q_m)
    track_s = _sampled_track_s(track.total_length_m, count)
    sampled_track = sample_track_stations(track, track_s)
    base_x, base_y, *_ = baseline.smooth_line.spline.evaluate(track_s)
    candidate_x, candidate_y, *_ = candidate.smooth_line.spline.evaluate(track_s)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "track_s_m", "track_center_x_m", "track_center_y_m",
            "baseline_x_m", "baseline_y_m", "candidate_x_m", "candidate_y_m",
        ))
        writer.writerows(zip(
            track_s,
            sampled_track.x_m,
            sampled_track.y_m,
            base_x,
            base_y,
            candidate_x,
            candidate_y,
        ))


def write_racing_line_png(path, track, baseline, candidate, deleted_index, *, margin_m, dpi):
    """Plot baseline and best deletion candidate with the removed guide identified."""
    checked_s = candidate.smooth_line.evaluated_track_s_m
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
    baseline_x, baseline_y, *_ = baseline.smooth_line.spline.evaluate(checked_s)
    candidate_x, candidate_y, *_ = candidate.smooth_line.spline.evaluate(checked_s)

    deleted_x = baseline.smooth_line.guide_x_m[deleted_index]
    deleted_y = baseline.smooth_line.guide_y_m[deleted_index]

    figure, axis = plt.subplots(figsize=(12, 9))
    axis.plot(left_x, left_y, linewidth=0.35, label="Left track edge")
    axis.plot(right_x, right_y, linewidth=0.35, label="Right track edge")
    axis.plot(margin_left_x, margin_left_y, linewidth=0.3, linestyle="--", label="Margin corridor")
    axis.plot(margin_right_x, margin_right_y, linewidth=0.3, linestyle="--")
    axis.plot(checked_track.x_m, checked_track.y_m, linewidth=0.25, linestyle=":",
              label="Centreline")
    axis.plot(baseline_x, baseline_y, linewidth=0.55, linestyle="--", label="Baseline line")
    axis.plot(candidate_x, candidate_y, linewidth=0.8, label="Deleted-control line")
    axis.scatter(candidate.smooth_line.guide_x_m, candidate.smooth_line.guide_y_m,
                 s=7, marker="o", linewidths=0.3, label="Retained control points", zorder=5)
    axis.scatter([deleted_x], [deleted_y], s=22, marker="x", linewidths=0.7,
                 label=f"Deleted control {deleted_index}", zorder=6)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(f"Mallala control-deletion screen - {margin_m:.3f} m edge margin")
    axis.legend(fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, args.margin_m)
    controls = phase8.load_initial_controls_csv(args.start_controls_csv, stations, lower, upper)
    protected = protected_control_mask(track, stations)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    baseline_screen = _require_feasible(
        evaluate_planar_racing_line(
            controls, track, bike, stations,
            sample_spacing_m=args.optimisation_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend=args.speed_backend,
        ),
        "baseline deletion-screen line",
    )
    baseline_common = _require_feasible(
        evaluate_planar_racing_line(
            controls, track, bike, stations,
            sample_spacing_m=args.common_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend="python",
        ),
        "baseline common-grid line",
    )

    rows = []
    feasible_candidates = []
    for index, (station, offset) in enumerate(zip(stations, controls)):
        row = {
            "original_index": index,
            "control_s_m": float(station),
            "start_offset_m": float(offset),
            "protected": bool(protected[index]),
            "protected_reason": "primitive_boundary" if protected[index] else "",
            "screen_feasible": False,
            "screen_lap_s": math.nan,
            "screen_delta_vs_baseline_s": math.nan,
            "maximum_line_displacement_m": math.nan,
            "rms_line_displacement_m": math.nan,
            "minimum_usable_clearance_m": math.nan,
            "common_grid_evaluated": False,
            "common_grid_lap_s": math.nan,
            "common_grid_delta_vs_baseline_s": math.nan,
            "common_grid_minimum_usable_clearance_m": math.nan,
            "failure_reason": "",
        }
        if protected[index]:
            rows.append(row)
            continue

        candidate_stations, candidate_controls = deletion_arrays(stations, controls, index)
        evaluation = evaluate_planar_racing_line(
            candidate_controls, track, bike, candidate_stations,
            sample_spacing_m=args.optimisation_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend=args.speed_backend,
        )
        if not evaluation.feasible:
            row["failure_reason"] = evaluation.failure_reason or "infeasible"
            rows.append(row)
            continue

        max_disp, rms_disp = geometry_displacement(
            baseline_screen.smooth_line.spline,
            evaluation.smooth_line.spline,
            track.total_length_m,
            args.boundary_check_spacing_m,
        )
        row.update({
            "screen_feasible": True,
            "screen_lap_s": float(evaluation.lap_time_s),
            "screen_delta_vs_baseline_s": float(evaluation.lap_time_s - baseline_screen.lap_time_s),
            "maximum_line_displacement_m": max_disp,
            "rms_line_displacement_m": rms_disp,
            "minimum_usable_clearance_m": float(evaluation.smooth_line.minimum_boundary_clearance_m),
        })
        rows.append(row)
        feasible_candidates.append((float(evaluation.lap_time_s), index,
                                    candidate_stations, candidate_controls))

    feasible_candidates.sort(key=lambda item: (item[0], item[1]))
    common_results = {}
    for _, index, candidate_stations, candidate_controls in feasible_candidates[:args.common_grid_top]:
        common = _require_feasible(
            evaluate_planar_racing_line(
                candidate_controls, track, bike, candidate_stations,
                sample_spacing_m=args.common_spacing_m,
                boundary_margin_m=args.margin_m,
                boundary_check_spacing_m=args.boundary_check_spacing_m,
                speed_backend="python",
            ),
            f"control {index} deletion on common grid",
        )
        common_results[index] = common
        row = rows[index]
        row["common_grid_evaluated"] = True
        row["common_grid_lap_s"] = float(common.lap_time_s)
        row["common_grid_delta_vs_baseline_s"] = float(common.lap_time_s - baseline_common.lap_time_s)
        row["common_grid_minimum_usable_clearance_m"] = float(
            common.smooth_line.minimum_boundary_clearance_m)

    summary_csv = args.output_dir / "phase11_control_deletion_screen.csv"
    _write_summary(summary_csv, rows)

    best_index = None
    if common_results:
        best_index = min(common_results, key=lambda idx: (common_results[idx].lap_time_s, idx))
        best = common_results[best_index]
        racing_csv = args.output_dir / "best_deletion_racing_line.csv"
        racing_png = args.output_dir / "best_deletion_racing_line.png"
        write_racing_line_csv(racing_csv, track, baseline_common, best)
        write_racing_line_png(
            racing_png, track, baseline_common, best, best_index,
            margin_m=args.margin_m, dpi=args.plot_dpi)
    else:
        racing_csv = None
        racing_png = None

    elapsed = time.perf_counter() - started
    eligible_count = int(np.count_nonzero(~protected))
    feasible_count = len(feasible_candidates)
    print(f"start_controls_csv={args.start_controls_csv}")
    print(f"control_count={len(stations)}")
    print(f"protected_primitive_boundary_controls={int(np.count_nonzero(protected))}")
    print(f"eligible_deletion_controls={eligible_count}")
    print(f"feasible_screen_deletions={feasible_count}")
    print(f"margin_m={args.margin_m:.6f}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print("deletion_note=one control is removed at a time without station relocation or offset re-optimisation")
    print(f"optimisation_sample_spacing_m={args.optimisation_spacing_m:.6f}")
    print(f"common_ranking_spacing_m={args.common_spacing_m:.6f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.6f}")
    print(f"speed_backend={args.speed_backend}")
    print(f"baseline_screen_lap_s={baseline_screen.lap_time_s:.9f}")
    print(f"baseline_common_grid_lap_s={baseline_common.lap_time_s:.9f}")
    if best_index is not None:
        best_row = rows[best_index]
        print(f"best_common_grid_deletion_original_index={best_index}")
        print(f"best_common_grid_deletion_control_s_m={best_row['control_s_m']:.9f}")
        print(f"best_common_grid_deletion_start_offset_m={best_row['start_offset_m']:.9f}")
        print(f"best_common_grid_deletion_lap_s={best_row['common_grid_lap_s']:.9f}")
        print(f"best_common_grid_deletion_delta_s={best_row['common_grid_delta_vs_baseline_s']:.9f}")
        print(f"best_common_grid_deletion_maximum_line_displacement_m={best_row['maximum_line_displacement_m']:.9f}")
        print(f"best_common_grid_deletion_rms_line_displacement_m={best_row['rms_line_displacement_m']:.9f}")
        print(f"racing_line_csv={racing_csv}")
        print(f"racing_line_png={racing_png}")
    print(f"summary_csv={summary_csv}")
    print(f"elapsed_s={elapsed:.3f}")
    return rows


if __name__ == "__main__":
    main()
