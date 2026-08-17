"""Bounded Phase 11 longitudinal control-station relocation screen.

This diagnostic keeps the retained 52 lateral offsets fixed and moves one
eligible control station at a time along track chainage. Primitive-boundary
anchors are protected except the periodic seam control at s=0, which is allowed
to move forward within the start/finish straight. The best feasible relocation
is ranked on the authoritative 0.125 m Python common grid.

This is a screening experiment only. A promising relocated station must be
followed by lateral-offset re-optimisation before it can be judged useful.
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


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_station_relocation")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_station_relocation")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_station_relocation")
phase11screen = _load_sibling("r6_phase11_control_deletion_screen.py", "phase11_station_relocation_base")

DEFAULT_MARGIN_M = 0.25
DEFAULT_MAX_ROLL_RATE_RADPS = 0.8
DEFAULT_SCREEN_SHIFTS_M = (-20.0, -10.0, -5.0, 5.0, 10.0, 20.0)
DEFAULT_MINIMUM_STATION_GAP_M = 5.0
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
    parser.add_argument("--screen-shifts-m", type=float, nargs="+",
                        default=DEFAULT_SCREEN_SHIFTS_M)
    parser.add_argument("--minimum-station-gap-m", type=_positive_float,
                        default=DEFAULT_MINIMUM_STATION_GAP_M)
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


def _primitive_boundary_mask(track, stations, *, atol=1e-9):
    boundaries = phase11screen.primitive_boundary_stations(track)
    return np.array([
        bool(np.any(np.isclose(value, boundaries, rtol=0.0, atol=atol)))
        for value in np.asarray(stations, dtype=float)
    ])


def relocation_eligible_mask(track, stations):
    """Protect primitive boundaries, but allow the periodic seam control at s=0."""
    protected = _primitive_boundary_mask(track, stations)
    if len(protected) and math.isclose(float(stations[0]), 0.0, abs_tol=1e-12):
        protected[0] = False
    return ~protected


def relocated_stations(stations, index, shift_m, track_length_m, minimum_gap_m):
    """Move one station without changing order or crossing neighbouring controls."""
    values = np.asarray(stations, dtype=float).copy()
    if isinstance(index, bool) or not isinstance(index, (int, np.integer)):
        raise ValueError("relocation index must be an integer")
    if index < 0 or index >= len(values):
        raise ValueError("relocation index is outside the control array")
    if not math.isfinite(shift_m):
        raise ValueError("station shift must be finite")
    if not math.isfinite(minimum_gap_m) or minimum_gap_m <= 0.0:
        raise ValueError("minimum station gap must be finite and positive")

    candidate = float(values[index] + shift_m)
    if index == 0:
        lower = 0.0
        upper = float(values[1] - minimum_gap_m)
    elif index == len(values) - 1:
        lower = float(values[-2] + minimum_gap_m)
        upper = float(track_length_m - minimum_gap_m)
    else:
        lower = float(values[index - 1] + minimum_gap_m)
        upper = float(values[index + 1] - minimum_gap_m)
    if lower > upper:
        raise ValueError("minimum station gap leaves no relocation interval")
    if candidate < lower or candidate > upper:
        return None
    values[index] = candidate
    if np.any(np.diff(values) <= 0.0) or values[0] < 0.0 or values[-1] >= track_length_m:
        raise RuntimeError("relocated stations violate strict cyclic ordering")
    return values


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


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


def _write_summary(path, rows):
    fields = (
        "original_index", "original_station_m", "shift_m", "candidate_station_m",
        "eligible", "screen_feasible", "screen_lap_s", "screen_delta_s",
        "common_grid_evaluated", "common_grid_lap_s", "common_grid_delta_s",
        "maximum_line_displacement_m", "rms_line_displacement_m",
        "minimum_usable_clearance_m", "failure_reason",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_racing_line_png(path, track, baseline, candidate, index, *, margin_m, dpi):
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

    original_x = float(baseline.smooth_line.guide_x_m[index])
    original_y = float(baseline.smooth_line.guide_y_m[index])
    moved_x = float(candidate.smooth_line.guide_x_m[index])
    moved_y = float(candidate.smooth_line.guide_y_m[index])
    sf_left, sf_right = _start_finish_segment(track)

    figure, axis = plt.subplots(figsize=(12, 9))
    axis.plot(left_x, left_y, linewidth=0.35, label="Left track edge")
    axis.plot(right_x, right_y, linewidth=0.35, label="Right track edge")
    axis.plot(margin_left_x, margin_left_y, linewidth=0.3, linestyle="--", label="Margin corridor")
    axis.plot(margin_right_x, margin_right_y, linewidth=0.3, linestyle="--")
    axis.plot(checked_track.x_m, checked_track.y_m, linewidth=0.18, linestyle=":",
              label="Centreline")
    axis.plot(baseline_x, baseline_y, linewidth=0.5, linestyle="--", label="Baseline line")
    axis.plot(candidate_x, candidate_y, linewidth=0.8, label="Relocated-station line")
    axis.scatter(candidate.smooth_line.guide_x_m, candidate.smooth_line.guide_y_m,
                 s=7, marker="o", linewidths=0.3, label="Control points", zorder=5)
    axis.scatter([original_x], [original_y], s=24, marker="x", linewidths=0.7,
                 label=f"Original control {index}", zorder=6)
    axis.scatter([moved_x], [moved_y], s=22, marker="s", linewidths=0.5,
                 facecolors="none", label=f"Moved control {index}", zorder=6)
    axis.plot([sf_left[0], sf_right[0]], [sf_left[1], sf_right[1]],
              linewidth=1.0, label="Start / finish", zorder=7)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(f"Mallala station-relocation screen - {margin_m:.3f} m edge margin")
    axis.legend(fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def _write_racing_line_csv(path, track, baseline, candidate):
    count = len(candidate.smooth_line.sampled_path.q_m)
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    sampled_track = sample_track_stations(track, track_s)
    baseline_x, baseline_y, *_ = baseline.smooth_line.spline.evaluate(track_s)
    candidate_x, candidate_y, *_ = candidate.smooth_line.spline.evaluate(track_s)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("track_s_m", "track_center_x_m", "track_center_y_m",
                         "baseline_x_m", "baseline_y_m", "candidate_x_m", "candidate_y_m"))
        writer.writerows(zip(track_s, sampled_track.x_m, sampled_track.y_m,
                             baseline_x, baseline_y, candidate_x, candidate_y))


def main(argv=None):
    args = build_parser().parse_args(argv)
    shifts = tuple(float(value) for value in args.screen_shifts_m)
    if not shifts or any(not math.isfinite(value) or value == 0.0 for value in shifts):
        raise ValueError("screen shifts must be finite non-zero values")
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, args.margin_m)
    controls = phase8.load_initial_controls_csv(args.start_controls_csv, stations, lower, upper)

    baseline_screen = _require_feasible(
        evaluate_planar_racing_line(
            controls, track, bike, stations,
            sample_spacing_m=args.optimisation_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend=args.speed_backend),
        "station-relocation baseline screen",
    )
    baseline_common = _require_feasible(
        evaluate_planar_racing_line(
            controls, track, bike, stations,
            sample_spacing_m=args.common_spacing_m,
            boundary_margin_m=args.margin_m,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            speed_backend="python"),
        "station-relocation baseline common grid",
    )

    eligible = relocation_eligible_mask(track, stations)
    rows = []
    feasible = []
    started = time.perf_counter()
    for index, station in enumerate(stations):
        if not eligible[index]:
            rows.append({
                "original_index": index, "original_station_m": float(station), "shift_m": "",
                "candidate_station_m": "", "eligible": False, "screen_feasible": False,
                "screen_lap_s": "", "screen_delta_s": "", "common_grid_evaluated": False,
                "common_grid_lap_s": "", "common_grid_delta_s": "",
                "maximum_line_displacement_m": "", "rms_line_displacement_m": "",
                "minimum_usable_clearance_m": "", "failure_reason": "primitive boundary protected",
            })
            continue
        for shift in shifts:
            moved = relocated_stations(
                stations, index, shift, track.total_length_m, args.minimum_station_gap_m)
            if moved is None:
                rows.append({
                    "original_index": index, "original_station_m": float(station), "shift_m": shift,
                    "candidate_station_m": float(station + shift), "eligible": True,
                    "screen_feasible": False, "screen_lap_s": "", "screen_delta_s": "",
                    "common_grid_evaluated": False, "common_grid_lap_s": "",
                    "common_grid_delta_s": "", "maximum_line_displacement_m": "",
                    "rms_line_displacement_m": "", "minimum_usable_clearance_m": "",
                    "failure_reason": "shift exceeds bounded neighbour interval",
                })
                continue
            evaluation = evaluate_planar_racing_line(
                controls, track, bike, moved,
                sample_spacing_m=args.optimisation_spacing_m,
                boundary_margin_m=args.margin_m,
                boundary_check_spacing_m=args.boundary_check_spacing_m,
                speed_backend=args.speed_backend,
            )
            row = {
                "original_index": index,
                "original_station_m": float(station),
                "shift_m": shift,
                "candidate_station_m": float(moved[index]),
                "eligible": True,
                "screen_feasible": bool(evaluation.feasible),
                "screen_lap_s": float(evaluation.lap_time_s) if evaluation.feasible else "",
                "screen_delta_s": (float(evaluation.lap_time_s - baseline_screen.lap_time_s)
                                   if evaluation.feasible else ""),
                "common_grid_evaluated": False,
                "common_grid_lap_s": "",
                "common_grid_delta_s": "",
                "maximum_line_displacement_m": "",
                "rms_line_displacement_m": "",
                "minimum_usable_clearance_m": (float(evaluation.smooth_line.minimum_boundary_clearance_m)
                                                if evaluation.feasible else ""),
                "failure_reason": "" if evaluation.feasible else evaluation.failure_reason,
            }
            rows.append(row)
            if evaluation.feasible:
                feasible.append((float(evaluation.lap_time_s), index, shift, moved, row))

    feasible.sort(key=lambda item: (item[0], item[1], item[2]))
    common_candidates = feasible[:args.common_grid_top]
    best_common = None
    for _, index, shift, moved, row in common_candidates:
        evaluation = _require_feasible(
            evaluate_planar_racing_line(
                controls, track, bike, moved,
                sample_spacing_m=args.common_spacing_m,
                boundary_margin_m=args.margin_m,
                boundary_check_spacing_m=args.boundary_check_spacing_m,
                speed_backend="python"),
            f"relocation candidate {index} shift {shift:g} m on common grid",
        )
        max_disp, rms_disp = phase11screen.geometry_displacement(
            baseline_common.smooth_line.spline,
            evaluation.smooth_line.spline,
            track.total_length_m,
            args.boundary_check_spacing_m,
        )
        row["common_grid_evaluated"] = True
        row["common_grid_lap_s"] = float(evaluation.lap_time_s)
        row["common_grid_delta_s"] = float(evaluation.lap_time_s - baseline_common.lap_time_s)
        row["maximum_line_displacement_m"] = max_disp
        row["rms_line_displacement_m"] = rms_disp
        if best_common is None or evaluation.lap_time_s < best_common[0]:
            best_common = (float(evaluation.lap_time_s), index, shift, moved, evaluation, max_disp, rms_disp)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = args.output_dir / "phase11_control_station_relocation_screen.csv"
    _write_summary(summary, rows)

    print(f"start_controls_csv={args.start_controls_csv}")
    print(f"control_count={len(stations)}")
    print(f"eligible_relocation_controls={int(np.sum(eligible))}")
    print(f"screen_shift_count={len(shifts)}")
    print(f"feasible_screen_relocations={len(feasible)}")
    print(f"baseline_screen_lap_s={baseline_screen.lap_time_s:.9f}")
    print(f"baseline_common_grid_lap_s={baseline_common.lap_time_s:.9f}")

    if best_common is not None:
        lap, index, shift, moved, evaluation, max_disp, rms_disp = best_common
        racing_csv = args.output_dir / "best_relocation_racing_line.csv"
        racing_png = args.output_dir / "best_relocation_racing_line.png"
        _write_racing_line_csv(racing_csv, track, baseline_common, evaluation)
        _write_racing_line_png(
            racing_png, track, baseline_common, evaluation, index,
            margin_m=args.margin_m, dpi=args.plot_dpi)
        print(f"best_common_grid_relocation_original_index={index}")
        print(f"best_common_grid_relocation_original_station_m={stations[index]:.9f}")
        print(f"best_common_grid_relocation_shift_m={shift:.9f}")
        print(f"best_common_grid_relocation_station_m={moved[index]:.9f}")
        print(f"best_common_grid_relocation_lap_s={lap:.9f}")
        print(f"best_common_grid_relocation_delta_s={lap - baseline_common.lap_time_s:.9f}")
        print(f"best_common_grid_relocation_maximum_line_displacement_m={max_disp:.9f}")
        print(f"best_common_grid_relocation_rms_line_displacement_m={rms_disp:.9f}")
        print(f"racing_line_csv={racing_csv}")
        print(f"racing_line_png={racing_png}")
    else:
        print("best_common_grid_relocation=none")

    elapsed = time.perf_counter() - started
    print(f"summary_csv={summary}")
    print(f"elapsed_s={elapsed:.3f}")
    return rows


if __name__ == "__main__":
    main()
