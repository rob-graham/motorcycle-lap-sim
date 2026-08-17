"""Diagnose Mallala Phase 12A corner-region segmentation before event extraction.

This command intentionally does not relax the nine-corner acceptance gate.  It
reproduces the retained Phase 11 representative line, runs the current
lean-hysteresis corner detector, and writes enough information to show where an
intended Mallala corner is being split into multiple regions.
"""

import argparse
import csv
import importlib.util
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.coaching import EventDetectionConfig, detect_corner_regions


EXPECTED_MALLALA_CORNERS = 9


def _load_phase12a():
    path = Path(__file__).resolve().with_name("r6_phase12a_coaching_events.py")
    spec = importlib.util.spec_from_file_location("phase12a_corner_diagnostics_base", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Phase 12A script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_parser():
    base = _load_phase12a()
    parser = argparse.ArgumentParser()
    parser.add_argument("representative_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--delete-index", type=int, default=base.DEFAULT_DELETE_INDEX)
    parser.add_argument("--margin-m", type=float, default=base.DEFAULT_MARGIN_M)
    parser.add_argument("--max-roll-rate-radps", type=float, default=base.DEFAULT_MAX_ROLL_RATE_RADPS)
    parser.add_argument("--spacing-m", type=float, default=base.DEFAULT_SPACING_M)
    parser.add_argument(
        "--boundary-check-spacing-m", type=float,
        default=base.DEFAULT_BOUNDARY_CHECK_SPACING_M,
    )
    parser.add_argument("--expected-lap-s", type=float, default=base.DEFAULT_EXPECTED_LAP_S)
    parser.add_argument("--lap-tolerance-s", type=float, default=base.DEFAULT_LAP_TOLERANCE_S)
    parser.add_argument("--plot-dpi", type=int, default=220)
    return parser


def _evaluate(args):
    base = _load_phase12a()
    phase8 = base._load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_phase12a_diag")
    phase9 = base._load_sibling("r6_phase9_baseline_check.py", "phase9_phase12a_diag")
    phase9f = base._load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_phase12a_diag")
    trajectory = base._load_sibling("r6_phase10_trajectory_export.py", "trajectory_phase12a_diag")

    from dataclasses import replace
    from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
    from motorcycle_lap_sim.optimisation import (
        REFERENCE_PLANAR_CONTROL_POLICY,
        evaluate_planar_racing_line,
        generate_planar_control_stations,
        planar_control_bounds,
    )
    from motorcycle_lap_sim.track import Track

    phase9f._require_canonical_inputs()
    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    bike = replace(
        base_bike,
        handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps),
    )
    standard_stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    representative_stations = np.delete(standard_stations, args.delete_index)
    lower, upper = planar_control_bounds(track, representative_stations, args.margin_m)
    controls = phase8.load_initial_controls_csv(
        args.representative_controls_csv, representative_stations, lower, upper)
    evaluation = evaluate_planar_racing_line(
        controls,
        track,
        bike,
        representative_stations,
        sample_spacing_m=args.spacing_m,
        boundary_margin_m=args.margin_m,
        boundary_check_spacing_m=args.boundary_check_spacing_m,
        speed_backend="python",
    )
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"retained representative line is infeasible: {evaluation.failure_reason}")
    lap_delta = float(evaluation.lap_time_s - args.expected_lap_s)
    if abs(lap_delta) > args.lap_tolerance_s:
        raise RuntimeError(
            "representative line lap does not reproduce retained Phase 11 reference: "
            f"actual={evaluation.lap_time_s:.9f} expected={args.expected_lap_s:.9f} "
            f"delta={lap_delta:+.9f} s")
    columns = trajectory.trajectory_columns(
        track, evaluation.smooth_line, evaluation.speed_profile, bike)
    return track, evaluation, columns, lap_delta


def _region_rows(columns, regions):
    s = np.asarray(columns["track_s_m"], dtype=float)
    q = np.asarray(columns["path_q_m"], dtype=float)
    lean = np.asarray(columns["roll_angle_deg"], dtype=float)
    curvature = np.asarray(columns["path_curvature_1pm"], dtype=float)
    speed = np.asarray(columns["speed_mps"], dtype=float)
    rows = []
    for number, (start, end) in enumerate(regions, start=1):
        local_lean = lean[start:end + 1]
        local_curvature = curvature[start:end + 1]
        peak_rel = int(np.argmax(np.abs(local_lean)))
        apex_rel = int(np.argmax(np.abs(local_curvature)))
        sign = 1 if float(local_lean[peak_rel]) >= 0.0 else -1
        gap_to_next_m = ""
        min_abs_lean_in_gap_deg = ""
        if number < len(regions):
            next_start = regions[number][0]
            gap_to_next_m = float(q[next_start] - q[end])
            if next_start > end + 1:
                min_abs_lean_in_gap_deg = float(np.min(np.abs(lean[end + 1:next_start])))
        rows.append({
            "detected_region": number,
            "turn_sign": sign,
            "start_index": start,
            "end_index": end,
            "start_track_s_m": float(s[start]),
            "end_track_s_m": float(s[end]),
            "start_path_q_m": float(q[start]),
            "end_path_q_m": float(q[end]),
            "length_path_m": float(q[end] - q[start]),
            "peak_abs_lean_deg": float(abs(local_lean[peak_rel])),
            "peak_lean_track_s_m": float(s[start + peak_rel]),
            "peak_abs_curvature_1pm": float(abs(local_curvature[apex_rel])),
            "curvature_apex_track_s_m": float(s[start + apex_rel]),
            "minimum_speed_kph": float(np.min(speed[start:end + 1]) * 3.6),
            "gap_to_next_path_m": gap_to_next_m,
            "min_abs_lean_in_gap_deg": min_abs_lean_in_gap_deg,
        })
    return rows


def _write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _write_plot(path, columns, regions, config, *, dpi):
    import matplotlib.pyplot as plt

    s = np.asarray(columns["track_s_m"], dtype=float)
    lean = np.asarray(columns["roll_angle_deg"], dtype=float)
    figure, axis = plt.subplots(figsize=(14, 6))
    axis.plot(s, lean, linewidth=0.9, label="Demanded lean")
    axis.axhline(config.corner_lean_on_deg, linewidth=0.6, linestyle="--", label="lean on/off thresholds")
    axis.axhline(-config.corner_lean_on_deg, linewidth=0.6, linestyle="--")
    axis.axhline(config.corner_lean_off_deg, linewidth=0.5, linestyle=":")
    axis.axhline(-config.corner_lean_off_deg, linewidth=0.5, linestyle=":")
    for number, (start, end) in enumerate(regions, start=1):
        axis.axvspan(s[start], s[end], alpha=0.12)
        midpoint = 0.5 * (s[start] + s[end])
        peak = float(np.max(np.abs(lean[start:end + 1])))
        sign = 1.0 if lean[start + int(np.argmax(np.abs(lean[start:end + 1])))] >= 0.0 else -1.0
        axis.text(midpoint, sign * max(8.0, 0.75 * peak), f"R{number}", ha="center", va="center", fontsize=8)
    axis.set_xlabel("Track chainage (m)")
    axis.set_ylabel("Demanded lean (deg)")
    axis.set_title("Phase 12A Mallala corner-region diagnostic")
    axis.grid(True, linewidth=0.25)
    axis.legend(fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def main(argv=None):
    args = build_parser().parse_args(argv)
    track, evaluation, columns, lap_delta = _evaluate(args)
    config = EventDetectionConfig()
    regions = detect_corner_regions(columns, config)
    rows = _region_rows(columns, regions)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "phase12a_detected_corner_regions.csv"
    png_path = args.output_dir / "phase12a_detected_corner_regions.png"
    _write_csv(csv_path, rows)
    _write_plot(png_path, columns, regions, config, dpi=args.plot_dpi)

    print("phase=12A_corner_region_diagnostics")
    print(f"lap_s={evaluation.lap_time_s:.9f}")
    print(f"lap_delta_from_phase11_reference_s={lap_delta:+.9f}")
    print(f"detected_corner_regions={len(regions)}")
    print(f"expected_mallala_corners={EXPECTED_MALLALA_CORNERS}")
    print(f"corner_lean_on_deg={config.corner_lean_on_deg:.6f}")
    print(f"corner_lean_off_deg={config.corner_lean_off_deg:.6f}")
    print(f"minimum_corner_length_m={config.minimum_corner_length_m:.6f}")
    print(f"merge_same_direction_gap_m={config.merge_same_direction_gap_m:.6f}")
    for row in rows:
        print(
            "region="
            f"{row['detected_region']} sign={row['turn_sign']:+d} "
            f"track_s_m={row['start_track_s_m']:.3f}:{row['end_track_s_m']:.3f} "
            f"length_m={row['length_path_m']:.3f} "
            f"peak_lean_deg={row['peak_abs_lean_deg']:.3f} "
            f"gap_to_next_m={row['gap_to_next_path_m']} "
            f"min_abs_lean_in_gap_deg={row['min_abs_lean_in_gap_deg']}"
        )
    print(f"regions_csv={csv_path}")
    print(f"regions_png={png_path}")
    if len(regions) != EXPECTED_MALLALA_CORNERS:
        print("status=diagnostic_mismatch_preserved")
        print("next_step=inspect split regions before changing thresholds or merge rules")
    else:
        print("status=nine_regions_detected")


if __name__ == "__main__":
    main()
