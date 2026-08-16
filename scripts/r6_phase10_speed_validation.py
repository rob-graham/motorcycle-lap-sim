"""Compare the frozen Mallala R6 ideal-response speed with measured speed envelope.

This is a validation diagnostic, not a calibration command.  It evaluates the
frozen 52-control geometry, maps its speed profile onto nominal track chainage,
and compares that profile with the post-registration measured speed envelope.
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np

import r6_phase9_baseline_check as phase9
from motorcycle_lap_sim.telemetry import (
    compare_speed_envelope,
    summarize_speed_comparison,
    uniform_closed_parameter_grid,
)


REQUIRED_ENVELOPE_COLUMNS = (
    "chainage_m",
    "speed_lap_count",
    "speed_median_mps",
    "speed_p10_mps",
    "speed_p90_mps",
)


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("envelope_csv", type=Path,
                        help="post-registration Phase 10 envelope CSV")
    parser.add_argument("--required-lap-count", type=int, default=5)
    parser.add_argument("--sim-spacing-m", type=float, default=1.0,
                        help="frozen fixed-line output spacing; must be 1.0, 0.5 or 0.25 m")
    parser.add_argument("--comparison-csv", type=Path, default=None)
    return parser


def _load_measured_speed_envelope(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = tuple(reader.fieldnames or ())
        missing = [name for name in REQUIRED_ENVELOPE_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError("speed envelope CSV is missing required columns: " + ", ".join(missing))
        rows = list(reader)
    if not rows:
        raise ValueError("speed envelope CSV contains no data rows")

    def numeric(name):
        result = []
        for row_number, row in enumerate(rows, start=2):
            value = row[name]
            if value is None or value.strip() == "":
                result.append(math.nan)
                continue
            try:
                result.append(float(value))
            except ValueError as exc:
                raise ValueError(
                    f"speed envelope row {row_number} column {name} is not numeric") from exc
        return np.asarray(result, dtype=float)

    return (
        numeric("chainage_m"),
        numeric("speed_median_mps"),
        numeric("speed_p10_mps"),
        numeric("speed_p90_mps"),
        numeric("speed_lap_count"),
    )


def _simulation_at_spacing(spacing_m):
    try:
        index = phase9.OUTPUT_SPACINGS_M.index(float(spacing_m))
    except ValueError as exc:
        raise ValueError(
            f"sim-spacing-m must be one of {phase9.OUTPUT_SPACINGS_M}") from exc
    track, _, _, evaluations = phase9.evaluate_baseline(speed_backend="python")
    evaluation = evaluations[index]
    speed_profile = evaluation.speed_profile
    if speed_profile is None:
        raise RuntimeError("frozen baseline evaluation did not return a speed profile")
    nominal_track_s = uniform_closed_parameter_grid(
        track.total_length_m, len(speed_profile.speed_mps))
    return track, evaluation, nominal_track_s, speed_profile.speed_mps


def _write_comparison_csv(path, comparison):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "chainage_m", "speed_lap_count", "speed_evidence_complete",
            "measured_speed_median_mps", "measured_speed_p10_mps",
            "measured_speed_p90_mps", "simulated_speed_mps",
            "sim_minus_measured_median_mps", "sim_within_measured_p10_p90",
            "sim_above_measured_p90", "sim_below_measured_p10",
        ))
        for index, chainage in enumerate(comparison.chainage_m):
            eligible = bool(comparison.eligible_mask[index])
            simulated = comparison.simulated_mps[index]
            p10 = comparison.measured_p10_mps[index]
            p90 = comparison.measured_p90_mps[index]
            writer.writerow((
                chainage,
                comparison.measured_lap_count[index],
                int(eligible),
                comparison.measured_median_mps[index],
                p10,
                p90,
                simulated,
                comparison.sim_minus_median_mps[index] if eligible else "",
                int(p10 <= simulated <= p90) if eligible else "",
                int(simulated > p90) if eligible else "",
                int(simulated < p10) if eligible else "",
            ))


def main(argv=None):
    args = build_parser().parse_args(argv)
    chainage, median, p10, p90, lap_count = _load_measured_speed_envelope(args.envelope_csv)
    track, evaluation, simulated_s, simulated_speed = _simulation_at_spacing(args.sim_spacing_m)
    comparison = compare_speed_envelope(
        chainage, median, p10, p90, lap_count,
        simulated_s, simulated_speed, track.total_length_m,
        required_lap_count=args.required_lap_count,
    )
    summary = summarize_speed_comparison(comparison)

    print(f"envelope_csv={args.envelope_csv}")
    print(f"required_lap_count={args.required_lap_count}")
    print(f"sim_spacing_m={args.sim_spacing_m:.2f}")
    print(f"simulated_fixed_line_lap_s={evaluation.lap_time_s:.9f}")
    print(f"comparison_bins={len(comparison.chainage_m)}")
    print(f"eligible_complete_lap_bins={summary.eligible_bins}/{len(comparison.chainage_m)}")
    print(f"mean_sim_minus_measured_median_mps={summary.mean_bias_mps:.9f}")
    print(f"median_sim_minus_measured_median_mps={summary.median_bias_mps:.9f}")
    print(f"mean_absolute_speed_error_mps={summary.mean_absolute_error_mps:.9f}")
    print(f"rms_speed_error_mps={summary.rms_error_mps:.9f}")
    print(f"p95_absolute_speed_error_mps={summary.p95_absolute_error_mps:.9f}")
    print(f"maximum_absolute_speed_error_mps={summary.maximum_absolute_error_mps:.9f}")
    print(f"maximum_absolute_speed_error_chainage_m={summary.maximum_absolute_error_chainage_m:.9f}")
    print(f"sim_within_measured_p10_p90_bins={summary.within_p10_p90_bins}/{summary.eligible_bins}")
    print(f"sim_above_measured_p90_bins={summary.above_p90_bins}/{summary.eligible_bins}")
    print(f"sim_below_measured_p10_bins={summary.below_p10_bins}/{summary.eligible_bins}")
    print("comparison_note=nominal track chainage is the common comparison coordinate; the simulator line and measured rider lines differ geometrically, so speed discrepancies are not uniquely attributable to motorcycle parameters")
    print("calibration_note=no motorcycle or track parameters are tuned by this command")

    if args.comparison_csv is not None:
        _write_comparison_csv(args.comparison_csv, comparison)
        print(f"comparison_csv={args.comparison_csv}")


if __name__ == "__main__":
    main()
