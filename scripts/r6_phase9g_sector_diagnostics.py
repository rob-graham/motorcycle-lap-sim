"""Sector-level Phase 9G comparison for the Mallala R6 validation case.

This diagnostic attributes the whole-lap differences between three reviewed
simulator cases to fixed nominal-track-chainage sectors:

1. frozen 52-control ideal-response line,
2. the same frozen line with an explicit Level-1 finite roll-rate scenario,
3. a supplied roll-aware re-optimised line with that same roll-rate scenario.

The sector table is intended to answer where finite roll changes the lap and
measured-speed agreement, and where re-optimising the line earns time back.
It also reports curvature-transition severity, roll binding, longitudinal
acceleration range, measured lateral-offset agreement, and line movement.
Nothing in this command calibrates motorcycle, rider, track, registration, or
roll-rate parameters.
"""

import argparse
import csv
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


DEFAULT_SECTOR_LENGTH_M = 100.0
DEFAULT_TOP_SECTORS = 6
TIME_SUM_ATOL_S = 1e-9

CSV_FIELDS = (
    "sector_index",
    "sector_start_m",
    "sector_end_m",
    "sector_length_m",
    "measured_speed_bins",
    "measured_offset_bins",
    "ideal_time_s",
    "frozen_roll_time_s",
    "roll_aware_time_s",
    "finite_roll_time_penalty_s",
    "roll_aware_time_gain_s",
    "roll_aware_vs_ideal_time_delta_s",
    "ideal_speed_mae_mps",
    "frozen_roll_speed_mae_mps",
    "roll_aware_speed_mae_mps",
    "finite_roll_speed_mae_improvement_mps",
    "roll_aware_speed_mae_change_vs_frozen_mps",
    "ideal_offset_mae_m",
    "roll_aware_offset_mae_m",
    "roll_aware_offset_mae_change_m",
    "frozen_roll_samples",
    "roll_aware_samples",
    "frozen_roll_binding_samples",
    "roll_aware_binding_samples",
    "frozen_roll_binding_fraction",
    "roll_aware_binding_fraction",
    "frozen_roll_peak_abs_roll_rate_radps",
    "roll_aware_peak_abs_roll_rate_radps",
    "frozen_roll_peak_abs_curvature_gradient_1pm2",
    "roll_aware_peak_abs_curvature_gradient_1pm2",
    "frozen_roll_min_longitudinal_acceleration_mps2",
    "frozen_roll_max_longitudinal_acceleration_mps2",
    "roll_aware_min_longitudinal_acceleration_mps2",
    "roll_aware_max_longitudinal_acceleration_mps2",
    "line_mean_abs_offset_change_m",
    "line_rms_offset_change_m",
    "line_max_abs_offset_change_m",
)


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


spatial = _load_sibling(
    "r6_phase9f_spatial_comparison.py", "r6_phase9f_for_sector_diagnostics")
phase8 = spatial.phase8
phase9 = spatial.phase9


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def _positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("envelope_csv", type=Path)
    parser.add_argument("roll_aware_controls_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument(
        "--sim-spacing-m", type=float, choices=spatial.SIM_SPACINGS_M, default=0.25)
    parser.add_argument(
        "--sector-length-m", type=_positive_float, default=DEFAULT_SECTOR_LENGTH_M)
    parser.add_argument("--top-sectors", type=_positive_int, default=DEFAULT_TOP_SECTORS)
    parser.add_argument("--required-lap-count", type=_positive_int, default=5)
    parser.add_argument("--expected-complete-lap-bins", type=int, default=219)
    return parser


def _sector_edges(total_length_m, sector_length_m):
    if not math.isfinite(total_length_m) or total_length_m <= 0.0:
        raise ValueError("total length must be finite and positive")
    if not math.isfinite(sector_length_m) or sector_length_m <= 0.0:
        raise ValueError("sector length must be finite and positive")
    count = int(math.ceil(total_length_m / sector_length_m))
    starts = np.arange(count, dtype=float) * sector_length_m
    ends = np.minimum(starts + sector_length_m, total_length_m)
    return starts, ends


def _sector_indices(chainage_m, total_length_m, sector_length_m, sector_count):
    values = np.mod(np.asarray(chainage_m, dtype=float), total_length_m)
    indices = np.floor(values / sector_length_m).astype(int)
    return np.minimum(indices, sector_count - 1)


def _segment_times(evaluation):
    if evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise ValueError("evaluation must include a smooth line and speed profile")
    path = evaluation.smooth_line.sampled_path
    speed = evaluation.speed_profile
    following = np.roll(speed.speed_mps, -1)
    denominator = speed.speed_mps + following
    if np.any(denominator <= 0.0):
        raise RuntimeError("segment-time calculation encountered non-positive speed sum")
    segment_time = 2.0 * path.segment_lengths_m / denominator
    total = float(np.sum(segment_time))
    if not math.isclose(total, speed.lap_time_s, rel_tol=0.0, abs_tol=TIME_SUM_ATOL_S):
        raise RuntimeError(
            f"segment times sum to {total:.12f} s but solver lap is {speed.lap_time_s:.12f} s")
    return segment_time


def _sector_time_totals(track, evaluation, sector_length_m, sector_count):
    segment_time = _segment_times(evaluation)
    count = len(segment_time)
    track_step = track.total_length_m / count
    segment_midpoint_s = np.mod(
        np.arange(count, dtype=float) * track_step + 0.5 * track_step,
        track.total_length_m,
    )
    indices = _sector_indices(
        segment_midpoint_s, track.total_length_m, sector_length_m, sector_count)
    totals = np.bincount(indices, weights=segment_time, minlength=sector_count)
    if not math.isclose(
            float(np.sum(totals)), evaluation.speed_profile.lap_time_s,
            rel_tol=0.0, abs_tol=TIME_SUM_ATOL_S):
        raise RuntimeError("sector time totals do not reproduce solver lap time")
    return totals


def _masked_mae(simulated, measured, mask):
    selected = np.asarray(mask, dtype=bool)
    if not np.any(selected):
        return math.nan
    error = np.asarray(simulated, dtype=float)[selected] - np.asarray(measured, dtype=float)[selected]
    return float(np.mean(np.abs(error)))


def _masked_peak_abs(values, mask):
    selected = np.asarray(mask, dtype=bool)
    if not np.any(selected):
        return math.nan
    return float(np.max(np.abs(np.asarray(values, dtype=float)[selected])))


def _masked_min(values, mask):
    selected = np.asarray(mask, dtype=bool)
    if not np.any(selected):
        return math.nan
    return float(np.min(np.asarray(values, dtype=float)[selected]))


def _masked_max(values, mask):
    selected = np.asarray(mask, dtype=bool)
    if not np.any(selected):
        return math.nan
    return float(np.max(np.asarray(values, dtype=float)[selected]))


def _line_change_on_reference_grid(track, cases):
    reference_s = cases["ideal"]["track_s_m"]
    ideal_offset = cases["ideal"]["lateral_offset_m"]
    roll_aware_offset = spatial._periodic_interp(
        reference_s,
        cases["roll_aware"]["track_s_m"],
        cases["roll_aware"]["lateral_offset_m"],
        track.total_length_m,
    )
    return reference_s, roll_aware_offset - ideal_offset


def _build_sector_rows(
        track, evaluations, cases, sampled, envelope,
        eligible_speed, eligible_offset, sector_length_m):
    starts, ends = _sector_edges(track.total_length_m, sector_length_m)
    sector_count = len(starts)
    times = {
        name: _sector_time_totals(track, evaluation, sector_length_m, sector_count)
        for name, evaluation in evaluations.items()
    }

    envelope_sector = _sector_indices(
        envelope["chainage_m"], track.total_length_m, sector_length_m, sector_count)
    case_sector = {
        name: _sector_indices(
            case["track_s_m"], track.total_length_m, sector_length_m, sector_count)
        for name, case in cases.items()
    }
    reference_s, line_change = _line_change_on_reference_grid(track, cases)
    line_sector = _sector_indices(
        reference_s, track.total_length_m, sector_length_m, sector_count)

    measured_speed = envelope["speed_median_mps"]
    measured_offset = envelope["offset_median_m"]
    rows = []
    for sector in range(sector_count):
        speed_mask = eligible_speed & (envelope_sector == sector)
        offset_mask = eligible_offset & (envelope_sector == sector)
        frozen_mask = case_sector["frozen_roll"] == sector
        aware_mask = case_sector["roll_aware"] == sector
        line_mask = line_sector == sector

        ideal_speed_mae = _masked_mae(
            sampled["ideal"]["speed_mps"], measured_speed, speed_mask)
        frozen_speed_mae = _masked_mae(
            sampled["frozen_roll"]["speed_mps"], measured_speed, speed_mask)
        aware_speed_mae = _masked_mae(
            sampled["roll_aware"]["speed_mps"], measured_speed, speed_mask)
        ideal_offset_mae = _masked_mae(
            sampled["ideal"]["lateral_offset_m"], measured_offset, offset_mask)
        aware_offset_mae = _masked_mae(
            sampled["roll_aware"]["lateral_offset_m"], measured_offset, offset_mask)

        frozen_binding = np.asarray(cases["frozen_roll"]["roll_binding"], dtype=bool)
        aware_binding = np.asarray(cases["roll_aware"]["roll_binding"], dtype=bool)
        frozen_samples = int(np.count_nonzero(frozen_mask))
        aware_samples = int(np.count_nonzero(aware_mask))
        frozen_binding_count = int(np.count_nonzero(frozen_mask & frozen_binding))
        aware_binding_count = int(np.count_nonzero(aware_mask & aware_binding))

        line_values = line_change[line_mask]
        if len(line_values):
            line_mean_abs = float(np.mean(np.abs(line_values)))
            line_rms = float(np.sqrt(np.mean(line_values ** 2)))
            line_max = float(np.max(np.abs(line_values)))
        else:
            line_mean_abs = line_rms = line_max = math.nan

        frozen_time = float(times["frozen_roll"][sector])
        ideal_time = float(times["ideal"][sector])
        aware_time = float(times["roll_aware"][sector])
        rows.append({
            "sector_index": sector,
            "sector_start_m": float(starts[sector]),
            "sector_end_m": float(ends[sector]),
            "sector_length_m": float(ends[sector] - starts[sector]),
            "measured_speed_bins": int(np.count_nonzero(speed_mask)),
            "measured_offset_bins": int(np.count_nonzero(offset_mask)),
            "ideal_time_s": ideal_time,
            "frozen_roll_time_s": frozen_time,
            "roll_aware_time_s": aware_time,
            "finite_roll_time_penalty_s": frozen_time - ideal_time,
            "roll_aware_time_gain_s": frozen_time - aware_time,
            "roll_aware_vs_ideal_time_delta_s": aware_time - ideal_time,
            "ideal_speed_mae_mps": ideal_speed_mae,
            "frozen_roll_speed_mae_mps": frozen_speed_mae,
            "roll_aware_speed_mae_mps": aware_speed_mae,
            "finite_roll_speed_mae_improvement_mps": ideal_speed_mae - frozen_speed_mae,
            "roll_aware_speed_mae_change_vs_frozen_mps": aware_speed_mae - frozen_speed_mae,
            "ideal_offset_mae_m": ideal_offset_mae,
            "roll_aware_offset_mae_m": aware_offset_mae,
            "roll_aware_offset_mae_change_m": aware_offset_mae - ideal_offset_mae,
            "frozen_roll_samples": frozen_samples,
            "roll_aware_samples": aware_samples,
            "frozen_roll_binding_samples": frozen_binding_count,
            "roll_aware_binding_samples": aware_binding_count,
            "frozen_roll_binding_fraction": (
                frozen_binding_count / frozen_samples if frozen_samples else math.nan),
            "roll_aware_binding_fraction": (
                aware_binding_count / aware_samples if aware_samples else math.nan),
            "frozen_roll_peak_abs_roll_rate_radps": _masked_peak_abs(
                cases["frozen_roll"]["roll_rate_model_radps"], frozen_mask),
            "roll_aware_peak_abs_roll_rate_radps": _masked_peak_abs(
                cases["roll_aware"]["roll_rate_model_radps"], aware_mask),
            "frozen_roll_peak_abs_curvature_gradient_1pm2": _masked_peak_abs(
                cases["frozen_roll"]["curvature_gradient_1pm2"], frozen_mask),
            "roll_aware_peak_abs_curvature_gradient_1pm2": _masked_peak_abs(
                cases["roll_aware"]["curvature_gradient_1pm2"], aware_mask),
            "frozen_roll_min_longitudinal_acceleration_mps2": _masked_min(
                evaluations["frozen_roll"].speed_profile.longitudinal_acceleration_mps2,
                frozen_mask),
            "frozen_roll_max_longitudinal_acceleration_mps2": _masked_max(
                evaluations["frozen_roll"].speed_profile.longitudinal_acceleration_mps2,
                frozen_mask),
            "roll_aware_min_longitudinal_acceleration_mps2": _masked_min(
                evaluations["roll_aware"].speed_profile.longitudinal_acceleration_mps2,
                aware_mask),
            "roll_aware_max_longitudinal_acceleration_mps2": _masked_max(
                evaluations["roll_aware"].speed_profile.longitudinal_acceleration_mps2,
                aware_mask),
            "line_mean_abs_offset_change_m": line_mean_abs,
            "line_rms_offset_change_m": line_rms,
            "line_max_abs_offset_change_m": line_max,
        })
    return rows


def _require_sector_sum(rows, key, expected, label):
    actual = float(sum(row[key] for row in rows))
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=TIME_SUM_ATOL_S):
        raise RuntimeError(
            f"{label} sector sum {actual:.12f} s does not match whole-lap value {expected:.12f} s")
    return actual


def _finite_rows(rows, key):
    return [row for row in rows if math.isfinite(float(row[key]))]


def _print_top(rows, key, label, count, descending=True):
    candidates = _finite_rows(rows, key)
    candidates.sort(key=lambda row: float(row[key]), reverse=descending)
    print(f"{label}:")
    for row in candidates[:min(count, len(candidates))]:
        print(
            f"  sector={row['sector_index']:02d} "
            f"chainage_m={row['sector_start_m']:.1f}:{row['sector_end_m']:.1f} "
            f"{key}={float(row[key]):.9f}")


def _csv_value(value):
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return ""
    return value


def _write_csv(path, rows):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        for row in rows:
            writer.writerow([_csv_value(row[field]) for field in CSV_FIELDS])


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.expected_complete_lap_bins < 0:
        raise ValueError("expected-complete-lap-bins must be non-negative")

    spatial._require_canonical_inputs()
    envelope = spatial._load_envelope(args.envelope_csv)
    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    roll_bike = SimpleNamespace(value=None)
    roll_bike = __import__("dataclasses").replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    frozen_controls = phase9.load_frozen_controls(
        phase9.DEFAULT_CONTROLS, stations, lower, upper)
    roll_aware_controls = phase8.load_initial_controls_csv(
        args.roll_aware_controls_csv, stations, lower, upper)

    evaluations = {
        "ideal": spatial._evaluate(
            track, base_bike, stations, frozen_controls, args.sim_spacing_m),
        "frozen_roll": spatial._evaluate(
            track, roll_bike, stations, frozen_controls, args.sim_spacing_m),
        "roll_aware": spatial._evaluate(
            track, roll_bike, stations, roll_aware_controls, args.sim_spacing_m),
    }
    cases = {
        name: spatial._sim_columns(track, evaluation)
        for name, evaluation in evaluations.items()
    }
    sampled = {
        name: spatial._sample_case(case, envelope["chainage_m"], track.total_length_m)
        for name, case in cases.items()
    }

    comparisons = {
        name: spatial._speed_summary(
            envelope, case, track.total_length_m, args.required_lap_count)[0]
        for name, case in cases.items()
    }
    eligible_speed = comparisons["ideal"].eligible_mask
    if not all(np.array_equal(comparison.eligible_mask, eligible_speed)
               for comparison in comparisons.values()):
        raise RuntimeError("speed-comparison eligibility differs between simulator cases")
    eligible_speed_count = int(np.count_nonzero(eligible_speed))
    if eligible_speed_count != args.expected_complete_lap_bins:
        raise RuntimeError(
            f"measured envelope has {eligible_speed_count}/{len(eligible_speed)} complete-lap speed bins, "
            f"expected {args.expected_complete_lap_bins}/{len(eligible_speed)}")

    eligible_offset = np.asarray(envelope["offset_lap_count"]) >= args.required_lap_count
    eligible_offset &= np.isfinite(envelope["offset_median_m"])
    eligible_offset &= np.isfinite(envelope["offset_p10_m"])
    eligible_offset &= np.isfinite(envelope["offset_p90_m"])

    rows = _build_sector_rows(
        track, evaluations, cases, sampled, envelope,
        eligible_speed, eligible_offset, args.sector_length_m)

    ideal_lap = evaluations["ideal"].lap_time_s
    frozen_lap = evaluations["frozen_roll"].lap_time_s
    aware_lap = evaluations["roll_aware"].lap_time_s
    finite_roll_penalty = frozen_lap - ideal_lap
    roll_aware_gain = frozen_lap - aware_lap
    _require_sector_sum(
        rows, "finite_roll_time_penalty_s", finite_roll_penalty,
        "finite-roll time penalty")
    _require_sector_sum(
        rows, "roll_aware_time_gain_s", roll_aware_gain,
        "roll-aware time gain")

    print(f"envelope_csv={args.envelope_csv}")
    print(f"envelope_sha256={spatial._sha256_file(args.envelope_csv)}")
    print(f"roll_aware_controls_csv={args.roll_aware_controls_csv}")
    print(f"roll_aware_controls_sha256={spatial._sha256_file(args.roll_aware_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"sim_spacing_m={args.sim_spacing_m:.2f}")
    print(f"sector_length_m={args.sector_length_m:.3f}")
    print(f"sector_count={len(rows)}")
    print(f"eligible_complete_speed_bins={eligible_speed_count}/{len(eligible_speed)}")
    print(f"eligible_complete_offset_bins={np.count_nonzero(eligible_offset)}/{len(eligible_offset)}")
    print(f"ideal_lap_s={ideal_lap:.9f}")
    print(f"frozen_roll_lap_s={frozen_lap:.9f}")
    print(f"roll_aware_lap_s={aware_lap:.9f}")
    print(f"finite_roll_total_time_penalty_s={finite_roll_penalty:.9f}")
    print(f"roll_aware_total_time_gain_s={roll_aware_gain:.9f}")
    print(f"roll_aware_vs_ideal_total_time_delta_s={aware_lap - ideal_lap:.9f}")

    _print_top(
        rows, "finite_roll_time_penalty_s",
        "top_finite_roll_time_penalty_sectors", args.top_sectors)
    _print_top(
        rows, "roll_aware_time_gain_s",
        "top_roll_aware_time_gain_sectors", args.top_sectors)
    _print_top(
        rows, "finite_roll_speed_mae_improvement_mps",
        "top_finite_roll_measured_speed_improvement_sectors", args.top_sectors)
    _print_top(
        rows, "roll_aware_speed_mae_change_vs_frozen_mps",
        "top_roll_aware_measured_speed_worsening_sectors", args.top_sectors)
    _print_top(
        rows, "line_max_abs_offset_change_m",
        "top_roll_aware_line_movement_sectors", args.top_sectors)

    print("sector_note=fixed nominal-track-chainage sectors are diagnostic bins, not hand-labelled corners; sector time deltas sum exactly to the whole-lap deltas")
    print("comparison_note=measured speed and lateral offsets use nominal track chainage; local reference-geometry mismatch and rider line choice remain independent sources of discrepancy")
    print("calibration_note=no motorcycle, rider, track, registration, or roll-rate parameters are tuned by this command")
    _write_csv(args.output_csv, rows)
    print(f"output_csv={args.output_csv}")


if __name__ == "__main__":
    main()
