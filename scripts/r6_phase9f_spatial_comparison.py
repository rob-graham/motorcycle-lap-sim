"""Compare frozen and roll-aware Mallala lines against the measured envelope.

This diagnostic is deliberately comparative rather than calibrating.  It puts
three simulator cases on the same nominal track-chainage bins used by the
hardened Phase 10 telemetry envelope:

1. frozen 52-control ideal-response line without finite roll limiting,
2. the same frozen line with one explicit Level-1 finite roll-rate scenario,
3. a supplied roll-aware re-optimised 52-control line with that same scenario.

The output CSV combines measured lateral/speed envelopes with simulator lateral
position, speed, curvature, curvature gradient, demanded Level-1 roll rate, and
roll-ceiling binding state so local adaptations can be reviewed directly.
"""

import argparse
import csv
from dataclasses import replace
import hashlib
import importlib.util
import math
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.telemetry import compare_speed_envelope, summarize_speed_comparison
from motorcycle_lap_sim.track import Track, sample_track_stations


REQUIRED_ENVELOPE_COLUMNS = (
    "chainage_m",
    "offset_lap_count",
    "offset_median_m",
    "offset_p10_m",
    "offset_p90_m",
    "speed_lap_count",
    "speed_median_mps",
    "speed_p10_mps",
    "speed_p90_mps",
)
SIM_SPACINGS_M = (1.0, 0.5, 0.25)
SPEED_BINDING_RTOL = 1e-8
SPEED_BINDING_ATOL_MPS = 1e-6


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling(
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_spatial_comparison")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_spatial_comparison")


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("envelope_csv", type=Path)
    parser.add_argument("roll_aware_controls_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--sim-spacing-m", type=float, choices=SIM_SPACINGS_M, default=0.25)
    parser.add_argument("--required-lap-count", type=int, default=5)
    parser.add_argument("--expected-complete-lap-bins", type=int, default=219)
    return parser


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_canonical_inputs():
    identities = (
        (phase9.DEFAULT_CONTROLS, phase9.EXPECTED_CONTROLS_SHA256, "controls"),
        (phase9.DEFAULT_TRACK, phase9.EXPECTED_TRACK_SHA256, "track"),
        (phase9.DEFAULT_MOTORCYCLE, phase9.EXPECTED_MOTORCYCLE_SHA256, "motorcycle"),
    )
    for path, expected, label in identities:
        actual = phase9.sha256_file(path)
        if actual != expected:
            raise RuntimeError(
                f"canonical {label} SHA-256 {actual} does not match frozen {expected}")


def _load_envelope(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = tuple(reader.fieldnames or ())
        missing = [name for name in REQUIRED_ENVELOPE_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError("envelope CSV is missing required columns: " + ", ".join(missing))
        rows = list(reader)
    if not rows:
        raise ValueError("envelope CSV contains no data rows")

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
                    f"envelope row {row_number} column {name} is not numeric") from exc
        return np.asarray(result, dtype=float)

    return {name: numeric(name) for name in REQUIRED_ENVELOPE_COLUMNS}


def _evaluate(track, bike, stations, controls, spacing):
    evaluation = evaluate_planar_racing_line(
        controls, track, bike, stations,
        sample_spacing_m=spacing,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        speed_backend="python",
    )
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"simulator case is infeasible: {evaluation.failure_reason}")
    return evaluation


def _sim_columns(track, evaluation):
    path = evaluation.smooth_line.sampled_path
    speed = evaluation.speed_profile
    count = len(path.q_m)
    if len(speed.speed_mps) != count:
        raise RuntimeError("speed profile and sampled path lengths differ")
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    reference = sample_track_stations(track, track_s)
    spline_x, spline_y, *_ = evaluation.smooth_line.spline.evaluate(track_s)
    if (not np.allclose(spline_x, path.x_m, rtol=0.0, atol=1e-10)
            or not np.allclose(spline_y, path.y_m, rtol=0.0, atol=1e-10)):
        raise RuntimeError("nominal track-chainage grid does not match racing-line samples")
    dx = path.x_m - reference.x_m
    dy = path.y_m - reference.y_m
    lateral_offset = dx * reference.normal_x + dy * reference.normal_y
    finite_roll = np.isfinite(speed.speed_limit_roll_rate_mps)
    roll_binding = finite_roll & np.isclose(
        speed.speed_mps, speed.speed_limit_roll_rate_mps,
        rtol=SPEED_BINDING_RTOL, atol=SPEED_BINDING_ATOL_MPS)
    return {
        "track_s_m": track_s,
        "lateral_offset_m": lateral_offset,
        "speed_mps": np.asarray(speed.speed_mps),
        "curvature_1pm": np.asarray(path.curvature_1pm),
        "curvature_gradient_1pm2": np.asarray(speed.curvature_gradient_1pm2),
        "roll_rate_model_radps": np.asarray(speed.demanded_roll_rate_radps),
        "roll_binding": roll_binding,
    }


def _periodic_interp(query_m, source_s_m, values, period_m):
    query = np.mod(np.asarray(query_m, dtype=float), period_m)
    source = np.asarray(source_s_m, dtype=float)
    values = np.asarray(values, dtype=float)
    if source.ndim != 1 or values.shape != source.shape or len(source) == 0:
        raise ValueError("periodic interpolation inputs must be equal non-empty 1D arrays")
    extended_s = np.r_[source[-1] - period_m, source, source[0] + period_m]
    extended_values = np.r_[values[-1], values, values[0]]
    return np.interp(query, extended_s, extended_values)


def _nearest_periodic_indices(query_m, sample_count, period_m):
    query = np.mod(np.asarray(query_m, dtype=float), period_m)
    return np.mod(np.rint(query * sample_count / period_m).astype(int), sample_count)


def _sample_case(case, chainage_m, period_m):
    nearest = _nearest_periodic_indices(chainage_m, len(case["track_s_m"]), period_m)
    return {
        "lateral_offset_m": _periodic_interp(
            chainage_m, case["track_s_m"], case["lateral_offset_m"], period_m),
        "speed_mps": _periodic_interp(
            chainage_m, case["track_s_m"], case["speed_mps"], period_m),
        "curvature_1pm": _periodic_interp(
            chainage_m, case["track_s_m"], case["curvature_1pm"], period_m),
        "curvature_gradient_1pm2": _periodic_interp(
            chainage_m, case["track_s_m"], case["curvature_gradient_1pm2"], period_m),
        "roll_rate_model_radps": _periodic_interp(
            chainage_m, case["track_s_m"], case["roll_rate_model_radps"], period_m),
        "roll_binding": case["roll_binding"][nearest].astype(int),
    }


def _speed_summary(envelope, case, track_length_m, required_lap_count):
    comparison = compare_speed_envelope(
        envelope["chainage_m"],
        envelope["speed_median_mps"],
        envelope["speed_p10_mps"],
        envelope["speed_p90_mps"],
        envelope["speed_lap_count"],
        case["track_s_m"], case["speed_mps"], track_length_m,
        required_lap_count=required_lap_count,
    )
    return comparison, summarize_speed_comparison(comparison)


def _print_speed_summary(label, lap_time_s, summary):
    print(f"{label}_lap_s={lap_time_s:.9f}")
    print(f"{label}_mean_sim_minus_measured_median_mps={summary.mean_bias_mps:.9f}")
    print(f"{label}_mean_absolute_speed_error_mps={summary.mean_absolute_error_mps:.9f}")
    print(f"{label}_rms_speed_error_mps={summary.rms_error_mps:.9f}")
    print(f"{label}_p95_absolute_speed_error_mps={summary.p95_absolute_error_mps:.9f}")
    print(f"{label}_within_measured_p10_p90_bins={summary.within_p10_p90_bins}/{summary.eligible_bins}")
    print(f"{label}_above_measured_p90_bins={summary.above_p90_bins}/{summary.eligible_bins}")
    print(f"{label}_below_measured_p10_bins={summary.below_p10_bins}/{summary.eligible_bins}")


def _write_csv(path, envelope, sampled, eligible_speed, eligible_offset):
    fields = (
        "chainage_m",
        "speed_evidence_complete", "offset_evidence_complete",
        "measured_speed_median_mps", "measured_speed_p10_mps", "measured_speed_p90_mps",
        "measured_offset_median_m", "measured_offset_p10_m", "measured_offset_p90_m",
        "ideal_speed_mps", "frozen_roll_speed_mps", "roll_aware_speed_mps",
        "ideal_offset_m", "roll_aware_offset_m", "roll_aware_minus_ideal_offset_m",
        "ideal_curvature_1pm", "roll_aware_curvature_1pm",
        "roll_aware_minus_ideal_curvature_1pm",
        "ideal_curvature_gradient_1pm2", "frozen_roll_curvature_gradient_1pm2",
        "roll_aware_curvature_gradient_1pm2",
        "ideal_roll_rate_model_radps", "frozen_roll_rate_model_radps",
        "roll_aware_roll_rate_model_radps",
        "frozen_roll_binding", "roll_aware_roll_binding",
        "ideal_minus_measured_speed_mps", "frozen_roll_minus_measured_speed_mps",
        "roll_aware_minus_measured_speed_mps",
    )
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index, chainage in enumerate(envelope["chainage_m"]):
            speed_ok = bool(eligible_speed[index])
            offset_ok = bool(eligible_offset[index])
            measured_speed = envelope["speed_median_mps"][index]
            ideal_speed = sampled["ideal"]["speed_mps"][index]
            frozen_roll_speed = sampled["frozen_roll"]["speed_mps"][index]
            roll_aware_speed = sampled["roll_aware"]["speed_mps"][index]
            ideal_offset = sampled["ideal"]["lateral_offset_m"][index]
            roll_aware_offset = sampled["roll_aware"]["lateral_offset_m"][index]
            writer.writerow((
                chainage,
                int(speed_ok), int(offset_ok),
                measured_speed,
                envelope["speed_p10_mps"][index], envelope["speed_p90_mps"][index],
                envelope["offset_median_m"][index], envelope["offset_p10_m"][index],
                envelope["offset_p90_m"][index],
                ideal_speed, frozen_roll_speed, roll_aware_speed,
                ideal_offset, roll_aware_offset, roll_aware_offset - ideal_offset,
                sampled["ideal"]["curvature_1pm"][index],
                sampled["roll_aware"]["curvature_1pm"][index],
                sampled["roll_aware"]["curvature_1pm"][index]
                - sampled["ideal"]["curvature_1pm"][index],
                sampled["ideal"]["curvature_gradient_1pm2"][index],
                sampled["frozen_roll"]["curvature_gradient_1pm2"][index],
                sampled["roll_aware"]["curvature_gradient_1pm2"][index],
                sampled["ideal"]["roll_rate_model_radps"][index],
                sampled["frozen_roll"]["roll_rate_model_radps"][index],
                sampled["roll_aware"]["roll_rate_model_radps"][index],
                sampled["frozen_roll"]["roll_binding"][index],
                sampled["roll_aware"]["roll_binding"][index],
                ideal_speed - measured_speed if speed_ok else "",
                frozen_roll_speed - measured_speed if speed_ok else "",
                roll_aware_speed - measured_speed if speed_ok else "",
            ))


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.required_lap_count <= 0:
        raise ValueError("required-lap-count must be positive")
    if args.expected_complete_lap_bins < 0:
        raise ValueError("expected-complete-lap-bins must be non-negative")

    _require_canonical_inputs()
    envelope_hash = _sha256_file(args.envelope_csv)
    envelope = _load_envelope(args.envelope_csv)
    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    roll_bike = replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    frozen_controls = phase9.load_frozen_controls(
        phase9.DEFAULT_CONTROLS, stations, lower, upper)
    roll_aware_controls = phase8.load_initial_controls_csv(
        args.roll_aware_controls_csv, stations, lower, upper)

    evaluations = {
        "ideal": _evaluate(track, base_bike, stations, frozen_controls, args.sim_spacing_m),
        "frozen_roll": _evaluate(track, roll_bike, stations, frozen_controls, args.sim_spacing_m),
        "roll_aware": _evaluate(track, roll_bike, stations, roll_aware_controls, args.sim_spacing_m),
    }
    cases = {name: _sim_columns(track, evaluation)
             for name, evaluation in evaluations.items()}
    sampled = {name: _sample_case(case, envelope["chainage_m"], track.total_length_m)
               for name, case in cases.items()}

    comparisons = {}
    summaries = {}
    for name, case in cases.items():
        comparisons[name], summaries[name] = _speed_summary(
            envelope, case, track.total_length_m, args.required_lap_count)

    eligible_speed = comparisons["ideal"].eligible_mask
    if not all(np.array_equal(comparison.eligible_mask, eligible_speed)
               for comparison in comparisons.values()):
        raise RuntimeError("speed-comparison eligibility differs between simulator cases")
    eligible_count = int(np.count_nonzero(eligible_speed))
    if eligible_count != args.expected_complete_lap_bins:
        raise RuntimeError(
            f"measured envelope has {eligible_count}/{len(eligible_speed)} complete-lap speed bins, "
            f"expected {args.expected_complete_lap_bins}/{len(eligible_speed)}")

    eligible_offset = (
        np.asarray(envelope["offset_lap_count"]) >= args.required_lap_count)
    eligible_offset &= np.isfinite(envelope["offset_median_m"])
    eligible_offset &= np.isfinite(envelope["offset_p10_m"])
    eligible_offset &= np.isfinite(envelope["offset_p90_m"])

    print(f"envelope_csv={args.envelope_csv}")
    print(f"envelope_sha256={envelope_hash}")
    print(f"roll_aware_controls_csv={args.roll_aware_controls_csv}")
    print(f"roll_aware_controls_sha256={_sha256_file(args.roll_aware_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"sim_spacing_m={args.sim_spacing_m:.2f}")
    print(f"eligible_complete_speed_bins={eligible_count}/{len(eligible_speed)}")
    print(f"eligible_complete_offset_bins={np.count_nonzero(eligible_offset)}/{len(eligible_offset)}")
    for name in ("ideal", "frozen_roll", "roll_aware"):
        _print_speed_summary(name, evaluations[name].lap_time_s, summaries[name])

    eligible_indices = np.flatnonzero(eligible_speed)
    frozen_error = np.abs(
        sampled["frozen_roll"]["speed_mps"] - envelope["speed_median_mps"])
    roll_aware_error = np.abs(
        sampled["roll_aware"]["speed_mps"] - envelope["speed_median_mps"])
    closer = eligible_speed & (roll_aware_error < frozen_error)
    farther = eligible_speed & (roll_aware_error > frozen_error)
    print(f"roll_aware_closer_to_measured_median_bins={np.count_nonzero(closer)}/{eligible_count}")
    print(f"roll_aware_farther_from_measured_median_bins={np.count_nonzero(farther)}/{eligible_count}")
    print("mean_absolute_error_change_roll_aware_minus_frozen_roll_mps="
          f"{np.mean(roll_aware_error[eligible_indices] - frozen_error[eligible_indices]):.9f}")

    offset_change = sampled["roll_aware"]["lateral_offset_m"] - sampled["ideal"]["lateral_offset_m"]
    worst_offset_index = int(np.argmax(np.abs(offset_change)))
    print(f"maximum_abs_line_offset_change_m={abs(offset_change[worst_offset_index]):.9f}")
    print(f"maximum_abs_line_offset_change_chainage_m={envelope['chainage_m'][worst_offset_index]:.9f}")
    print(f"rms_line_offset_change_m={np.sqrt(np.mean(offset_change ** 2)):.9f}")
    print("comparison_note=nominal track chainage is the common coordinate; measured and simulated lines differ geometrically and local reference-geometry error remains possible")
    print("calibration_note=no motorcycle, rider, track, or registration parameters are tuned by this command")

    _write_csv(args.output_csv, envelope, sampled, eligible_speed, eligible_offset)
    print(f"output_csv={args.output_csv}")


if __name__ == "__main__":
    main()
