"""Sweep simple finite roll-rate limits on the frozen Mallala R6 racing line.

This Phase 10 diagnostic keeps the frozen 52-control geometry fixed.  It does
not optimise the line or calibrate motorcycle parameters.  Each candidate uses
one constant maximum Level-1 roll rate and is compared with the same measured
speed envelope used by the fixed-line speed-validation command.
"""

import argparse
from dataclasses import replace
import hashlib
import importlib.util
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.telemetry import (
    compare_speed_envelope,
    summarize_speed_comparison,
    uniform_closed_parameter_grid,
)


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase9 = _load_sibling("r6_phase9_baseline_check.py", "r6_phase9_baseline_for_roll_sweep")
speed_validation = _load_sibling(
    "r6_phase10_speed_validation.py", "r6_phase10_speed_validation_for_roll_sweep")

DEFAULT_ENVELOPE = Path("../../motorcycle-lap-sim-results/phase10_mallala_envelope_hardened.csv")
DEFAULT_ROLL_RATES_RADPS = (0.5, 0.7, 0.9)


def _positive_float(text):
    value = float(text)
    if not np.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("roll-rate values must be finite and positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "envelope_csv", nargs="?", type=Path, default=DEFAULT_ENVELOPE,
        help="post-registration Phase 10 envelope CSV")
    parser.add_argument(
        "--roll-rates-radps", type=_positive_float, nargs="+",
        default=DEFAULT_ROLL_RATES_RADPS,
        help="constant maximum roll rates to sweep")
    parser.add_argument("--expected-complete-lap-bins", type=int, default=219)
    parser.add_argument("--required-lap-count", type=int, default=5)
    return parser


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _comparison(track, speed_mps, measured, required_lap_count):
    chainage, median, p10, p90, lap_count = measured
    simulated_s = uniform_closed_parameter_grid(track.total_length_m, len(speed_mps))
    comparison = compare_speed_envelope(
        chainage, median, p10, p90, lap_count,
        simulated_s, speed_mps, track.total_length_m,
        required_lap_count=required_lap_count,
    )
    return comparison, summarize_speed_comparison(comparison)


def _print_case(label, result, summary):
    finite_roll = np.isfinite(result.speed_limit_roll_rate_mps)
    maximum_rate = float(np.max(np.abs(result.demanded_roll_rate_radps)))
    print(f"case={label}")
    print(f"lap_s={result.lap_time_s:.9f}")
    print(f"roll_limited_samples={np.count_nonzero(finite_roll)}/{len(finite_roll)}")
    print(f"maximum_level1_demanded_roll_rate_radps={maximum_rate:.9f}")
    print(f"eligible_complete_lap_bins={summary.eligible_bins}")
    print(f"mean_sim_minus_measured_median_mps={summary.mean_bias_mps:.9f}")
    print(f"mean_absolute_speed_error_mps={summary.mean_absolute_error_mps:.9f}")
    print(f"rms_speed_error_mps={summary.rms_error_mps:.9f}")
    print(f"p95_absolute_speed_error_mps={summary.p95_absolute_error_mps:.9f}")
    print(f"sim_within_measured_p10_p90_bins={summary.within_p10_p90_bins}/{summary.eligible_bins}")
    print(f"sim_above_measured_p90_bins={summary.above_p90_bins}/{summary.eligible_bins}")
    print(f"sim_below_measured_p10_bins={summary.below_p10_bins}/{summary.eligible_bins}")


def main(argv=None):
    args = build_parser().parse_args(argv)
    measured = speed_validation._load_measured_speed_envelope(args.envelope_csv)
    envelope_hash = _sha256_file(args.envelope_csv)

    track, _, _, evaluations = phase9.evaluate_baseline(speed_backend="python")
    baseline_evaluation = evaluations[0]
    if baseline_evaluation.speed_profile is None or baseline_evaluation.smooth_line is None:
        raise RuntimeError("Phase 9 fixed-line baseline did not return required artifacts")
    path = baseline_evaluation.smooth_line.sampled_path
    baseline = baseline_evaluation.speed_profile

    bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling limits enabled")

    baseline_comparison, baseline_summary = _comparison(
        track, baseline.speed_mps, measured, args.required_lap_count)
    speed_validation._require_expected_complete_bins(
        baseline_summary.eligible_bins, len(baseline_comparison.chainage_m),
        args.expected_complete_lap_bins)

    print(f"envelope_csv={args.envelope_csv}")
    print(f"envelope_sha256={envelope_hash}")
    print("model=level1_constant_max_roll_rate")
    print("model_note=roll demand is from changing path curvature at locally constant speed; longitudinal-acceleration contribution to lean-rate is intentionally omitted")
    print("calibration_note=measured roll-rate telemetry is not fitted by this command")
    _print_case("unconstrained", baseline, baseline_summary)

    for roll_rate in args.roll_rates_radps:
        handling = HandlingConfig(max_roll_rate_radps=float(roll_rate))
        result = solve_speed_profile(path, replace(bike, handling=handling))
        comparison, summary = _comparison(
            track, result.speed_mps, measured, args.required_lap_count)
        speed_validation._require_expected_complete_bins(
            summary.eligible_bins, len(comparison.chainage_m),
            args.expected_complete_lap_bins)
        print(f"max_roll_rate_radps={roll_rate:.9f}")
        _print_case(f"roll_{roll_rate:.3f}_radps", result, summary)


if __name__ == "__main__":
    main()
