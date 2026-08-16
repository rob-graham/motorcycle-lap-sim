"""Test coarse-to-reference warm-starting for Mallala from a generic centreline.

Phase 11A showed that direct 52-control coordinate search from centreline is
inefficient. This diagnostic changes no physics and introduces no new optimiser:
it first optimises the existing coarse planar control policy, transfers that
result periodically onto the reference 52-control policy, and then runs the
ordinary reference-policy optimiser. The final result is re-ranked on the same
0.25 m Python grid used for optimisation assurance.
"""

import argparse
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import time

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    COARSE_PLANAR_CONTROL_POLICY,
    REFERENCE_PLANAR_CONTROL_POLICY,
    PlanarOptimisationConfig,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    optimise_planar_racing_line,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


DEFAULT_RANKING_SPACING_M = 0.25


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling(
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_phase11b")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_phase11b")
phase9f = _load_sibling(
    "r6_phase9f_roll_aware_optimisation.py", "r6_phase9f_for_phase11b")
phase11a = _load_sibling(
    "r6_phase11a_multistart_assurance.py", "r6_phase11a_for_phase11b")


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
    parser.add_argument(
        "reviewed_reference_controls_csv", type=Path,
        help="reviewed Phase 9F reference-policy controls used only for final comparison",
    )
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--coarse-max-evaluations", type=_positive_int, default=2500)
    parser.add_argument("--coarse-max-sweeps", type=_positive_int, default=20)
    parser.add_argument("--reference-max-evaluations", type=_positive_int, default=4000)
    parser.add_argument("--reference-max-sweeps", type=_positive_int, default=30)
    parser.add_argument("--initial-step-m", type=_positive_float, default=1.0)
    parser.add_argument("--workers", type=_positive_int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument("--ranking-spacing-m", type=_positive_float, default=DEFAULT_RANKING_SPACING_M)
    return parser


def periodic_linear_transfer(source_s_m, source_controls_m, target_s_m, lap_length_m):
    """Periodically linearly interpolate control offsets onto target stations."""
    source_s = np.asarray(source_s_m, dtype=float)
    source_controls = np.asarray(source_controls_m, dtype=float)
    target_s = np.asarray(target_s_m, dtype=float)
    if source_s.ndim != 1 or source_controls.shape != source_s.shape or target_s.ndim != 1:
        raise ValueError("source stations/controls and target stations must be 1D with matching source shapes")
    if len(source_s) < 2 or len(target_s) == 0:
        raise ValueError("transfer requires at least two source controls and one target station")
    if (not np.all(np.isfinite(source_s)) or not np.all(np.isfinite(source_controls))
            or not np.all(np.isfinite(target_s))):
        raise ValueError("transfer inputs must be finite")
    if not math.isfinite(lap_length_m) or lap_length_m <= 0.0:
        raise ValueError("lap length must be finite and positive")
    if np.any(np.diff(source_s) <= 0.0) or source_s[0] < 0.0 or source_s[-1] >= lap_length_m:
        raise ValueError("source stations must be strictly increasing within one lap")

    wrapped_target = np.mod(target_s, lap_length_m)
    extended_s = np.concatenate((source_s[-1:] - lap_length_m, source_s, source_s[:1] + lap_length_m))
    extended_controls = np.concatenate((source_controls[-1:], source_controls, source_controls[:1]))
    return np.interp(wrapped_target, extended_s, extended_controls)


def _evaluate(track, bike, stations, controls, spacing):
    evaluation = evaluate_planar_racing_line(
        controls, track, bike, stations,
        sample_spacing_m=spacing,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        speed_backend="python",
    )
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(
            f"candidate controls are infeasible at {spacing:.3f} m: {evaluation.failure_reason}")
    return evaluation


def _config(args, max_evaluations, max_sweeps):
    return PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m,
        max_sweeps=max_sweeps,
        max_evaluations=max_evaluations,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        optimisation_sample_spacing_m=1.0,
        parallel_workers=args.workers,
        speed_backend=args.speed_backend,
    )


def _run(label, track, bike, policy, config, initial_controls):
    started = time.perf_counter()
    result = optimise_planar_racing_line(
        track, bike, policy, config, initial_controls_m=initial_controls)
    elapsed = time.perf_counter() - started
    print(
        f"stage={label} controls={len(result.control_s_m)} "
        f"initial_lap_s={result.initial_lap_time_s:.9f} "
        f"final_lap_s={result.best_lap_time_s:.9f} "
        f"improvement_s={result.improvement_s:.9f} "
        f"evaluations={result.evaluations} sweeps={result.sweeps} "
        f"final_step_m={result.final_step_m:.9f} "
        f"termination={result.termination_reason!r} elapsed_s={elapsed:.3f}")
    return result


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    coarse_s = generate_planar_control_stations(track, COARSE_PLANAR_CONTROL_POLICY)
    coarse_lower, coarse_upper = planar_control_bounds(
        track, coarse_s, phase9.BOUNDARY_MARGIN_M)
    coarse_centreline = np.clip(np.zeros_like(coarse_s), coarse_lower, coarse_upper)

    reference_s = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    reference_lower, reference_upper = planar_control_bounds(
        track, reference_s, phase9.BOUNDARY_MARGIN_M)
    reference_centreline = np.clip(
        np.zeros_like(reference_s), reference_lower, reference_upper)
    reviewed_controls = phase8.load_initial_controls_csv(
        args.reviewed_reference_controls_csv, reference_s, reference_lower, reference_upper)

    args.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"reviewed_reference_controls_csv={args.reviewed_reference_controls_csv}")
    print(f"reviewed_reference_controls_sha256={phase9.sha256_file(args.reviewed_reference_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"coarse_control_count={len(coarse_s)}")
    print(f"reference_control_count={len(reference_s)}")
    print(f"workers={args.workers}")
    print(f"speed_backend={args.speed_backend}")
    print(f"common_ranking_spacing_m={args.ranking_spacing_m:.3f}")

    coarse = _run(
        "coarse_from_centreline", track, bike, COARSE_PLANAR_CONTROL_POLICY,
        _config(args, args.coarse_max_evaluations, args.coarse_max_sweeps),
        coarse_centreline,
    )
    phase8.atomic_write_controls_csv(
        args.output_dir / "coarse_final_controls.csv",
        coarse.control_s_m, coarse.best_controls_m,
        coarse.lower_bounds_m, coarse.upper_bounds_m,
    )

    proposed_reference = periodic_linear_transfer(
        coarse.control_s_m, coarse.best_controls_m, reference_s, track.total_length_m)
    proposed_reference = np.clip(proposed_reference, reference_lower, reference_upper)

    def reference_feasible(controls):
        for spacing in (1.0, args.ranking_spacing_m):
            evaluation = evaluate_planar_racing_line(
                controls, track, bike, reference_s,
                sample_spacing_m=spacing,
                boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
                boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
                speed_backend="python",
            )
            if (not evaluation.feasible or evaluation.smooth_line is None
                    or evaluation.speed_profile is None):
                return False
        return True

    transferred, transfer_scale = phase11a.backoff_to_feasible(
        proposed_reference, reference_centreline, reference_feasible)
    if transfer_scale == 0.0:
        raise RuntimeError("coarse-to-reference transfer collapsed to centreline")
    transferred_common = _evaluate(
        track, bike, reference_s, transferred, args.ranking_spacing_m)
    print(f"coarse_to_reference_feasibility_scale={transfer_scale:.6f}")
    print(f"transferred_reference_common_grid_lap_s={transferred_common.lap_time_s:.9f}")

    reference = _run(
        "reference_from_coarse_transfer", track, bike, REFERENCE_PLANAR_CONTROL_POLICY,
        _config(args, args.reference_max_evaluations, args.reference_max_sweeps),
        transferred,
    )
    phase8.atomic_write_controls_csv(
        args.output_dir / "reference_final_controls.csv",
        reference.control_s_m, reference.best_controls_m,
        reference.lower_bounds_m, reference.upper_bounds_m,
    )

    hierarchy_common = _evaluate(
        track, bike, reference_s, reference.best_controls_m, args.ranking_spacing_m)
    reviewed_common = _evaluate(
        track, bike, reference_s, reviewed_controls, args.ranking_spacing_m)
    delta_controls = np.asarray(reference.best_controls_m) - reviewed_controls
    delta_lap = hierarchy_common.lap_time_s - reviewed_common.lap_time_s

    print(f"hierarchical_common_grid_lap_s={hierarchy_common.lap_time_s:.9f}")
    print(f"reviewed_common_grid_lap_s={reviewed_common.lap_time_s:.9f}")
    print(f"hierarchical_minus_reviewed_s={delta_lap:.9f}")
    print(f"maximum_abs_control_delta_to_reviewed_m={np.max(np.abs(delta_controls)):.9f}")
    print(f"rms_control_delta_to_reviewed_m={np.sqrt(np.mean(delta_controls ** 2)):.9f}")
    print("interpretation_note=if a generic centreline reaches a comparably good reference-policy result through the existing coarse-to-reference hierarchy at reasonable cost, prescribe hierarchical warm-starting before considering a new optimiser algorithm")
    print("calibration_note=no motorcycle, rider, track, or roll-rate parameter is fitted by this command")


if __name__ == "__main__":
    main()
