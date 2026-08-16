"""Warm-start Mallala re-optimisation with the Level-1 finite-roll constraint.

This Phase 9F diagnostic starts from the frozen 52-control ideal-response line,
then lets the existing deterministic planar optimiser adapt the line to one
explicit constant maximum roll-rate scenario.  The roll-rate value is a
sensitivity scenario, not a calibrated R6 or rider constant.
"""

import argparse
from dataclasses import replace
import importlib.util
import math
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    PlanarOptimisationConfig,
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    optimise_planar_racing_line,
    planar_control_bounds,
    resample_planar_result,
)
from motorcycle_lap_sim.track import Track


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling(
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_roll_optimisation")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_roll_optimisation")


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_controls_csv", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--max-evaluations", type=int, default=1500)
    parser.add_argument("--max-sweeps", type=int, default=30)
    parser.add_argument("--initial-step-m", type=_positive_float, default=1.0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument(
        "--initial-controls-csv", type=Path,
        help="strict same-policy controls CSV used to restart from an earlier Phase 9F result",
    )
    parser.add_argument("--checkpoint-controls-csv", type=Path)
    return parser


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


def _binding_roll_count(speed_profile):
    finite = np.isfinite(speed_profile.speed_limit_roll_rate_mps)
    binding = finite & np.isclose(
        speed_profile.speed_mps, speed_profile.speed_limit_roll_rate_mps,
        rtol=1e-8, atol=1e-6)
    return int(np.count_nonzero(binding)), len(binding)


def _evaluate_controls(track, bike, stations, controls, spacing):
    evaluation = evaluate_planar_racing_line(
        controls, track, bike, stations,
        sample_spacing_m=spacing,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        speed_backend="python",
    )
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(
            f"saved controls are infeasible at {spacing:.2f} m: {evaluation.failure_reason}")
    return evaluation


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.max_evaluations <= 0 or args.max_sweeps <= 0 or args.workers <= 0:
        raise ValueError("evaluation, sweep and worker limits must be positive integers")

    _require_canonical_inputs()
    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    frozen_controls = phase9.load_frozen_controls(
        phase9.DEFAULT_CONTROLS, stations, lower, upper)
    if args.initial_controls_csv is None:
        initial_controls = frozen_controls
        initial_source = str(phase9.DEFAULT_CONTROLS)
    else:
        initial_controls = phase8.load_initial_controls_csv(
            args.initial_controls_csv, stations, lower, upper)
        initial_source = str(args.initial_controls_csv)

    fixed_frozen = _evaluate_controls(
        track, bike, stations, frozen_controls, spacing=1.0)

    config = PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m,
        max_sweeps=args.max_sweeps,
        max_evaluations=args.max_evaluations,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        optimisation_sample_spacing_m=1.0,
        parallel_workers=args.workers,
        speed_backend=args.speed_backend,
    )

    callback = None
    if args.checkpoint_controls_csv is not None:
        args.checkpoint_controls_csv.parent.mkdir(parents=True, exist_ok=True)
        callback = phase8.checkpoint_callback(
            args.checkpoint_controls_csv, stations, lower, upper)

    result = optimise_planar_racing_line(
        track, bike, REFERENCE_PLANAR_CONTROL_POLICY, config,
        initial_controls_m=initial_controls,
        progress_callback=callback,
    )

    args.output_controls_csv.parent.mkdir(parents=True, exist_ok=True)
    phase8.atomic_write_controls_csv(
        args.output_controls_csv, result.control_s_m, result.best_controls_m,
        result.lower_bounds_m, result.upper_bounds_m)

    delta = result.best_controls_m - frozen_controls
    binding, samples = _binding_roll_count(result.speed_profile)
    total_improvement = fixed_frozen.lap_time_s - result.best_lap_time_s
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"speed_backend={args.speed_backend}")
    print(f"workers={args.workers}")
    print(f"control_count={len(result.control_s_m)}")
    print(f"initial_controls_source={initial_source}")
    print(f"fixed_frozen_line_with_roll_lap_s={fixed_frozen.lap_time_s:.9f}")
    print(f"restart_initial_line_with_roll_lap_s={result.initial_lap_time_s:.9f}")
    print(f"roll_aware_optimised_lap_s={result.best_lap_time_s:.9f}")
    print(f"restart_improvement_s={result.improvement_s:.9f}")
    print(f"total_line_adaptation_improvement_from_frozen_s={total_improvement:.9f}")
    print("total_line_adaptation_improvement_from_frozen_percent="
          f"{100.0 * total_improvement / fixed_frozen.lap_time_s:.9f}")
    print(f"evaluations={result.evaluations}")
    print(f"sweeps={result.sweeps}")
    print(f"final_step_m={result.final_step_m:.9f}")
    print(f"termination_reason={result.termination_reason}")
    print(f"minimum_boundary_clearance_m={result.minimum_boundary_clearance_m:.9f}")
    print(f"maximum_control_change_from_frozen_m={np.max(np.abs(delta)):.9f}")
    print(f"rms_control_change_from_frozen_m={np.sqrt(np.mean(delta ** 2)):.9f}")
    print(f"binding_roll_ceiling_samples={binding}/{samples}")
    print("maximum_level1_demanded_roll_rate_radps="
          f"{np.max(np.abs(result.speed_profile.demanded_roll_rate_radps)):.9f}")
    print(f"output_controls_csv={args.output_controls_csv}")

    for spacing in phase9.OUTPUT_SPACINGS_M:
        frozen = _evaluate_controls(
            track, bike, stations, frozen_controls, spacing)
        path, speed = resample_planar_result(result, bike, spacing)
        frozen_binding, frozen_samples = _binding_roll_count(frozen.speed_profile)
        optimised_binding, optimised_samples = _binding_roll_count(speed)
        spacing_improvement = frozen.lap_time_s - speed.lap_time_s
        print(
            f"grid_spacing_m={spacing:.2f} "
            f"frozen_lap_s={frozen.lap_time_s:.9f} "
            f"roll_aware_lap_s={speed.lap_time_s:.9f} "
            f"line_adaptation_improvement_s={spacing_improvement:.9f} "
            f"frozen_path_length_m={frozen.smooth_line.sampled_path.total_length_m:.9f} "
            f"roll_aware_path_length_m={path.total_length_m:.9f} "
            f"frozen_binding_roll_samples={frozen_binding}/{frozen_samples} "
            f"roll_aware_binding_roll_samples={optimised_binding}/{optimised_samples} "
            f"frozen_maximum_level1_roll_rate_radps="
            f"{np.max(np.abs(frozen.speed_profile.demanded_roll_rate_radps)):.9f} "
            f"roll_aware_maximum_level1_roll_rate_radps="
            f"{np.max(np.abs(speed.demanded_roll_rate_radps)):.9f}")


if __name__ == "__main__":
    main()
