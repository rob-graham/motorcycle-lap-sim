"""Test low-dimensional latent basin search on the 52-guide Mallala representation.

Phase 11A/11B showed that direct coordinate search spends most of its budget on
large complete polls and remains strongly warm-start dependent. Phase 11C keeps
the reference 52-guide spline representation, but first searches it through a
small set of smooth periodic latent variables. The resulting feasible reference
controls then warm-start the unchanged 52-control planar optimiser.

This is an optimisation-assurance experiment only: no motorcycle, rider, track,
or roll parameter is calibrated and no physics model is changed.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import importlib.util
import math
import multiprocessing
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    PeriodicCubicParameterisation,
    PlanarOptimisationConfig,
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    optimise_planar_racing_line,
    planar_control_bounds,
)
from motorcycle_lap_sim.optimisation.planar import (
    _PlanarWorkerEvaluation,
    _best_improvement_pattern_search,
)
from motorcycle_lap_sim.track import Track


DEFAULT_LATENT_COUNT = 12
DEFAULT_LATENT_BOUND_M = 4.0
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
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_phase11c")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_phase11c")
phase9f = _load_sibling(
    "r6_phase9f_roll_aware_optimisation.py", "r6_phase9f_for_phase11c")


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
        help="reviewed Phase 9F reference controls used only for final comparison",
    )
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--latent-count", type=_positive_int, default=DEFAULT_LATENT_COUNT)
    parser.add_argument("--latent-bound-m", type=_positive_float, default=DEFAULT_LATENT_BOUND_M)
    parser.add_argument("--latent-max-evaluations", type=_positive_int, default=1800)
    parser.add_argument("--latent-max-sweeps", type=_positive_int, default=30)
    parser.add_argument("--reference-max-evaluations", type=_positive_int, default=4000)
    parser.add_argument("--reference-max-sweeps", type=_positive_int, default=30)
    parser.add_argument("--initial-step-m", type=_positive_float, default=1.0)
    parser.add_argument("--workers", type=_positive_int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument("--ranking-spacing-m", type=_positive_float, default=DEFAULT_RANKING_SPACING_M)
    return parser


def latent_to_reference_controls(
        latent_controls_m, reference_s_m, lap_length_m, lower_bounds_m, upper_bounds_m):
    """Map smooth periodic latent metre offsets onto bounded reference guides."""
    latent = np.asarray(latent_controls_m, dtype=float)
    reference_s = np.asarray(reference_s_m, dtype=float)
    lower = np.asarray(lower_bounds_m, dtype=float)
    upper = np.asarray(upper_bounds_m, dtype=float)
    if latent.ndim != 1 or len(latent) < 4 or not np.all(np.isfinite(latent)):
        raise ValueError("latent controls must be a finite 1D array with at least four values")
    if reference_s.ndim != 1 or lower.shape != reference_s.shape or upper.shape != reference_s.shape:
        raise ValueError("reference stations and bounds must be matching 1D arrays")
    if not np.all(np.isfinite(reference_s)) or not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError("reference stations and bounds must be finite")
    if not math.isfinite(lap_length_m) or lap_length_m <= 0.0:
        raise ValueError("lap length must be finite and positive")
    if np.any(lower > upper):
        raise ValueError("reference lower bounds must not exceed upper bounds")

    parameterisation = PeriodicCubicParameterisation(len(latent))
    sampled_reference = SimpleNamespace(s_m=reference_s, total_length_m=lap_length_m)
    smooth_offsets = parameterisation.latent_values(latent, sampled_reference)
    return np.clip(smooth_offsets, lower, upper)


_latent_worker_context = None


def _initialise_latent_worker(
        track, bike, reference_s, lower, upper, latent_count, sample_spacing_m,
        boundary_margin_m, boundary_check_spacing_m, speed_backend):
    global _latent_worker_context
    _latent_worker_context = (
        track, bike, np.asarray(reference_s), np.asarray(lower), np.asarray(upper),
        latent_count, sample_spacing_m, boundary_margin_m, boundary_check_spacing_m,
        speed_backend,
    )


def _evaluate_latent_worker(latent_controls_m):
    context = _latent_worker_context
    if context is None:
        raise RuntimeError("latent worker context was not initialised")
    (track, bike, reference_s, lower, upper, latent_count, sample_spacing_m,
     boundary_margin_m, boundary_check_spacing_m, speed_backend) = context
    latent = np.asarray(latent_controls_m, dtype=float)
    if latent.shape != (latent_count,):
        raise ValueError("worker latent-control shape mismatch")
    reference_controls = latent_to_reference_controls(
        latent, reference_s, track.total_length_m, lower, upper)
    evaluation = evaluate_planar_racing_line(
        reference_controls, track, bike, reference_s,
        sample_spacing_m=sample_spacing_m,
        boundary_margin_m=boundary_margin_m,
        boundary_check_spacing_m=boundary_check_spacing_m,
        speed_backend=speed_backend,
    )
    return _PlanarWorkerEvaluation(
        evaluation.feasible, evaluation.lap_time_s, evaluation.failure_reason)


def _evaluate_reference(track, bike, reference_s, controls, spacing):
    evaluation = evaluate_planar_racing_line(
        controls, track, bike, reference_s,
        sample_spacing_m=spacing,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        speed_backend="python",
    )
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(
            f"reference controls infeasible at {spacing:.3f} m: {evaluation.failure_reason}")
    return evaluation


def _reference_config(args):
    return PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m,
        max_sweeps=args.reference_max_sweeps,
        max_evaluations=args.reference_max_evaluations,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        optimisation_sample_spacing_m=1.0,
        parallel_workers=args.workers,
        speed_backend=args.speed_backend,
    )


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.latent_count < 4:
        raise ValueError("latent-count must be at least four")
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    reference_s = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, reference_s, phase9.BOUNDARY_MARGIN_M)
    reviewed_controls = phase8.load_initial_controls_csv(
        args.reviewed_reference_controls_csv, reference_s, lower, upper)
    centreline_controls = np.zeros(len(reference_s))

    args.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"reviewed_reference_controls_csv={args.reviewed_reference_controls_csv}")
    print(f"reviewed_reference_controls_sha256={phase9.sha256_file(args.reviewed_reference_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"reference_control_count={len(reference_s)}")
    print(f"latent_control_count={args.latent_count}")
    print(f"latent_bound_m={args.latent_bound_m:.9f}")
    print(f"workers={args.workers}")
    print(f"speed_backend={args.speed_backend}")
    print(f"common_ranking_spacing_m={args.ranking_spacing_m:.3f}")

    latent_config = PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m,
        max_sweeps=args.latent_max_sweeps,
        max_evaluations=args.latent_max_evaluations,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        optimisation_sample_spacing_m=1.0,
        parallel_workers=1,
        speed_backend=args.speed_backend,
    )
    latent_lower = np.full(args.latent_count, -args.latent_bound_m)
    latent_upper = np.full(args.latent_count, args.latent_bound_m)
    latent_initial = np.zeros(args.latent_count)

    def evaluate_latent(latent_controls):
        reference_controls = latent_to_reference_controls(
            latent_controls, reference_s, track.total_length_m, lower, upper)
        return evaluate_planar_racing_line(
            reference_controls, track, bike, reference_s,
            sample_spacing_m=1.0,
            boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
            boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
            speed_backend=args.speed_backend,
        )

    latent_initial_evaluation = evaluate_latent(latent_initial)
    if not latent_initial_evaluation.feasible:
        raise RuntimeError(
            f"reference-policy centreline unexpectedly infeasible: "
            f"{latent_initial_evaluation.failure_reason}")

    def materialise_latent(latent_controls, worker_evaluation):
        regenerated = evaluate_latent(latent_controls)
        if (not regenerated.feasible
                or abs(regenerated.lap_time_s - worker_evaluation.lap_time_s) > 1e-9):
            raise RuntimeError("latent poll winner materialisation disagreed with worker")
        return regenerated

    started = time.perf_counter()
    if args.workers == 1:
        latent_result = _best_improvement_pattern_search(
            latent_initial, latent_lower, latent_upper, latent_initial_evaluation,
            evaluate_latent, latent_config)
    else:
        with ProcessPoolExecutor(
                max_workers=args.workers,
                mp_context=multiprocessing.get_context("spawn"),
                initializer=_initialise_latent_worker,
                initargs=(
                    track, bike, reference_s, lower, upper, args.latent_count, 1.0,
                    phase9.BOUNDARY_MARGIN_M, phase9.BOUNDARY_CHECK_SPACING_M,
                    args.speed_backend,
                )) as executor:
            latent_result = _best_improvement_pattern_search(
                latent_initial, latent_lower, latent_upper, latent_initial_evaluation,
                evaluate_latent, latent_config,
                lambda candidates: executor.map(_evaluate_latent_worker, candidates),
                materialise_latent,
            )
    latent_elapsed = time.perf_counter() - started
    (best_latent, best_latent_evaluation, latent_evaluations, latent_sweeps,
     latent_step, latent_reason) = latent_result
    latent_reference_controls = latent_to_reference_controls(
        best_latent, reference_s, track.total_length_m, lower, upper)
    latent_common = _evaluate_reference(
        track, bike, reference_s, latent_reference_controls, args.ranking_spacing_m)
    phase8.atomic_write_controls_csv(
        args.output_dir / "latent_warm_start_reference_controls.csv",
        reference_s, latent_reference_controls, lower, upper)

    print(
        f"stage=latent_reference_search initial_lap_s={latent_initial_evaluation.lap_time_s:.9f} "
        f"final_lap_s={best_latent_evaluation.lap_time_s:.9f} "
        f"evaluations={latent_evaluations} sweeps={latent_sweeps} "
        f"final_step_m={latent_step:.9f} termination={latent_reason!r} "
        f"elapsed_s={latent_elapsed:.3f}")
    print(f"latent_common_grid_lap_s={latent_common.lap_time_s:.9f}")

    started = time.perf_counter()
    reference = optimise_planar_racing_line(
        track, bike, REFERENCE_PLANAR_CONTROL_POLICY, _reference_config(args),
        initial_controls_m=latent_reference_controls,
    )
    reference_elapsed = time.perf_counter() - started
    phase8.atomic_write_controls_csv(
        args.output_dir / "reference_final_controls.csv",
        reference.control_s_m, reference.best_controls_m,
        reference.lower_bounds_m, reference.upper_bounds_m)
    print(
        f"stage=reference_from_latent initial_lap_s={reference.initial_lap_time_s:.9f} "
        f"final_lap_s={reference.best_lap_time_s:.9f} "
        f"evaluations={reference.evaluations} sweeps={reference.sweeps} "
        f"final_step_m={reference.final_step_m:.9f} "
        f"termination={reference.termination_reason!r} elapsed_s={reference_elapsed:.3f}")

    final_common = _evaluate_reference(
        track, bike, reference_s, reference.best_controls_m, args.ranking_spacing_m)
    reviewed_common = _evaluate_reference(
        track, bike, reference_s, reviewed_controls, args.ranking_spacing_m)
    delta_controls = np.asarray(reference.best_controls_m) - reviewed_controls
    delta_lap = final_common.lap_time_s - reviewed_common.lap_time_s

    print(f"latent_hierarchy_common_grid_lap_s={final_common.lap_time_s:.9f}")
    print(f"reviewed_common_grid_lap_s={reviewed_common.lap_time_s:.9f}")
    print(f"latent_hierarchy_minus_reviewed_s={delta_lap:.9f}")
    print(f"maximum_abs_control_delta_to_reviewed_m={np.max(np.abs(delta_controls)):.9f}")
    print(f"rms_control_delta_to_reviewed_m={np.sqrt(np.mean(delta_controls ** 2)):.9f}")
    print("interpretation_note=Phase 11C tests whether reducing search dimension while retaining the full reference guide representation improves basin finding from a generic centreline")
    print("calibration_note=no motorcycle, rider, track, or roll-rate parameter is fitted by this command")


if __name__ == "__main__":
    main()
