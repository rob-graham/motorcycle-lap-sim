"""Bounded deterministic global screening before Mallala reference optimisation.

Phase 11A-C showed that local coordinate-pattern variants remain strongly
warm-start dependent. Phase 11D keeps the same 52-guide reference geometry and
physics evaluator, but replaces the first local walk from centreline with an
independent low-discrepancy screen of smooth latent racing-line candidates.

The screen is deterministic, parallelisable and deliberately bounded. Its best
feasible reference-guide candidate warm-starts the unchanged 52-control planar
optimiser. No physical parameter is calibrated by this diagnostic.
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
from motorcycle_lap_sim.track import Track


DEFAULT_LATENT_COUNT = 12
DEFAULT_LATENT_BOUND_M = 4.0
DEFAULT_SCREEN_CANDIDATES = 512
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
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_phase11d")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_phase11d")
phase9f = _load_sibling(
    "r6_phase9f_roll_aware_optimisation.py", "r6_phase9f_for_phase11d")


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
    parser.add_argument("reviewed_reference_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--latent-count", type=_positive_int, default=DEFAULT_LATENT_COUNT)
    parser.add_argument("--latent-bound-m", type=_positive_float, default=DEFAULT_LATENT_BOUND_M)
    parser.add_argument("--screen-candidates", type=_positive_int, default=DEFAULT_SCREEN_CANDIDATES)
    parser.add_argument("--reference-max-evaluations", type=_positive_int, default=4000)
    parser.add_argument("--reference-max-sweeps", type=_positive_int, default=30)
    parser.add_argument("--initial-step-m", type=_positive_float, default=1.0)
    parser.add_argument("--workers", type=_positive_int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument("--ranking-spacing-m", type=_positive_float, default=DEFAULT_RANKING_SPACING_M)
    return parser


def first_primes(count):
    """Return the first ``count`` primes for a dependency-free Halton sequence."""
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("prime count must be a positive integer")
    primes = []
    candidate = 2
    while len(primes) < count:
        limit = int(math.sqrt(candidate))
        if all(candidate % prime for prime in primes if prime <= limit):
            primes.append(candidate)
        candidate += 1
    return tuple(primes)


def radical_inverse(index, base):
    if isinstance(index, bool) or not isinstance(index, int) or index <= 0:
        raise ValueError("Halton index must be a positive integer")
    if isinstance(base, bool) or not isinstance(base, int) or base < 2:
        raise ValueError("Halton base must be an integer of at least two")
    value = 0.0
    factor = 1.0 / base
    while index:
        index, digit = divmod(index, base)
        value += digit * factor
        factor /= base
    return value


def halton_latent_candidates(count, dimension, bound_m):
    """Return centreline plus deterministic low-discrepancy latent candidates."""
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("candidate count must be a positive integer")
    if isinstance(dimension, bool) or not isinstance(dimension, int) or dimension < 4:
        raise ValueError("latent dimension must be an integer of at least four")
    if not math.isfinite(bound_m) or bound_m <= 0.0:
        raise ValueError("latent bound must be finite and positive")
    candidates = np.zeros((count, dimension), dtype=float)
    if count == 1:
        return candidates
    bases = first_primes(dimension)
    for row in range(1, count):
        index = row
        candidates[row] = bound_m * np.array([
            2.0 * radical_inverse(index, base) - 1.0 for base in bases
        ])
    return candidates


def latent_to_reference_controls(
        latent_controls_m, reference_s_m, lap_length_m, lower_bounds_m, upper_bounds_m):
    latent = np.asarray(latent_controls_m, dtype=float)
    reference_s = np.asarray(reference_s_m, dtype=float)
    lower = np.asarray(lower_bounds_m, dtype=float)
    upper = np.asarray(upper_bounds_m, dtype=float)
    if latent.ndim != 1 or len(latent) < 4 or not np.all(np.isfinite(latent)):
        raise ValueError("latent controls must be a finite 1D array with at least four values")
    if reference_s.ndim != 1 or lower.shape != reference_s.shape or upper.shape != reference_s.shape:
        raise ValueError("reference stations and bounds must be matching 1D arrays")
    parameterisation = PeriodicCubicParameterisation(len(latent))
    sampled_reference = SimpleNamespace(s_m=reference_s, total_length_m=lap_length_m)
    smooth_offsets = parameterisation.latent_values(latent, sampled_reference)
    return np.clip(smooth_offsets, lower, upper)


def best_screened_candidate(candidates, evaluations):
    """Return the stable best feasible candidate/evaluation/index triple."""
    candidates = np.asarray(candidates, dtype=float)
    if candidates.ndim != 2 or len(candidates) != len(evaluations):
        raise ValueError("candidates and evaluations must have matching rows")
    feasible = [
        (evaluation.lap_time_s, index, candidate, evaluation)
        for index, (candidate, evaluation) in enumerate(zip(candidates, evaluations))
        if evaluation.feasible and math.isfinite(evaluation.lap_time_s)
    ]
    if not feasible:
        raise RuntimeError("global latent screen produced no feasible candidate")
    _, index, candidate, evaluation = min(feasible, key=lambda item: (item[0], item[1]))
    return candidate.copy(), evaluation, index, len(feasible)


_worker_context = None


def _initialise_worker(
        track, bike, reference_s, lower, upper, sample_spacing_m,
        boundary_margin_m, boundary_check_spacing_m, speed_backend):
    global _worker_context
    _worker_context = (
        track, bike, np.asarray(reference_s), np.asarray(lower), np.asarray(upper),
        sample_spacing_m, boundary_margin_m, boundary_check_spacing_m, speed_backend,
    )


def _evaluate_worker(latent_controls_m):
    context = _worker_context
    if context is None:
        raise RuntimeError("global-screen worker context was not initialised")
    (track, bike, reference_s, lower, upper, sample_spacing_m,
     boundary_margin_m, boundary_check_spacing_m, speed_backend) = context
    controls = latent_to_reference_controls(
        latent_controls_m, reference_s, track.total_length_m, lower, upper)
    evaluation = evaluate_planar_racing_line(
        controls, track, bike, reference_s,
        sample_spacing_m=sample_spacing_m,
        boundary_margin_m=boundary_margin_m,
        boundary_check_spacing_m=boundary_check_spacing_m,
        speed_backend=speed_backend,
    )
    return SimpleNamespace(
        feasible=evaluation.feasible,
        lap_time_s=evaluation.lap_time_s,
        failure_reason=evaluation.failure_reason,
    )


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
    args.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"reviewed_reference_controls_csv={args.reviewed_reference_controls_csv}")
    print(f"reviewed_reference_controls_sha256={phase9.sha256_file(args.reviewed_reference_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"reference_control_count={len(reference_s)}")
    print(f"latent_control_count={args.latent_count}")
    print(f"latent_bound_m={args.latent_bound_m:.9f}")
    print(f"screen_candidate_count={args.screen_candidates}")
    print(f"workers={args.workers}")
    print(f"speed_backend={args.speed_backend}")
    print(f"common_ranking_spacing_m={args.ranking_spacing_m:.3f}")

    candidates = halton_latent_candidates(
        args.screen_candidates, args.latent_count, args.latent_bound_m)
    started = time.perf_counter()
    if args.workers == 1:
        _initialise_worker(
            track, bike, reference_s, lower, upper, 1.0,
            phase9.BOUNDARY_MARGIN_M, phase9.BOUNDARY_CHECK_SPACING_M,
            args.speed_backend)
        evaluations = [_evaluate_worker(candidate) for candidate in candidates]
    else:
        with ProcessPoolExecutor(
                max_workers=args.workers,
                mp_context=multiprocessing.get_context("spawn"),
                initializer=_initialise_worker,
                initargs=(
                    track, bike, reference_s, lower, upper, 1.0,
                    phase9.BOUNDARY_MARGIN_M, phase9.BOUNDARY_CHECK_SPACING_M,
                    args.speed_backend,
                )) as executor:
            evaluations = list(executor.map(_evaluate_worker, candidates))
    screen_elapsed = time.perf_counter() - started

    best_latent, best_evaluation, best_index, feasible_count = best_screened_candidate(
        candidates, evaluations)
    screened_reference_controls = latent_to_reference_controls(
        best_latent, reference_s, track.total_length_m, lower, upper)
    screened_common = _evaluate_reference(
        track, bike, reference_s, screened_reference_controls, args.ranking_spacing_m)
    phase8.atomic_write_controls_csv(
        args.output_dir / "screened_warm_start_reference_controls.csv",
        reference_s, screened_reference_controls, lower, upper)

    print(f"screen_feasible_count={feasible_count}")
    print(f"screen_infeasible_count={args.screen_candidates - feasible_count}")
    print(f"screen_best_index={best_index}")
    print(f"screen_best_optimisation_grid_lap_s={best_evaluation.lap_time_s:.9f}")
    print(f"screen_best_common_grid_lap_s={screened_common.lap_time_s:.9f}")
    print(f"screen_elapsed_s={screen_elapsed:.3f}")

    reference_config = PlanarOptimisationConfig(
        initial_step_m=args.initial_step_m,
        max_sweeps=args.reference_max_sweeps,
        max_evaluations=args.reference_max_evaluations,
        boundary_margin_m=phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=phase9.BOUNDARY_CHECK_SPACING_M,
        optimisation_sample_spacing_m=1.0,
        parallel_workers=args.workers,
        speed_backend=args.speed_backend,
    )
    started = time.perf_counter()
    reference = optimise_planar_racing_line(
        track, bike, REFERENCE_PLANAR_CONTROL_POLICY, reference_config,
        initial_controls_m=screened_reference_controls)
    reference_elapsed = time.perf_counter() - started
    phase8.atomic_write_controls_csv(
        args.output_dir / "reference_final_controls.csv",
        reference.control_s_m, reference.best_controls_m,
        reference.lower_bounds_m, reference.upper_bounds_m)
    print(
        f"stage=reference_from_global_screen initial_lap_s={reference.initial_lap_time_s:.9f} "
        f"final_lap_s={reference.best_lap_time_s:.9f} evaluations={reference.evaluations} "
        f"sweeps={reference.sweeps} final_step_m={reference.final_step_m:.9f} "
        f"termination={reference.termination_reason!r} elapsed_s={reference_elapsed:.3f}")

    final_common = _evaluate_reference(
        track, bike, reference_s, reference.best_controls_m, args.ranking_spacing_m)
    reviewed_common = _evaluate_reference(
        track, bike, reference_s, reviewed_controls, args.ranking_spacing_m)
    delta_controls = np.asarray(reference.best_controls_m) - reviewed_controls
    delta_lap = final_common.lap_time_s - reviewed_common.lap_time_s
    print(f"global_screen_common_grid_lap_s={final_common.lap_time_s:.9f}")
    print(f"reviewed_common_grid_lap_s={reviewed_common.lap_time_s:.9f}")
    print(f"global_screen_minus_reviewed_s={delta_lap:.9f}")
    print(f"maximum_abs_control_delta_to_reviewed_m={np.max(np.abs(delta_controls)):.9f}")
    print(f"rms_control_delta_to_reviewed_m={np.sqrt(np.mean(delta_controls ** 2)):.9f}")
    print("interpretation_note=Phase 11D tests deterministic global basin screening before the unchanged local reference optimiser")
    print("calibration_note=no motorcycle, rider, track, or roll-rate parameter is fitted by this command")


if __name__ == "__main__":
    main()
