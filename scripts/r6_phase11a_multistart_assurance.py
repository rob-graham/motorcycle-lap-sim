"""Screen Mallala roll-aware optimisation sensitivity to deterministic starting lines.

This Phase 11A diagnostic does not introduce a new optimiser or new physics.  It
runs the existing deterministic planar optimiser from a small set of materially
different starting controls, then re-evaluates every final candidate on one
common fine grid using the Python speed backend.

The objective is optimisation assurance for track-layout work: determine whether
credible starts collapse toward essentially the same solution, or whether local
optima remain important enough to justify improving the optimiser.
"""

import argparse
import csv
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import time

import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    PlanarOptimisationConfig,
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    optimise_planar_racing_line,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


DEFAULT_PERTURBATION_AMPLITUDE_M = 1.0
DEFAULT_RANKING_SPACING_M = 0.25
RANKING_SPACINGS_M = (1.0, 0.5, 0.25)


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling(
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_multistart_assurance")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_multistart_assurance")
phase9f = _load_sibling(
    "r6_phase9f_roll_aware_optimisation.py", "r6_phase9f_for_multistart_assurance")


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
        "reference_controls_csv", type=Path,
        help="reviewed roll-aware controls used as the current-best start and perturbation base",
    )
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--max-evaluations", type=_positive_int, default=2500)
    parser.add_argument("--max-sweeps", type=_positive_int, default=20)
    parser.add_argument("--initial-step-m", type=_positive_float, default=1.0)
    parser.add_argument("--workers", type=_positive_int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument(
        "--ranking-spacing-m", type=float, choices=RANKING_SPACINGS_M,
        default=DEFAULT_RANKING_SPACING_M,
    )
    parser.add_argument(
        "--perturbation-amplitude-m", type=_positive_float,
        default=DEFAULT_PERTURBATION_AMPLITUDE_M,
    )
    return parser


def bounded_smooth_perturbation(
        base_controls_m, control_s_m, lap_length_m,
        lower_bounds_m, upper_bounds_m, amplitude_m, direction):
    """Return a deterministic low-frequency perturbation that remains in bounds."""
    base = np.asarray(base_controls_m, dtype=float)
    stations = np.asarray(control_s_m, dtype=float)
    lower = np.asarray(lower_bounds_m, dtype=float)
    upper = np.asarray(upper_bounds_m, dtype=float)
    if base.shape != stations.shape or lower.shape != base.shape or upper.shape != base.shape:
        raise ValueError("controls, stations and bounds must have identical shapes")
    if (base.ndim != 1 or len(base) == 0
            or not np.all(np.isfinite(base))
            or not np.all(np.isfinite(stations))
            or not np.all(np.isfinite(lower))
            or not np.all(np.isfinite(upper))):
        raise ValueError("controls, stations and bounds must be non-empty finite 1D arrays")
    if not math.isfinite(lap_length_m) or lap_length_m <= 0.0:
        raise ValueError("lap length must be finite and positive")
    if not math.isfinite(amplitude_m) or amplitude_m <= 0.0:
        raise ValueError("perturbation amplitude must be finite and positive")
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or +1")
    if np.any(lower > upper) or np.any(base < lower) or np.any(base > upper):
        raise ValueError("base controls must lie within valid bounds")

    phase = 2.0 * math.pi * stations / lap_length_m
    wave = np.sin(phase + 0.35) + 0.45 * np.sin(3.0 * phase - 0.7)
    peak = float(np.max(np.abs(wave)))
    if peak == 0.0:
        raise RuntimeError("deterministic perturbation unexpectedly has zero amplitude")
    desired = direction * amplitude_m * wave / peak
    available = np.where(desired >= 0.0, upper - base, base - lower)
    scale = np.ones_like(desired)
    nonzero = np.abs(desired) > 0.0
    scale[nonzero] = np.minimum(1.0, available[nonzero] / np.abs(desired[nonzero]))
    candidate = base + desired * scale
    return np.clip(candidate, lower, upper)


def build_starting_controls(
        track, stations, lower, upper, frozen_controls, reference_controls,
        perturbation_amplitude_m):
    """Return the fixed Phase 11A starting set in deterministic order."""
    centreline = np.clip(np.zeros_like(stations, dtype=float), lower, upper)
    starts = (
        ("reviewed_roll_aware", np.asarray(reference_controls, dtype=float).copy()),
        ("frozen_phase8", np.asarray(frozen_controls, dtype=float).copy()),
        ("centreline", centreline),
        (
            "perturb_plus",
            bounded_smooth_perturbation(
                reference_controls, stations, track.total_length_m,
                lower, upper, perturbation_amplitude_m, +1),
        ),
        (
            "perturb_minus",
            bounded_smooth_perturbation(
                reference_controls, stations, track.total_length_m,
                lower, upper, perturbation_amplitude_m, -1),
        ),
    )
    return starts


def rank_candidates(rows):
    """Return a stable ascending common-grid ranking without hiding ties."""
    return sorted(rows, key=lambda row: (row["common_grid_lap_s"], row["start_name"]))


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
            f"candidate controls are infeasible at {spacing:.2f} m: {evaluation.failure_reason}")
    return evaluation


def _write_summary(path, rows):
    fields = (
        "rank", "start_name", "optimisation_initial_lap_s", "optimisation_final_lap_s",
        "common_grid_lap_s", "common_grid_delta_to_best_s", "evaluations", "sweeps",
        "final_step_m", "termination_reason", "minimum_boundary_clearance_m",
        "maximum_abs_control_delta_to_best_m", "rms_control_delta_to_best_m",
        "output_controls_csv",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in fields})


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    frozen_controls = phase9.load_frozen_controls(
        phase9.DEFAULT_CONTROLS, stations, lower, upper)
    reference_controls = phase8.load_initial_controls_csv(
        args.reference_controls_csv, stations, lower, upper)

    starts = build_starting_controls(
        track, stations, lower, upper, frozen_controls, reference_controls,
        args.perturbation_amplitude_m,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)

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

    print(f"reference_controls_csv={args.reference_controls_csv}")
    print(f"reference_controls_sha256={phase9.sha256_file(args.reference_controls_csv)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"start_count={len(starts)}")
    print(f"max_evaluations_per_start={args.max_evaluations}")
    print(f"max_sweeps_per_start={args.max_sweeps}")
    print(f"initial_step_m={args.initial_step_m:.9f}")
    print(f"workers={args.workers}")
    print(f"speed_backend={args.speed_backend}")
    print(f"common_ranking_spacing_m={args.ranking_spacing_m:.2f}")
    print(f"perturbation_amplitude_m={args.perturbation_amplitude_m:.9f}")

    rows = []
    final_controls = {}
    for start_name, initial_controls in starts:
        initial_common = _evaluate_controls(
            track, bike, stations, initial_controls, args.ranking_spacing_m)
        started = time.perf_counter()
        result = optimise_planar_racing_line(
            track, bike, REFERENCE_PLANAR_CONTROL_POLICY, config,
            initial_controls_m=initial_controls,
        )
        elapsed = time.perf_counter() - started
        output_controls = args.output_dir / f"{start_name}_final_controls.csv"
        phase8.atomic_write_controls_csv(
            output_controls, result.control_s_m, result.best_controls_m,
            result.lower_bounds_m, result.upper_bounds_m)
        common = _evaluate_controls(
            track, bike, stations, result.best_controls_m, args.ranking_spacing_m)
        final_controls[start_name] = np.asarray(result.best_controls_m, dtype=float).copy()
        rows.append({
            "start_name": start_name,
            "optimisation_initial_lap_s": result.initial_lap_time_s,
            "optimisation_final_lap_s": result.best_lap_time_s,
            "common_grid_initial_lap_s": initial_common.lap_time_s,
            "common_grid_lap_s": common.lap_time_s,
            "evaluations": result.evaluations,
            "sweeps": result.sweeps,
            "final_step_m": result.final_step_m,
            "termination_reason": result.termination_reason,
            "minimum_boundary_clearance_m": result.minimum_boundary_clearance_m,
            "elapsed_s": elapsed,
            "output_controls_csv": str(output_controls),
        })
        print(
            f"start={start_name} "
            f"common_grid_initial_lap_s={initial_common.lap_time_s:.9f} "
            f"optimisation_initial_lap_s={result.initial_lap_time_s:.9f} "
            f"optimisation_final_lap_s={result.best_lap_time_s:.9f} "
            f"common_grid_final_lap_s={common.lap_time_s:.9f} "
            f"evaluations={result.evaluations} sweeps={result.sweeps} "
            f"final_step_m={result.final_step_m:.9f} "
            f"termination={result.termination_reason!r} elapsed_s={elapsed:.3f}")

    ranked = rank_candidates(rows)
    best = ranked[0]
    best_controls = final_controls[best["start_name"]]
    best_lap = float(best["common_grid_lap_s"])
    for rank, row in enumerate(ranked, start=1):
        delta_controls = final_controls[row["start_name"]] - best_controls
        row["rank"] = rank
        row["common_grid_delta_to_best_s"] = float(row["common_grid_lap_s"] - best_lap)
        row["maximum_abs_control_delta_to_best_m"] = float(np.max(np.abs(delta_controls)))
        row["rms_control_delta_to_best_m"] = float(np.sqrt(np.mean(delta_controls ** 2)))

    print("common_grid_ranking:")
    for row in ranked:
        print(
            f"  rank={row['rank']} start={row['start_name']} "
            f"lap_s={row['common_grid_lap_s']:.9f} "
            f"delta_to_best_s={row['common_grid_delta_to_best_s']:.9f} "
            f"max_control_delta_to_best_m={row['maximum_abs_control_delta_to_best_m']:.6f} "
            f"rms_control_delta_to_best_m={row['rms_control_delta_to_best_m']:.6f}")

    spread = float(ranked[-1]["common_grid_lap_s"] - best_lap)
    print(f"common_grid_best_start={best['start_name']}")
    print(f"common_grid_best_lap_s={best_lap:.9f}")
    print(f"common_grid_full_spread_s={spread:.9f}")
    print("interpretation_note=this is a bounded multistart screening diagnostic, not proof of a global optimum; materially different final solutions justify further optimiser work, while close convergence supports retaining the simple deterministic search")
    print("calibration_note=no motorcycle, rider, track, or roll-rate parameter is fitted by this command")

    summary_csv = args.output_dir / "phase11a_multistart_summary.csv"
    _write_summary(summary_csv, ranked)
    print(f"summary_csv={summary_csv}")


if __name__ == "__main__":
    main()
