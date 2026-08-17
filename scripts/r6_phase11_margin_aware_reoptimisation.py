"""Re-optimise the reviewed Mallala roll-aware line with explicit edge margins.

This bounded Phase 11 diagnostic follows the resolution/margin sensitivity study.
It keeps the current reference 52-control representation and finite-roll physics,
changes only the required track-edge margin, and re-optimises each margin case.
Final candidates are re-evaluated on one common 0.125 m Python grid with 0.125 m
dense corridor checking so lap-time cost and achieved edge clearance are directly
comparable.
"""

import argparse
import csv
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
)
from motorcycle_lap_sim.track import Track


DEFAULT_MARGINS_M = (0.25, 0.50)
DEFAULT_COMMON_SPACING_M = 0.125
DEFAULT_BOUNDARY_CHECK_SPACING_M = 0.125
BACKOFF_SCALES = (1.0, 0.75, 0.5, 0.25, 0.125, 0.0625, 0.0)


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_margin_reopt")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_margin_reopt")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_margin_reopt")


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def _nonnegative_float(text):
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return value


def _positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def margin_controls_filename(margin_m):
    """Return the deliberately three-decimal margin controls filename."""
    return f"margin_{float(margin_m):.3f}m_final_controls.csv"


def require_unique_margin_control_filenames(margins):
    """Fail closed when distinct exact margins collide after filename formatting."""
    seen = {}
    for margin in margins:
        margin = float(margin)
        filename = margin_controls_filename(margin)
        if filename in seen and seen[filename] != margin:
            raise ValueError(
                f"margin values {seen[filename]!r} and {margin!r} map to the same "
                f"controls filename {filename!r}; choose margins distinguishable at "
                "0.001 m filename precision"
            )
        seen[filename] = margin


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("reviewed_reference_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--margins-m", type=_nonnegative_float, nargs="+", default=DEFAULT_MARGINS_M)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--max-evaluations", type=_positive_int, default=4000)
    parser.add_argument("--max-sweeps", type=_positive_int, default=30)
    parser.add_argument("--initial-step-m", type=_positive_float, default=0.5)
    parser.add_argument("--workers", type=_positive_int, default=1)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    parser.add_argument("--common-spacing-m", type=_positive_float, default=DEFAULT_COMMON_SPACING_M)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float, default=DEFAULT_BOUNDARY_CHECK_SPACING_M)
    return parser


def projected_seed(reference_controls, lower, upper):
    """Project only guide values into the requested margin bounds."""
    reference = np.asarray(reference_controls, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if reference.shape != lower.shape or lower.shape != upper.shape or reference.ndim != 1:
        raise ValueError("reference controls and bounds must be equal 1D arrays")
    if np.any(lower > upper):
        raise ValueError("lower bounds exceed upper bounds")
    return np.clip(reference, lower, upper)


def backoff_seed_to_feasible(projected, fallback, feasibility_check):
    """Back off a projected seed toward a known-safe fallback deterministically."""
    projected = np.asarray(projected, dtype=float)
    fallback = np.asarray(fallback, dtype=float)
    if projected.shape != fallback.shape or projected.ndim != 1 or len(projected) == 0:
        raise ValueError("projected and fallback controls must be equal non-empty 1D arrays")
    for scale in BACKOFF_SCALES:
        candidate = fallback + scale * (projected - fallback)
        if feasibility_check(candidate):
            return candidate, scale
    raise RuntimeError("fallback controls are unexpectedly infeasible")


def _evaluate(track, bike, stations, controls, spacing, margin, check_spacing):
    return evaluate_planar_racing_line(
        controls, track, bike, stations,
        sample_spacing_m=spacing,
        boundary_margin_m=margin,
        boundary_check_spacing_m=check_spacing,
        speed_backend="python",
    )


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


def _write_summary(path, rows):
    fields = (
        "margin_m", "seed_backoff_scale", "optimisation_initial_lap_s",
        "optimisation_final_lap_s", "evaluations", "sweeps", "final_step_m",
        "termination_reason", "common_grid_lap_s", "delta_to_reviewed_line_s",
        "minimum_usable_clearance_m", "minimum_track_edge_clearance_m",
        "maximum_abs_control_delta_from_reviewed_m", "rms_control_delta_from_reviewed_m",
        "output_controls_csv",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()

    margins = tuple(sorted(set(float(value) for value in args.margins_m)))
    if not margins:
        raise ValueError("at least one margin is required")
    require_unique_margin_control_filenames(margins)

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    canonical_lower, canonical_upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    reviewed = phase8.load_initial_controls_csv(
        args.reviewed_reference_controls_csv, stations, canonical_lower, canonical_upper)

    reviewed_eval = _require_feasible(
        _evaluate(track, bike, stations, reviewed, args.common_spacing_m, 0.0,
                  args.boundary_check_spacing_m),
        "reviewed physical line on zero-margin comparison corridor",
    )
    reviewed_lap = float(reviewed_eval.lap_time_s)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"reviewed_reference_controls_csv={args.reviewed_reference_controls_csv}")
    print(f"reviewed_reference_controls_sha256={phase9.sha256_file(args.reviewed_reference_controls_csv)}")
    print(f"control_count={len(stations)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"margins_m={','.join(f'{value:.6f}' for value in margins)}")
    print(f"optimisation_sample_spacing_m=1.000000")
    print(f"common_ranking_spacing_m={args.common_spacing_m:.6f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.6f}")
    print(f"max_evaluations_per_margin={args.max_evaluations}")
    print(f"max_sweeps_per_margin={args.max_sweeps}")
    print(f"initial_step_m={args.initial_step_m:.9f}")
    print(f"workers={args.workers}")
    print(f"speed_backend={args.speed_backend}")
    print(f"reviewed_fixed_line_common_grid_lap_s={reviewed_lap:.9f}")

    rows = []
    centreline = np.zeros_like(reviewed)
    for margin in margins:
        lower, upper = planar_control_bounds(track, stations, margin)
        fallback = np.clip(centreline, lower, upper)
        projected = projected_seed(reviewed, lower, upper)

        def feasible(candidate):
            evaluation = _evaluate(
                track, bike, stations, candidate, 1.0, margin,
                args.boundary_check_spacing_m)
            return bool(evaluation.feasible)

        seed, scale = backoff_seed_to_feasible(projected, fallback, feasible)
        seed_common = _require_feasible(
            _evaluate(track, bike, stations, seed, args.common_spacing_m, margin,
                      args.boundary_check_spacing_m),
            f"margin {margin:.3f} m seed on common grid",
        )

        config = PlanarOptimisationConfig(
            initial_step_m=args.initial_step_m,
            max_sweeps=args.max_sweeps,
            max_evaluations=args.max_evaluations,
            boundary_margin_m=margin,
            boundary_check_spacing_m=args.boundary_check_spacing_m,
            optimisation_sample_spacing_m=1.0,
            parallel_workers=args.workers,
            speed_backend=args.speed_backend,
        )
        result = optimise_planar_racing_line(
            track, bike, REFERENCE_PLANAR_CONTROL_POLICY, config,
            initial_controls_m=seed,
        )
        common = _require_feasible(
            _evaluate(track, bike, stations, result.best_controls_m,
                      args.common_spacing_m, margin, args.boundary_check_spacing_m),
            f"margin {margin:.3f} m final line on common grid",
        )
        output_controls = args.output_dir / margin_controls_filename(margin)
        phase8.atomic_write_controls_csv(
            output_controls, result.control_s_m, result.best_controls_m,
            result.lower_bounds_m, result.upper_bounds_m)

        delta = np.asarray(result.best_controls_m) - reviewed
        usable_clearance = float(common.smooth_line.minimum_boundary_clearance_m)
        row = {
            "margin_m": margin,
            "seed_backoff_scale": scale,
            "optimisation_initial_lap_s": float(result.initial_lap_time_s),
            "optimisation_final_lap_s": float(result.best_lap_time_s),
            "evaluations": int(result.evaluations),
            "sweeps": int(result.sweeps),
            "final_step_m": float(result.final_step_m),
            "termination_reason": result.termination_reason,
            "common_grid_lap_s": float(common.lap_time_s),
            "delta_to_reviewed_line_s": float(common.lap_time_s - reviewed_lap),
            "minimum_usable_clearance_m": usable_clearance,
            "minimum_track_edge_clearance_m": usable_clearance + margin,
            "maximum_abs_control_delta_from_reviewed_m": float(np.max(np.abs(delta))),
            "rms_control_delta_from_reviewed_m": float(np.sqrt(np.mean(delta ** 2))),
            "output_controls_csv": str(output_controls),
        }
        rows.append(row)
        print(
            f"margin_m={margin:.6f} seed_backoff_scale={scale:.6f} "
            f"seed_common_lap_s={seed_common.lap_time_s:.9f} "
            f"optimisation_initial_lap_s={result.initial_lap_time_s:.9f} "
            f"optimisation_final_lap_s={result.best_lap_time_s:.9f} "
            f"common_grid_lap_s={common.lap_time_s:.9f} "
            f"delta_to_reviewed_line_s={row['delta_to_reviewed_line_s']:.9f} "
            f"minimum_track_edge_clearance_m={row['minimum_track_edge_clearance_m']:.9f} "
            f"evaluations={result.evaluations} sweeps={result.sweeps} "
            f"final_step_m={result.final_step_m:.9f} "
            f"termination_reason={result.termination_reason!r}")

    summary = args.output_dir / "phase11_margin_aware_summary.csv"
    _write_summary(summary, rows)
    print(f"summary_csv={summary}")
    return rows


if __name__ == "__main__":
    main()
