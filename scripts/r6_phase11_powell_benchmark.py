"""Bounded independent Powell benchmark for Phase 11 optimisation assurance.

This diagnostic does not replace the retained deterministic planar optimiser.
It starts from saved margin-specific controls produced by that method, applies
SciPy's bounded derivative-free Powell method to the same 52 physical controls,
and ranks the starting and final feasible lines on the same authoritative
Python common grid.

SciPy is an optional benchmark dependency and is imported only when the
benchmark is executed.
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
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
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


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_powell_benchmark")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_powell_benchmark")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_powell_benchmark")
phase11 = _load_sibling("r6_phase11_margin_aware_reoptimisation.py", "phase11_powell_benchmark_base")

DEFAULT_MARGINS_M = (0.25, 0.50)
INFEASIBLE_OBJECTIVE_S = 1.0e6


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


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("reviewed_reference_controls_csv", type=Path)
    parser.add_argument("start_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--margins-m", type=_nonnegative_float, nargs="+", default=DEFAULT_MARGINS_M)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float, default=0.8)
    parser.add_argument("--max-evaluations", type=_positive_int, default=4000)
    parser.add_argument("--xtol", type=_positive_float, default=1e-4)
    parser.add_argument("--ftol", type=_positive_float, default=1e-8)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="numba")
    parser.add_argument("--common-spacing-m", type=_positive_float, default=0.125)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float, default=0.125)
    return parser


def _load_scipy_minimize():
    try:
        from scipy.optimize import minimize
    except (ImportError, ModuleNotFoundError) as error:
        raise RuntimeError(
            "SciPy is required only for this benchmark; install it in the active "
            "environment with 'python -m pip install scipy'") from error
    return minimize


def _evaluate(track, bike, stations, controls, spacing, margin, check_spacing, backend):
    return evaluate_planar_racing_line(
        controls, track, bike, stations,
        sample_spacing_m=spacing,
        boundary_margin_m=margin,
        boundary_check_spacing_m=check_spacing,
        speed_backend=backend,
    )


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


class FeasibleObjective:
    """Track the best feasible candidate while returning a finite Powell objective."""

    def __init__(self, evaluate):
        self._evaluate = evaluate
        self.evaluations = 0
        self.infeasible_evaluations = 0
        self.best_controls = None
        self.best_lap_time_s = math.inf

    def __call__(self, controls):
        candidate = np.asarray(controls, dtype=float)
        evaluation = self._evaluate(candidate)
        self.evaluations += 1
        if not evaluation.feasible:
            self.infeasible_evaluations += 1
            return INFEASIBLE_OBJECTIVE_S
        lap = float(evaluation.lap_time_s)
        if lap < self.best_lap_time_s:
            self.best_lap_time_s = lap
            self.best_controls = candidate.copy()
        return lap


def _write_summary(path, rows):
    fields = (
        "margin_m", "start_common_grid_lap_s", "powell_objective_lap_s",
        "final_common_grid_lap_s", "delta_final_minus_start_common_s",
        "delta_final_minus_reviewed_common_s", "minimum_usable_clearance_m",
        "minimum_track_edge_clearance_m", "evaluations", "infeasible_evaluations",
        "nit", "success", "status", "message", "elapsed_s",
        "maximum_abs_control_delta_from_start_m", "rms_control_delta_from_start_m",
        "output_controls_csv",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()
    minimize = _load_scipy_minimize()

    margins = tuple(sorted(set(float(value) for value in args.margins_m)))
    if not margins:
        raise ValueError("at least one margin is required")
    phase11.require_unique_margin_control_filenames(margins)

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)

    canonical_lower, canonical_upper = planar_control_bounds(track, stations, phase9.BOUNDARY_MARGIN_M)
    reviewed = phase8.load_initial_controls_csv(
        args.reviewed_reference_controls_csv, stations, canonical_lower, canonical_upper)
    reviewed_common = _require_feasible(
        _evaluate(track, bike, stations, reviewed, args.common_spacing_m, 0.0,
                  args.boundary_check_spacing_m, "python"),
        "reviewed physical line on zero-margin comparison corridor",
    )
    reviewed_lap = float(reviewed_common.lap_time_s)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"reviewed_reference_controls_csv={args.reviewed_reference_controls_csv}")
    print(f"reviewed_reference_controls_sha256={phase9.sha256_file(args.reviewed_reference_controls_csv)}")
    print(f"start_dir={args.start_dir}")
    print(f"control_count={len(stations)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print("benchmark_method=scipy.optimize.minimize(method='Powell')")
    print("benchmark_note=independent bounded diagnostic; not a production optimiser replacement")
    print("optimisation_sample_spacing_m=1.000000")
    print(f"common_ranking_spacing_m={args.common_spacing_m:.6f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.6f}")
    print(f"max_evaluations_per_margin={args.max_evaluations}")
    print(f"xtol={args.xtol:.12g}")
    print(f"ftol={args.ftol:.12g}")
    print(f"speed_backend={args.speed_backend}")
    print(f"reviewed_fixed_line_common_grid_lap_s={reviewed_lap:.9f}")

    rows = []
    for margin in margins:
        lower, upper = planar_control_bounds(track, stations, margin)
        start_csv = args.start_dir / phase11.margin_controls_filename(margin)
        start = phase8.load_initial_controls_csv(start_csv, stations, lower, upper)
        start_common = _require_feasible(
            _evaluate(track, bike, stations, start, args.common_spacing_m, margin,
                      args.boundary_check_spacing_m, "python"),
            f"margin {margin:.3f} m benchmark start on common grid",
        )

        objective = FeasibleObjective(lambda controls: _evaluate(
            track, bike, stations, controls, 1.0, margin,
            args.boundary_check_spacing_m, args.speed_backend))
        # Prime the tracker so a feasible result is always available even if
        # Powell exhausts its budget while probing infeasible spline geometries.
        objective(start)
        started = time.perf_counter()
        result = minimize(
            objective,
            start,
            method="Powell",
            bounds=list(zip(lower, upper)),
            options={
                "maxfev": args.max_evaluations,
                "xtol": args.xtol,
                "ftol": args.ftol,
                "disp": False,
            },
        )
        elapsed = time.perf_counter() - started

        if objective.best_controls is None:
            raise RuntimeError(f"margin {margin:.3f} m Powell benchmark found no feasible candidate")
        best_controls = objective.best_controls
        final_common = _require_feasible(
            _evaluate(track, bike, stations, best_controls, args.common_spacing_m, margin,
                      args.boundary_check_spacing_m, "python"),
            f"margin {margin:.3f} m Powell best line on common grid",
        )
        output_controls = args.output_dir / phase11.margin_controls_filename(margin)
        phase8.atomic_write_controls_csv(output_controls, stations, best_controls, lower, upper)

        delta = np.asarray(best_controls) - start
        usable_clearance = float(final_common.smooth_line.minimum_boundary_clearance_m)
        row = {
            "margin_m": margin,
            "start_common_grid_lap_s": float(start_common.lap_time_s),
            "powell_objective_lap_s": float(objective.best_lap_time_s),
            "final_common_grid_lap_s": float(final_common.lap_time_s),
            "delta_final_minus_start_common_s": float(final_common.lap_time_s - start_common.lap_time_s),
            "delta_final_minus_reviewed_common_s": float(final_common.lap_time_s - reviewed_lap),
            "minimum_usable_clearance_m": usable_clearance,
            "minimum_track_edge_clearance_m": usable_clearance + margin,
            "evaluations": int(objective.evaluations),
            "infeasible_evaluations": int(objective.infeasible_evaluations),
            "nit": int(getattr(result, "nit", -1)),
            "success": bool(result.success),
            "status": int(result.status),
            "message": str(result.message),
            "elapsed_s": float(elapsed),
            "maximum_abs_control_delta_from_start_m": float(np.max(np.abs(delta))),
            "rms_control_delta_from_start_m": float(np.sqrt(np.mean(delta ** 2))),
            "output_controls_csv": str(output_controls),
        }
        rows.append(row)
        print(
            f"margin_m={margin:.6f} start_controls_csv={start_csv} "
            f"start_common_lap_s={start_common.lap_time_s:.9f} "
            f"powell_objective_lap_s={objective.best_lap_time_s:.9f} "
            f"final_common_grid_lap_s={final_common.lap_time_s:.9f} "
            f"delta_final_minus_start_common_s={row['delta_final_minus_start_common_s']:.9f} "
            f"minimum_track_edge_clearance_m={row['minimum_track_edge_clearance_m']:.9f} "
            f"evaluations={objective.evaluations} infeasible_evaluations={objective.infeasible_evaluations} "
            f"nit={row['nit']} success={row['success']} status={row['status']} "
            f"elapsed_s={elapsed:.3f} message={row['message']!r}")

    if len(rows) == 2:
        print(f"final_common_margin_gap_s={rows[1]['final_common_grid_lap_s'] - rows[0]['final_common_grid_lap_s']:.9f}")
    summary = args.output_dir / "phase11_powell_benchmark_summary.csv"
    _write_summary(summary, rows)
    print(f"summary_csv={summary}")
    return rows


if __name__ == "__main__":
    main()
