"""Nonlinear-constrained Phase 11 optimisation benchmark using SciPy COBYQA.

This diagnostic does not replace the retained deterministic planar optimiser.
It starts from saved margin-specific controls, keeps the same fixed 52-control
representation, exposes dense corridor and forward-progress requirements as
explicit nonlinear constraints, and ranks the best production-feasible line on
the same authoritative Python common grid.

Routine outputs include saved controls plus a sampled racing-line CSV and a
high-resolution thin-line PNG for each margin. Control-point markers are shown
by default during development and can be disabled for presentation output.
"""

import argparse
import csv
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import time

import matplotlib.pyplot as plt
import numpy as np

from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.racing_line import PeriodicPlanarSpline
from motorcycle_lap_sim.speed_solver import solve_speed_profile
from motorcycle_lap_sim.track import Track, sample_track_stations


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_constrained_benchmark")
phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_constrained_benchmark")
phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_constrained_benchmark")
phase11 = _load_sibling("r6_phase11_margin_aware_reoptimisation.py", "phase11_constrained_benchmark_base")

DEFAULT_MARGINS_M = (0.25, 0.50)
DEFAULT_INITIAL_TR_RADIUS_M = 0.05
DEFAULT_FINAL_TR_RADIUS_M = 0.002
DEFAULT_FEASIBILITY_TOL = 1e-10
DEFAULT_PLOT_DPI = 400
PRODUCTION_BOUNDARY_TOL_M = 1e-10
INVALID_OBJECTIVE_S = 1.0e6


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
    parser.add_argument("--max-iterations", type=_positive_int, default=2000)
    parser.add_argument("--initial-tr-radius-m", type=_positive_float, default=DEFAULT_INITIAL_TR_RADIUS_M)
    parser.add_argument("--final-tr-radius-m", type=_positive_float, default=DEFAULT_FINAL_TR_RADIUS_M)
    parser.add_argument("--feasibility-tol", type=_positive_float, default=DEFAULT_FEASIBILITY_TOL)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="numba")
    parser.add_argument("--common-spacing-m", type=_positive_float, default=0.125)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float, default=0.125)
    parser.add_argument("--plot-dpi", type=_positive_int, default=DEFAULT_PLOT_DPI)
    parser.add_argument(
        "--show-control-points",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="show control-point markers in development plots",
    )
    return parser


def _load_scipy_tools():
    try:
        from scipy.optimize import NonlinearConstraint, minimize
    except (ImportError, ModuleNotFoundError) as error:
        raise RuntimeError(
            "SciPy is required only for this benchmark; install the benchmark extra "
            "with 'python -m pip install -e \".[test,accelerated,benchmark]\"'") from error
    return minimize, NonlinearConstraint


def _speed_solver(speed_backend):
    if speed_backend == "python":
        return solve_speed_profile
    if speed_backend != "numba":
        raise ValueError(f"unsupported speed backend {speed_backend!r}")
    try:
        from motorcycle_lap_sim.speed_solver.numba_backend import solve_speed_profile_numba
    except (ImportError, ModuleNotFoundError) as error:
        raise RuntimeError(
            "Numba speed backend is unavailable; install the accelerated extra with "
            "'python -m pip install -e \".[test,accelerated,benchmark]\"'") from error
    return solve_speed_profile_numba


def _evaluate(track, bike, stations, controls, spacing, margin, check_spacing, backend):
    return evaluate_planar_racing_line(
        controls,
        track,
        bike,
        stations,
        sample_spacing_m=spacing,
        boundary_margin_m=margin,
        boundary_check_spacing_m=check_spacing,
        speed_backend=backend,
    )


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


def production_feasible_constraint_values(values):
    """Mirror production dense-corridor and forward-progress acceptance."""
    values = np.asarray(values, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        return False
    left_clearance, right_clearance, minimum_forward = values
    return bool(
        left_clearance >= -PRODUCTION_BOUNDARY_TOL_M
        and right_clearance >= -PRODUCTION_BOUNDARY_TOL_M
        and minimum_forward > 0.0
    )


class ConstrainedCandidateEvaluator:
    """Share one spline construction between COBYQA objective and constraints."""

    def __init__(
        self,
        track,
        bike,
        stations,
        margin_m,
        boundary_check_spacing_m,
        optimisation_sample_spacing_m,
        speed_backend,
    ):
        self.track = track
        self.bike = bike
        self.stations = np.asarray(stations, dtype=float)
        self.margin_m = float(margin_m)
        self.optimisation_sample_spacing_m = float(optimisation_sample_spacing_m)
        self.solver = _speed_solver(speed_backend)

        self.guide_track = sample_track_stations(track, self.stations)
        check_count = max(4, math.ceil(track.total_length_m / boundary_check_spacing_m))
        self.check_s_m = np.arange(check_count, dtype=float) * track.total_length_m / check_count
        self.checked_track = sample_track_stations(track, self.check_s_m)

        self.objective_evaluations = 0
        self.constraint_calls = 0
        self.geometry_evaluations = 0
        self.invalid_objective_evaluations = 0
        self.best_controls = None
        self.best_lap_time_s = math.inf
        self.best_constraint_values = None

        self._cache_controls = None
        self._cache_spline = None
        self._cache_constraints = None

    def _geometry(self, controls):
        candidate = np.asarray(controls, dtype=float)
        if candidate.shape != self.stations.shape or not np.all(np.isfinite(candidate)):
            raise ValueError("candidate controls must be finite and match control stations")
        if self._cache_controls is not None and np.array_equal(candidate, self._cache_controls):
            return self._cache_spline, self._cache_constraints

        gx = self.guide_track.x_m + candidate * self.guide_track.normal_x
        gy = self.guide_track.y_m + candidate * self.guide_track.normal_y
        spline = PeriodicPlanarSpline(self.stations, gx, gy, self.track.total_length_m)
        px, py, dx, dy, *_ = spline.evaluate(self.check_s_m)
        delta_x = px - self.checked_track.x_m
        delta_y = py - self.checked_track.y_m
        projected = (
            delta_x * self.checked_track.normal_x
            + delta_y * self.checked_track.normal_y
        )
        left_clearance = self.checked_track.width_left_m - self.margin_m - projected
        right_clearance = self.checked_track.width_right_m - self.margin_m + projected
        forward = dx * self.checked_track.tangent_x + dy * self.checked_track.tangent_y
        constraints = np.array(
            [
                float(np.min(left_clearance)),
                float(np.min(right_clearance)),
                float(np.min(forward)),
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(constraints)):
            constraints = np.full(3, -1.0e6)

        self.geometry_evaluations += 1
        self._cache_controls = candidate.copy()
        self._cache_spline = spline
        self._cache_constraints = constraints
        return spline, constraints

    def constraint_values(self, controls):
        self.constraint_calls += 1
        try:
            _, values = self._geometry(controls)
        except (ValueError, RuntimeError, FloatingPointError):
            return np.full(3, -1.0e6)
        return values.copy()

    def objective(self, controls):
        self.objective_evaluations += 1
        try:
            spline, constraint_values = self._geometry(controls)
            path = spline.sampled_path(self.optimisation_sample_spacing_m)
            speed = self.solver(path, self.bike)
            lap = float(speed.lap_time_s)
        except (ValueError, RuntimeError, FloatingPointError):
            self.invalid_objective_evaluations += 1
            return INVALID_OBJECTIVE_S

        if production_feasible_constraint_values(constraint_values) and lap < self.best_lap_time_s:
            self.best_lap_time_s = lap
            self.best_controls = np.asarray(controls, dtype=float).copy()
            self.best_constraint_values = constraint_values.copy()
        return lap


def racing_line_artifact_paths(output_dir, margin_m):
    stem = f"margin_{float(margin_m):.3f}m_racing_line"
    output_dir = Path(output_dir)
    return output_dir / f"{stem}.csv", output_dir / f"{stem}.png"


def _sampled_track_s(track_length_m, sample_count):
    return np.arange(sample_count, dtype=float) * track_length_m / sample_count


def write_racing_line_csv(path, track, evaluation, margin_m):
    """Write a reusable sampled path/track artifact on the common evaluation grid."""
    smooth = evaluation.smooth_line
    sampled_path = smooth.sampled_path
    speed = evaluation.speed_profile
    count = len(sampled_path.q_m)
    track_s = _sampled_track_s(track.total_length_m, count)
    sampled_track = sample_track_stations(track, track_s)
    delta_x = sampled_path.x_m - sampled_track.x_m
    delta_y = sampled_path.y_m - sampled_track.y_m
    projected = delta_x * sampled_track.normal_x + delta_y * sampled_track.normal_y

    left_edge_x = sampled_track.x_m + sampled_track.width_left_m * sampled_track.normal_x
    left_edge_y = sampled_track.y_m + sampled_track.width_left_m * sampled_track.normal_y
    right_edge_x = sampled_track.x_m - sampled_track.width_right_m * sampled_track.normal_x
    right_edge_y = sampled_track.y_m - sampled_track.width_right_m * sampled_track.normal_y
    usable_left = sampled_track.width_left_m - margin_m
    usable_right = sampled_track.width_right_m - margin_m
    margin_left_x = sampled_track.x_m + usable_left * sampled_track.normal_x
    margin_left_y = sampled_track.y_m + usable_left * sampled_track.normal_y
    margin_right_x = sampled_track.x_m - usable_right * sampled_track.normal_x
    margin_right_y = sampled_track.y_m - usable_right * sampled_track.normal_y

    fields = (
        "track_s_m",
        "path_q_m",
        "x_m",
        "y_m",
        "projected_offset_m",
        "curvature_1pm",
        "speed_mps",
        "track_center_x_m",
        "track_center_y_m",
        "left_edge_x_m",
        "left_edge_y_m",
        "right_edge_x_m",
        "right_edge_y_m",
        "margin_left_x_m",
        "margin_left_y_m",
        "margin_right_x_m",
        "margin_right_y_m",
    )
    arrays = (
        track_s,
        sampled_path.q_m,
        sampled_path.x_m,
        sampled_path.y_m,
        projected,
        sampled_path.curvature_1pm,
        speed.speed_mps,
        sampled_track.x_m,
        sampled_track.y_m,
        left_edge_x,
        left_edge_y,
        right_edge_x,
        right_edge_y,
        margin_left_x,
        margin_left_y,
        margin_right_x,
        margin_right_y,
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(zip(*arrays))


def write_racing_line_png(path, track, evaluation, margin_m, *, dpi, show_control_points):
    """Write a high-resolution, thin-line development plot of one racing line."""
    smooth = evaluation.smooth_line
    check_s = smooth.evaluated_track_s_m
    checked_track = sample_track_stations(track, check_s)
    left_edge_x = checked_track.x_m + checked_track.width_left_m * checked_track.normal_x
    left_edge_y = checked_track.y_m + checked_track.width_left_m * checked_track.normal_y
    right_edge_x = checked_track.x_m - checked_track.width_right_m * checked_track.normal_x
    right_edge_y = checked_track.y_m - checked_track.width_right_m * checked_track.normal_y
    usable_left = checked_track.width_left_m - margin_m
    usable_right = checked_track.width_right_m - margin_m
    margin_left_x = checked_track.x_m + usable_left * checked_track.normal_x
    margin_left_y = checked_track.y_m + usable_left * checked_track.normal_y
    margin_right_x = checked_track.x_m - usable_right * checked_track.normal_x
    margin_right_y = checked_track.y_m - usable_right * checked_track.normal_y
    px, py, *_ = smooth.spline.evaluate(check_s)

    figure, axis = plt.subplots(figsize=(12, 9))
    axis.plot(left_edge_x, left_edge_y, linewidth=0.35, label="Left track edge")
    axis.plot(right_edge_x, right_edge_y, linewidth=0.35, label="Right track edge")
    axis.plot(margin_left_x, margin_left_y, linewidth=0.3, linestyle="--", label="Margin corridor")
    axis.plot(margin_right_x, margin_right_y, linewidth=0.3, linestyle="--")
    axis.plot(checked_track.x_m, checked_track.y_m, linewidth=0.25, linestyle=":", label="Centreline")
    axis.plot(px, py, linewidth=0.7, label="Derived racing line")
    if show_control_points:
        axis.scatter(
            smooth.guide_x_m,
            smooth.guide_y_m,
            s=7,
            marker="o",
            linewidths=0.3,
            label="Control points",
            zorder=5,
        )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(f"Mallala derived racing line - {margin_m:.3f} m edge margin")
    axis.legend(fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def _write_summary(path, rows):
    fields = (
        "margin_m",
        "start_common_grid_lap_s",
        "cobyqa_objective_lap_s",
        "final_common_grid_lap_s",
        "delta_final_minus_start_common_s",
        "delta_final_minus_reviewed_common_s",
        "minimum_usable_clearance_m",
        "minimum_track_edge_clearance_m",
        "objective_evaluations",
        "constraint_calls",
        "geometry_evaluations",
        "invalid_objective_evaluations",
        "nit",
        "success",
        "status",
        "message",
        "max_constraint_violation",
        "terminal_left_clearance_m",
        "terminal_right_clearance_m",
        "terminal_minimum_forward_progress",
        "elapsed_s",
        "maximum_abs_control_delta_from_start_m",
        "rms_control_delta_from_start_m",
        "output_controls_csv",
        "racing_line_csv",
        "racing_line_png",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.final_tr_radius_m >= args.initial_tr_radius_m:
        raise ValueError("final trust-region radius must be smaller than initial trust-region radius")
    phase9f._require_canonical_inputs()
    minimize, NonlinearConstraint = _load_scipy_tools()

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
        _evaluate(
            track,
            bike,
            stations,
            reviewed,
            args.common_spacing_m,
            0.0,
            args.boundary_check_spacing_m,
            "python",
        ),
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
    print("benchmark_method=scipy.optimize.minimize(method='COBYQA')")
    print("benchmark_note=nonlinear-constrained derivative-free diagnostic; not a production optimiser replacement")
    print("constraint_note=dense left/right corridor clearance and forward progress are explicit nonlinear constraints")
    print("optimisation_sample_spacing_m=1.000000")
    print(f"common_ranking_spacing_m={args.common_spacing_m:.6f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.6f}")
    print(f"max_evaluations_per_margin={args.max_evaluations}")
    print(f"max_iterations_per_margin={args.max_iterations}")
    print(f"initial_tr_radius_m={args.initial_tr_radius_m:.9f}")
    print(f"final_tr_radius_m={args.final_tr_radius_m:.9f}")
    print(f"feasibility_tol={args.feasibility_tol:.12g}")
    print(f"speed_backend={args.speed_backend}")
    print(f"plot_dpi={args.plot_dpi}")
    print(f"show_control_points={args.show_control_points}")
    print(f"reviewed_fixed_line_common_grid_lap_s={reviewed_lap:.9f}")

    rows = []
    for margin in margins:
        lower, upper = planar_control_bounds(track, stations, margin)
        start_csv = args.start_dir / phase11.margin_controls_filename(margin)
        start = phase8.load_initial_controls_csv(start_csv, stations, lower, upper)
        start_common = _require_feasible(
            _evaluate(
                track,
                bike,
                stations,
                start,
                args.common_spacing_m,
                margin,
                args.boundary_check_spacing_m,
                "python",
            ),
            f"margin {margin:.3f} m benchmark start on common grid",
        )

        evaluator = ConstrainedCandidateEvaluator(
            track,
            bike,
            stations,
            margin,
            args.boundary_check_spacing_m,
            1.0,
            args.speed_backend,
        )
        starting_constraints = evaluator.constraint_values(start)
        if not production_feasible_constraint_values(starting_constraints):
            raise RuntimeError(
                f"margin {margin:.3f} m benchmark start violates nonlinear constraints: "
                f"{starting_constraints.tolist()}")
        evaluator.objective(start)
        nonlinear_constraint = NonlinearConstraint(
            evaluator.constraint_values,
            lb=np.zeros(3),
            ub=np.full(3, np.inf),
        )

        started = time.perf_counter()
        result = minimize(
            evaluator.objective,
            start,
            method="COBYQA",
            bounds=list(zip(lower, upper)),
            constraints=(nonlinear_constraint,),
            options={
                "maxfev": args.max_evaluations,
                "maxiter": args.max_iterations,
                "feasibility_tol": args.feasibility_tol,
                "initial_tr_radius": args.initial_tr_radius_m,
                "final_tr_radius": args.final_tr_radius_m,
                "scale": False,
                "disp": False,
            },
        )
        elapsed = time.perf_counter() - started

        if evaluator.best_controls is None:
            raise RuntimeError(f"margin {margin:.3f} m COBYQA benchmark found no production-feasible candidate")
        best_controls = evaluator.best_controls
        final_common = _require_feasible(
            _evaluate(
                track,
                bike,
                stations,
                best_controls,
                args.common_spacing_m,
                margin,
                args.boundary_check_spacing_m,
                "python",
            ),
            f"margin {margin:.3f} m COBYQA best line on common grid",
        )
        output_controls = args.output_dir / phase11.margin_controls_filename(margin)
        phase8.atomic_write_controls_csv(output_controls, stations, best_controls, lower, upper)

        racing_csv, racing_png = racing_line_artifact_paths(args.output_dir, margin)
        write_racing_line_csv(racing_csv, track, final_common, margin)
        write_racing_line_png(
            racing_png,
            track,
            final_common,
            margin,
            dpi=args.plot_dpi,
            show_control_points=args.show_control_points,
        )

        terminal_constraints = evaluator.constraint_values(np.asarray(result.x, dtype=float))
        max_constraint_violation = float(max(0.0, -float(np.min(terminal_constraints))))
        delta = np.asarray(best_controls) - start
        usable_clearance = float(final_common.smooth_line.minimum_boundary_clearance_m)
        row = {
            "margin_m": margin,
            "start_common_grid_lap_s": float(start_common.lap_time_s),
            "cobyqa_objective_lap_s": float(evaluator.best_lap_time_s),
            "final_common_grid_lap_s": float(final_common.lap_time_s),
            "delta_final_minus_start_common_s": float(final_common.lap_time_s - start_common.lap_time_s),
            "delta_final_minus_reviewed_common_s": float(final_common.lap_time_s - reviewed_lap),
            "minimum_usable_clearance_m": usable_clearance,
            "minimum_track_edge_clearance_m": usable_clearance + margin,
            "objective_evaluations": int(evaluator.objective_evaluations),
            "constraint_calls": int(evaluator.constraint_calls),
            "geometry_evaluations": int(evaluator.geometry_evaluations),
            "invalid_objective_evaluations": int(evaluator.invalid_objective_evaluations),
            "nit": int(getattr(result, "nit", -1)),
            "success": bool(result.success),
            "status": int(result.status),
            "message": str(result.message),
            "max_constraint_violation": max_constraint_violation,
            "terminal_left_clearance_m": float(terminal_constraints[0]),
            "terminal_right_clearance_m": float(terminal_constraints[1]),
            "terminal_minimum_forward_progress": float(terminal_constraints[2]),
            "elapsed_s": float(elapsed),
            "maximum_abs_control_delta_from_start_m": float(np.max(np.abs(delta))),
            "rms_control_delta_from_start_m": float(np.sqrt(np.mean(delta ** 2))),
            "output_controls_csv": str(output_controls),
            "racing_line_csv": str(racing_csv),
            "racing_line_png": str(racing_png),
        }
        rows.append(row)
        print(
            f"margin_m={margin:.6f} start_controls_csv={start_csv} "
            f"start_common_lap_s={start_common.lap_time_s:.9f} "
            f"cobyqa_objective_lap_s={evaluator.best_lap_time_s:.9f} "
            f"final_common_grid_lap_s={final_common.lap_time_s:.9f} "
            f"delta_final_minus_start_common_s={row['delta_final_minus_start_common_s']:.9f} "
            f"minimum_track_edge_clearance_m={row['minimum_track_edge_clearance_m']:.9f} "
            f"objective_evaluations={evaluator.objective_evaluations} "
            f"constraint_calls={evaluator.constraint_calls} "
            f"geometry_evaluations={evaluator.geometry_evaluations} "
            f"invalid_objective_evaluations={evaluator.invalid_objective_evaluations} "
            f"nit={row['nit']} success={row['success']} status={row['status']} "
            f"max_constraint_violation={max_constraint_violation:.12g} "
            f"elapsed_s={elapsed:.3f} message={row['message']!r}")
        print(f"racing_line_csv={racing_csv}")
        print(f"racing_line_png={racing_png}")

    if len(rows) == 2:
        print(
            "final_common_margin_gap_s="
            f"{rows[1]['final_common_grid_lap_s'] - rows[0]['final_common_grid_lap_s']:.9f}")
    summary = args.output_dir / "phase11_constrained_benchmark_summary.csv"
    _write_summary(summary, rows)
    print(f"summary_csv={summary}")
    return rows


if __name__ == "__main__":
    main()
