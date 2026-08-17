"""Evaluate Mallala Phase 11 resolution and boundary-margin sensitivity.

This diagnostic changes no optimiser algorithm and no motorcycle physics.  It
loads the reviewed Phase 9F 52-control roll-aware line and separates three
numerical/geometric questions that should not be conflated with optimiser
warm-start sensitivity:

1. fixed-spline output-grid resolution: rebuild the identical control-defined
   spline at several fixed-path sample spacings;
2. corridor-check resolution: keep the fixed-path output spacing and boundary
   margin fixed while changing only the dense geometric validation spacing;
3. boundary-margin sensitivity: keep the same physical controls and evaluate
   whether that same spline remains feasible as the usable-track margin changes.

Changing output spacing does not change guide points or the underlying spline.
Changing boundary-check spacing or boundary margin does not optimise, clip or
project the line.  An infeasible case is reported as infeasible.  This makes the
results suitable as bounded Phase 11 assurance evidence before deciding whether
a separate margin-aware re-optimisation experiment has an engineering payoff.
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
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


DEFAULT_MAX_ROLL_RATE_RADPS = 0.8
DEFAULT_OUTPUT_SPACINGS_M = (1.0, 0.5, 0.25, 0.125)
DEFAULT_BOUNDARY_CHECK_SPACINGS_M = (1.0, 0.5, 0.25, 0.125)
DEFAULT_BOUNDARY_MARGINS_M = (0.0, 0.125, 0.25, 0.5)
DEFAULT_COMMON_OUTPUT_SPACING_M = 0.25
DEFAULT_COMMON_BOUNDARY_CHECK_SPACING_M = 0.125


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase8 = _load_sibling(
    "r6_phase8_planar_optimisation_check.py", "r6_phase8_for_phase11_resolution_margin")
phase9 = _load_sibling(
    "r6_phase9_baseline_check.py", "r6_phase9_for_phase11_resolution_margin")
phase9f = _load_sibling(
    "r6_phase9f_roll_aware_optimisation.py", "r6_phase9f_for_phase11_resolution_margin")


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


def _unique_sorted(values):
    """Return deterministic ascending unique float values."""
    return tuple(sorted(set(float(value) for value in values)))


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "reviewed_reference_controls_csv", type=Path,
        help="reviewed Phase 9F 52-control roll-aware controls",
    )
    parser.add_argument(
        "--output-csv", type=Path,
        default=Path("phase11_resolution_margin_sensitivity.csv"),
    )
    parser.add_argument(
        "--max-roll-rate-radps", type=_positive_float,
        default=DEFAULT_MAX_ROLL_RATE_RADPS,
    )
    parser.add_argument(
        "--output-spacings-m", type=_positive_float, nargs="+",
        default=DEFAULT_OUTPUT_SPACINGS_M,
    )
    parser.add_argument(
        "--boundary-check-spacings-m", type=_positive_float, nargs="+",
        default=DEFAULT_BOUNDARY_CHECK_SPACINGS_M,
    )
    parser.add_argument(
        "--boundary-margins-m", type=_nonnegative_float, nargs="+",
        default=DEFAULT_BOUNDARY_MARGINS_M,
    )
    parser.add_argument(
        "--common-output-spacing-m", type=_positive_float,
        default=DEFAULT_COMMON_OUTPUT_SPACING_M,
    )
    parser.add_argument(
        "--common-boundary-check-spacing-m", type=_positive_float,
        default=DEFAULT_COMMON_BOUNDARY_CHECK_SPACING_M,
    )
    return parser


def build_case_matrix(args):
    """Build the three independent sensitivity sweeps in deterministic order."""
    output_spacings = _unique_sorted(args.output_spacings_m)
    check_spacings = _unique_sorted(args.boundary_check_spacings_m)
    margins = _unique_sorted(args.boundary_margins_m)

    rows = []
    for spacing in output_spacings:
        rows.append({
            "study": "fixed_spline_output_resolution",
            "sample_spacing_m": spacing,
            "boundary_check_spacing_m": phase9.BOUNDARY_CHECK_SPACING_M,
            "boundary_margin_m": phase9.BOUNDARY_MARGIN_M,
        })
    for spacing in check_spacings:
        rows.append({
            "study": "corridor_check_resolution",
            "sample_spacing_m": args.common_output_spacing_m,
            "boundary_check_spacing_m": spacing,
            "boundary_margin_m": phase9.BOUNDARY_MARGIN_M,
        })
    for margin in margins:
        rows.append({
            "study": "boundary_margin",
            "sample_spacing_m": args.common_output_spacing_m,
            "boundary_check_spacing_m": args.common_boundary_check_spacing_m,
            "boundary_margin_m": margin,
        })
    return rows


def evaluation_row(case, evaluation):
    """Convert one planar evaluation into a stable machine-readable row."""
    row = dict(case)
    row.update({
        "feasible": bool(evaluation.feasible),
        "lap_time_s": math.nan,
        "path_length_m": math.nan,
        "sample_count": 0,
        "minimum_boundary_clearance_m": math.nan,
        "minimum_edge_clearance_m": math.nan,
        "minimum_forward_progress": math.nan,
        "curvature_min_1pm": math.nan,
        "curvature_max_1pm": math.nan,
        "failure_reason": evaluation.failure_reason or "",
    })
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        return row

    path = evaluation.smooth_line.sampled_path
    curvature = np.asarray(path.curvature_1pm, dtype=float)
    clearance = float(evaluation.smooth_line.minimum_boundary_clearance_m)
    margin = float(case["boundary_margin_m"])
    row.update({
        "lap_time_s": float(evaluation.lap_time_s),
        "path_length_m": float(path.total_length_m),
        "sample_count": int(len(path.q_m)),
        "minimum_boundary_clearance_m": clearance,
        # Clearance to the physical track edge is the remaining usable-corridor
        # clearance plus the requested edge margin.  This is useful for checking
        # that a margin sweep is changing validity, not silently moving the path.
        "minimum_edge_clearance_m": clearance + margin,
        "minimum_forward_progress": float(evaluation.smooth_line.minimum_forward_progress),
        "curvature_min_1pm": float(np.min(curvature)),
        "curvature_max_1pm": float(np.max(curvature)),
        "failure_reason": "",
    })
    return row


def _write_csv(path, rows):
    fields = (
        "study", "sample_spacing_m", "boundary_check_spacing_m", "boundary_margin_m",
        "feasible", "lap_time_s", "path_length_m", "sample_count",
        "minimum_boundary_clearance_m", "minimum_edge_clearance_m",
        "minimum_forward_progress", "curvature_min_1pm", "curvature_max_1pm",
        "failure_reason",
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in fields})


def _print_row(row):
    prefix = (
        f"study={row['study']} sample_spacing_m={row['sample_spacing_m']:.6f} "
        f"boundary_check_spacing_m={row['boundary_check_spacing_m']:.6f} "
        f"boundary_margin_m={row['boundary_margin_m']:.6f}"
    )
    if not row["feasible"]:
        print(f"{prefix} feasible=false failure={row['failure_reason']!r}")
        return
    print(
        f"{prefix} feasible=true lap_s={row['lap_time_s']:.9f} "
        f"path_length_m={row['path_length_m']:.9f} samples={row['sample_count']} "
        f"minimum_boundary_clearance_m={row['minimum_boundary_clearance_m']:.9f} "
        f"minimum_edge_clearance_m={row['minimum_edge_clearance_m']:.9f}")


def main(argv=None):
    args = build_parser().parse_args(argv)
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        base_bike,
        handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps),
    )

    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    canonical_lower, canonical_upper = planar_control_bounds(
        track, stations, phase9.BOUNDARY_MARGIN_M)
    controls = phase8.load_initial_controls_csv(
        args.reviewed_reference_controls_csv,
        stations,
        canonical_lower,
        canonical_upper,
    )

    print(f"reviewed_reference_controls_csv={args.reviewed_reference_controls_csv}")
    print(
        "reviewed_reference_controls_sha256="
        f"{phase9.sha256_file(args.reviewed_reference_controls_csv)}")
    print(f"control_count={len(stations)}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print(
        "scenario_note=finite roll rate is a sensitivity scenario, not a calibrated "
        "R6/rider constant")
    print(
        "study_note=output spacing, boundary-check spacing and boundary margin are "
        "varied independently on the same physical control-defined line; no optimisation, "
        "clipping or projection is performed")

    rows = []
    for case in build_case_matrix(args):
        evaluation = evaluate_planar_racing_line(
            controls,
            track,
            bike,
            stations,
            sample_spacing_m=case["sample_spacing_m"],
            boundary_margin_m=case["boundary_margin_m"],
            boundary_check_spacing_m=case["boundary_check_spacing_m"],
            speed_backend="python",
        )
        row = evaluation_row(case, evaluation)
        rows.append(row)
        _print_row(row)

    _write_csv(args.output_csv, rows)
    print(f"output_csv={args.output_csv}")
    print(f"case_count={len(rows)}")

    canonical_rows = [
        row for row in rows
        if row["study"] == "fixed_spline_output_resolution"
        and math.isclose(row["sample_spacing_m"], phase9.BOUNDARY_CHECK_SPACING_M)
    ]
    if canonical_rows and canonical_rows[0]["feasible"]:
        print(f"canonical_0p25m_output_lap_s={canonical_rows[0]['lap_time_s']:.9f}")

    return rows


if __name__ == "__main__":
    main()
