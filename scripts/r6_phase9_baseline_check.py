"""Evaluate and verify the frozen Phase 9 Mallala R6 reference geometry.

The command loads the recovered 52-control artifact without optimisation,
validates its identity/stations/bounds, evaluates the fixed geometry at the
frozen output spacings, and fails closed if the canonical baseline no longer
matches the executable regression values established on 16 August 2026.

Baseline text identities are SHA-256 hashes of UTF-8 content after newline
normalisation to LF. This makes the identity check independent of Git working-
tree CRLF conversion while remaining sensitive to every other text change.
"""

import argparse
import csv
import hashlib
import math
from pathlib import Path
import platform

import numpy as np

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track


DEFAULT_CONTROLS = Path("cases/mallala_r6/baseline/phase8_reference_controls.csv")
DEFAULT_TRACK = Path("examples/tracks/mallala_reference.yaml")
DEFAULT_MOTORCYCLE = Path("examples/motorcycles/r6_2017plus_reference.yaml")
EXPECTED_CONTROL_COUNT = 52
EXPECTED_CONTROLS_SHA256 = "5727dd1326c7892682f1d7dc1b78a67cede5c7b1c769577a6d26ae9ad564bf83"
EXPECTED_TRACK_SHA256 = "a213f9f15a3797ddb73f4a2a5969f0a1afa8b7dfccc4c057dd0c1e14e4e67959"
EXPECTED_MOTORCYCLE_SHA256 = "8c3ed9d3ac13b483dd441e6d9b500ada573cd4e5679581c614768117e5f63aee"
HISTORICAL_REFERENCE_LABEL_LAP_S = 69.354897583
BOUNDARY_MARGIN_M = 0.25
BOUNDARY_CHECK_SPACING_M = 0.25
OUTPUT_SPACINGS_M = (1.0, 0.5, 0.25)
EXPECTED_LAP_TIMES_S = (69.354897583, 69.321493766, 69.305349182)
EXPECTED_PATH_LENGTH_M = 2510.660863823
EXPECTED_MINIMUM_BOUNDARY_CLEARANCE_M = 0.000014708
EXPECTED_PROJECTED_OFFSET_MIN_M = -3.749985292
EXPECTED_PROJECTED_OFFSET_MAX_M = 4.749689886
EXPECTED_CURVATURE_MIN_1PM = (-0.101362936, -0.101793650, -0.101472758)
EXPECTED_CURVATURE_MAX_1PM = (0.027468564, 0.027609254, 0.027597204)

# Tight numerical-regression tolerances. Hash checks separately require exact
# canonical text identity of the controls, track and motorcycle inputs.
LAP_TIME_ATOL_S = 1e-6
PATH_LENGTH_ATOL_M = 1e-6
CLEARANCE_ATOL_M = 1e-7
OFFSET_ATOL_M = 1e-6
CURVATURE_ATOL_1PM = 1e-6


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controls", type=Path, default=DEFAULT_CONTROLS)
    parser.add_argument("--track", type=Path, default=DEFAULT_TRACK)
    parser.add_argument("--motorcycle", type=Path, default=DEFAULT_MOTORCYCLE)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    return parser


def canonical_text_sha256(path):
    """Hash UTF-8 text after normalising CRLF and CR newlines to LF.

    Git may materialise committed LF text as CRLF in a Windows working tree
    when ``core.autocrlf=true``. Baseline identity is therefore defined over
    canonical text rather than platform-specific working-tree bytes.
    """
    text = Path(path).read_bytes().decode("utf-8")
    canonical = text.replace("\r\n", "\n").replace("\r", "\n")
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


# Backward-compatible name used by sibling diagnostics that predate the
# cross-platform canonical-text identity change. It deliberately retains the
# canonical-text semantics; it is not a raw working-tree byte hash.
sha256_file = canonical_text_sha256


def load_frozen_controls(path, stations, lower_bounds, upper_bounds):
    required = ("index", "control_s_m", "best_offset_m", "lower_bound_m", "upper_bound_m")
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != required:
            raise ValueError(
                "frozen controls CSV schema must be exactly: " + ", ".join(required))
        rows = list(reader)

    stations = np.asarray(stations, dtype=float)
    lower = np.asarray(lower_bounds, dtype=float)
    upper = np.asarray(upper_bounds, dtype=float)
    if len(rows) != EXPECTED_CONTROL_COUNT or len(rows) != len(stations):
        raise ValueError(
            f"frozen controls must contain exactly {EXPECTED_CONTROL_COUNT} current-policy rows")

    parsed = np.empty((len(rows), 4), dtype=float)
    for expected_index, row in enumerate(rows):
        try:
            index = int(row["index"])
            if str(index) != row["index"].strip() or index != expected_index:
                raise ValueError
            parsed[expected_index] = [float(row[name]) for name in required[1:]]
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"frozen controls row {expected_index + 2} is not a valid sequential numeric row") from exc
        if not np.all(np.isfinite(parsed[expected_index])):
            raise ValueError(f"frozen controls row {expected_index + 2} contains non-finite values")

    saved_stations, controls, saved_lower, saved_upper = parsed.T
    tolerance = dict(rtol=0.0, atol=1e-9)
    if not np.allclose(saved_stations, stations, **tolerance):
        raise ValueError("frozen control stations do not match the current reference policy")
    if not np.allclose(saved_lower, lower, **tolerance):
        raise ValueError("frozen lower bounds do not match the current track/margin inputs")
    if not np.allclose(saved_upper, upper, **tolerance):
        raise ValueError("frozen upper bounds do not match the current track/margin inputs")
    outside = (controls < lower) | (controls > upper)
    if np.any(outside):
        index = int(np.flatnonzero(outside)[0])
        raise ValueError(f"frozen control {index} is outside its current stored bounds")
    return controls


def evaluate_baseline(controls_path=DEFAULT_CONTROLS, track_path=DEFAULT_TRACK,
                      motorcycle_path=DEFAULT_MOTORCYCLE, *, speed_backend="python"):
    track = Track.from_yaml(track_path)
    motorcycle = load_motorcycle_config(motorcycle_path)
    stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(track, stations, BOUNDARY_MARGIN_M)
    controls = load_frozen_controls(controls_path, stations, lower, upper)

    evaluations = []
    for spacing in OUTPUT_SPACINGS_M:
        evaluation = evaluate_planar_racing_line(
            controls, track, motorcycle, stations,
            sample_spacing_m=spacing,
            boundary_margin_m=BOUNDARY_MARGIN_M,
            boundary_check_spacing_m=BOUNDARY_CHECK_SPACING_M,
            speed_backend=speed_backend,
        )
        if not evaluation.feasible or evaluation.smooth_line is None:
            raise RuntimeError(
                f"frozen baseline is infeasible at {spacing:.2f} m: {evaluation.failure_reason}")
        evaluations.append(evaluation)
    return track, stations, controls, tuple(evaluations)


def _require_close(label, actual, expected, atol):
    if not math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=atol):
        raise RuntimeError(
            f"Phase 9 baseline regression: {label}={actual:.12g} differs from "
            f"expected {expected:.12g} by more than {atol:.12g}")


def verify_default_regression(controls_hash, track_hash, motorcycle_hash, evaluations):
    """Fail closed if the canonical Python baseline differs from the frozen package."""
    identities = (
        ("controls", controls_hash, EXPECTED_CONTROLS_SHA256),
        ("track", track_hash, EXPECTED_TRACK_SHA256),
        ("motorcycle", motorcycle_hash, EXPECTED_MOTORCYCLE_SHA256),
    )
    for label, actual, expected in identities:
        if actual != expected:
            raise RuntimeError(
                f"Phase 9 baseline regression: {label} canonical text SHA-256 "
                f"{actual} does not match {expected}")

    if len(evaluations) != len(OUTPUT_SPACINGS_M):
        raise RuntimeError("Phase 9 baseline regression: output-spacing evaluation count changed")

    for spacing, evaluation, expected_lap, expected_kmin, expected_kmax in zip(
            OUTPUT_SPACINGS_M, evaluations, EXPECTED_LAP_TIMES_S,
            EXPECTED_CURVATURE_MIN_1PM, EXPECTED_CURVATURE_MAX_1PM):
        smooth = evaluation.smooth_line
        path = smooth.sampled_path
        _require_close(f"lap time at {spacing:.2f} m [s]", evaluation.lap_time_s,
                       expected_lap, LAP_TIME_ATOL_S)
        _require_close(f"path length at {spacing:.2f} m [m]", path.total_length_m,
                       EXPECTED_PATH_LENGTH_M, PATH_LENGTH_ATOL_M)
        _require_close(f"minimum clearance at {spacing:.2f} m [m]",
                       smooth.minimum_boundary_clearance_m,
                       EXPECTED_MINIMUM_BOUNDARY_CLEARANCE_M, CLEARANCE_ATOL_M)
        _require_close(f"projected offset minimum at {spacing:.2f} m [m]",
                       np.min(smooth.projected_offset_m), EXPECTED_PROJECTED_OFFSET_MIN_M,
                       OFFSET_ATOL_M)
        _require_close(f"projected offset maximum at {spacing:.2f} m [m]",
                       np.max(smooth.projected_offset_m), EXPECTED_PROJECTED_OFFSET_MAX_M,
                       OFFSET_ATOL_M)
        _require_close(f"curvature minimum at {spacing:.2f} m [1/m]",
                       np.min(path.curvature_1pm), expected_kmin, CURVATURE_ATOL_1PM)
        _require_close(f"curvature maximum at {spacing:.2f} m [1/m]",
                       np.max(path.curvature_1pm), expected_kmax, CURVATURE_ATOL_1PM)


def main(argv=None):
    args = build_parser().parse_args(argv)
    controls_hash = canonical_text_sha256(args.controls)
    track_hash = canonical_text_sha256(args.track)
    motorcycle_hash = canonical_text_sha256(args.motorcycle)

    track, stations, controls, evaluations = evaluate_baseline(
        args.controls, args.track, args.motorcycle, speed_backend=args.speed_backend)

    print(f"python_version={platform.python_version()}")
    print(f"numpy_version={np.__version__}")
    print("input_identity_hash_method=sha256_utf8_text_normalized_to_lf")
    print(f"controls_path={args.controls}")
    print(f"controls_sha256={controls_hash}")
    print(f"track_path={args.track}")
    print(f"track_sha256={track_hash}")
    print(f"motorcycle_path={args.motorcycle}")
    print(f"motorcycle_sha256={motorcycle_hash}")
    print(f"reference_policy_max_spacing_m={REFERENCE_PLANAR_CONTROL_POLICY.max_spacing_m:.9f}")
    print("reference_policy_max_arc_heading_change_deg="
          f"{math.degrees(REFERENCE_PLANAR_CONTROL_POLICY.max_arc_heading_change_rad):.9f}")
    print(f"control_count={len(stations)}")
    print(f"control_min_m={np.min(controls):.9f}")
    print(f"control_max_m={np.max(controls):.9f}")
    print(f"track_length_m={track.total_length_m:.9f}")

    for spacing, evaluation in zip(OUTPUT_SPACINGS_M, evaluations):
        smooth = evaluation.smooth_line
        path = smooth.sampled_path
        print(
            f"spacing_m={spacing:.2f} lap_s={evaluation.lap_time_s:.9f} "
            f"path_length_m={path.total_length_m:.9f} "
            f"minimum_boundary_clearance_m={smooth.minimum_boundary_clearance_m:.9f} "
            f"projected_offset_min_m={np.min(smooth.projected_offset_m):.9f} "
            f"projected_offset_max_m={np.max(smooth.projected_offset_m):.9f} "
            f"curvature_min_1pm={np.min(path.curvature_1pm):.9f} "
            f"curvature_max_1pm={np.max(path.curvature_1pm):.9f}")

    one_metre_lap = evaluations[0].lap_time_s
    delta = one_metre_lap - HISTORICAL_REFERENCE_LABEL_LAP_S
    print(f"historical_reference_label_lap_s={HISTORICAL_REFERENCE_LABEL_LAP_S:.9f}")
    print(f"one_metre_minus_historical_label_s={delta:.9f}")
    print("historical_reference_label_status="
          + ("reproduced" if math.isclose(one_metre_lap, HISTORICAL_REFERENCE_LABEL_LAP_S,
                                          rel_tol=0.0, abs_tol=LAP_TIME_ATOL_S)
             else "not_reproduced"))

    canonical = (args.controls == DEFAULT_CONTROLS and args.track == DEFAULT_TRACK
                 and args.motorcycle == DEFAULT_MOTORCYCLE and args.speed_backend == "python")
    if canonical:
        verify_default_regression(controls_hash, track_hash, motorcycle_hash, evaluations)
        print("executable_baseline_regression_status=passed")
    else:
        print("executable_baseline_regression_status=not_checked_noncanonical_inputs")

    print("baseline_note=the recovered historical run label is independently reproduced by the canonical fixed-geometry repository evaluation and is frozen as an executable regression value")


if __name__ == "__main__":
    main()