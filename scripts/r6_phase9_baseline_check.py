"""Evaluate the recovered Phase 9 Mallala R6 reference controls without optimisation.

This command is deliberately a fixed-geometry reproduction check.  It validates
artifact identity, control-station policy and stored bounds before evaluating the
saved controls at fixed output spacings.  It does not claim that the recovered
historical 69.354897583 s run label is reproduced until the calculation says so.
"""

import argparse
import csv
import hashlib
import math
from pathlib import Path

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
EXPECTED_CONTROLS_SHA256 = "2290d07de682fa0ced7701d6cfb6f8459a9e0a96bfd662b0f37c931b8ea5d368"
HISTORICAL_REFERENCE_LABEL_LAP_S = 69.354897583
BOUNDARY_MARGIN_M = 0.25
BOUNDARY_CHECK_SPACING_M = 0.25
OUTPUT_SPACINGS_M = (1.0, 0.5, 0.25)


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controls", type=Path, default=DEFAULT_CONTROLS)
    parser.add_argument("--track", type=Path, default=DEFAULT_TRACK)
    parser.add_argument("--motorcycle", type=Path, default=DEFAULT_MOTORCYCLE)
    parser.add_argument("--speed-backend", choices=("python", "numba"), default="python")
    return parser


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
    if np.any((controls < lower) | (controls > upper)):
        index = int(np.flatnonzero((controls < lower) | (controls > upper))[0])
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


def main(argv=None):
    args = build_parser().parse_args(argv)
    controls_hash = sha256_file(args.controls)
    track_hash = sha256_file(args.track)
    motorcycle_hash = sha256_file(args.motorcycle)
    if args.controls == DEFAULT_CONTROLS and controls_hash != EXPECTED_CONTROLS_SHA256:
        raise RuntimeError(
            "frozen controls SHA-256 does not match the recovered Phase 9 artifact")

    track, stations, controls, evaluations = evaluate_baseline(
        args.controls, args.track, args.motorcycle, speed_backend=args.speed_backend)

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
                                          rel_tol=0.0, abs_tol=1e-9)
             else "not_reproduced"))
    print("baseline_note=the historical 69.354897583 s value is a recovered run label; only a matching fixed-geometry evaluation establishes it as the executable repository regression value")


if __name__ == "__main__":
    main()
