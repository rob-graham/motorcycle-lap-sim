"""Build a representative Phase 11 racing line and optimiser-spread envelope.

This diagnostic does not optimise a new path. It re-evaluates the retained
Mallala Phase 11 candidate lines on one authoritative Python common grid,
quantifies their geometric spread at identical analytic track stations, and
selects a representative from explicitly eligible, optimisation-assured
candidates. The envelope is optimiser/basis spread, not physical or statistical
uncertainty.
"""

import argparse
import csv
from dataclasses import dataclass, replace
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
from motorcycle_lap_sim.track import Track, sample_track_stations


DEFAULT_DELETE_INDEX = 26
DEFAULT_RELOCATE_INDEX = 27
DEFAULT_RELOCATE_SHIFT_M = 5.0
DEFAULT_MINIMUM_STATION_GAP_M = 5.0
DEFAULT_MARGIN_M = 0.25
DEFAULT_MAX_ROLL_RATE_RADPS = 0.8
DEFAULT_COMMON_SPACING_M = 0.125
DEFAULT_BOUNDARY_CHECK_SPACING_M = 0.125
DEFAULT_REPRESENTATIVE_MAX_LAP_DELTA_S = 0.05
DEFAULT_PLOT_DPI = 400


@dataclass(frozen=True)
class CandidateSpec:
    label: str
    basis_kind: str
    controls_csv: Path
    controls_sha256: str
    stations_m: np.ndarray
    controls_m: np.ndarray
    representative_eligible: bool
    source_note: str


@dataclass(frozen=True)
class CandidateResult:
    spec: CandidateSpec
    evaluation: object
    x_m: np.ndarray
    y_m: np.ndarray
    projected_offset_m: np.ndarray
    minimum_forward_progress: float


def _load_sibling(filename, module_name):
    path = Path(__file__).resolve().with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load sibling script from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def _nonnegative_int(text):
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return value


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_controls_csv", type=Path)
    parser.add_argument("reduced_controls_csv", type=Path)
    parser.add_argument("relocated_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--delete-index", type=_nonnegative_int, default=DEFAULT_DELETE_INDEX)
    parser.add_argument("--relocate-index", type=_nonnegative_int, default=DEFAULT_RELOCATE_INDEX)
    parser.add_argument("--relocate-shift-m", type=float, default=DEFAULT_RELOCATE_SHIFT_M)
    parser.add_argument("--minimum-station-gap-m", type=_positive_float,
                        default=DEFAULT_MINIMUM_STATION_GAP_M)
    parser.add_argument("--margin-m", type=_nonnegative_float, default=DEFAULT_MARGIN_M)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float,
                        default=DEFAULT_MAX_ROLL_RATE_RADPS)
    parser.add_argument("--common-spacing-m", type=_positive_float,
                        default=DEFAULT_COMMON_SPACING_M)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float,
                        default=DEFAULT_BOUNDARY_CHECK_SPACING_M)
    parser.add_argument(
        "--representative-max-lap-delta-s", type=_nonnegative_float,
        default=DEFAULT_REPRESENTATIVE_MAX_LAP_DELTA_S,
        help=("maximum common-grid lap-time penalty allowed for the eligible geometric medoid; "
              "if exceeded, use the fastest eligible candidate"),
    )
    parser.add_argument("--plot-dpi", type=int, default=DEFAULT_PLOT_DPI)
    return parser


def _common_track_stations(track_length_m, spacing_m):
    count = max(4, math.ceil(track_length_m / spacing_m))
    return np.arange(count, dtype=float) * track_length_m / count


def optimiser_spread_envelope(offsets_by_label, labels):
    """Return min, median, max and width of lateral optimiser spread."""
    if len(labels) < 2:
        raise ValueError("at least two candidate labels are required for a spread envelope")
    arrays = [np.asarray(offsets_by_label[label], dtype=float) for label in labels]
    shape = arrays[0].shape
    if any(array.shape != shape for array in arrays) or len(shape) != 1:
        raise ValueError("candidate offset arrays must have one identical one-dimensional shape")
    if not all(np.all(np.isfinite(array)) for array in arrays):
        raise ValueError("candidate offset arrays must be finite")
    stacked = np.vstack(arrays)
    minimum = np.min(stacked, axis=0)
    median = np.median(stacked, axis=0)
    maximum = np.max(stacked, axis=0)
    return minimum, median, maximum, maximum - minimum


def pairwise_geometry(points_by_label, labels):
    """Return pairwise maximum and RMS Euclidean displacement matrices."""
    if len(labels) < 2:
        raise ValueError("at least two candidate labels are required")
    points = [np.asarray(points_by_label[label], dtype=float) for label in labels]
    shape = points[0].shape
    if len(shape) != 2 or shape[1] != 2:
        raise ValueError("candidate point arrays must have shape (N, 2)")
    if any(point.shape != shape for point in points):
        raise ValueError("candidate point arrays must have identical shapes")
    if not all(np.all(np.isfinite(point)) for point in points):
        raise ValueError("candidate point arrays must be finite")

    count = len(labels)
    maximum = np.zeros((count, count), dtype=float)
    rms = np.zeros((count, count), dtype=float)
    for i in range(count):
        for j in range(i + 1, count):
            displacement = np.linalg.norm(points[i] - points[j], axis=1)
            maximum[i, j] = maximum[j, i] = float(np.max(displacement))
            rms[i, j] = rms[j, i] = float(np.sqrt(np.mean(displacement ** 2)))
    return maximum, rms


def select_representative_candidate(
        labels, rms_matrix, lap_times_s, representative_eligible, max_lap_delta_s):
    """Select an optimisation-assured representative with a lap-time guardrail.

    The geometric medoid is calculated only among candidates explicitly marked
    representative-eligible, so sensitivity-only perturbations cannot influence
    representative centrality. If that eligible medoid is slower than the
    fastest eligible candidate by more than ``max_lap_delta_s``, selection falls
    back to the fastest eligible candidate. No synthetic median path is created.
    """
    if len(labels) < 2:
        raise ValueError("at least two candidate labels are required")
    if not math.isfinite(max_lap_delta_s) or max_lap_delta_s < 0.0:
        raise ValueError("representative maximum lap delta must be finite and non-negative")
    matrix = np.asarray(rms_matrix, dtype=float)
    if matrix.shape != (len(labels), len(labels)) or not np.all(np.isfinite(matrix)):
        raise ValueError("RMS matrix must be finite and square for the candidate labels")
    if not np.allclose(matrix, matrix.T, rtol=0.0, atol=1e-12):
        raise ValueError("RMS matrix must be symmetric")
    if not np.allclose(np.diag(matrix), 0.0, rtol=0.0, atol=1e-12):
        raise ValueError("RMS matrix diagonal must be zero")
    if set(representative_eligible) != set(labels):
        raise ValueError("representative eligibility must be supplied for every candidate label")
    if set(lap_times_s) != set(labels):
        raise ValueError("lap times must be supplied for every candidate label")

    eligible_indices = [
        index for index, label in enumerate(labels) if bool(representative_eligible[label])]
    if not eligible_indices:
        raise ValueError("at least one representative-eligible candidate is required")

    eligible_labels = [labels[index] for index in eligible_indices]
    fastest_eligible_label = min(
        eligible_labels, key=lambda label: (float(lap_times_s[label]), label))

    eligible_means = {label: math.nan for label in labels}
    if len(eligible_indices) == 1:
        medoid_label = eligible_labels[0]
        eligible_means[medoid_label] = 0.0
    else:
        for index in eligible_indices:
            peer_distances = [
                matrix[index, peer] for peer in eligible_indices if peer != index]
            eligible_means[labels[index]] = float(np.mean(peer_distances))
        medoid_label = min(
            eligible_labels,
            key=lambda label: (
                eligible_means[label], float(lap_times_s[label]), label),
        )

    medoid_lap_delta_s = float(
        lap_times_s[medoid_label] - lap_times_s[fastest_eligible_label])
    if medoid_lap_delta_s <= max_lap_delta_s + 1e-12:
        representative_label = medoid_label
        selection_reason = "eligible_geometric_medoid_within_lap_delta"
    else:
        representative_label = fastest_eligible_label
        selection_reason = "fastest_eligible_fallback_medoid_exceeds_lap_delta"

    return (
        representative_label,
        medoid_label,
        fastest_eligible_label,
        eligible_means,
        medoid_lap_delta_s,
        selection_reason,
    )


def _require_feasible(evaluation, label):
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"{label} is infeasible: {evaluation.failure_reason}")
    return evaluation


def _minimum_forward(track, evaluation):
    checked_s = evaluation.smooth_line.evaluated_track_s_m
    checked_track = sample_track_stations(track, checked_s)
    _, _, dx, dy, *_ = evaluation.smooth_line.spline.evaluate(checked_s)
    return float(np.min(dx * checked_track.tangent_x + dy * checked_track.tangent_y))


def _start_finish_segment(track):
    sampled = sample_track_stations(track, np.array([0.0]))
    cx = float(sampled.x_m[0])
    cy = float(sampled.y_m[0])
    nx = float(sampled.normal_x[0])
    ny = float(sampled.normal_y[0])
    return (
        (cx + float(sampled.width_left_m[0]) * nx,
         cy + float(sampled.width_left_m[0]) * ny),
        (cx - float(sampled.width_right_m[0]) * nx,
         cy - float(sampled.width_right_m[0]) * ny),
    )


def _evaluate_candidate(spec, track, bike, track_s, sampled_track,
                        common_spacing_m, margin_m, boundary_check_spacing_m):
    evaluation = _require_feasible(
        evaluate_planar_racing_line(
            spec.controls_m,
            track,
            bike,
            spec.stations_m,
            sample_spacing_m=common_spacing_m,
            boundary_margin_m=margin_m,
            boundary_check_spacing_m=boundary_check_spacing_m,
            speed_backend="python",
        ),
        spec.label,
    )
    x_m, y_m, *_ = evaluation.smooth_line.spline.evaluate(track_s)
    projected = ((x_m - sampled_track.x_m) * sampled_track.normal_x
                 + (y_m - sampled_track.y_m) * sampled_track.normal_y)
    return CandidateResult(
        spec=spec,
        evaluation=evaluation,
        x_m=np.asarray(x_m, dtype=float),
        y_m=np.asarray(y_m, dtype=float),
        projected_offset_m=np.asarray(projected, dtype=float),
        minimum_forward_progress=_minimum_forward(track, evaluation),
    )


def _write_candidate_summary(
        path, results, labels, fastest_label, fastest_eligible_label,
        representative_label, medoid_label, eligible_means, maximum_matrix, rms_matrix,
        margin_m, max_lap_delta_s):
    fastest_lap = float(results[fastest_label].evaluation.lap_time_s)
    fastest_eligible_lap = float(results[fastest_eligible_label].evaluation.lap_time_s)
    representative_index = labels.index(representative_label)
    fields = (
        "label", "basis_kind", "controls_csv", "controls_sha256", "control_count",
        "common_grid_lap_s", "delta_from_fastest_s", "delta_from_fastest_eligible_s",
        "representative_eligible", "within_representative_lap_acceptance",
        "eligible_mean_pairwise_rms_displacement_m", "is_eligible_geometric_medoid",
        "minimum_usable_clearance_m", "minimum_track_edge_clearance_m",
        "minimum_forward_progress", "maximum_displacement_from_representative_m",
        "rms_displacement_from_representative_m", "is_fastest", "is_fastest_eligible",
        "is_representative", "source_note",
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, label in enumerate(labels):
            result = results[label]
            usable = float(result.evaluation.smooth_line.minimum_boundary_clearance_m)
            eligible = bool(result.spec.representative_eligible)
            eligible_delta = float(result.evaluation.lap_time_s - fastest_eligible_lap)
            eligible_mean = eligible_means[label]
            writer.writerow({
                "label": label,
                "basis_kind": result.spec.basis_kind,
                "controls_csv": str(result.spec.controls_csv),
                "controls_sha256": result.spec.controls_sha256,
                "control_count": len(result.spec.controls_m),
                "common_grid_lap_s": float(result.evaluation.lap_time_s),
                "delta_from_fastest_s": float(result.evaluation.lap_time_s - fastest_lap),
                "delta_from_fastest_eligible_s": eligible_delta if eligible else "",
                "representative_eligible": eligible,
                "within_representative_lap_acceptance": (
                    eligible and eligible_delta <= max_lap_delta_s + 1e-12),
                "eligible_mean_pairwise_rms_displacement_m": (
                    "" if math.isnan(eligible_mean) else eligible_mean),
                "is_eligible_geometric_medoid": label == medoid_label,
                "minimum_usable_clearance_m": usable,
                "minimum_track_edge_clearance_m": usable + margin_m,
                "minimum_forward_progress": result.minimum_forward_progress,
                "maximum_displacement_from_representative_m": float(
                    maximum_matrix[index, representative_index]),
                "rms_displacement_from_representative_m": float(
                    rms_matrix[index, representative_index]),
                "is_fastest": label == fastest_label,
                "is_fastest_eligible": label == fastest_eligible_label,
                "is_representative": label == representative_label,
                "source_note": result.spec.source_note,
            })


def _write_pairwise_csv(path, labels, maximum_matrix, rms_matrix):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("label_a", "label_b", "maximum_displacement_m", "rms_displacement_m"))
        for i, label_a in enumerate(labels):
            for j in range(i + 1, len(labels)):
                writer.writerow((label_a, labels[j], maximum_matrix[i, j], rms_matrix[i, j]))


def _write_envelope_csv(path, track, track_s, sampled_track, results, labels,
                        representative_label, fastest_label, minimum, median, maximum, spread,
                        margin_m):
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
    envelope_min_x = sampled_track.x_m + minimum * sampled_track.normal_x
    envelope_min_y = sampled_track.y_m + minimum * sampled_track.normal_y
    envelope_max_x = sampled_track.x_m + maximum * sampled_track.normal_x
    envelope_max_y = sampled_track.y_m + maximum * sampled_track.normal_y

    representative = results[representative_label]
    fastest = results[fastest_label]
    fixed_fields = [
        "track_s_m", "track_center_x_m", "track_center_y_m",
        "left_edge_x_m", "left_edge_y_m", "right_edge_x_m", "right_edge_y_m",
        "margin_left_x_m", "margin_left_y_m", "margin_right_x_m", "margin_right_y_m",
        "envelope_min_offset_m", "envelope_median_offset_m", "envelope_max_offset_m",
        "envelope_width_m", "envelope_min_x_m", "envelope_min_y_m",
        "envelope_max_x_m", "envelope_max_y_m",
        "representative_x_m", "representative_y_m", "representative_offset_m",
        "fastest_x_m", "fastest_y_m", "fastest_offset_m",
    ]
    candidate_fields = [f"candidate_{label}_offset_m" for label in labels]
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fixed_fields + candidate_fields)
        for index in range(len(track_s)):
            row = [
                track_s[index], sampled_track.x_m[index], sampled_track.y_m[index],
                left_edge_x[index], left_edge_y[index], right_edge_x[index], right_edge_y[index],
                margin_left_x[index], margin_left_y[index], margin_right_x[index], margin_right_y[index],
                minimum[index], median[index], maximum[index], spread[index],
                envelope_min_x[index], envelope_min_y[index], envelope_max_x[index], envelope_max_y[index],
                representative.x_m[index], representative.y_m[index],
                representative.projected_offset_m[index], fastest.x_m[index], fastest.y_m[index],
                fastest.projected_offset_m[index],
            ]
            row.extend(results[label].projected_offset_m[index] for label in labels)
            writer.writerow(row)


def _write_reference_line_csv(path, track, result, margin_m):
    smooth = result.evaluation.smooth_line
    sampled_path = smooth.sampled_path
    speed = result.evaluation.speed_profile
    count = len(sampled_path.q_m)
    track_s = np.arange(count, dtype=float) * track.total_length_m / count
    sampled_track = sample_track_stations(track, track_s)
    projected = ((sampled_path.x_m - sampled_track.x_m) * sampled_track.normal_x
                 + (sampled_path.y_m - sampled_track.y_m) * sampled_track.normal_y)
    fields = (
        "track_s_m", "path_q_m", "x_m", "y_m", "projected_offset_m",
        "curvature_1pm", "speed_mps", "track_center_x_m", "track_center_y_m",
    )
    arrays = (
        track_s, sampled_path.q_m, sampled_path.x_m, sampled_path.y_m, projected,
        sampled_path.curvature_1pm, speed.speed_mps, sampled_track.x_m, sampled_track.y_m,
    )
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(zip(*arrays))


def _write_png(path, track, sampled_track, results, labels, representative_label, fastest_label,
               minimum, maximum, margin_m, dpi):
    import matplotlib.pyplot as plt

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
    envelope_min_x = sampled_track.x_m + minimum * sampled_track.normal_x
    envelope_min_y = sampled_track.y_m + minimum * sampled_track.normal_y
    envelope_max_x = sampled_track.x_m + maximum * sampled_track.normal_x
    envelope_max_y = sampled_track.y_m + maximum * sampled_track.normal_y
    sf_left, sf_right = _start_finish_segment(track)

    figure, axis = plt.subplots(figsize=(12, 9))
    axis.plot(left_edge_x, left_edge_y, linewidth=0.35, label="Left track edge")
    axis.plot(right_edge_x, right_edge_y, linewidth=0.35, label="Right track edge")
    axis.plot(margin_left_x, margin_left_y, linewidth=0.3, linestyle="--", label="Margin corridor")
    axis.plot(margin_right_x, margin_right_y, linewidth=0.3, linestyle="--")
    axis.plot(sampled_track.x_m, sampled_track.y_m, linewidth=0.18, linestyle=":",
              label="Centreline")
    axis.plot(envelope_min_x, envelope_min_y, linewidth=0.45, linestyle="-.",
              label="Optimiser-spread envelope")
    axis.plot(envelope_max_x, envelope_max_y, linewidth=0.45, linestyle="-.")

    for label in labels:
        if label in (representative_label, fastest_label):
            continue
        axis.plot(results[label].x_m, results[label].y_m, linewidth=0.35,
                  label=f"Candidate: {label}")

    representative = results[representative_label]
    axis.plot(representative.x_m, representative.y_m, linewidth=0.9,
              label=f"Representative: {representative_label}")
    if fastest_label != representative_label:
        fastest = results[fastest_label]
        axis.plot(fastest.x_m, fastest.y_m, linewidth=0.7, linestyle="--",
                  label=f"Fastest: {fastest_label}")
    axis.plot([sf_left[0], sf_right[0]], [sf_left[1], sf_right[1]],
              linewidth=1.0, label="Start / finish", zorder=7)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(
        f"Mallala Phase 11 representative line and optimiser spread - {margin_m:.3f} m margin")
    axis.legend(fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.plot_dpi <= 0:
        raise ValueError("plot DPI must be positive")
    if not math.isfinite(args.relocate_shift_m) or args.relocate_shift_m == 0.0:
        raise ValueError("relocation shift must be finite and non-zero")

    phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_robust_line")
    phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_robust_line")
    phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_robust_line")
    deletion = _load_sibling("r6_phase11_control_deletion_screen.py", "phase11_deletion_robust_line")
    relocation = _load_sibling(
        "r6_phase11_control_station_relocation_screen.py", "phase11_relocation_robust_line")
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    standard_stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    standard_lower, standard_upper = planar_control_bounds(track, standard_stations, args.margin_m)
    baseline_controls = phase8.load_initial_controls_csv(
        args.baseline_controls_csv, standard_stations, standard_lower, standard_upper)

    reduced_stations, _ = deletion.deletion_arrays(
        standard_stations, baseline_controls, args.delete_index)
    reduced_lower, reduced_upper = planar_control_bounds(track, reduced_stations, args.margin_m)
    reduced_controls = phase8.load_initial_controls_csv(
        args.reduced_controls_csv, reduced_stations, reduced_lower, reduced_upper)

    relocated_stations = relocation.relocated_stations(
        standard_stations,
        args.relocate_index,
        args.relocate_shift_m,
        track.total_length_m,
        args.minimum_station_gap_m,
    )
    if relocated_stations is None:
        raise ValueError("requested relocation exceeds bounded neighbour interval")
    relocated_lower, relocated_upper = planar_control_bounds(track, relocated_stations, args.margin_m)
    relocated_controls = phase8.load_initial_controls_csv(
        args.relocated_controls_csv, relocated_stations, relocated_lower, relocated_upper)

    baseline_hash = phase9.sha256_file(args.baseline_controls_csv)
    candidates = (
        CandidateSpec(
            "baseline_restart3_52", "reference_52", args.baseline_controls_csv, baseline_hash,
            standard_stations, baseline_controls, True,
            "retained restart3 52-control line",
        ),
        CandidateSpec(
            "reduced_reoptimised_51", f"delete_{args.delete_index}_51",
            args.reduced_controls_csv, phase9.sha256_file(args.reduced_controls_csv),
            reduced_stations, reduced_controls, True,
            f"PR #58 reduced basis after deleting original control {args.delete_index}",
        ),
        CandidateSpec(
            "relocated_fixed_offsets_52", f"relocate_{args.relocate_index}_fixed_offsets",
            args.baseline_controls_csv, baseline_hash, relocated_stations, baseline_controls, False,
            f"PR #59 fixed-offset relocation sensitivity of control {args.relocate_index} "
            f"by {args.relocate_shift_m:+.3f} m; spread-only, not representative-eligible",
        ),
        CandidateSpec(
            "relocated_reoptimised_52", f"relocate_{args.relocate_index}_reoptimised",
            args.relocated_controls_csv, phase9.sha256_file(args.relocated_controls_csv),
            relocated_stations, relocated_controls, True,
            f"PR #60 bounded re-optimisation after control {args.relocate_index} relocation",
        ),
    )
    labels = [candidate.label for candidate in candidates]

    track_s = _common_track_stations(track.total_length_m, args.common_spacing_m)
    sampled_track = sample_track_stations(track, track_s)
    started = time.perf_counter()
    results = {
        candidate.label: _evaluate_candidate(
            candidate,
            track,
            bike,
            track_s,
            sampled_track,
            args.common_spacing_m,
            args.margin_m,
            args.boundary_check_spacing_m,
        )
        for candidate in candidates
    }
    elapsed = time.perf_counter() - started

    offsets_by_label = {label: results[label].projected_offset_m for label in labels}
    points_by_label = {
        label: np.column_stack((results[label].x_m, results[label].y_m)) for label in labels
    }
    minimum, median, maximum, spread = optimiser_spread_envelope(offsets_by_label, labels)
    maximum_matrix, rms_matrix = pairwise_geometry(points_by_label, labels)
    lap_times = {label: float(results[label].evaluation.lap_time_s) for label in labels}
    fastest_label = min(labels, key=lambda label: (lap_times[label], label))
    representative_eligible = {
        candidate.label: candidate.representative_eligible for candidate in candidates}
    (
        representative_label,
        medoid_label,
        fastest_eligible_label,
        eligible_means,
        medoid_lap_delta_s,
        selection_reason,
    ) = select_representative_candidate(
        labels,
        rms_matrix,
        lap_times,
        representative_eligible,
        args.representative_max_lap_delta_s,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = args.output_dir / "phase11_robust_line_candidate_summary.csv"
    pairwise_csv = args.output_dir / "phase11_robust_line_pairwise_spread.csv"
    envelope_csv = args.output_dir / "phase11_optimiser_spread_envelope.csv"
    reference_csv = args.output_dir / "phase11_representative_reference_line.csv"
    racing_png = args.output_dir / "phase11_representative_line_and_spread.png"

    _write_candidate_summary(
        summary_csv,
        results,
        labels,
        fastest_label,
        fastest_eligible_label,
        representative_label,
        medoid_label,
        eligible_means,
        maximum_matrix,
        rms_matrix,
        args.margin_m,
        args.representative_max_lap_delta_s,
    )
    _write_pairwise_csv(pairwise_csv, labels, maximum_matrix, rms_matrix)
    _write_envelope_csv(
        envelope_csv, track, track_s, sampled_track, results, labels,
        representative_label, fastest_label, minimum, median, maximum, spread, args.margin_m)
    _write_reference_line_csv(reference_csv, track, results[representative_label], args.margin_m)
    _write_png(
        racing_png, track, sampled_track, results, labels, representative_label, fastest_label,
        minimum, maximum, args.margin_m, args.plot_dpi)

    representative_index = labels.index(representative_label)
    fastest_index = labels.index(fastest_label)
    print(f"candidate_count={len(labels)}")
    print(f"candidate_labels={','.join(labels)}")
    print(f"margin_m={args.margin_m:.6f}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"common_spacing_m={args.common_spacing_m:.6f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.6f}")
    print("speed_backend=python")
    print("spread_interpretation=optimiser_and_control_basis_spread_not_physical_or_statistical_uncertainty")
    print("representative_eligibility=retained_or_reoptimised_candidates_only")
    print(f"representative_max_lap_delta_s={args.representative_max_lap_delta_s:.9f}")
    print(f"representative_selection_reason={selection_reason}")
    for label in labels:
        result = results[label]
        eligible_mean = eligible_means[label]
        eligible_mean_text = "na" if math.isnan(eligible_mean) else f"{eligible_mean:.9f}"
        print(
            f"candidate={label} basis={result.spec.basis_kind} controls={len(result.spec.controls_m)} "
            f"representative_eligible={str(result.spec.representative_eligible).lower()} "
            f"eligible_mean_pairwise_rms_m={eligible_mean_text} "
            f"lap_s={result.evaluation.lap_time_s:.9f} "
            f"minimum_track_edge_clearance_m={result.evaluation.smooth_line.minimum_boundary_clearance_m + args.margin_m:.9f} "
            f"minimum_forward_progress={result.minimum_forward_progress:.12g}")
    print(f"fastest_label={fastest_label}")
    print(f"fastest_common_grid_lap_s={lap_times[fastest_label]:.9f}")
    print(f"fastest_eligible_label={fastest_eligible_label}")
    print(f"eligible_geometric_medoid_label={medoid_label}")
    print(f"eligible_geometric_medoid_delta_from_fastest_s={medoid_lap_delta_s:.9f}")
    print(f"representative_label={representative_label}")
    print(f"representative_common_grid_lap_s={lap_times[representative_label]:.9f}")
    print(f"representative_delta_from_fastest_s={lap_times[representative_label] - lap_times[fastest_label]:.9f}")
    representative_mean = eligible_means[representative_label]
    print(f"representative_eligible_mean_pairwise_rms_displacement_m={representative_mean:.9f}")
    fastest_mean = eligible_means[fastest_label]
    fastest_mean_text = "na" if math.isnan(fastest_mean) else f"{fastest_mean:.9f}"
    print(f"fastest_eligible_mean_pairwise_rms_displacement_m={fastest_mean_text}")
    print(f"maximum_displacement_from_fastest_to_representative_m={maximum_matrix[fastest_index, representative_index]:.9f}")
    print(f"rms_displacement_from_fastest_to_representative_m={rms_matrix[fastest_index, representative_index]:.9f}")
    print(f"maximum_envelope_width_m={float(np.max(spread)):.9f}")
    print(f"rms_envelope_width_m={float(np.sqrt(np.mean(spread ** 2))):.9f}")
    print(f"evaluation_elapsed_s={elapsed:.3f}")
    print(f"candidate_summary_csv={summary_csv}")
    print(f"pairwise_spread_csv={pairwise_csv}")
    print(f"optimiser_spread_envelope_csv={envelope_csv}")
    print(f"representative_reference_line_csv={reference_csv}")
    print(f"racing_line_png={racing_png}")
    return {
        "results": results,
        "fastest_label": fastest_label,
        "fastest_eligible_label": fastest_eligible_label,
        "eligible_geometric_medoid_label": medoid_label,
        "representative_label": representative_label,
        "selection_reason": selection_reason,
        "maximum_envelope_width_m": float(np.max(spread)),
        "rms_envelope_width_m": float(np.sqrt(np.mean(spread ** 2))),
    }


if __name__ == "__main__":
    main()
