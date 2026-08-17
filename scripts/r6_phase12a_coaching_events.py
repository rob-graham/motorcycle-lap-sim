"""Extract and plot Phase 12A rider-facing events on the retained Mallala line.

The input controls are the Phase 11 retained representative 51-control line.
The line is re-evaluated with the same 0.25 m corridor margin, 0.8 rad/s
finite-roll sensitivity scenario and authoritative 0.125 m Python common grid
used for Phase 11 representative selection.

This command deliberately does not define run-off departure seeds or a
simulator-to-run-off export contract.  Phase 12A ends with numerical and visual
review of the coaching landmarks.
"""

import argparse
import csv
from dataclasses import asdict, replace
import importlib.util
import math
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.coaching import (
    EventDetectionConfig,
    detect_corner_regions,
    extract_coaching_events,
)
from motorcycle_lap_sim.motorcycle.config import HandlingConfig, load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.track import Track, sample_track_stations


DEFAULT_DELETE_INDEX = 26
DEFAULT_MARGIN_M = 0.25
DEFAULT_MAX_ROLL_RATE_RADPS = 0.8
DEFAULT_SPACING_M = 0.125
DEFAULT_BOUNDARY_CHECK_SPACING_M = 0.125
DEFAULT_EXPECTED_LAP_S = 71.396583646
DEFAULT_LAP_TOLERANCE_S = 2e-6
DEFAULT_PLOT_DPI = 400
EXPECTED_MALLALA_CORNERS = 9

# Groups of analytic reference-track primitives that represent Mallala T1-T9.
# T3, T6 and T7 are compound same-direction bends in the current approximate
# reference geometry.  This case-specific mapping consolidates raw lean regions without changing
# the generic lean-hysteresis detector.
MALLALA_CORNER_PRIMITIVE_GROUPS = (
    (1,),
    (3,),
    (5, 6, 7),
    (9,),
    (11,),
    (13, 14),
    (16, 17),
    (19,),
    (21,),
)

EVENT_FIELDS = (
    "corner", "event_type", "sample_index", "track_s_m", "path_q_m", "x_m", "y_m",
    "speed_mps", "speed_kph", "longitudinal_acceleration_mps2", "curvature_1pm",
    "lean_angle_deg", "roll_rate_radps", "gear", "rpm", "source_rule", "confidence",
    "display_on_map",
)

CORNER_REVIEW_FIELDS = (
    "raw_region_index", "nominal_corner", "raw_start_s_m", "raw_end_s_m",
    "consolidated_start_s_m", "consolidated_end_s_m", "turn_sign",
    "peak_abs_lean_deg", "peak_abs_curvature_1pm", "assignment_rule", "confidence",
)

MAP_EVENT_TYPES = (
    "braking_onset",
    "turn_in",
    "geometric_apex",
    "positive_drive_pickup",
    "corner_exit",
)

EVENT_ABBREVIATIONS = {
    "braking_onset": "BRK",
    "brake_release": "REL",
    "turn_in": "TURN",
    "geometric_apex": "APEX",
    "positive_drive_pickup": "GAS",
    "corner_exit": "EXIT",
}

EVENT_MARKERS = {
    "braking_onset": "v",
    "brake_release": "s",
    "turn_in": ">",
    "geometric_apex": "o",
    "positive_drive_pickup": "^",
    "corner_exit": "x",
}


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


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("representative_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--delete-index", type=int, default=DEFAULT_DELETE_INDEX)
    parser.add_argument("--margin-m", type=_nonnegative_float, default=DEFAULT_MARGIN_M)
    parser.add_argument("--max-roll-rate-radps", type=_positive_float,
                        default=DEFAULT_MAX_ROLL_RATE_RADPS)
    parser.add_argument("--spacing-m", type=_positive_float, default=DEFAULT_SPACING_M)
    parser.add_argument("--boundary-check-spacing-m", type=_positive_float,
                        default=DEFAULT_BOUNDARY_CHECK_SPACING_M)
    parser.add_argument("--expected-lap-s", type=_positive_float, default=DEFAULT_EXPECTED_LAP_S)
    parser.add_argument("--lap-tolerance-s", type=_nonnegative_float,
                        default=DEFAULT_LAP_TOLERANCE_S)
    parser.add_argument("--plot-dpi", type=int, default=DEFAULT_PLOT_DPI)
    return parser


def _mallala_corner_windows(track):
    starts = np.asarray(track.primitive_start_s_m, dtype=float)
    windows = []
    for group in MALLALA_CORNER_PRIMITIVE_GROUPS:
        first = min(group)
        last = max(group)
        if tuple(range(first, last + 1)) != tuple(group):
            raise RuntimeError("Mallala corner primitive groups must be contiguous")
        if first < 0 or last >= len(track.primitives):
            raise RuntimeError("Mallala corner primitive group is outside the reference track")
        windows.append((float(starts[first]), float(starts[last + 1])))
    return tuple(windows)


def _mallala_corner_ownership_intervals(track):
    """Return T1--T9 ownership intervals bounded at adjacent-straight midpoints.

    The corner arcs are deliberately not used as hard detection bounds.  Each
    nominal corner owns half of its approach and exit straight, which admits
    racing-line turn-in and exit chainages outside the centreline arc.  T1's
    interval wraps across start/finish and is represented by two intervals.
    """
    starts = np.asarray(track.primitive_start_s_m, dtype=float)
    lap_length = float(starts[-1])
    ownership = []
    for group in MALLALA_CORNER_PRIMITIVE_GROUPS:
        first, last = min(group), max(group)
        before = first - 1
        after = last + 1
        if before < 0 or after >= len(track.primitives):
            raise RuntimeError("Mallala corner must be bounded by straight primitives")
        start = 0.5 * (starts[before] + starts[first])
        end = 0.5 * (starts[after] + starts[after + 1])
        if first == 1:  # preceding straight is primitive 22 across start/finish
            start = 0.5 * (starts[-2] + lap_length)
            ownership.append(((start, lap_length), (0.0, end)))
        else:
            ownership.append(((float(start), float(end)),))
    return tuple(ownership)


def _interval_overlap_m(start_a, end_a, start_b, end_b):
    return max(0.0, min(end_a, end_b) - max(start_a, start_b))


def _consolidate_mallala_corner_regions(
        track, columns, raw_regions, *, allow_unassigned=False):
    """Assign generic lean regions to nominal Mallala corners and consolidate.

    Ownership uses the primitive-defined corner plus half of each neighbouring
    straight.  Direction must agree with the reference arcs.  Regions which do
    not satisfy both rules are never silently discarded: callers either get a
    clear exception or, for the review-producing CLI, an explicit unassigned
    review row.  More than one raw region may belong to a compound corner.
    """
    track_s = np.asarray(columns["track_s_m"], dtype=float)
    lean = np.asarray(columns["roll_angle_deg"], dtype=float)
    curvature = np.asarray(columns["path_curvature_1pm"], dtype=float)
    ownership = _mallala_corner_ownership_intervals(track)
    expected_signs = tuple(
        1 if track.primitives[group[0]].turn_angle_rad > 0.0 else -1
        for group in MALLALA_CORNER_PRIMITIVE_GROUPS)
    assignments = [[] for _ in MALLALA_CORNER_PRIMITIVE_GROUPS]
    raw_details = []

    for raw_index, (start, end) in enumerate(raw_regions, start=1):
        raw_start, raw_end = float(track_s[start]), float(track_s[end])
        sign = 1 if lean[start + int(np.argmax(np.abs(lean[start:end + 1])))] >= 0 else -1
        overlaps = [
            sum(_interval_overlap_m(raw_start, raw_end, a, b) for a, b in intervals)
            if sign == expected_signs[number] else 0.0
            for number, intervals in enumerate(ownership)
        ]
        maximum = max(overlaps, default=0.0)
        candidates = [i for i, overlap in enumerate(overlaps)
                      if overlap > 0.0 and np.isclose(overlap, maximum, atol=1e-9)]
        if len(candidates) > 1:
            names = ", ".join(f"T{i + 1}" for i in candidates)
            raise ValueError(f"raw region {raw_index} has ambiguous Mallala ownership: {names}")
        corner_index = candidates[0] if candidates else None
        if corner_index is None and not allow_unassigned:
            raise ValueError(
                f"raw region {raw_index} is unassigned: no direction-compatible "
                "Mallala ownership interval")
        if corner_index is not None:
            assignments[corner_index].append((start, end))
        raw_details.append((raw_index, start, end, sign, corner_index, maximum))

    missing = [f"T{i + 1}" for i, assigned in enumerate(assignments) if not assigned]
    if missing:
        raise ValueError(f"nominal Mallala corners have no assigned raw region: {', '.join(missing)}")
    consolidated = tuple((min(x[0] for x in assigned), max(x[1] for x in assigned))
                         for assigned in assignments)
    review = []
    for raw_index, start, end, sign, corner_index, overlap in raw_details:
        consolidated_region = consolidated[corner_index] if corner_index is not None else None
        review.append({
            "raw_region_index": raw_index,
            "nominal_corner": "" if corner_index is None else f"T{corner_index + 1}",
            "raw_start_s_m": float(track_s[start]),
            "raw_end_s_m": float(track_s[end]),
            "consolidated_start_s_m": "" if consolidated_region is None else
                float(track_s[consolidated_region[0]]),
            "consolidated_end_s_m": "" if consolidated_region is None else
                float(track_s[consolidated_region[1]]),
            "turn_sign": sign,
            "peak_abs_lean_deg": float(np.max(np.abs(lean[start:end + 1]))),
            "peak_abs_curvature_1pm": float(np.max(np.abs(curvature[start:end + 1]))),
            "assignment_rule": ("unassigned_direction_or_ownership_mismatch" if corner_index is None
                                else "maximum_chainage_overlap_with_mid_straight_ownership_and_turn_sign"),
            "confidence": "review" if corner_index is None else ("high" if overlap > 0.0 else "low"),
        })
    return consolidated, tuple(review)


def _write_corner_review_csv(path, review_rows):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CORNER_REVIEW_FIELDS)
        writer.writeheader()
        writer.writerows(review_rows)


def _write_events_csv(path, events):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=EVENT_FIELDS)
        writer.writeheader()
        for event in events:
            row = asdict(event)
            row["speed_kph"] = event.speed_mps * 3.6
            writer.writerow({field: row[field] for field in EVENT_FIELDS})


def _start_finish_segment(track):
    sample = sample_track_stations(track, np.array([0.0]))
    cx, cy = float(sample.x_m[0]), float(sample.y_m[0])
    nx, ny = float(sample.normal_x[0]), float(sample.normal_y[0])
    return (
        (cx + float(sample.width_left_m[0]) * nx,
         cy + float(sample.width_left_m[0]) * ny),
        (cx - float(sample.width_right_m[0]) * nx,
         cy - float(sample.width_right_m[0]) * ny),
    )


def _event_label(event):
    code = EVENT_ABBREVIATIONS[event.event_type]
    speed = int(round(event.speed_mps * 3.6))
    gear = "" if event.gear is None else f" G{event.gear}"
    if event.event_type == "geometric_apex":
        return f"{event.corner} {code}\n{speed} km/h {abs(event.lean_angle_deg):.0f} deg"
    return f"{event.corner} {code}\n{speed} km/h{gear}"


def _write_coaching_png(path, track, columns, events, *, max_roll_rate_radps, dpi):
    import matplotlib.pyplot as plt

    track_s = np.asarray(columns["track_s_m"], dtype=float)
    sampled_track = sample_track_stations(track, track_s)
    left_x = sampled_track.x_m + sampled_track.width_left_m * sampled_track.normal_x
    left_y = sampled_track.y_m + sampled_track.width_left_m * sampled_track.normal_y
    right_x = sampled_track.x_m - sampled_track.width_right_m * sampled_track.normal_x
    right_y = sampled_track.y_m - sampled_track.width_right_m * sampled_track.normal_y
    sf_left, sf_right = _start_finish_segment(track)

    figure, axis = plt.subplots(figsize=(13, 10))
    axis.plot(left_x, left_y, linewidth=0.45, label="Track edge")
    axis.plot(right_x, right_y, linewidth=0.45)
    axis.plot(columns["bike_x_m"], columns["bike_y_m"], linewidth=1.05,
              label="Representative racing line")
    axis.plot([sf_left[0], sf_right[0]], [sf_left[1], sf_right[1]],
              linewidth=1.0, linestyle="--", label="Start / finish")

    map_events = [event for event in events
                  if event.display_on_map and event.event_type in MAP_EVENT_TYPES]
    for event_type in MAP_EVENT_TYPES:
        selected = [event for event in map_events if event.event_type == event_type]
        if not selected:
            continue
        axis.scatter(
            [event.x_m for event in selected], [event.y_m for event in selected],
            marker=EVENT_MARKERS[event_type], s=24, linewidths=0.8,
            label=EVENT_ABBREVIATIONS[event_type], zorder=5,
        )

    # Short rider-facing labels are offset in alternating directions.  No
    # optimiser controls, corridor/envelope, centreline or diagnostic metrics
    # are drawn on this image.
    offsets = ((5, 5), (5, -16), (-44, 5), (-44, -16))
    for index, event in enumerate(map_events):
        dx, dy = offsets[index % len(offsets)]
        axis.annotate(
            _event_label(event),
            (event.x_m, event.y_m),
            xytext=(dx, dy), textcoords="offset points", fontsize=5.7,
            arrowprops={"arrowstyle": "-", "linewidth": 0.35},
            bbox={"boxstyle": "round,pad=0.15", "fc": "white", "alpha": 0.78,
                  "linewidth": 0.25},
            zorder=6,
        )

    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(
        "Mallala Phase 12A coaching marks - retained representative line\n"
        f"R6 provisional scenario; finite-roll sensitivity {max_roll_rate_radps:.2f} rad/s")
    axis.legend(fontsize="small", ncol=4)
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.plot_dpi <= 0:
        raise ValueError("plot DPI must be positive")

    phase8 = _load_sibling("r6_phase8_planar_optimisation_check.py", "phase8_phase12a")
    phase9 = _load_sibling("r6_phase9_baseline_check.py", "phase9_phase12a")
    phase9f = _load_sibling("r6_phase9f_roll_aware_optimisation.py", "phase9f_phase12a")
    trajectory = _load_sibling("r6_phase10_trajectory_export.py", "trajectory_phase12a")
    phase9f._require_canonical_inputs()

    track = Track.from_yaml(phase9.DEFAULT_TRACK)
    base_bike = load_motorcycle_config(phase9.DEFAULT_MOTORCYCLE)
    if base_bike.handling is not None:
        raise RuntimeError("canonical R6 baseline unexpectedly has handling enabled")
    bike = replace(
        base_bike, handling=HandlingConfig(max_roll_rate_radps=args.max_roll_rate_radps))

    standard_stations = generate_planar_control_stations(track, REFERENCE_PLANAR_CONTROL_POLICY)
    if args.delete_index < 0 or args.delete_index >= len(standard_stations):
        raise ValueError("delete index is outside the standard Phase 11 control basis")
    representative_stations = np.delete(standard_stations, args.delete_index)
    lower, upper = planar_control_bounds(track, representative_stations, args.margin_m)
    controls = phase8.load_initial_controls_csv(
        args.representative_controls_csv, representative_stations, lower, upper)

    evaluation = evaluate_planar_racing_line(
        controls,
        track,
        bike,
        representative_stations,
        sample_spacing_m=args.spacing_m,
        boundary_margin_m=args.margin_m,
        boundary_check_spacing_m=args.boundary_check_spacing_m,
        speed_backend="python",
    )
    if not evaluation.feasible or evaluation.smooth_line is None or evaluation.speed_profile is None:
        raise RuntimeError(f"retained representative line is infeasible: {evaluation.failure_reason}")
    lap_delta = float(evaluation.lap_time_s - args.expected_lap_s)
    if abs(lap_delta) > args.lap_tolerance_s:
        raise RuntimeError(
            "representative line lap does not reproduce the retained Phase 11 reference: "
            f"actual={evaluation.lap_time_s:.9f} expected={args.expected_lap_s:.9f} "
            f"delta={lap_delta:+.9f} s")

    columns = trajectory.trajectory_columns(
        track, evaluation.smooth_line, evaluation.speed_profile, bike)
    config = EventDetectionConfig()
    raw_regions = detect_corner_regions(columns, config)
    corner_regions, corner_review = _consolidate_mallala_corner_regions(
        track, columns, raw_regions, allow_unassigned=True)
    events = extract_coaching_events(
        columns, config, corner_regions=corner_regions,
        expected_corner_count=EXPECTED_MALLALA_CORNERS)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_csv = args.output_dir / "phase12a_representative_trajectory.csv"
    events_csv = args.output_dir / "phase12a_coaching_events.csv"
    corner_review_csv = args.output_dir / "phase12a_corner_regions_review.csv"
    coaching_png = args.output_dir / "phase12a_coaching_overview.png"
    trajectory.write_trajectory_csv(trajectory_csv, columns)
    _write_events_csv(events_csv, events)
    _write_corner_review_csv(corner_review_csv, corner_review)
    _write_coaching_png(
        coaching_png, track, columns, events,
        max_roll_rate_radps=args.max_roll_rate_radps, dpi=args.plot_dpi)

    counts = {event_type: 0 for event_type in (
        "local_max_speed", "braking_onset", "maximum_braking", "brake_release",
        "turn_in", "geometric_apex", "speed_apex", "maximum_lean",
        "positive_drive_pickup", "corner_exit", "roll_transition", "gear_shift")}
    for event in events:
        counts[event.event_type] = counts.get(event.event_type, 0) + 1

    print("phase=12A_coaching_event_extraction")
    print("phase11_status=closed_representative_line_retained")
    print("representative_label=reduced_reoptimised_51")
    print(f"representative_controls_csv={args.representative_controls_csv}")
    print(f"representative_controls_sha256={phase9.sha256_file(args.representative_controls_csv)}")
    print(f"deleted_original_control_index={args.delete_index}")
    print(f"control_count={len(controls)}")
    print(f"margin_m={args.margin_m:.6f}")
    print(f"max_roll_rate_radps={args.max_roll_rate_radps:.9f}")
    print("scenario_note=finite roll rate is a sensitivity scenario, not a calibrated R6/rider constant")
    print(f"common_spacing_m={args.spacing_m:.6f}")
    print(f"boundary_check_spacing_m={args.boundary_check_spacing_m:.6f}")
    print("speed_backend=python")
    print(f"lap_s={evaluation.lap_time_s:.9f}")
    print(f"lap_delta_from_phase11_reference_s={lap_delta:+.9f}")
    print(f"raw_corner_region_count={len(raw_regions)}")
    print(f"nominal_corner_count={counts['turn_in']}")
    for event_type, count in counts.items():
        print(f"event_count_{event_type}={count}")
    print(f"trajectory_csv={trajectory_csv}")
    print(f"coaching_events_csv={events_csv}")
    print(f"corner_regions_review_csv={corner_review_csv}")
    print(f"coaching_overview_png={coaching_png}")
    print("visual_review_required=true")
    print("runoff_export_contract_status=deferred_until_after_visual_event_review")
    return {
        "evaluation": evaluation,
        "columns": columns,
        "raw_regions": raw_regions,
        "corner_regions": corner_regions,
        "events": events,
    }


if __name__ == "__main__":
    main()
