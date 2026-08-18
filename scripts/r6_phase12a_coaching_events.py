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

OVERVIEW_EVENT_TYPES = (
    "braking_onset",
    "turn_in",
    "geometric_apex",
    "positive_drive_pickup",
    "corner_exit",
)
MAP_EVENT_TYPES = OVERVIEW_EVENT_TYPES

DETAIL_EVENT_TYPES = (
    "braking_onset", "brake_release", "turn_in", "geometric_apex",
    "positive_drive_pickup", "corner_exit", "speed_apex", "maximum_curvature",
)

EVENT_ABBREVIATIONS = {
    "braking_onset": "BRK",
    "brake_release": "REL",
    "turn_in": "TURN",
    "geometric_apex": "APEX",
    "positive_drive_pickup": "GAS",
    "corner_exit": "EXIT",
    "speed_apex": "V-APEX",
    "maximum_curvature": "K-MAX",
}

EVENT_MARKERS = {
    "braking_onset": "v",
    "brake_release": "s",
    "turn_in": ">",
    "geometric_apex": "o",
    "positive_drive_pickup": "^",
    "corner_exit": "x",
    "speed_apex": "D",
    "maximum_curvature": "P",
}

VISUAL_OUTPUT_FILENAMES = {
    "coaching_overview": "phase12a_coaching_overview.png",
    "speed_map": "phase12a_speed_map.png",
    "detail_T1_T3": "phase12a_T1_T3_detail.png",
    "detail_T4_T6": "phase12a_T4_T6_detail.png",
    "detail_T7_T9": "phase12a_T7_T9_detail.png",
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


def _mallala_corner_ownership_windows(track):
    """Return non-overlapping T1-T9 intervals including adjacent straights.

    Boundaries between nominal arc groups are the midpoint of the intervening
    straight.  The lap start/end bound the T1 approach and T9 departure.  This
    is geometry-derived case ownership, not a hard-coded event chainage.
    """
    nominal = _mallala_corner_windows(track)
    boundaries = [0.0]
    boundaries.extend(
        0.5 * (nominal[index][1] + nominal[index + 1][0])
        for index in range(len(nominal) - 1))
    boundaries.append(float(track.total_length_m))
    return tuple((float(boundaries[index]), float(boundaries[index + 1]))
                 for index in range(len(nominal)))


def _mallala_corner_turn_signs(track):
    signs = []
    for group in MALLALA_CORNER_PRIMITIVE_GROUPS:
        curvatures = [float(track.primitives[index].curvature_1pm) for index in group]
        nonzero = [value for value in curvatures if value != 0.0]
        if not nonzero or any(np.sign(value) != np.sign(nonzero[0]) for value in nonzero):
            raise RuntimeError("Mallala nominal corner has ambiguous reference turn direction")
        signs.append(1 if nonzero[0] > 0.0 else -1)
    return tuple(signs)


def _interval_overlap_m(start_a, end_a, start_b, end_b):
    return max(0.0, min(end_a, end_b) - max(start_a, start_b))


def _select_mallala_corner_regions(track, columns, raw_regions):
    """Consolidate generic lean regions into geometry-owned Mallala corners.

    Ownership includes adjacent straights, validates turn direction and permits
    multiple same-direction raw fragments for compound nominal corners.  A raw
    region is assigned to exactly one owner by greatest overlap; equal-overlap
    ambiguity and direction conflicts fail closed.  Non-overlapping artefacts
    are explicitly reported as unassigned rather than silently discarded.
    """
    track_s = np.asarray(columns["track_s_m"], dtype=float)
    lean = np.asarray(columns["roll_angle_deg"], dtype=float)
    if lean.shape != track_s.shape or not np.all(np.isfinite(lean)):
        raise ValueError("Mallala consolidation requires finite roll_angle_deg samples")
    nominal_windows = _mallala_corner_windows(track)
    ownership_windows = _mallala_corner_ownership_windows(track)
    expected_signs = _mallala_corner_turn_signs(track)
    selected = []
    diagnostics = []
    assignments = [[] for _ in nominal_windows]

    for raw_index, (start, end) in enumerate(raw_regions):
        region_start, region_end = float(track_s[start]), float(track_s[end])
        overlaps = np.asarray([
            _interval_overlap_m(region_start, region_end, owner_start, owner_end)
            for owner_start, owner_end in ownership_windows])
        maximum = float(np.max(overlaps))
        if maximum <= 0.0:
            diagnostics.append({"raw_region": raw_index + 1, "status": "unassigned",
                                "reason": "no positive ownership overlap"})
            continue
        owners = np.flatnonzero(np.isclose(overlaps, maximum, rtol=0.0, atol=1e-9))
        if owners.size != 1:
            raise ValueError(
                f"raw lean region {raw_index + 1} has ambiguous equal Mallala ownership")
        owner = int(owners[0])
        observed_sign = 1 if lean[start + int(np.argmax(np.abs(lean[start:end + 1])))] >= 0 else -1
        if observed_sign != expected_signs[owner]:
            diagnostics.append({"raw_region": raw_index + 1, "status": "unassigned",
                                "reason": f"turn direction conflicts with T{owner + 1}"})
            continue
        assignments[owner].append((raw_index, start, end, maximum))

    for owner, assigned in enumerate(assignments):
        nominal_start, nominal_end = nominal_windows[owner]
        if not assigned or not any(
                _interval_overlap_m(float(track_s[start]), float(track_s[end]),
                                    nominal_start, nominal_end) > 0.0
                for _, start, end, _ in assigned):
            raise ValueError(f"no direction-valid raw lean region resolves Mallala T{owner + 1}")
        start = min(item[1] for item in assigned)
        end = max(item[2] for item in assigned)
        selected.append((start, end))
        owner_start, owner_end = ownership_windows[owner]
        diagnostics.append({
            "corner": owner + 1,
            "raw_regions": tuple(item[0] + 1 for item in assigned),
            "status": "assigned_consolidated",
            "reason": "greatest ownership overlap and matching turn direction",
            "overlap_m": sum(item[3] for item in assigned),
            "nominal_window": (nominal_start, nominal_end),
            "ownership_window": (owner_start, owner_end),
            "turn_sign": expected_signs[owner],
        })

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


def _track_plot_data(columns):
    return tuple(np.asarray(columns[name], dtype=float) for name in (
        "left_boundary_x_m", "left_boundary_y_m", "right_boundary_x_m",
        "right_boundary_y_m", "bike_x_m", "bike_y_m"))


def _draw_base(axis, columns):
    left_x, left_y, right_x, right_y, bike_x, bike_y = _track_plot_data(columns)
    axis.plot(left_x, left_y, color="0.45", linewidth=0.55, label="Track edge")
    axis.plot(right_x, right_y, color="0.45", linewidth=0.55)
    axis.plot(bike_x, bike_y, color="C0", linewidth=1.15,
              label="Representative racing line")


def _draw_start_finish(axis, track):
    sf_left, sf_right = _start_finish_segment(track)
    axis.plot([sf_left[0], sf_right[0]], [sf_left[1], sf_right[1]],
              color="black", linewidth=1.0, linestyle="--", label="Start / finish")


def _corner_events(events, first, last):
    names = {f"T{number}" for number in range(first, last + 1)}
    return [event for event in events if event.corner in names]


def _draw_corner_labels(axis, events, first=1, last=9):
    for number in range(first, last + 1):
        apex = next(event for event in events
                    if event.corner == f"T{number}" and event.event_type == "geometric_apex")
        axis.annotate(f"T{number}", (apex.x_m, apex.y_m), xytext=(7, 7),
                      textcoords="offset points", fontsize=8, fontweight="bold")


def _scatter_events(axis, events, event_types, *, annotate=False):
    for event_type in event_types:
        selected = [event for event in events if event.event_type == event_type]
        if not selected:
            continue
        axis.scatter([event.x_m for event in selected], [event.y_m for event in selected],
                     marker=EVENT_MARKERS[event_type], s=24, linewidths=0.8,
                     label=EVENT_ABBREVIATIONS[event_type], zorder=5)
        if annotate:
            for event in selected:
                speed = int(round(event.speed_mps * 3.6))
                axis.annotate(f"{event.corner} {EVENT_ABBREVIATIONS[event_type]} {speed}",
                              (event.x_m, event.y_m), xytext=(5, 5),
                              textcoords="offset points", fontsize=6,
                              bbox={"boxstyle": "round,pad=0.12", "fc": "white",
                                    "alpha": 0.78, "linewidth": 0.25})


def _finish_plot(figure, axis, path, title, dpi):
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Local x (m)")
    axis.set_ylabel("Local y (m)")
    axis.set_title(title)
    axis.legend(fontsize="small", ncol=4)
    figure.tight_layout()
    figure.savefig(path, dpi=dpi)
    import matplotlib.pyplot as plt
    plt.close(figure)


def _write_coaching_overview(path, track, columns, events, *, dpi):
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(13, 10))
    _draw_base(axis, columns)
    _draw_start_finish(axis, track)
    _scatter_events(axis, events, OVERVIEW_EVENT_TYPES)
    _draw_corner_labels(axis, events)
    _finish_plot(figure, axis, path,
                 "Mallala Phase 12A coaching overview — visual review required", dpi)


def _write_speed_map(path, track, columns, events, *, dpi):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    _, _, _, _, bike_x, bike_y = _track_plot_data(columns)
    points = np.column_stack((bike_x, bike_y))
    segments = np.stack((points[:-1], points[1:]), axis=1)
    speed_kph = np.asarray(columns["speed_mps"], dtype=float) * 3.6
    figure, axis = plt.subplots(figsize=(13, 10))
    left_x, left_y, right_x, right_y, _, _ = _track_plot_data(columns)
    axis.plot(left_x, left_y, color="0.65", linewidth=0.5)
    axis.plot(right_x, right_y, color="0.65", linewidth=0.5)
    collection = LineCollection(segments, cmap="viridis", linewidth=1.8)
    collection.set_array(0.5 * (speed_kph[:-1] + speed_kph[1:]))
    axis.add_collection(collection)
    figure.colorbar(collection, ax=axis, label="Speed (km/h)")
    _draw_start_finish(axis, track)
    _scatter_events(axis, events,
                    ("braking_onset", "geometric_apex", "positive_drive_pickup"))
    _draw_corner_labels(axis, events)
    _finish_plot(figure, axis, path,
                 "Mallala retained racing line — authoritative solved speed", dpi)


def _write_detail(path, columns, events, first, last, *, dpi):
    import matplotlib.pyplot as plt
    selected = _corner_events(events, first, last)
    samples = [event.sample_index for event in selected]
    if not samples:
        raise RuntimeError(f"no events found for T{first}-T{last} detail plot")
    lo, hi = min(samples), max(samples)
    left_x, left_y, right_x, right_y, bike_x, bike_y = _track_plot_data(columns)
    sl = slice(lo, hi + 1)
    figure, axis = plt.subplots(figsize=(12, 9))
    axis.plot(left_x[sl], left_y[sl], color="0.45", linewidth=0.7, label="Track edge")
    axis.plot(right_x[sl], right_y[sl], color="0.45", linewidth=0.7)
    axis.plot(bike_x[sl], bike_y[sl], color="C0", linewidth=1.3,
              label="Representative racing line")
    _scatter_events(axis, selected, DETAIL_EVENT_TYPES, annotate=True)
    _draw_corner_labels(axis, events, first, last)
    margin = 25.0
    axis.set_xlim(min(left_x[sl].min(), right_x[sl].min()) - margin,
                  max(left_x[sl].max(), right_x[sl].max()) + margin)
    axis.set_ylim(min(left_y[sl].min(), right_y[sl].min()) - margin,
                  max(left_y[sl].max(), right_y[sl].max()) + margin)
    _finish_plot(figure, axis, path,
                 f"Mallala Phase 12A event detail — T{first} to T{last}", dpi)


def _write_corner_review_csv(path, columns, corner_regions, mapping):
    fields = ("corner", "raw_regions", "status", "reason", "turn_sign", "start_index",
              "end_index", "start_track_s_m", "end_track_s_m",
              "nominal_window_start_m", "nominal_window_end_m",
              "ownership_window_start_m", "ownership_window_end_m", "overlap_m")
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for diagnostic in mapping:
            if "corner" not in diagnostic:
                writer.writerow({"raw_regions": diagnostic["raw_region"],
                                 "status": diagnostic["status"],
                                 "reason": diagnostic["reason"]})
                continue
            number = int(diagnostic["corner"])
            start, end = corner_regions[number - 1]
            nominal_start, nominal_end = diagnostic["nominal_window"]
            owner_start, owner_end = diagnostic["ownership_window"]
            writer.writerow({
                "corner": f"T{number}",
                "raw_regions": ";".join(map(str, diagnostic["raw_regions"])),
                "status": diagnostic["status"], "reason": diagnostic["reason"],
                "turn_sign": diagnostic["turn_sign"], "start_index": start,
                "end_index": end, "start_track_s_m": columns["track_s_m"][start],
                "end_track_s_m": columns["track_s_m"][end],
                "nominal_window_start_m": nominal_start,
                "nominal_window_end_m": nominal_end,
                "ownership_window_start_m": owner_start,
                "ownership_window_end_m": owner_end,
                "overlap_m": diagnostic["overlap_m"],
            })

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
    visual_paths = {
        name: args.output_dir / filename for name, filename in VISUAL_OUTPUT_FILENAMES.items()
    }
    trajectory.write_trajectory_csv(trajectory_csv, columns)
    _write_events_csv(events_csv, events)
    _write_corner_review_csv(corner_review_csv, columns, corner_regions, corner_mapping)
    _write_coaching_overview(
        visual_paths["coaching_overview"], track, columns, events, dpi=args.plot_dpi)
    _write_speed_map(visual_paths["speed_map"], track, columns, events, dpi=args.plot_dpi)
    for name, first, last in (("detail_T1_T3", 1, 3), ("detail_T4_T6", 4, 6),
                              ("detail_T7_T9", 7, 9)):
        _write_detail(visual_paths[name], columns, events, first, last, dpi=args.plot_dpi)

    counts = {event_type: 0 for event_type in (
        "local_max_speed", "braking_onset", "maximum_braking", "brake_release",
        "turn_in", "geometric_apex", "maximum_curvature", "speed_apex", "maximum_lean",
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
    print(f"raw_lean_region_count={len(raw_regions)}")
    print(f"corner_count={counts['turn_in']}")
    print(f"nominal_corner_count={len(corner_regions)}")
    for diagnostic in corner_mapping:
        if "corner" not in diagnostic:
            print(
                f"raw_region_{diagnostic['raw_region']}_status={diagnostic['status']} "
                f"reason={diagnostic['reason']}")
            continue
        corner_number = diagnostic["corner"]
        raw_numbers = ",".join(map(str, diagnostic["raw_regions"]))
        overlap = diagnostic["overlap_m"]
        window_start, window_end = diagnostic["nominal_window"]
        owner_start, owner_end = diagnostic["ownership_window"]
        print(
            f"corner_mapping_T{corner_number}=raw_regions_{raw_numbers} "
            f"overlap_m={overlap:.6f} nominal_window_m={window_start:.3f}:{window_end:.3f} "
            f"ownership_window_m={owner_start:.3f}:{owner_end:.3f}")
    for event_type, count in counts.items():
        print(f"event_count_{event_type}={count}")
    print(f"trajectory_csv={trajectory_csv}")
    print(f"coaching_events_csv={events_csv}")
    print(f"corner_regions_review_csv={corner_review_csv}")
    print(f"coaching_overview_png={visual_paths['coaching_overview']}")
    print(f"speed_map_png={visual_paths['speed_map']}")
    print(f"detail_T1_T3_png={visual_paths['detail_T1_T3']}")
    print(f"detail_T4_T6_png={visual_paths['detail_T4_T6']}")
    print(f"detail_T7_T9_png={visual_paths['detail_T7_T9']}")
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
