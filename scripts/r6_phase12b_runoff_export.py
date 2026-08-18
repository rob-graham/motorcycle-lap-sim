"""Build the deterministic Phase 12B run-off bundle for retained Mallala."""

import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import importlib.util
import io
from pathlib import Path
import subprocess

from motorcycle_lap_sim.runoff import (
    RUNOFF_BUNDLE_VERSION,
    RUNOFF_INTERFACE_VERSION,
    build_runoff_input_package,
    write_runoff_bundle,
)


REPRESENTATIVE_LABEL = "reduced_reoptimised_51"
EXPECTED_CONTROLS_SHA256 = "4aa138e5af35e3a9180efc7a79abca7628dac99914ca082019d0140a8dfb02b3"
SCENARIO_ID = "mallala_r6_reduced_reoptimised_51_roll_0.8_radps"
TRACK_ID = "mallala_analytic_reference_track"
RUNOFF_TRAJECTORY_FIELDS = (
    "sample_index", "track_s_m", "path_q_m", "bike_x_m", "bike_y_m",
    "left_boundary_x_m", "left_boundary_y_m", "right_boundary_x_m",
    "right_boundary_y_m", "speed_mps", "longitudinal_acceleration_mps2",
    "lateral_acceleration_signed_mps2", "path_curvature_1pm", "roll_angle_rad",
    "roll_rate_model_radps", "gear", "rpm", "roll_rate_limited",
    "lateral_grip_limited", "powertrain_speed_limited", "wheelie_limited",
    "stoppie_limited", "traction_limited", "engine_power_limited",
    "longitudinal_limit_reason",
)


def _load_phase12a():
    path = Path(__file__).resolve().with_name("r6_phase12a_coaching_events.py")
    spec = importlib.util.spec_from_file_location("phase12a_for_phase12b", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Phase 12A pipeline from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase12a = _load_phase12a()


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("representative_controls_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--delete-index", type=int, default=phase12a.DEFAULT_DELETE_INDEX)
    parser.add_argument("--margin-m", type=phase12a._nonnegative_float,
                        default=phase12a.DEFAULT_MARGIN_M)
    parser.add_argument("--max-roll-rate-radps", type=phase12a._positive_float,
                        default=phase12a.DEFAULT_MAX_ROLL_RATE_RADPS)
    parser.add_argument("--spacing-m", type=phase12a._positive_float,
                        default=phase12a.DEFAULT_SPACING_M)
    parser.add_argument("--boundary-check-spacing-m", type=phase12a._positive_float,
                        default=phase12a.DEFAULT_BOUNDARY_CHECK_SPACING_M)
    parser.add_argument("--expected-lap-s", type=phase12a._positive_float,
                        default=phase12a.DEFAULT_EXPECTED_LAP_S)
    parser.add_argument("--lap-tolerance-s", type=phase12a._nonnegative_float,
                        default=phase12a.DEFAULT_LAP_TOLERANCE_S)
    return parser


def _sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _event_set_id(events):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=phase12a.EVENT_FIELDS, lineterminator="\n")
    writer.writeheader()
    for event in events:
        row = asdict(event)
        row["speed_kph"] = event.speed_mps * 3.6
        writer.writerow({field: row[field] for field in phase12a.EVENT_FIELDS})
    return "sha256:" + hashlib.sha256(stream.getvalue().encode("utf-8")).hexdigest()


def _git_commit():
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True,
        cwd=Path(__file__).resolve().parents[1]).stdout.strip()


def _require_retained_acceptance_provenance(args, controls_sha256):
    """Reject settings or controls that are not the frozen acceptance case."""
    canonical_settings = {
        "delete_index": phase12a.DEFAULT_DELETE_INDEX,
        "margin_m": phase12a.DEFAULT_MARGIN_M,
        "max_roll_rate_radps": phase12a.DEFAULT_MAX_ROLL_RATE_RADPS,
        "spacing_m": phase12a.DEFAULT_SPACING_M,
        "boundary_check_spacing_m": phase12a.DEFAULT_BOUNDARY_CHECK_SPACING_M,
        "expected_lap_s": phase12a.DEFAULT_EXPECTED_LAP_S,
        "lap_tolerance_s": phase12a.DEFAULT_LAP_TOLERANCE_S,
    }
    mismatches = [
        f"{name}={getattr(args, name)!r} (required {expected!r})"
        for name, expected in canonical_settings.items()
        if getattr(args, name) != expected
    ]
    if mismatches:
        raise RuntimeError(
            "Phase 12B retained acceptance requires canonical settings: "
            + "; ".join(mismatches))
    if controls_sha256 != EXPECTED_CONTROLS_SHA256:
        raise RuntimeError(
            "retained controls SHA-256 mismatch: "
            f"actual={controls_sha256} expected={EXPECTED_CONTROLS_SHA256}")


def assemble_runoff_package(retained, *, controls_sha256, simulator_commit, args):
    """Assemble the real retained-case interface package from Phase 12A output."""
    _require_retained_acceptance_provenance(args, controls_sha256)
    columns = retained["columns"]
    missing = [field for field in RUNOFF_TRAJECTORY_FIELDS if field not in columns]
    if missing:
        raise RuntimeError(f"retained trajectory lacks run-off fields: {missing}")
    runoff_columns = {field: columns[field] for field in RUNOFF_TRAJECTORY_FIELDS}
    path = retained["evaluation"].smooth_line.sampled_path
    path_length_m = float(path.segment_lengths_m.sum())
    track = retained["track"]
    events = retained["events"]
    metadata = {
        "scenario_id": SCENARIO_ID,
        "simulator_commit": simulator_commit,
        "track_id": TRACK_ID,
        "event_set_id": _event_set_id(events),
        "representative_label": REPRESENTATIVE_LABEL,
        "retained_controls_sha256": controls_sha256,
        "track_config_sha256": _sha256_file(retained["phase9"].DEFAULT_TRACK),
        "motorcycle_config_sha256": _sha256_file(retained["phase9"].DEFAULT_MOTORCYCLE),
        "margin_m": format(args.margin_m, ".17g"),
        "max_roll_rate_radps": format(args.max_roll_rate_radps, ".17g"),
        "common_spacing_m": format(args.spacing_m, ".17g"),
        "boundary_check_spacing_m": format(args.boundary_check_spacing_m, ".17g"),
        "speed_backend": "python",
    }
    warnings = (
        "INTERNAL-PROTOTYPE; NOT-EXTERNALLY-ACCEPTED",
        "Retained path is an engineering analysis trajectory, not a recommended riding line.",
        "Departure seeds are candidates, not safety criteria or occurrence predictions.",
        "Mallala reference geometry is approximate and not survey-grade or georeferenced.",
    )
    return build_runoff_input_package(
        runoff_columns, events, track_length_m=track.total_length_m,
        path_length_m=path_length_m, scenario_metadata=metadata, warnings=warnings)


def main(argv=None):
    args = build_parser().parse_args(argv)
    controls_hash = _sha256_file(args.representative_controls_csv)
    _require_retained_acceptance_provenance(args, controls_hash)
    retained = phase12a.calculate_retained_case(args)
    commit = _git_commit()
    package = assemble_runoff_package(
        retained, controls_sha256=controls_hash, simulator_commit=commit, args=args)
    files = write_runoff_bundle(package, args.output_dir)
    source_counts = Counter(seed.source_event_type for seed in package.departure_seeds)
    seed_counts = Counter(seed.seed_type for seed in package.departure_seeds)
    event_counts = Counter(event.event_type for event in retained["events"])
    print("phase=12B_retained_mallala_runoff_export")
    print(f"runoff_interface_version={RUNOFF_INTERFACE_VERSION}")
    print(f"runoff_bundle_version={RUNOFF_BUNDLE_VERSION}")
    print(f"simulator_commit={commit}")
    print(f"representative_controls_sha256={controls_hash}")
    print(f"lap_s={retained['evaluation'].lap_time_s:.9f}")
    print(f"lap_delta_from_phase11_reference_s={retained['lap_delta_s']:+.9f}")
    for prefix, values, total in (
            ("track", package.trajectory["track_s_m"], package.track_length_m),
            ("path", package.trajectory["path_q_m"], package.path_length_m)):
        print(f"{prefix}_length_m={total:.17g}")
        print(f"final_stored_{prefix}_chainage_m={float(values[-1]):.17g}")
        print(f"{prefix}_wrap_distance_m={total - float(values[-1]):.17g}")
    print(f"trajectory_sample_count={len(package.trajectory['sample_index'])}")
    for event_type in sorted(event_counts):
        print(f"phase12a_event_count_{event_type}={event_counts[event_type]}")
    print(f"departure_seed_count={len(package.departure_seeds)}")
    for name, count in sorted(source_counts.items()):
        print(f"departure_source_event_count_{name}={count}")
    for name, count in sorted(seed_counts.items()):
        print(f"departure_seed_type_count_{name}={count}")
        seed = next(value for value in package.departure_seeds if value.seed_type == name)
        print(
            f"departure_seed_sample_{name}=seed_id:{seed.seed_id},track_s_m:{seed.track_s_m:.9f},"
            f"speed_mps:{seed.speed_mps:.9f},heading_rad:{seed.heading_rad:.9f}")
    print(f"event_set_id={package.scenario_metadata['event_set_id']}")
    for filename, details in files.items():
        print(f"output_{filename}={details['path']} sha256={details['sha256']}")
    return package


if __name__ == "__main__":
    main()
