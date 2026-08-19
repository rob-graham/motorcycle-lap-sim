import csv
import io
import json
import math
from dataclasses import replace

import numpy as np
import pytest

from motorcycle_lap_sim.runoff import (
    DEPARTURE_SEED_FIELDS,
    RUNOFF_BUNDLE_VERSION,
    build_runoff_input_package,
    serialize_runoff_bundle,
    write_runoff_bundle,
)


def _package(event_types=("braking_onset",), warnings=("synthetic warning",)):
    count = 6
    theta = np.arange(count) * 2 * np.pi / count
    x, y = 10 * np.cos(theta), 10 * np.sin(theta)
    segment = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)
    q = np.r_[0.0, np.cumsum(segment[:-1])]
    columns = {
        "sample_index": np.arange(count), "track_s_m": q, "path_q_m": q,
        "bike_x_m": x, "bike_y_m": y,
        "left_boundary_x_m": 12 * np.cos(theta), "left_boundary_y_m": 12 * np.sin(theta),
        "right_boundary_x_m": 8 * np.cos(theta), "right_boundary_y_m": 8 * np.sin(theta),
        "speed_mps": np.array([20.123456789012345, 21, 22, 23, 24, 25], dtype=float),
        "longitudinal_acceleration_mps2": np.linspace(-2, 1, count),
        "lateral_acceleration_signed_mps2": np.linspace(2, 3, count),
        "path_curvature_1pm": np.full(count, 0.1),
        "roll_angle_rad": np.linspace(0.2, 0.3, count),
        "roll_rate_model_radps": np.linspace(-0.2, 0.2, count),
        "gear": np.arange(count) + 1,
        "longitudinal_limit_reason": np.array(["profile/other"] * count),
    }
    events = []
    for index, event_type in enumerate(event_types, 1):
        events.append({
            "corner": f"T{index}", "event_type": event_type, "sample_index": index,
            "track_s_m": q[index], "path_q_m": q[index], "x_m": x[index], "y_m": y[index],
            "speed_mps": columns["speed_mps"][index],
            "longitudinal_acceleration_mps2": columns["longitudinal_acceleration_mps2"][index],
            "curvature_1pm": 0.1,
            "lean_angle_deg": math.degrees(columns["roll_angle_rad"][index]),
            "roll_rate_radps": columns["roll_rate_model_radps"][index],
            "source_rule": "synthetic exact rule", "confidence": "high",
        })
    total = float(segment.sum())
    return build_runoff_input_package(
        columns, events, track_length_m=total, path_length_m=total,
        scenario_metadata={"scenario_id": "case", "simulator_commit": "abc123",
                           "track_id": "circle", "event_set_id": "sha256:123"},
        warnings=warnings)


def _read_csv(data):
    return list(csv.DictReader(io.StringIO(data.decode("utf-8"))))


def test_repeated_exports_are_byte_stable_and_hashes_match(tmp_path):
    package = _package(("braking_onset", "turn_in", "geometric_apex"))
    first = serialize_runoff_bundle(package)
    second = serialize_runoff_bundle(package)
    assert first == second
    written = write_runoff_bundle(package, tmp_path)
    assert {name: (tmp_path / name).read_bytes() for name in first} == first
    import hashlib
    assert all(written[name]["sha256"] == hashlib.sha256(data).hexdigest()
               for name, data in first.items())


def test_manifest_schema_order_counts_metadata_and_warnings():
    package = _package(("local_max_speed", "turn_in"), warnings=("one", "two"))
    files = serialize_runoff_bundle(package)
    manifest = json.loads(files["manifest.json"])
    assert manifest["bundle_version"] == RUNOFF_BUNDLE_VERSION
    assert manifest["runoff_interface_version"] == "0.1.0"
    assert manifest["trajectory"]["fields"] == list(package.trajectory)
    assert manifest["trajectory"]["row_count"] == 6
    assert "heading_rad" in manifest["trajectory"]["fields"]
    assert manifest["departure_seeds"]["fields"] == list(DEPARTURE_SEED_FIELDS)
    assert manifest["departure_seeds"]["count"] == 2
    assert manifest["departure_seeds"]["counts_by_source_event_type"] == {
        "local_max_speed": 1, "turn_in": 1}
    assert manifest["scenario_metadata"] == dict(package.scenario_metadata)
    assert manifest["warnings"] == ["one", "two"]
    assert files["trajectory.csv"].splitlines()[0].decode().split(",") == list(package.trajectory)
    assert files["departure_seeds.csv"].splitlines()[0].decode().split(",") == list(DEPARTURE_SEED_FIELDS)


def test_float_csv_round_trip_preserves_binary_float_values():
    package = _package()
    rows = _read_csv(serialize_runoff_bundle(package)["trajectory.csv"])
    for index, value in enumerate(package.trajectory["speed_mps"]):
        assert float(rows[index]["speed_mps"]) == float(value)


def test_zero_seed_and_multi_seed_bundles():
    zero = _package(())
    assert _read_csv(serialize_runoff_bundle(zero)["departure_seeds.csv"]) == []
    multi = _package(("local_max_speed", "braking_onset", "turn_in",
                      "geometric_apex", "positive_drive_pickup"))
    assert len(_read_csv(serialize_runoff_bundle(multi)["departure_seeds.csv"])) == 5


def test_serialization_does_not_mutate_package():
    package = _package(("turn_in",))
    arrays = {name: value.tobytes() for name, value in package.trajectory.items()}
    metadata = dict(package.scenario_metadata)
    seeds = package.departure_seeds
    serialize_runoff_bundle(package)
    assert arrays == {name: value.tobytes() for name, value in package.trajectory.items()}
    assert metadata == dict(package.scenario_metadata)
    assert seeds == package.departure_seeds


@pytest.mark.parametrize("change,match", [
    (lambda package: replace(package, track_length_m=math.nan), "non-finite"),
    (lambda package: replace(package, warnings=(math.inf,)), "non-finite"),
])
def test_malformed_nonfinite_manual_package_fails_closed(change, match):
    with pytest.raises(ValueError, match=match):
        serialize_runoff_bundle(change(_package()))
