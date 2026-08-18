import math

import numpy as np
import pytest

from motorcycle_lap_sim.runoff import (
    RUNOFF_INTERFACE_VERSION,
    build_departure_seeds,
    build_runoff_input_package,
    derive_closed_path_heading_rad,
)


def _columns(count=8, clockwise=False, nonuniform=False):
    if nonuniform:
        fractions = np.linspace(0.0, 1.0, count, endpoint=False) ** 1.35
        theta = fractions * 2.0 * np.pi
    else:
        theta = np.arange(count, dtype=float) * 2.0 * np.pi / count
    if clockwise:
        theta = -theta
    radius = 20.0
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    q = np.empty(count, dtype=float)
    q[0] = 0.0
    if count > 1:
        segments = np.hypot(np.diff(x), np.diff(y))
        q[1:] = np.cumsum(segments)
    path_length = float(q[-1] + np.hypot(x[0] - x[-1], y[0] - y[-1]))
    speed = np.linspace(20.0, 24.0, count)
    curvature = np.full(count, (-1.0 if clockwise else 1.0) / radius)
    lateral = speed ** 2 * curvature
    return {
        "sample_index": np.arange(count),
        "track_s_m": q.copy(),
        "path_q_m": q.copy(),
        "bike_x_m": x,
        "bike_y_m": y,
        "left_boundary_x_m": (radius + 4.0) * np.cos(theta),
        "left_boundary_y_m": (radius + 4.0) * np.sin(theta),
        "right_boundary_x_m": (radius - 4.0) * np.cos(theta),
        "right_boundary_y_m": (radius - 4.0) * np.sin(theta),
        "speed_mps": speed,
        "longitudinal_acceleration_mps2": np.linspace(-2.0, 1.0, count),
        "lateral_acceleration_signed_mps2": lateral,
        "path_curvature_1pm": curvature,
        "roll_angle_rad": np.arctan(lateral / 9.81),
        "roll_rate_model_radps": np.linspace(-0.3, 0.3, count),
        "gear": np.full(count, 3),
        "rpm": np.linspace(7000.0, 9000.0, count),
        "_track_length_m": path_length,
        "_path_length_m": path_length,
    }


def _event(columns, index, event_type, corner="T1", *, roll_rate_radps=None):
    if roll_rate_radps is None:
        roll_rate_radps = float(columns["roll_rate_model_radps"][index])
    return {
        "corner": corner,
        "event_type": event_type,
        "sample_index": index,
        "track_s_m": float(columns["track_s_m"][index]),
        "path_q_m": float(columns["path_q_m"][index]),
        "x_m": float(columns["bike_x_m"][index]),
        "y_m": float(columns["bike_y_m"][index]),
        "speed_mps": float(columns["speed_mps"][index]),
        "longitudinal_acceleration_mps2": float(
            columns["longitudinal_acceleration_mps2"][index]),
        "curvature_1pm": float(columns["path_curvature_1pm"][index]),
        "lean_angle_deg": math.degrees(float(columns["roll_angle_rad"][index])),
        "roll_rate_radps": roll_rate_radps,
        "source_rule": "synthetic reviewed event",
        "confidence": "high",
    }


def _clean_columns(columns):
    return {key: value for key, value in columns.items() if not key.startswith("_")}


def _heading_vectors(heading):
    return np.column_stack((np.cos(heading), np.sin(heading)))


def test_closed_circle_heading_follows_tangent_direction():
    columns = _columns(64)
    heading = derive_closed_path_heading_rad(
        columns["bike_x_m"], columns["bike_y_m"], columns["path_q_m"],
        columns["_path_length_m"])
    theta = np.arange(64, dtype=float) * 2.0 * np.pi / 64
    expected = np.column_stack((-np.sin(theta), np.cos(theta)))
    assert np.allclose(_heading_vectors(heading), expected, atol=2e-3)
    assert not heading.flags.writeable


def test_nonuniform_circle_heading_is_spacing_aware_and_wraps():
    columns = _columns(96, nonuniform=True)
    heading = derive_closed_path_heading_rad(
        columns["bike_x_m"], columns["bike_y_m"], columns["path_q_m"],
        columns["_path_length_m"])
    theta = np.arctan2(columns["bike_y_m"], columns["bike_x_m"])
    expected = np.column_stack((-np.sin(theta), np.cos(theta)))
    assert np.max(np.linalg.norm(_heading_vectors(heading) - expected, axis=1)) < 0.03
    assert np.linalg.norm(_heading_vectors(heading)[0] - expected[0]) < 0.03


def test_clockwise_circle_heading_direction():
    columns = _columns(64, clockwise=True)
    heading = derive_closed_path_heading_rad(
        columns["bike_x_m"], columns["bike_y_m"], columns["path_q_m"],
        columns["_path_length_m"])
    theta = -np.arange(64, dtype=float) * 2.0 * np.pi / 64
    expected = np.column_stack((np.sin(theta), -np.cos(theta)))
    assert np.allclose(_heading_vectors(heading), expected, atol=2e-3)


def test_heading_wrap_is_compared_as_direction_not_raw_angle():
    columns = _columns(128)
    heading = derive_closed_path_heading_rad(
        columns["bike_x_m"], columns["bike_y_m"], columns["path_q_m"],
        columns["_path_length_m"])
    vectors = _heading_vectors(heading)
    assert np.allclose(np.linalg.norm(vectors, axis=1), 1.0, atol=1e-12)
    assert np.min(heading) < -3.0
    assert np.max(heading) > 3.0


def test_closed_heading_rejects_degenerate_local_path():
    with pytest.raises(ValueError, match="degenerate"):
        derive_closed_path_heading_rad([0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 2.0], 3.0)


def test_departure_seed_mapping_is_explicit_and_traceable():
    columns = _columns()
    clean = _clean_columns(columns)
    events = [
        _event(columns, 1, "local_max_speed"),
        _event(columns, 2, "braking_onset"),
        _event(columns, 3, "turn_in"),
        _event(columns, 4, "geometric_apex"),
        _event(columns, 5, "positive_drive_pickup"),
        _event(columns, 6, "maximum_braking"),
    ]
    seeds = build_departure_seeds(clean, events, path_length_m=columns["_path_length_m"])
    assert [seed.seed_type for seed in seeds] == [
        "missed_braking_candidate",
        "upright_overrun_candidate",
        "entry_lowside_turn_in_candidate",
        "entry_lowside_apex_candidate",
        "exit_highside_candidate",
    ]
    assert all(seed.source_rule == "synthetic reviewed event" for seed in seeds)
    assert all(seed.confidence == "high" for seed in seeds)
    assert all("candidate" in seed.interpretation for seed in seeds)


@pytest.mark.parametrize("field,delta", [
    ("speed_mps", 0.01),
    ("x_m", 0.01),
    ("track_s_m", 0.01),
    ("longitudinal_acceleration_mps2", 0.01),
    ("curvature_1pm", 0.001),
    ("lean_angle_deg", 1.0),
    ("roll_rate_radps", 0.01),
])
def test_departure_seed_rejects_stale_event_trajectory_pairs(field, delta):
    columns = _columns()
    clean = _clean_columns(columns)
    event = _event(columns, 2, "braking_onset")
    event[field] += delta
    with pytest.raises(ValueError, match="does not match trajectory"):
        build_departure_seeds(clean, [event], path_length_m=columns["_path_length_m"])


def test_nonfinite_event_roll_rate_is_explicit_missing_copy_and_trajectory_is_authoritative():
    columns = _columns()
    clean = _clean_columns(columns)
    event = _event(columns, 3, "turn_in", roll_rate_radps=math.nan)
    seed = build_departure_seeds(clean, [event], path_length_m=columns["_path_length_m"])[0]
    assert seed.roll_rate_radps == pytest.approx(columns["roll_rate_model_radps"][3])
    assert seed.lean_angle_rad == pytest.approx(columns["roll_angle_rad"][3])


@pytest.mark.parametrize("bad_index", [2.9, -1, 8, math.nan, math.inf, True, "2", [2]])
def test_event_sample_index_fails_closed(bad_index):
    columns = _columns()
    clean = _clean_columns(columns)
    event = _event(columns, 2, "braking_onset")
    event["sample_index"] = bad_index
    with pytest.raises(ValueError, match="sample_index|numeric scalar"):
        build_departure_seeds(clean, [event], path_length_m=columns["_path_length_m"])


def test_package_has_closed_loop_lengths_provenance_and_strong_read_only_arrays():
    columns = _columns()
    clean = _clean_columns(columns)
    event = _event(columns, 2, "braking_onset")
    package = build_runoff_input_package(
        clean,
        [event],
        track_length_m=columns["_track_length_m"],
        path_length_m=columns["_path_length_m"],
        scenario_metadata={
            "scenario_id": "synthetic",
            "simulator_commit": "deadbeef",
            "track_id": "unit_circle",
            "event_set_id": "synthetic-reviewed-events-v1",
        },
        warnings=("synthetic data",),
    )
    assert package.interface_version == RUNOFF_INTERFACE_VERSION
    assert package.coordinate_frame == "local_cartesian_m"
    assert package.track_length_m == pytest.approx(columns["_track_length_m"])
    assert package.path_length_m == pytest.approx(columns["_path_length_m"])
    assert package.sampling_convention == "closed loop; duplicated endpoint omitted"
    assert "path_heading_rad" in package.trajectory
    assert package.departure_seeds[0].seed_type == "upright_overrun_candidate"
    assert package.warnings == ("synthetic data",)
    with pytest.raises(TypeError):
        package.trajectory["new"] = np.array([1.0])
    with pytest.raises(ValueError):
        package.trajectory["speed_mps"][0] = 0.0
    with pytest.raises(ValueError):
        package.trajectory["speed_mps"].setflags(write=True)


def test_package_requires_identity_and_event_set_metadata():
    columns = _columns()
    clean = _clean_columns(columns)
    with pytest.raises(ValueError, match="scenario metadata"):
        build_runoff_input_package(
            clean,
            [],
            track_length_m=columns["_track_length_m"],
            path_length_m=columns["_path_length_m"],
            scenario_metadata={"scenario_id": "x", "simulator_commit": "y", "track_id": "z"},
        )


def test_required_trajectory_fields_fail_closed():
    columns = _columns()
    clean = _clean_columns(columns)
    clean.pop("lateral_acceleration_signed_mps2")
    with pytest.raises(ValueError, match="missing fields"):
        build_departure_seeds(clean, [], path_length_m=columns["_path_length_m"])


def test_malformed_event_provenance_is_rejected():
    columns = _columns()
    clean = _clean_columns(columns)
    event = _event(columns, 2, "braking_onset")
    event["source_rule"] = ""
    with pytest.raises(ValueError, match="source_rule"):
        build_departure_seeds(clean, [event], path_length_m=columns["_path_length_m"])


def test_unknown_confidence_is_rejected():
    columns = _columns()
    clean = _clean_columns(columns)
    event = _event(columns, 2, "braking_onset")
    event["confidence"] = "reviewed-ish"
    with pytest.raises(ValueError, match="confidence"):
        build_departure_seeds(clean, [event], path_length_m=columns["_path_length_m"])
