import math

import numpy as np
import pytest

from motorcycle_lap_sim.runoff import (
    RUNOFF_INTERFACE_VERSION,
    build_departure_seeds,
    build_runoff_input_package,
    derive_closed_path_heading_rad,
)


def _columns(count=8):
    theta = np.arange(count, dtype=float) * 2.0 * np.pi / count
    radius = 20.0
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    q = np.arange(count, dtype=float) * (2.0 * np.pi * radius / count)
    speed = np.linspace(20.0, 24.0, count)
    curvature = np.full(count, 1.0 / radius)
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
    }


def _event(columns, index, event_type, corner="T1", *, roll_rate_radps=0.1):
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


def test_closed_circle_heading_follows_tangent_direction():
    columns = _columns(32)
    heading = derive_closed_path_heading_rad(columns["bike_x_m"], columns["bike_y_m"])
    theta = np.arange(32, dtype=float) * 2.0 * np.pi / 32
    expected_x = -np.sin(theta)
    expected_y = np.cos(theta)
    actual_x = np.cos(heading)
    actual_y = np.sin(heading)
    assert np.allclose(actual_x, expected_x, atol=1e-12)
    assert np.allclose(actual_y, expected_y, atol=1e-12)
    assert not heading.flags.writeable


def test_closed_heading_rejects_degenerate_local_chord():
    with pytest.raises(ValueError, match="degenerate"):
        derive_closed_path_heading_rad([0.0, 1.0, 0.0], [0.0, 0.0, 0.0])


def test_departure_seed_mapping_is_explicit_and_traceable():
    columns = _columns()
    events = [
        _event(columns, 1, "local_max_speed"),
        _event(columns, 2, "braking_onset"),
        _event(columns, 3, "turn_in"),
        _event(columns, 4, "geometric_apex"),
        _event(columns, 5, "positive_drive_pickup"),
        _event(columns, 6, "maximum_braking"),
    ]
    seeds = build_departure_seeds(columns, events)
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


def test_departure_seed_rejects_stale_event_trajectory_pair():
    columns = _columns()
    event = _event(columns, 2, "braking_onset")
    event["speed_mps"] += 0.01
    with pytest.raises(ValueError, match="does not match trajectory"):
        build_departure_seeds(columns, [event])


def test_nonfinite_event_roll_rate_falls_back_to_model_field():
    columns = _columns()
    event = _event(columns, 3, "turn_in", roll_rate_radps=math.nan)
    seed = build_departure_seeds(columns, [event])[0]
    assert seed.roll_rate_radps == pytest.approx(columns["roll_rate_model_radps"][3])


def test_package_is_versioned_read_only_and_has_no_physics_profile():
    columns = _columns()
    event = _event(columns, 2, "braking_onset")
    package = build_runoff_input_package(
        columns,
        [event],
        scenario_metadata={
            "scenario_id": "synthetic",
            "simulator_commit": "deadbeef",
            "track_id": "unit_circle",
        },
        warnings=("synthetic data",),
    )
    assert package.interface_version == RUNOFF_INTERFACE_VERSION
    assert package.coordinate_frame == "local_cartesian_m"
    assert "path_heading_rad" in package.trajectory
    assert package.departure_seeds[0].seed_type == "upright_overrun_candidate"
    assert package.warnings == ("synthetic data",)
    assert "friction" not in package.scenario_metadata
    with pytest.raises(TypeError):
        package.trajectory["new"] = np.array([1.0])
    with pytest.raises(ValueError):
        package.trajectory["speed_mps"][0] = 0.0


def test_package_requires_identity_metadata():
    columns = _columns()
    with pytest.raises(ValueError, match="scenario metadata"):
        build_runoff_input_package(columns, [], scenario_metadata={"scenario_id": "x"})


def test_required_trajectory_fields_fail_closed():
    columns = _columns()
    columns.pop("lateral_acceleration_signed_mps2")
    with pytest.raises(ValueError, match="missing fields"):
        build_departure_seeds(columns, [])
