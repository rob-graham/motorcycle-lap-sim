import numpy as np
import pytest

from motorcycle_lap_sim.coaching import EventDetectionConfig, extract_coaching_events
from motorcycle_lap_sim.coaching.events import detect_corner_regions


def _columns():
    count = 241
    s = np.arange(count, dtype=float)
    lean = np.zeros(count)
    lean[50:91] = np.r_[np.linspace(4, 35, 16), np.full(10, 35), np.linspace(35, 4, 15)]
    lean[150:191] = -np.r_[np.linspace(4, 30, 16), np.full(10, 30), np.linspace(30, 4, 15)]
    curvature = lean / 2500.0
    speed = np.full(count, 40.0)
    speed[35:72] = np.linspace(40.0, 20.0, 37)
    speed[72:105] = np.linspace(20.0, 35.0, 33)
    speed[135:172] = np.linspace(39.0, 18.0, 37)
    speed[172:205] = np.linspace(18.0, 34.0, 33)
    acceleration = np.zeros(count)
    acceleration[35:70] = -2.0
    acceleration[70:76] = -0.1
    acceleration[80:110] = 1.0
    acceleration[135:170] = -2.2
    acceleration[170:176] = -0.1
    acceleration[180:210] = 0.9
    gear = np.full(count, 4.0)
    gear[100:] = 5.0
    return {
        "track_s_m": s,
        "path_q_m": s,
        "bike_x_m": s,
        "bike_y_m": np.zeros(count),
        "speed_mps": speed,
        "longitudinal_acceleration_mps2": acceleration,
        "path_curvature_1pm": curvature,
        "roll_angle_deg": lean,
        "roll_rate_model_radps": np.gradient(np.radians(lean)),
        "gear": gear,
        "rpm": np.full(count, 10000.0),
    }


def test_detect_corner_regions_uses_hysteresis_and_direction():
    regions = detect_corner_regions(_columns())
    assert len(regions) == 2
    assert regions[0][0] <= 50 < regions[0][1]
    assert regions[1][0] <= 150 < regions[1][1]


def test_extract_coaching_events_finds_core_rider_landmarks():
    events = extract_coaching_events(_columns(), expected_corner_count=2)
    by_corner = {
        corner: {event.event_type: event for event in events if event.corner == corner}
        for corner in ("T1", "T2")
    }
    for corner in ("T1", "T2"):
        assert {
            "local_max_speed", "braking_onset", "maximum_braking", "brake_release",
            "turn_in", "geometric_apex", "positive_drive_pickup", "corner_exit",
        }.issubset(by_corner[corner])
        assert by_corner[corner]["braking_onset"].track_s_m < by_corner[corner]["turn_in"].track_s_m
        assert by_corner[corner]["turn_in"].track_s_m <= by_corner[corner]["geometric_apex"].track_s_m
        assert by_corner[corner]["positive_drive_pickup"].track_s_m > by_corner[corner]["geometric_apex"].track_s_m
    assert any(event.event_type == "roll_transition" for event in events)
    assert any(event.event_type == "gear_shift" for event in events)


def test_extract_coaching_events_accepts_case_specific_corner_regions():
    columns = _columns()
    events = extract_coaching_events(
        columns, corner_regions=((50, 90), (150, 190)), expected_corner_count=2)
    turn_ins = [event for event in events if event.event_type == "turn_in"]
    assert [event.sample_index for event in turn_ins] == [50, 150]


def test_brake_release_is_not_fabricated_at_search_boundary():
    columns = _columns()
    acceleration = columns["longitudinal_acceleration_mps2"].copy()
    acceleration[35:76] = -2.0
    columns["longitudinal_acceleration_mps2"] = acceleration
    events = extract_coaching_events(
        columns, corner_regions=((50, 71),), expected_corner_count=1)
    types = {event.event_type for event in events if event.corner == "T1"}
    assert "braking_onset" in types
    assert "maximum_braking" in types
    assert "brake_release" not in types


def test_expected_corner_count_fails_closed():
    with pytest.raises(ValueError, match="expected 9"):
        extract_coaching_events(_columns(), expected_corner_count=9)


def test_config_rejects_inverted_lean_hysteresis():
    with pytest.raises(ValueError, match="lean-off"):
        EventDetectionConfig(corner_lean_on_deg=4.0, corner_lean_off_deg=6.0)
