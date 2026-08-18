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
    acceleration[72:110] = 1.0
    acceleration[135:170] = -2.2
    acceleration[170:176] = -0.1
    acceleration[172:210] = 0.9
    gear = np.full(count, 4.0)
    gear[100:] = 5.0
    bike_y = np.zeros(count)
    bike_y[50:91] = np.r_[np.linspace(0, 3.0, 21), np.linspace(3.0, 0, 20)]
    bike_y[150:191] = -np.r_[np.linspace(0, 3.0, 21), np.linspace(3.0, 0, 20)]
    return {
        "track_s_m": s,
        "path_q_m": s,
        "bike_x_m": s,
        "bike_y_m": bike_y,
        "left_boundary_x_m": s,
        "left_boundary_y_m": np.full(count, 5.0),
        "right_boundary_x_m": s,
        "right_boundary_y_m": np.full(count, -5.0),
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
    assert all(50 <= event.sample_index <= 190 for event in turn_ins)


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


def _event_map(columns, region):
    return {event.event_type: event for event in extract_coaching_events(
        columns, corner_regions=(region,), expected_corner_count=1)}


def test_geometric_apex_uses_inside_physical_edge_not_maximum_curvature():
    columns = _columns()
    curvature = columns["path_curvature_1pm"].copy()
    curvature[55] = 0.2
    columns["path_curvature_1pm"] = curvature
    events = _event_map(columns, (50, 90))
    assert events["geometric_apex"].sample_index == 70
    assert events["maximum_curvature"].sample_index == 55
    assert "inside physical left track edge" in events["geometric_apex"].source_rule


def test_geometric_apex_selects_right_edge_for_right_corner():
    columns = _columns()
    events = _event_map(columns, (150, 190))
    assert events["geometric_apex"].sample_index == 170
    assert "inside physical right track edge" in events["geometric_apex"].source_rule


@pytest.mark.parametrize("mode, expected", [
    ("crossing", True), ("already_positive", False), ("spike", False),
])
def test_positive_drive_requires_sustained_transition(mode, expected):
    columns = _columns()
    acceleration = np.zeros_like(columns["longitudinal_acceleration_mps2"])
    if mode == "crossing":
        acceleration[72:90] = 0.5
    elif mode == "already_positive":
        acceleration[50:90] = 0.5
    else:
        acceleration[72:74] = 0.5
    columns["longitudinal_acceleration_mps2"] = acceleration
    events = _event_map(columns, (50, 90))
    assert ("positive_drive_pickup" in events) is expected


def test_rider_turn_and_exit_ignore_raw_lean_region_extremes():
    events = _event_map(_columns(), (45, 110))
    assert events["turn_in"].sample_index > 45
    assert events["corner_exit"].sample_index < 110
    assert events["turn_in"].sample_index <= events["geometric_apex"].sample_index
    assert events["geometric_apex"].sample_index <= events["corner_exit"].sample_index


def test_compound_clearance_fallback_is_deterministic():
    columns = _columns()
    columns["bike_y_m"] = np.zeros_like(columns["bike_y_m"])
    first = _event_map(columns, (50, 90))
    second = _event_map(columns, (50, 90))
    assert first["turn_in"].sample_index == second["turn_in"].sample_index
    assert first["turn_in"].confidence == "medium"
    assert "fallback" in first["turn_in"].source_rule


def test_turn_uses_dominant_roll_in_before_curvature_peak_not_clearance_move():
    columns = _columns()
    # Hold the line away from the inside-edge movement until after the signed
    # lean/curvature build is established.
    columns["bike_y_m"][50:66] = 0.0
    columns["path_curvature_1pm"][65] = 0.2
    events = _event_map(columns, (50, 90))
    assert events["turn_in"].sample_index < 65
    assert events["turn_in"].sample_index < events["maximum_curvature"].sample_index
    assert "demanded-lean/curvature build" in events["turn_in"].source_rule


def test_exit_waits_for_apex_vmin_kmax_and_substantial_recovery_unwind():
    columns = _columns()
    columns["path_curvature_1pm"][75] = 0.2
    # A tiny apex departure precedes the real track-out.
    columns["bike_y_m"][70:78] = np.linspace(3.0, 2.9, 8)
    columns["bike_y_m"][78:91] = np.linspace(2.9, 0.0, 13)
    events = _event_map(columns, (50, 90))
    completion = max(events[name].sample_index for name in
                     ("geometric_apex", "speed_apex", "maximum_curvature"))
    assert events["corner_exit"].sample_index >= completion
    assert events["corner_exit"].sample_index > 78


def test_release_is_final_sustained_recovery_after_renewed_braking():
    columns = _columns()
    acceleration = np.zeros_like(columns["longitudinal_acceleration_mps2"])
    acceleration[35:45] = -2.0
    acceleration[45:51] = 0.0       # early sustained-looking recovery
    acceleration[51:61] = -1.8      # renewed meaningful pulse
    acceleration[61:] = 0.0
    columns["longitudinal_acceleration_mps2"] = acceleration
    events = _event_map(columns, (50, 90))
    assert events["brake_release"].sample_index == 61
    assert "final sustained" in events["brake_release"].source_rule


def test_drive_detects_crossing_at_search_start_and_is_not_exit_gated():
    columns = _columns()
    acceleration = np.zeros_like(columns["longitudinal_acceleration_mps2"])
    acceleration[71] = -0.2
    acceleration[72:] = 0.8
    columns["longitudinal_acceleration_mps2"] = acceleration
    events = _event_map(columns, (50, 90))
    assert events["positive_drive_pickup"].sample_index == 72

    # Force EXIT to the corner bound; DRIVE remains independently detectable.
    columns["bike_y_m"][:] = 0.0
    events = _event_map(columns, (50, 90))
    assert events["positive_drive_pickup"].sample_index == 72


def test_drive_rejects_early_patch_before_sustained_braking_and_uses_final_crossing():
    columns = _columns()
    acceleration = np.zeros_like(columns["longitudinal_acceleration_mps2"])
    acceleration[71] = -0.2
    acceleration[72:80] = 0.8
    acceleration[80:87] = -1.0
    acceleration[87:] = 0.8
    columns["longitudinal_acceleration_mps2"] = acceleration
    events = _event_map(columns, (50, 90))
    assert events["positive_drive_pickup"].sample_index == 87
