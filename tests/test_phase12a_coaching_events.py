import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from motorcycle_lap_sim.track import Track


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "r6_phase12a_coaching_events.py"
TRACK = ROOT / "examples" / "tracks" / "mallala_reference.yaml"


def _module():
    spec = importlib.util.spec_from_file_location("phase12a_test_module", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _assignment_columns(track_s, lean):
    return {
        "track_s_m": np.asarray(track_s, dtype=float),
        "roll_angle_deg": np.asarray(lean, dtype=float),
        "path_curvature_1pm": np.zeros_like(track_s, dtype=float),
    }


def test_parser_defaults_match_retained_phase11_scenario(tmp_path):
    module = _module()
    args = module.build_parser().parse_args(["representative.csv", str(tmp_path)])
    assert args.delete_index == 26
    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.expected_lap_s == 71.396583646


def test_main_writes_trajectory_from_calculated_retained_case(monkeypatch, tmp_path):
    module = _module()
    trajectory_export = SimpleNamespace(write_trajectory_csv=lambda path, columns: None)
    written = []
    trajectory_export.write_trajectory_csv = lambda path, columns: written.append((path, columns))
    columns = {"marker": np.asarray([1.0])}
    retained = {
        "track": object(),
        "bike": object(),
        "controls": np.zeros(51),
        "evaluation": SimpleNamespace(lap_time_s=71.396583646),
        "lap_delta_s": 0.0,
        "columns": columns,
        "raw_regions": (),
        "corner_regions": tuple((index, index) for index in range(9)),
        "corner_review": (),
        "events": (),
        "phase9": SimpleNamespace(sha256_file=lambda path: "retained-hash"),
        "trajectory_export": trajectory_export,
    }
    monkeypatch.setattr(module, "calculate_retained_case", lambda args: retained)
    monkeypatch.setattr(module, "_write_events_csv", lambda *args: None)
    monkeypatch.setattr(module, "_write_corner_review_csv", lambda *args: None)
    monkeypatch.setattr(module, "_limit_state_rows", lambda *args: [])
    monkeypatch.setattr(module, "_write_limit_state_csv", lambda *args: None)
    monkeypatch.setattr(module, "_write_coaching_overview", lambda *args, **kwargs: None)
    monkeypatch.setattr(module, "_write_speed_map", lambda *args, **kwargs: None)
    monkeypatch.setattr(module, "_write_detail", lambda *args, **kwargs: None)
    monkeypatch.setattr(module, "_write_limit_state_map", lambda *args, **kwargs: None)

    module.main(["representative.csv", str(tmp_path), "--plot-dpi", "1"])

    assert written == [(tmp_path / "phase12a_representative_trajectory.csv", columns)]


def test_map_contains_only_rider_facing_event_types():
    module = _module()
    assert module.MAP_EVENT_TYPES == (
        "braking_onset",
        "turn_in",
        "geometric_apex",
        "positive_drive_pickup",
        "corner_exit",
    )
    assert "maximum_braking" not in module.MAP_EVENT_TYPES
    assert "brake_release" not in module.MAP_EVENT_TYPES
    assert "gear_shift" not in module.MAP_EVENT_TYPES
    assert "roll_transition" not in module.MAP_EVENT_TYPES


def test_visual_output_names_are_the_phase12a_review_suite():
    module = _module()
    assert set(module.VISUAL_OUTPUT_FILENAMES.values()) == {
        "phase12a_coaching_overview.png", "phase12a_speed_map.png",
        "phase12a_T1_T3_detail.png", "phase12a_T4_T6_detail.png",
        "phase12a_T7_T9_detail.png",
        "phase12a_limit_state_map.png",
    }


def test_display_names_and_detail_callouts_are_grouped():
    module = _module()
    assert module.EVENT_ABBREVIATIONS["positive_drive_pickup"] == "DRIVE"
    assert module.EVENT_ABBREVIATIONS["speed_apex"] == "VMIN"
    assert "maximum_braking" in module.DETAIL_EVENT_TYPES

    from types import SimpleNamespace
    events = [
        SimpleNamespace(corner="T1", event_type="geometric_apex", track_s_m=100.0),
        SimpleNamespace(corner="T1", event_type="speed_apex", track_s_m=102.0),
        SimpleNamespace(corner="T1", event_type="maximum_curvature", track_s_m=103.0),
    ]
    rows = module._detail_callout_rows(events)
    assert len(rows) == 1
    assert "APEX/VMIN/K-MAX" in rows[0]


def test_limit_state_requires_capability_utilisation_and_trail_requires_brake_force():
    module = _module()
    state = module._classify_limit_state(2.0, 5.0, 8.0, "wheelie", "stoppie", -0.2)
    assert state[0] == "sub-max drive"
    state = module._classify_limit_state(4.95, 5.0, 8.0, "wheelie", "stoppie", -0.2)
    assert state[0] == "wheelie-limited drive"
    assert not module._is_trail_braking_proxy(20.0, 0.0)
    assert not module._is_trail_braking_proxy(0.0, 500.0)
    assert module._is_trail_braking_proxy(20.0, 500.0)


def test_mallala_reference_filter_rejects_three_straight_setup_regions():
    module = _module()
    track = Track.from_yaml(TRACK)
    track_s = np.arange(2558, dtype=float)
    raw_regions = (
        (0, 66),
        (80, 263),
        (269, 518),
        (694, 1020),
        (1087, 1493),
        (1507, 1568),
        (1574, 1706),
        (1711, 2147),
        (2155, 2314),
        (2319, 2386),
        (2390, 2524),
        (2533, 2557),
    )
    lean = np.zeros_like(track_s)
    signs = module._mallala_corner_turn_signs(track)
    intended = (1, 2, 3, 4, 6, 7, 8, 9, 10)
    for corner, raw_index in enumerate(intended):
        start, end = raw_regions[raw_index]
        lean[start:end + 1] = 10.0 * signs[corner]
    for raw_index in (0, 5, 11):
        start, end = raw_regions[raw_index]
        overlaps = [module._interval_overlap_m(
            track_s[start], track_s[end], *window)
            for window in module._mallala_corner_ownership_windows(track)]
        owner = int(np.argmax(overlaps))
        lean[start:end + 1] = -10.0 * signs[owner]

    selected, review = module._consolidate_mallala_corner_regions(
        track, _assignment_columns(track_s, lean), raw_regions, allow_unassigned=True)

    assert selected == tuple(raw_regions[index] for index in (1, 2, 3, 4, 6, 7, 8, 9, 10))
    assert [row["nominal_corner"] for row in review if row["status"] == "assigned"] == [
        "T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9"]
    assert {row["raw_region_index"] for row in review if row["status"] == "unassigned"} == {
        1, 6, 12}
    assert len(selected) == module.EXPECTED_MALLALA_CORNERS


def test_mallala_corner_windows_follow_reference_primitive_groups():
    module = _module()
    track = Track.from_yaml(TRACK)
    windows = module._mallala_corner_windows(track)

    assert len(windows) == 9
    assert np.allclose(windows[0], (178.593280835, 217.55908050670806), atol=1e-9)
    assert np.allclose(windows[2], (773.0806313974115, 961.758495318458), atol=1e-9)
    assert np.allclose(windows[-1], (2409.3274154371034, 2448.9194817642556), atol=1e-9)


def test_mallala_ownership_windows_partition_lap_at_adjacent_straight_midpoints():
    module = _module()
    track = Track.from_yaml(TRACK)
    nominal = module._mallala_corner_windows(track)
    ownership = module._mallala_corner_ownership_windows(track)
    assert ownership[0][0] == 0.0
    assert ownership[-1][1] == track.total_length_m
    assert all(ownership[index][1] == ownership[index + 1][0]
               for index in range(8))
    assert ownership[0][1] == pytest.approx(0.5 * (nominal[0][1] + nominal[1][0]))


def test_mallala_compound_corner_consolidates_multiple_raw_regions():
    module = _module()
    track = Track.from_yaml(TRACK)
    track_s = np.arange(2558, dtype=float)
    nominal = module._mallala_corner_windows(track)
    signs = module._mallala_corner_turn_signs(track)
    raw_regions = []
    lean = np.zeros_like(track_s)
    for corner, (start_m, end_m) in enumerate(nominal):
        start, end = int(np.floor(start_m)), int(np.ceil(end_m))
        if corner == 2:
            fragments = ((start, (start + end) // 2 - 2),
                         ((start + end) // 2 + 2, end))
        else:
            fragments = ((start, end),)
        for fragment in fragments:
            raw_regions.append(fragment)
            lean[fragment[0]:fragment[1] + 1] = 10.0 * signs[corner]

    selected, review = module._consolidate_mallala_corner_regions(
        track, _assignment_columns(track_s, lean), tuple(raw_regions))

    t3 = [row for row in review if row.get("nominal_corner") == "T3"]
    assert len(t3) == 2
    assert {row["consolidated_start_index"] for row in t3} == {raw_regions[2][0]}
    assert {row["consolidated_end_index"] for row in t3} == {raw_regions[3][1]}
    assert selected[2] == (raw_regions[2][0], raw_regions[3][1])


def _one_raw_region_per_nominal_corner(module, track, track_s):
    raw_regions = []
    lean = np.zeros_like(track_s, dtype=float)
    for sign, (start_m, end_m) in zip(
            module._mallala_corner_turn_signs(track), module._mallala_corner_windows(track)):
        region = (int(np.floor(start_m)), int(np.ceil(end_m)))
        raw_regions.append(region)
        lean[region[0]:region[1] + 1] = 10.0 * sign
    return raw_regions, lean


def test_mallala_unassigned_raw_region_fails_clearly_and_can_be_reviewed(tmp_path):
    module = _module()
    track = Track.from_yaml(TRACK)
    track_s = np.arange(2558, dtype=float)
    raw_regions, lean = _one_raw_region_per_nominal_corner(module, track, track_s)
    setup = (20, 40)
    raw_regions.append(setup)
    lean[setup[0]:setup[1] + 1] = -10.0 * module._mallala_corner_turn_signs(track)[0]
    columns = _assignment_columns(track_s, lean)

    with pytest.raises(ValueError, match="unassigned Mallala raw lean regions: 10"):
        module._consolidate_mallala_corner_regions(track, columns, tuple(raw_regions))

    regions, review = module._consolidate_mallala_corner_regions(
        track, columns, tuple(raw_regions), allow_unassigned=True)
    assert len(regions) == 9
    rejected = review[-1]
    assert rejected["raw_region_index"] == 10
    assert rejected["status"] == "unassigned"
    assert "turn direction conflicts" in rejected["assignment_rule"]

    output = tmp_path / "corner_review.csv"
    module._write_corner_review_csv(output, review)
    header, *rows = output.read_text(encoding="utf-8").splitlines()
    assert "peak_abs_curvature_1pm" in header
    assert "consolidated_start_track_s_m" in header
    assert len(rows) == len(raw_regions)


def test_mallala_compound_primitive_groups_remain_t3_t6_t7():
    module = _module()
    assert module.MALLALA_CORNER_PRIMITIVE_GROUPS[2] == (5, 6, 7)
    assert module.MALLALA_CORNER_PRIMITIVE_GROUPS[5] == (13, 14)
    assert module.MALLALA_CORNER_PRIMITIVE_GROUPS[6] == (16, 17)
