import importlib.util
from pathlib import Path

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


def test_parser_defaults_match_retained_phase11_scenario(tmp_path):
    module = _module()
    args = module.build_parser().parse_args(["representative.csv", str(tmp_path)])
    assert args.delete_index == 26
    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.expected_lap_s == 71.396583646


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
    }


def test_mallala_reference_filter_rejects_three_straight_setup_regions():
    module = _module()
    track = Track.from_yaml(TRACK)
    raw_regions = (
        (170, 230), (295, 455),
        (760, 825), (835, 970),  # two raw regions owned by compound T3
        (1150, 1430), (1620, 1680),
        (1710, 1810), (1820, 1970),  # compound T6
        (2220, 2260), (2270, 2310),  # compound T7
        (2315, 2390), (2395, 2460),
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

    selected, diagnostics = module._select_mallala_corner_regions(
        track, {"track_s_m": track_s, "roll_angle_deg": lean}, raw_regions)

    assert selected == tuple(raw_regions[index] for index in (1, 2, 3, 4, 6, 7, 8, 9, 10))
    assigned = [item for item in diagnostics if "corner" in item]
    assert tuple(item["raw_regions"] for item in assigned) == (
        (2,), (3,), (4,), (5,), (7,), (8,), (9,), (10,), (11,))
    assert {item["raw_region"] for item in diagnostics if "corner" not in item} == {1, 6, 12}
    assert len(selected) == module.EXPECTED_MALLALA_CORNERS
    assert selected[2] == (760, 970)
    assert selected[5] == (1710, 1970)
    assert selected[6] == (2220, 2310)
    assert [row["nominal_corner"] for row in review].count("T3") == 2


def test_mallala_unassigned_raw_region_fails_clearly():
    module = _module()
    track = Track.from_yaml(TRACK)
    raw_regions = ((170, 230),)
    columns = _assignment_columns(track, raw_regions, (1,))  # T1 turns negative
    with np.testing.assert_raises_regex(ValueError, "raw region 1 is unassigned"):
        module._consolidate_mallala_corner_regions(track, columns, raw_regions)


def test_mallala_corner_windows_follow_reference_primitive_groups():
    module = _module()
    track = Track.from_yaml(TRACK)
    windows = module._mallala_corner_windows(track)

    assert tuple(f"T{i}" for i in range(1, len(module.MALLALA_CORNER_PRIMITIVE_GROUPS) + 1)) == tuple(
        f"T{i}" for i in range(1, 10))
    assert module.MALLALA_CORNER_PRIMITIVE_GROUPS[2] == (5, 6, 7)
    assert module.MALLALA_CORNER_PRIMITIVE_GROUPS[5] == (13, 14)
    assert module.MALLALA_CORNER_PRIMITIVE_GROUPS[6] == (16, 17)
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

    selected, diagnostics = module._select_mallala_corner_regions(
        track, {"track_s_m": track_s, "roll_angle_deg": lean}, tuple(raw_regions))

    t3 = next(item for item in diagnostics if item.get("corner") == 3)
    assert len(t3["raw_regions"]) == 2
    assert selected[2] == (raw_regions[2][0], raw_regions[3][1])
