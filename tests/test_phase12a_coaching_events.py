import importlib.util
from pathlib import Path

import numpy as np

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
    assert "gear_shift" not in module.MAP_EVENT_TYPES
    assert "roll_transition" not in module.MAP_EVENT_TYPES


def _assignment_columns(track, raw_regions, signs):
    track_s = np.arange(int(np.ceil(track.primitive_start_s_m[-1])) + 1, dtype=float)
    lean = np.zeros_like(track_s)
    curvature = np.zeros_like(track_s)
    for (start, end), sign in zip(raw_regions, signs):
        lean[start:end + 1] = 20.0 * sign
        curvature[start:end + 1] = 0.01 * sign
    return {
        "track_s_m": track_s,
        "roll_angle_deg": lean,
        "path_curvature_1pm": curvature,
    }


def test_mallala_compound_corners_consolidate_multiple_raw_regions():
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
    signs = (-1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1)

    selected, review = module._consolidate_mallala_corner_regions(
        track, _assignment_columns(track, raw_regions, signs), raw_regions)

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
