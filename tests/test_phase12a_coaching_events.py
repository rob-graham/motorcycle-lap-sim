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
        "brake_release",
        "turn_in",
        "geometric_apex",
        "positive_drive_pickup",
        "corner_exit",
    )
    assert "maximum_braking" not in module.MAP_EVENT_TYPES
    assert "gear_shift" not in module.MAP_EVENT_TYPES
    assert "roll_transition" not in module.MAP_EVENT_TYPES


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

    selected, diagnostics = module._select_mallala_corner_regions(
        track, {"track_s_m": track_s}, raw_regions)

    assert selected == tuple(raw_regions[index] for index in (1, 2, 3, 4, 6, 7, 8, 9, 10))
    assert tuple(item[1] for item in diagnostics) == (2, 3, 4, 5, 7, 8, 9, 10, 11)
    assert len(selected) == module.EXPECTED_MALLALA_CORNERS


def test_mallala_corner_windows_follow_reference_primitive_groups():
    module = _module()
    track = Track.from_yaml(TRACK)
    windows = module._mallala_corner_windows(track)

    assert len(windows) == 9
    assert windows[0] == (178.593280835, 217.55918295035884)
    assert windows[2][0] == 773.080775992831
    assert windows[2][1] == 961.758497135805
    assert windows[-1][0] == 2409.326878374208
    assert windows[-1][1] == 2448.91948176492
