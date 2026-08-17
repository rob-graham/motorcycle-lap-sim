import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase12a_coaching_events.py"


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
