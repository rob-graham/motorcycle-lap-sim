import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase12b_runoff_export.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("phase12b_export_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parser_retains_authoritative_mallala_defaults():
    module = _load_script()
    assert module.EXPECTED_CONTROLS_SHA256 == (
        "7e7916fb998a59b366441f5586134d7c8406d42232243141378cf83b22db1a3d")
    args = module.build_parser().parse_args(["controls.csv", "bundle"])
    assert args.delete_index == 26
    assert args.margin_m == pytest.approx(0.25)
    assert args.max_roll_rate_radps == pytest.approx(0.8)
    assert args.spacing_m == pytest.approx(0.125)
    assert args.boundary_check_spacing_m == pytest.approx(0.125)
    assert args.expected_lap_s == pytest.approx(71.396583646)
    assert args.lap_tolerance_s == pytest.approx(2e-6)
    assert "heading_rad" in module.RUNOFF_TRAJECTORY_FIELDS


@pytest.mark.parametrize("option,value", [
    ("--delete-index", "25"),
    ("--margin-m", "0.3"),
    ("--max-roll-rate-radps", "0.9"),
    ("--spacing-m", "0.25"),
    ("--boundary-check-spacing-m", "0.25"),
    ("--expected-lap-s", "71.4"),
    ("--lap-tolerance-s", "0.001"),
])
def test_retained_acceptance_rejects_noncanonical_setting(option, value):
    module = _load_script()
    args = module.build_parser().parse_args(["controls.csv", "bundle", option, value])
    with pytest.raises(RuntimeError, match="requires canonical settings"):
        module._require_retained_acceptance_provenance(
            args, module.EXPECTED_CONTROLS_SHA256)


def test_retained_acceptance_controls_hash_is_fixed_not_caller_overridable():
    module = _load_script()
    args = module.build_parser().parse_args(["controls.csv", "bundle"])
    with pytest.raises(RuntimeError, match="retained controls SHA-256 mismatch"):
        module._require_retained_acceptance_provenance(args, "0" * 64)
    module._require_retained_acceptance_provenance(
        args, module.EXPECTED_CONTROLS_SHA256)


def test_retained_acceptance_pins_added_corner_exit_and_full_seed_counts():
    module = _load_script()
    seeds = [SimpleNamespace(seed_type="existing_candidate") for _ in range(43)]
    seeds += [SimpleNamespace(
        seed_type="entry_lowside_corner_exit_candidate") for _ in range(9)]
    module._require_retained_seed_counts(seeds)
    with pytest.raises(RuntimeError, match="departure seed count mismatch"):
        module._require_retained_seed_counts(seeds[:-1])


def test_event_set_id_is_content_tied_and_deterministic():
    module = _load_script()
    event = type("Event", (), {})()
    for field in module.phase12a.EVENT_FIELDS:
        setattr(event, field, 1 if field == "sample_index" else "value")
    # dataclasses.asdict deliberately requires the production CoachingEvent shape.
    from motorcycle_lap_sim.coaching.events import CoachingEvent
    values = {field: getattr(event, field) for field in module.phase12a.EVENT_FIELDS
              if field != "speed_kph"}
    values.update({"sample_index": 1, "track_s_m": 1.0, "path_q_m": 1.0,
                   "x_m": 2.0, "y_m": 3.0, "speed_mps": 4.0,
                   "longitudinal_acceleration_mps2": -1.0, "curvature_1pm": 0.01,
                   "lean_angle_deg": 5.0, "roll_rate_radps": 0.2, "gear": 2, "rpm": 8000.0,
                   "display_on_map": True})
    first = CoachingEvent(**values)
    assert module._event_set_id([first]) == module._event_set_id([first])
    values["speed_mps"] = 4.1
    assert module._event_set_id([first]) != module._event_set_id([CoachingEvent(**values)])


def test_integration_assembly_fails_closed_on_incomplete_retained_trajectory():
    module = _load_script()
    args = module.build_parser().parse_args(["controls.csv", "bundle"])
    with pytest.raises(RuntimeError, match="lacks run-off fields"):
        module.assemble_runoff_package(
            {"columns": {}}, controls_sha256=module.EXPECTED_CONTROLS_SHA256,
            simulator_commit="abc123", args=args)
