"""Producer consistency checks for the cross-repository run-off contract."""

import importlib.util
import json
from pathlib import Path

from motorcycle_lap_sim.runoff import (
    DEPARTURE_SEED_FIELDS,
    RUNOFF_BUNDLE_VERSION,
    RUNOFF_INTERFACE_VERSION,
)
from motorcycle_lap_sim.runoff import export, interface


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "contracts" / "runoff_interface_0.1.0.json"


def _load_phase12b_export():
    path = ROOT / "scripts" / "r6_phase12b_runoff_export.py"
    spec = importlib.util.spec_from_file_location("phase12b_for_contract_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_machine_readable_runoff_contract_matches_producer():
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    phase12b = _load_phase12b_export()

    assert contract["runoff_interface_version"] == RUNOFF_INTERFACE_VERSION
    assert contract["runoff_bundle_version"] == RUNOFF_BUNDLE_VERSION
    assert contract["bundle"]["runoff_interface_version"] == RUNOFF_INTERFACE_VERSION
    assert contract["bundle"]["bundle_version"] == RUNOFF_BUNDLE_VERSION
    assert contract["bundle"]["artifacts"] == [
        export.MANIFEST_FILENAME,
        export.TRAJECTORY_FILENAME,
        export.DEPARTURE_SEEDS_FILENAME,
    ]

    seed_fields = tuple(
        field["serialized_name"] for field in contract["departure_seeds"]["fields"])
    assert seed_fields == DEPARTURE_SEED_FIELDS

    trajectory_fields = tuple(
        field["serialized_name"] for field in contract["trajectory"]["fields"])
    assert trajectory_fields == phase12b.RUNOFF_TRAJECTORY_FIELDS
    assert "heading_rad" in trajectory_fields

    expected_seed_types = {
        "missed_braking_candidate",
        "upright_overrun_candidate",
        "entry_lowside_turn_in_candidate",
        "entry_lowside_apex_candidate",
        "entry_lowside_corner_exit_candidate",
        "exit_highside_candidate",
    }
    assert set(contract["departure_seeds"]["supported_seed_types"]) == expected_seed_types
    assert {seed_type for seed_type, _ in interface._EVENT_TO_SEED.values()} == expected_seed_types

    mappings = {
        field["serialized_name"]: field.get("downstream_name")
        for field in contract["trajectory"]["fields"]
    }
    expected_mappings = {
        "bike_x_m": "x_m",
        "bike_y_m": "y_m",
        "lateral_acceleration_signed_mps2": "lateral_acceleration_mps2",
        "path_curvature_1pm": "curvature_1pm",
        "roll_angle_rad": "lean_angle_rad",
        "roll_rate_model_radps": "roll_rate_radps",
        "heading_rad": "heading_rad",
    }
    assert {name: mappings[name] for name in expected_mappings} == expected_mappings
