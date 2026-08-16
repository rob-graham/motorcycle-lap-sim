import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_script():
    path = Path("scripts/r6_phase11b_hierarchical_warm_start.py").resolve()
    spec = importlib.util.spec_from_file_location("r6_phase11b_hierarchical_warm_start_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_periodic_linear_transfer_interpolates_across_lap_seam():
    diagnostic = _load_script()
    source_s = np.array([0.0, 25.0, 50.0, 75.0])
    source_controls = np.array([0.0, 1.0, 0.0, -1.0])
    target_s = np.array([87.5, 0.0, 12.5, 37.5, 62.5])

    transferred = diagnostic.periodic_linear_transfer(
        source_s, source_controls, target_s, 100.0)

    assert np.allclose(transferred, [-0.5, 0.0, 0.5, 0.5, -0.5])


def test_periodic_linear_transfer_wraps_target_stations():
    diagnostic = _load_script()
    source_s = np.array([0.0, 50.0])
    source_controls = np.array([0.0, 2.0])

    transferred = diagnostic.periodic_linear_transfer(
        source_s, source_controls, np.array([-25.0, 25.0, 125.0]), 100.0)

    assert np.allclose(transferred, [1.0, 1.0, 1.0])


def test_periodic_linear_transfer_rejects_unsorted_sources():
    diagnostic = _load_script()

    with pytest.raises(ValueError, match="strictly increasing"):
        diagnostic.periodic_linear_transfer(
            np.array([0.0, 50.0, 25.0]),
            np.array([0.0, 1.0, 2.0]),
            np.array([10.0]),
            100.0,
        )


def test_select_lowest_control_feasible_candidate_prefers_dimension_then_order():
    diagnostic = _load_script()
    rows = [
        {"name": "first_45", "control_count": 45, "feasible": True, "order": 0},
        {"name": "second_45", "control_count": 45, "feasible": True, "order": 1},
        {"name": "forty", "control_count": 40, "feasible": True, "order": 2},
        {"name": "thirty_infeasible", "control_count": 30, "feasible": False, "order": 3},
    ]

    selected = diagnostic.select_lowest_control_feasible_candidate(rows, 52)

    assert selected["name"] == "forty"


def test_select_lowest_control_feasible_candidate_rejects_reference_sized_only():
    diagnostic = _load_script()
    rows = [
        {"name": "infeasible_small", "control_count": 41, "feasible": False, "order": 0},
        {"name": "reference_sized", "control_count": 52, "feasible": True, "order": 1},
    ]

    with pytest.raises(RuntimeError, match="fewer controls than the reference policy"):
        diagnostic.select_lowest_control_feasible_candidate(rows, 52)
