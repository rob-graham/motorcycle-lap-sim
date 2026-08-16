import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_script():
    path = Path("scripts/r6_phase11a_multistart_assurance.py").resolve()
    spec = importlib.util.spec_from_file_location("r6_phase11a_multistart_assurance_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bounded_smooth_perturbation_is_deterministic_bounded_and_bidirectional():
    diagnostic = _load_script()
    stations = np.linspace(0.0, 900.0, 10, endpoint=False)
    base = np.zeros(10)
    lower = np.full(10, -0.4)
    upper = np.full(10, 0.6)

    plus_a = diagnostic.bounded_smooth_perturbation(
        base, stations, 1000.0, lower, upper, 1.0, +1)
    plus_b = diagnostic.bounded_smooth_perturbation(
        base, stations, 1000.0, lower, upper, 1.0, +1)
    minus = diagnostic.bounded_smooth_perturbation(
        base, stations, 1000.0, lower, upper, 1.0, -1)

    assert np.array_equal(plus_a, plus_b)
    assert np.all(plus_a >= lower)
    assert np.all(plus_a <= upper)
    assert np.all(minus >= lower)
    assert np.all(minus <= upper)
    assert not np.array_equal(plus_a, minus)
    assert np.max(np.abs(plus_a)) <= pytest.approx(0.6)


def test_rank_candidates_uses_common_grid_and_stable_name_tie_break():
    diagnostic = _load_script()
    rows = [
        {"start_name": "zeta", "common_grid_lap_s": 71.2},
        {"start_name": "beta", "common_grid_lap_s": 71.1},
        {"start_name": "alpha", "common_grid_lap_s": 71.1},
    ]

    ranked = diagnostic.rank_candidates(rows)
    assert [row["start_name"] for row in ranked] == ["alpha", "beta", "zeta"]
