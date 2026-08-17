import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_powell_benchmark.py"
spec = importlib.util.spec_from_file_location("phase11_powell_benchmark", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_parser_defaults_match_bounded_benchmark():
    args = phase11.build_parser().parse_args([
        "reviewed.csv",
        "start_dir",
        "output_dir",
    ])

    assert args.margins_m == phase11.DEFAULT_MARGINS_M
    assert args.max_roll_rate_radps == 0.8
    assert args.max_evaluations == 4000
    assert args.xtol == 1e-4
    assert args.ftol == 1e-8
    assert args.speed_backend == "numba"
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125


def test_feasible_objective_tracks_best_candidate_and_infeasible_count():
    class Evaluation:
        def __init__(self, feasible, lap_time_s):
            self.feasible = feasible
            self.lap_time_s = lap_time_s

    def evaluate(controls):
        value = float(np.asarray(controls)[0])
        if value < 0:
            return Evaluation(False, float("inf"))
        return Evaluation(True, 70.0 + value)

    objective = phase11.FeasibleObjective(evaluate)

    assert objective([2.0]) == 72.0
    assert objective([-1.0]) == phase11.INFEASIBLE_OBJECTIVE_S
    assert objective([1.0]) == 71.0
    assert objective.evaluations == 3
    assert objective.infeasible_evaluations == 1
    assert objective.best_lap_time_s == 71.0
    assert np.array_equal(objective.best_controls, np.array([1.0]))


def test_scipy_is_loaded_only_when_requested(monkeypatch):
    import builtins

    original_import = builtins.__import__

    def blocked_import(name, *args, **kwargs):
        if name == "scipy.optimize" or name.startswith("scipy"):
            raise ModuleNotFoundError("blocked for test")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked_import)
    with pytest.raises(RuntimeError, match="SciPy is required only for this benchmark"):
        phase11._load_scipy_minimize()
