import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def _load_script():
    path = Path("scripts/r6_phase11d_global_latent_screen.py").resolve()
    spec = importlib.util.spec_from_file_location("r6_phase11d_global_latent_screen_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_first_primes_are_deterministic_and_correct():
    diagnostic = _load_script()
    assert diagnostic.first_primes(8) == (2, 3, 5, 7, 11, 13, 17, 19)


def test_radical_inverse_known_values():
    diagnostic = _load_script()
    assert diagnostic.radical_inverse(1, 2) == 0.5
    assert diagnostic.radical_inverse(2, 2) == 0.25
    assert diagnostic.radical_inverse(3, 2) == 0.75
    assert diagnostic.radical_inverse(2, 3) == pytest.approx(2 / 3)


def test_halton_latent_candidates_include_centreline_and_stay_bounded():
    diagnostic = _load_script()
    candidates = diagnostic.halton_latent_candidates(17, 5, 4.0)

    assert candidates.shape == (17, 5)
    assert np.array_equal(candidates[0], np.zeros(5))
    assert np.all(candidates <= 4.0)
    assert np.all(candidates >= -4.0)
    assert np.array_equal(
        candidates, diagnostic.halton_latent_candidates(17, 5, 4.0))


def test_best_screened_candidate_uses_lap_then_input_order_and_counts_feasible():
    diagnostic = _load_script()
    candidates = np.arange(12, dtype=float).reshape(4, 3)
    evaluations = [
        SimpleNamespace(feasible=False, lap_time_s=float("inf")),
        SimpleNamespace(feasible=True, lap_time_s=10.0),
        SimpleNamespace(feasible=True, lap_time_s=9.0),
        SimpleNamespace(feasible=True, lap_time_s=9.0),
    ]

    candidate, evaluation, index, feasible_count = diagnostic.best_screened_candidate(
        candidates, evaluations)

    assert index == 2
    assert evaluation is evaluations[2]
    assert np.array_equal(candidate, candidates[2])
    assert feasible_count == 3


def test_best_screened_candidate_rejects_all_infeasible():
    diagnostic = _load_script()
    with pytest.raises(RuntimeError, match="no feasible candidate"):
        diagnostic.best_screened_candidate(
            np.zeros((2, 4)),
            [SimpleNamespace(feasible=False, lap_time_s=float("inf")) for _ in range(2)],
        )
