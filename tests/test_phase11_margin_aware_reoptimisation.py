import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_margin_aware_reoptimisation.py"
spec = importlib.util.spec_from_file_location("phase11_margin_reopt", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_projected_seed_clips_only_to_requested_bounds():
    reference = np.array([-2.0, -0.25, 0.5, 3.0])
    lower = np.array([-1.0, -1.0, -1.0, -1.0])
    upper = np.array([1.0, 1.0, 1.0, 2.0])

    projected = phase11.projected_seed(reference, lower, upper)

    np.testing.assert_allclose(projected, [-1.0, -0.25, 0.5, 2.0])


def test_backoff_seed_returns_first_feasible_scale():
    projected = np.array([2.0, -2.0])
    fallback = np.zeros(2)

    def feasible(candidate):
        return np.max(np.abs(candidate)) <= 0.6

    candidate, scale = phase11.backoff_seed_to_feasible(projected, fallback, feasible)

    assert scale == 0.25
    np.testing.assert_allclose(candidate, [0.5, -0.5])


def test_backoff_seed_accepts_projected_seed_without_change_when_feasible():
    projected = np.array([0.2, -0.3])
    fallback = np.zeros(2)

    candidate, scale = phase11.backoff_seed_to_feasible(
        projected, fallback, lambda controls: True)

    assert scale == 1.0
    np.testing.assert_allclose(candidate, projected)
