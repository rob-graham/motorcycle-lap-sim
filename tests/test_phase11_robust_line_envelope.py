import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_robust_line_envelope.py"
spec = importlib.util.spec_from_file_location("phase11_robust_line_envelope_test", SCRIPT)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def test_parser_defaults():
    args = module.build_parser().parse_args([
        "baseline.csv", "reduced.csv", "relocated.csv", "output"])
    assert args.delete_index == 26
    assert args.relocate_index == 27
    assert args.relocate_shift_m == 5.0
    assert args.minimum_station_gap_m == 5.0
    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.representative_max_lap_delta_s == 0.05
    assert args.plot_dpi == 400


def test_optimiser_spread_envelope_returns_pointwise_statistics():
    offsets = {
        "a": np.array([-1.0, 0.0, 2.0]),
        "b": np.array([0.0, 1.0, 1.0]),
        "c": np.array([1.0, 2.0, 3.0]),
    }

    minimum, median, maximum, spread = module.optimiser_spread_envelope(
        offsets, ["a", "b", "c"])

    np.testing.assert_allclose(minimum, [-1.0, 0.0, 1.0])
    np.testing.assert_allclose(median, [0.0, 1.0, 2.0])
    np.testing.assert_allclose(maximum, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(spread, [2.0, 2.0, 2.0])


def test_pairwise_geometry_is_symmetric_and_uses_euclidean_displacement():
    points = {
        "a": np.array([[0.0, 0.0], [1.0, 0.0]]),
        "b": np.array([[0.0, 1.0], [1.0, 1.0]]),
        "c": np.array([[0.0, 2.0], [1.0, 2.0]]),
    }

    maximum, rms = module.pairwise_geometry(points, ["a", "b", "c"])

    np.testing.assert_allclose(maximum, [[0.0, 1.0, 2.0], [1.0, 0.0, 1.0], [2.0, 1.0, 0.0]])
    np.testing.assert_allclose(rms, maximum)


def test_representative_selection_excludes_ineligible_central_perturbation():
    labels = ["left", "perturbation", "right"]
    rms = np.array([
        [0.0, 1.0, 2.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 0.0],
    ])
    lap_times = {"left": 71.40, "perturbation": 71.41, "right": 71.42}
    eligible = {"left": True, "perturbation": False, "right": True}

    representative, medoid, fastest, means, delta, reason = module.select_representative_candidate(
        labels, rms, lap_times, eligible, 0.05)

    assert representative == "left"
    assert medoid == "left"
    assert fastest == "left"
    assert np.isnan(means["perturbation"])
    assert delta == pytest.approx(0.0)
    assert reason == "eligible_geometric_medoid_within_lap_delta"


def test_representative_selection_allows_eligible_medoid_with_small_lap_penalty():
    labels = ["a", "b", "c"]
    rms = np.array([
        [0.0, 1.0, 2.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 0.0],
    ])
    lap_times = {"a": 71.40, "b": 71.43, "c": 71.45}
    eligible = {label: True for label in labels}

    representative, medoid, fastest, means, delta, reason = module.select_representative_candidate(
        labels, rms, lap_times, eligible, 0.05)

    assert representative == "b"
    assert medoid == "b"
    assert fastest == "a"
    assert means["b"] == pytest.approx(1.0)
    assert delta == pytest.approx(0.03)
    assert reason == "eligible_geometric_medoid_within_lap_delta"


def test_representative_selection_falls_back_when_medoid_too_slow():
    labels = ["a", "b", "c"]
    rms = np.array([
        [0.0, 1.0, 2.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 0.0],
    ])
    lap_times = {"a": 71.40, "b": 71.48, "c": 71.50}
    eligible = {label: True for label in labels}

    representative, medoid, fastest, _, delta, reason = module.select_representative_candidate(
        labels, rms, lap_times, eligible, 0.05)

    assert medoid == "b"
    assert fastest == "a"
    assert representative == "a"
    assert delta == pytest.approx(0.08)
    assert reason == "fastest_eligible_fallback_medoid_exceeds_lap_delta"


def test_spread_envelope_rejects_mismatched_arrays():
    with pytest.raises(ValueError, match="identical"):
        module.optimiser_spread_envelope(
            {"a": np.array([0.0]), "b": np.array([0.0, 1.0])}, ["a", "b"])
