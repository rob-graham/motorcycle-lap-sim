import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_robust_line_envelope.py"
spec = importlib.util.spec_from_file_location("phase11_robust_line_envelope_test", SCRIPT)
module = importlib.util.module_from_spec(spec)
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


def test_select_geometric_medoid_selects_existing_central_candidate():
    labels = ["a", "b", "c"]
    rms = np.array([
        [0.0, 1.0, 2.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 0.0],
    ])
    lap_times = {"a": 71.4, "b": 71.5, "c": 71.3}

    label, means = module.select_geometric_medoid(labels, rms, lap_times)

    assert label == "b"
    np.testing.assert_allclose(means, [1.5, 1.0, 1.5])


def test_select_geometric_medoid_uses_lap_time_only_as_tie_break():
    labels = ["a", "b"]
    rms = np.array([[0.0, 1.0], [1.0, 0.0]])
    lap_times = {"a": 71.5, "b": 71.4}

    label, _ = module.select_geometric_medoid(labels, rms, lap_times)

    assert label == "b"


def test_spread_envelope_rejects_mismatched_arrays():
    with pytest.raises(ValueError, match="identical"):
        module.optimiser_spread_envelope(
            {"a": np.array([0.0]), "b": np.array([0.0, 1.0])}, ["a", "b"])
