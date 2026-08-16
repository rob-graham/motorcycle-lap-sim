import importlib.util
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.motorcycle.config import load_motorcycle_config
from motorcycle_lap_sim.optimisation import (
    COARSE_PLANAR_CONTROL_POLICY,
    evaluate_planar_racing_line,
    generate_planar_control_stations,
)
from motorcycle_lap_sim.track import Track


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


def test_mallala_coarse_policy_accepts_generic_centreline_seed():
    """Guard the initial candidate used by the Phase 11B command."""
    track = Track.from_yaml("examples/tracks/mallala_reference.yaml")
    motorcycle = load_motorcycle_config("examples/motorcycles/r6_2017plus_reference.yaml")
    stations = generate_planar_control_stations(track, COARSE_PLANAR_CONTROL_POLICY)

    evaluation = evaluate_planar_racing_line(
        np.zeros(len(stations)), track, motorcycle, stations,
        sample_spacing_m=1.0,
        boundary_margin_m=0.25,
        boundary_check_spacing_m=0.25,
    )

    assert evaluation.feasible, evaluation.failure_reason
