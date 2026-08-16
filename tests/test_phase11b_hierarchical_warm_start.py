import importlib.util
from pathlib import Path

import numpy as np
import pytest

from motorcycle_lap_sim.optimisation import (
    REFERENCE_PLANAR_CONTROL_POLICY,
    generate_planar_control_stations,
    planar_control_bounds,
)
from motorcycle_lap_sim.racing_line import build_smooth_racing_line_path
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


def test_hierarchical_coarse_centreline_is_feasible_and_coarser_than_reference():
    diagnostic = _load_script()
    track = Track.from_yaml(diagnostic.phase9.DEFAULT_TRACK)
    coarse_s = generate_planar_control_stations(
        track, diagnostic.HIERARCHICAL_COARSE_CONTROL_POLICY)
    reference_s = generate_planar_control_stations(
        track, REFERENCE_PLANAR_CONTROL_POLICY)
    lower, upper = planar_control_bounds(
        track, coarse_s, diagnostic.phase9.BOUNDARY_MARGIN_M)
    centreline = np.clip(np.zeros_like(coarse_s), lower, upper)

    smooth = build_smooth_racing_line_path(
        track,
        centreline,
        guide_s_m=coarse_s,
        sample_spacing_m=1.0,
        boundary_margin_m=diagnostic.phase9.BOUNDARY_MARGIN_M,
        boundary_check_spacing_m=diagnostic.phase9.BOUNDARY_CHECK_SPACING_M,
    )

    assert len(coarse_s) < len(reference_s)
    assert smooth.sampled_path.closed
