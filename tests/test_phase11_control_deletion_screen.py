import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "r6_phase11_control_deletion_screen.py"
spec = importlib.util.spec_from_file_location("phase11_control_deletion_screen", SCRIPT)
phase11 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phase11)


def test_parser_defaults_match_deletion_screen_contract():
    args = phase11.build_parser().parse_args(["controls.csv", "out"])

    assert args.margin_m == 0.25
    assert args.max_roll_rate_radps == 0.8
    assert args.speed_backend == "numba"
    assert args.optimisation_spacing_m == 1.0
    assert args.common_spacing_m == 0.125
    assert args.boundary_check_spacing_m == 0.125
    assert args.common_grid_top == 8
    assert args.plot_dpi == 400


def test_deletion_arrays_remove_exact_pair():
    stations = np.array([0.0, 10.0, 20.0, 30.0, 40.0])
    controls = np.array([1.0, 2.0, 3.0, 4.0, 5.0])

    reduced_stations, reduced_controls = phase11.deletion_arrays(stations, controls, 2)

    assert np.array_equal(reduced_stations, [0.0, 10.0, 30.0, 40.0])
    assert np.array_equal(reduced_controls, [1.0, 2.0, 4.0, 5.0])


def test_deletion_arrays_reject_bad_index_and_small_basis():
    with pytest.raises(ValueError, match="at least five"):
        phase11.deletion_arrays(np.arange(4.0), np.arange(4.0), 1)
    with pytest.raises(ValueError, match="outside"):
        phase11.deletion_arrays(np.arange(5.0), np.arange(5.0), 5)
    with pytest.raises(ValueError, match="integer"):
        phase11.deletion_arrays(np.arange(5.0), np.arange(5.0), True)


def test_reference_mallala_protects_all_primitive_boundaries_but_not_all_controls():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track, phase11.REFERENCE_PLANAR_CONTROL_POLICY)
    mask = phase11.protected_control_mask(track, stations)
    boundaries = phase11.primitive_boundary_stations(track)

    assert len(stations) == 52
    assert 0 < np.count_nonzero(mask) < len(stations)
    for boundary in boundaries:
        matching = np.flatnonzero(np.isclose(stations, boundary, rtol=0.0, atol=1e-9))
        assert len(matching) == 1
        assert mask[matching[0]]


def test_reference_mallala_has_deletable_control_on_first_primitive():
    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track, phase11.REFERENCE_PLANAR_CONTROL_POLICY)
    mask = phase11.protected_control_mask(track, stations)
    first_primitive_end = track.primitives[0].length_m

    interior = np.flatnonzero((stations > 0.0) & (stations < first_primitive_end) & ~mask)
    assert len(interior) >= 1


def test_geometry_displacement_zero_for_identical_spline():
    from motorcycle_lap_sim.racing_line import PeriodicPlanarSpline

    track = phase11.Track.from_yaml(phase11.phase9.DEFAULT_TRACK)
    stations = phase11.generate_planar_control_stations(
        track, phase11.REFERENCE_PLANAR_CONTROL_POLICY)
    sampled = phase11.sample_track_stations(track, stations)
    candidate = PeriodicPlanarSpline(
        stations, sampled.x_m, sampled.y_m, track.total_length_m)

    maximum, rms = phase11.geometry_displacement(
        candidate, candidate, track.total_length_m, 5.0)

    assert maximum == pytest.approx(0.0)
    assert rms == pytest.approx(0.0)
